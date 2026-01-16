#include "Engine/app.h"
#include "Labs/Fluid Simulation/Case2dSPH.h"
#include "Labs/Common/ImGuiHelper.h"
#include <iostream>

namespace VCX::Labs::Animation {
    Case2dSPH::Case2dSPH():
        _program(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/SPH2d.vert"),
                Engine::GL::SharedShader("assets/shaders/SPH2d.frag") })),
        _particlesItem(Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream , 0)
            .Add<glm::vec3>("velocity", Engine::GL::DrawFrequency::Stream , 1), Engine::GL::PrimitiveType::Points) {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
        ResetSystem();
    }

    void Case2dSPH::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Reset System")) ResetSystem();
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
            ImGui::SliderInt("Part. Num (Col)", &_SPHParams.InitParticleColNumber, 10, 80);
            ImGui::SliderInt("Part. Num (Row)", &_SPHParams.InitParticleRowNumber, 10, 80);
            ImGui::SliderInt("Iterations per Frame", &_SPHParams.IterationsPerFrame, 1, 20);
            ImGui::SliderFloat("Viscosity", &_SPHParams.Viscosity, .01f, 1.f);
        }
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Appearance")) {
            ImGui::SliderFloat("Part. Size", &_particleSize, 1, 20);
        }
        ImGui::Spacing();
    }

    Common::CaseRenderResult Case2dSPH::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        _viewportSize = desiredSize;
        if (! _stopped) _SPHSystem2d.Advance2dSPHSystem(_SPHParams);

        _particlesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(_SPHSystem2d.positions));
        _particlesItem.UpdateVertexBuffer("velocity", Engine::make_span_bytes<glm::vec3>(_SPHSystem2d.velocities));
        _frame.Resize(desiredSize);
        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());
        gl_using(_frame);

        glPointSize(_particleSize);
        _program.GetUniforms().SetByName("u_MaxSpeed", 7.5f);
        _particlesItem.Draw({ _program.Use() });
       
        glPointSize(1.f);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void Case2dSPH::OnProcessInput(ImVec2 const & pos) {
        auto            window  = ImGui::GetCurrentWindow();
        ImGuiIO const & io      = ImGui::GetIO();
        bool            anyHeld = false;
        bool            hover   = false;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hover, &anyHeld);
        bool leftHeld  = anyHeld && ImGui::IsMouseDown(ImGuiMouseButton_Left);
        bool rightHeld = anyHeld && ImGui::IsMouseDown(ImGuiMouseButton_Right);
        
        if (!anyHeld) {
            _SPHParams.ExternalForceStrength = 0;
        } else {
            glm::mat4 projection = _camera.GetProjectionMatrix((float(_viewportSize.first) / _viewportSize.second));
            glm::mat4 view       = _camera.GetViewMatrix();
            glm::vec2 screensize = glm::vec2((float) _viewportSize.first, (float) _viewportSize.second);
            
            glm::vec2 worldLeftTop { -_SPHParams.BoundaryWidth / 2, _SPHParams.BoundaryHeight / 2 };
            glm::vec2 worldRightBottom { _SPHParams.BoundaryWidth / 2, -_SPHParams.BoundaryHeight / 2 };

            auto ScreenPos = [&](glm::vec2 worldPos, glm::mat4 proj, glm::mat4 view) {
                glm::vec4 clipPos = proj * view * glm::vec4(worldPos, 0, 1);
                glm::vec3 ndcPos = glm::vec3(clipPos) / clipPos.w;
                glm::vec2 screenPos { (ndcPos.x + 1.f) * 0.5f * screensize.x, (1.f - (ndcPos.y + 1.f) * 0.5f) * screensize.y };
                return screenPos;
            };

            glm::vec2 screenLeftTop = ScreenPos(worldLeftTop, projection, view);
            glm::vec2 screenRightBottom = ScreenPos(worldRightBottom, projection, view);
            if (pos.x < screenLeftTop.x || pos.x > screenRightBottom.x || pos.y < screenLeftTop.y || pos.y > screenRightBottom.y) {
                _SPHParams.ExternalForceStrength = 0;
            } else {
                _SPHParams.ExternalForceStrength = (20.f) * (leftHeld ? 1.f : -1.f);
                _SPHParams.ExternalForcePos      = glm::vec3 {
                    (pos.x - screenLeftTop.x) / (screenRightBottom.x - screenLeftTop.x) * _SPHParams.BoundaryWidth + worldLeftTop.x,
                    -(pos.y - screenLeftTop.y) / (screenRightBottom.y - screenLeftTop.y) * _SPHParams.BoundaryHeight + worldLeftTop.y,
                    0.f
                };
                _SPHSystem2d.params = _SPHParams;
            }
        }
    }

    void Case2dSPH::ResetSystem() {
        _SPHSystem2d = SPHSystem2d(_SPHParams);
        _SPHSystem2d.positions.clear();
        _SPHSystem2d.velocities.clear();

        auto addParticle = [&](float x, float y) {
            _SPHSystem2d.positions.push_back(glm::vec3(x, y, 0));
            _SPHSystem2d.velocities.push_back(glm::vec3(0));
        };

        float step = _SPHParams.H * 0.75f;
        for (std::size_t i = 0; i < _SPHSystem2d.params.InitParticleColNumber; ++i) {
            for (std::size_t j = 0; j < _SPHSystem2d.params.InitParticleRowNumber; ++j) {
                glm::vec2 pos = _SPHParams.InitPos;
                addParticle(pos.x + step * i, pos.y + step * j);
            }
        }

        _SPHSystem2d.numFluidParticles = _SPHSystem2d.positions.size();

        float w    = _SPHParams.BoundaryWidth;
        float h    = _SPHParams.BoundaryHeight;
        
        step = _SPHParams.H * 0.5f;
        for (float x = -w / 2; x <= w / 2; x += step) {
            addParticle(x, -h / 2);
            addParticle(x, h / 2);
        }
        for (float y = -h / 2; y <= h / 2; y += step) {
            addParticle(-w / 2, y);
            addParticle(w / 2, y);
        }
        _SPHSystem2d.numTotalParticles = _SPHSystem2d.positions.size();
        _SPHSystem2d.densities.assign(_SPHSystem2d.numTotalParticles, 0.0f);
        _SPHSystem2d.pressures.assign(_SPHSystem2d.numTotalParticles, 0.0f);
        _SPHSystem2d.forces.assign(_SPHSystem2d.numTotalParticles, glm::vec3(0.0f));
    }
}
