#include "Engine/app.h"
#include "Labs/Fluid Simulation/Case3dSPH.h"
#include "Labs/Common/ImGuiHelper.h"
#include <iostream>

namespace VCX::Labs::Animation {
    Case3dSPH::Case3dSPH():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/SPH3d.vert"),
                                        Engine::GL::SharedShader("assets/shaders/SPH3d.frag") })),
        _particlesItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0).Add<glm::vec3>("velocity", Engine::GL::DrawFrequency::Stream, 1), Engine::GL::PrimitiveType::Points) {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
        ResetSystem();
    }

    void Case3dSPH::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Reset System")) ResetSystem();
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
            ImGui::SliderInt("Part. Num X", &_SPHParams.InitXNumber, 5, 20);
            ImGui::SliderInt("Part. Num Y", &_SPHParams.InitYNumber, 5, 40);
            ImGui::SliderInt("Part. Num Z", &_SPHParams.InitZNumber, 5, 20);
            ImGui::SliderFloat("Gravity", &_SPHParams.Gravity, .1f, 20.f);
            ImGui::SliderFloat("Viscosity", &_SPHParams.Viscosity, .01f, 1.f);
        }
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Appearance")) {
            ImGui::SliderFloat("Part. Size", &_particleSize, 1, 20);
        }
        ImGui::Spacing();
    }

    Common::CaseRenderResult Case3dSPH::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (! _stopped) _SPHSystem3d.Advance3dSPHSystem(_SPHParams);

        std::vector<glm::vec3> positions(_SPHSystem3d.positions.begin(), _SPHSystem3d.positions.begin() + _SPHSystem3d.numFluidParticles);
        std::vector<glm::vec3> velocities(_SPHSystem3d.velocities.begin(), _SPHSystem3d.velocities.begin() + _SPHSystem3d.numFluidParticles);

        _particlesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(positions));
        _particlesItem.UpdateVertexBuffer("velocity", Engine::make_span_bytes<glm::vec3>(velocities));
        _frame.Resize(desiredSize);
        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());
        gl_using(_frame);

        glPointSize(_particleSize);
        _program.GetUniforms().SetByName("u_MaxSpeed", 5.f);
        _particlesItem.Draw({ _program.Use() });

        glPointSize(1.f);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void Case3dSPH::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void Case3dSPH::ResetSystem() {
        _SPHSystem3d = SPHSystem3d(_SPHParams);
        _SPHSystem3d.positions.clear();
        _SPHSystem3d.velocities.clear();

        auto addParticle = [&](float x, float y, float z) {
            _SPHSystem3d.positions.push_back(glm::vec3(x, y, z));
            _SPHSystem3d.velocities.push_back(glm::vec3(0));
        };

        // Initialize Fluid Particles in a 3D block
        float step = _SPHParams.H * 0.75f;
        for (int i = 0; i < _SPHParams.InitXNumber; ++i) {
            for (int j = 0; j < _SPHParams.InitYNumber; ++j) {
                for (int k = 0; k < _SPHParams.InitZNumber; ++k) {
                    glm::vec3 pos = _SPHParams.InitPos;
                    addParticle(pos.x + step * i, pos.y + step * j, pos.z + step * k);
                }
            }
        }

        _SPHSystem3d.numFluidParticles = _SPHSystem3d.positions.size();

        // Initialize Boundary Particles (Box Container)
        float bx = _SPHParams.BoundaryX;
        float by = _SPHParams.BoundaryY;
        float bz = _SPHParams.BoundaryZ;

        step         = _SPHParams.H * 0.5f;
        int   layers = 0;
        float margin = layers * step;

        auto addBlock = [&](float x0, float x1, float y0, float y1, float z0, float z1) {
            for (float x = x0; x <= x1 + 1e-5f; x += step) {
                for (float y = y0; y <= y1 + 1e-5f; y += step) {
                    for (float z = z0; z <= z1 + 1e-5f; z += step) {
                        addParticle(x, y, z);
                    }
                }
            }
        };

        // Floor and Ceiling (Full X, Full Z)
        addBlock(-bx / 2 - margin, bx / 2 + margin, -by / 2 - margin, -by / 2, -bz / 2 - margin, bz / 2 + margin);
        //addBlock(-bx / 2 - margin, bx / 2 + margin, by / 2, by / 2 + margin, -bz / 2 - margin, bz / 2 + margin);

        // Left and Right Walls (Inner Y, Full Z)
        addBlock(-bx / 2 - margin, -bx / 2, -by / 2 + step, by / 2 - step, -bz / 2 - margin, bz / 2 + margin);
        addBlock(bx / 2, bx / 2 + margin, -by / 2 + step, by / 2 - step, -bz / 2 - margin, bz / 2 + margin);

        // Front and Back Walls (Inner Y, Inner X)
        addBlock(-bx / 2 + step, bx / 2 - step, -by / 2 + step, by / 2 - step, -bz / 2 - margin, -bz / 2);
        addBlock(-bx / 2 + step, bx / 2 - step, -by / 2 + step, by / 2 - step, bz / 2, bz / 2 + margin);

        _SPHSystem3d.numTotalParticles = _SPHSystem3d.positions.size();
        _SPHSystem3d.densities.assign(_SPHSystem3d.numTotalParticles, 0.0f);
        _SPHSystem3d.pressures.assign(_SPHSystem3d.numTotalParticles, 0.0f);
        _SPHSystem3d.forces.assign(_SPHSystem3d.numTotalParticles, glm::vec3(0.0f));
    }
} // namespace VCX::Labs::Animation
