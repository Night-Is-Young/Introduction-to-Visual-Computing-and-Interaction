#include "Labs/Fluid Simulation/Case3dSPHRendering.h"
#include "Engine/app.h"
#include "Labs/Common/ImGuiHelper.h"
#include <iostream>

namespace VCX::Labs::Animation {
    Case3dSPHRendering::Case3dSPHRendering():
        _programDepth(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/depth.vert"),
                                        Engine::GL::SharedShader("assets/shaders/depth.frag") })),
        _programSmooth(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/full_screen.vert"),
                                        Engine::GL::SharedShader("assets/shaders/smooth.frag") })),
        _programShade(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/full_screen.vert"),
                                        Engine::GL::SharedShader("assets/shaders/shade.frag") })),
        _particlesItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0).Add<glm::vec3>("velocity", Engine::GL::DrawFrequency::Stream, 1), Engine::GL::PrimitiveType::Points) {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
        ResetSystem();
    }

    void Case3dSPHRendering::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Reset System")) ResetSystem();
            ImGui::SameLine();
            if (ImGui::Button(_stopped ? "Start Simulation" : "Stop Simulation")) _stopped = ! _stopped;
            ImGui::SliderInt("Part. Num X", &_SPHParams.InitXNumber, 5, 20);
            ImGui::SliderInt("Part. Num Y", &_SPHParams.InitYNumber, 5, 40);
            ImGui::SliderInt("Part. Num Z", &_SPHParams.InitZNumber, 5, 20);
            ImGui::SliderFloat("Min Render Density", &_minDensity, 0.0f, 2000.0f);
        }
        ImGui::Spacing();
    }

    Common::CaseRenderResult Case3dSPHRendering::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (! _stopped) _SPHSystem3d.Advance3dSPHSystem(_SPHParams);

        std::vector<glm::vec3> positions;
        positions.reserve(_SPHSystem3d.numFluidParticles);
        for (int i = 0; i < _SPHSystem3d.numFluidParticles; ++i) {
            if (_SPHSystem3d.densities[i] >= _minDensity) {
                positions.push_back(_SPHSystem3d.positions[i]);
            }
        }
        _particlesItem.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(positions));
        
        _cameraManager.Update(_camera);

        _depthFrame.Resize(desiredSize);
        _smoothFrame.Resize(desiredSize);
        _sFrame.Resize(desiredSize);
        _frame.Resize(desiredSize);

        {
            gl_using(_depthFrame);
            glClear(GL_DEPTH_BUFFER_BIT);

            _programDepth.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
            _programDepth.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());
            _programDepth.GetUniforms().SetByName("u_PointRadius", _SPHParams.H * 0.5f);
            _programDepth.GetUniforms().SetByName("u_ScreenHeight", float(desiredSize.second));

            glEnable(GL_PROGRAM_POINT_SIZE);
            glEnable(GL_DEPTH_TEST);
            _particlesItem.Draw({ _programDepth.Use() });
            glDisable(GL_PROGRAM_POINT_SIZE);
            glDisable(GL_DEPTH_TEST);
        }

        Engine::GL::UniqueTexture2D const * pResultTexture = &_depthFrame.GetDepthStencilAttachment();

        if (_filterIterations > 0) {
            {
                gl_using(_smoothFrame);
                glClearColor(1, 1, 1, 1);
                glClear(GL_COLOR_BUFFER_BIT);

                _programSmooth.GetUniforms().SetByName("u_DepthMap", 0);
                _programSmooth.GetUniforms().SetByName("u_TexelSize", glm::vec2(1.0f / desiredSize.first, 1.0f / desiredSize.second));
                _programSmooth.GetUniforms().SetByName("u_BlurScale", _blurScale);
                _programSmooth.GetUniforms().SetByName("u_BlurDepthFalloff", _blurDepthFalloff);

                auto const useProgram = _programSmooth.Use();
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, _depthFrame.GetDepthStencilAttachment().Get());

                glBindVertexArray(_emptyVAO.Get());
                glDrawArrays(GL_TRIANGLES, 0, 3);
            }

            pResultTexture = &_smoothFrame.GetColorAttachment();
            for (int i = 1; i < _filterIterations; ++i) {
                auto & target = (i % 2 == 1) ? _sFrame : _smoothFrame;
                auto & source = (i % 2 == 1) ? _smoothFrame : _sFrame;
                gl_using(target);
                glClearColor(1, 1, 1, 1);
                glClear(GL_COLOR_BUFFER_BIT);
                _programSmooth.GetUniforms().SetByName("u_DepthMap", 0);
                auto const useProgram = _programSmooth.Use();
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, source.GetColorAttachment().Get());
                glBindVertexArray(_emptyVAO.Get());
                glDrawArrays(GL_TRIANGLES, 0, 3);
                pResultTexture = &target.GetColorAttachment();
            }
        }

        {
            gl_using(_frame);
            glClearColor(0.f, 0.f, 0.f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glm::mat4 projection = _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second));

            _programShade.GetUniforms().SetByName("u_SmoothDepthMap", 0);
            _programShade.GetUniforms().SetByName("u_InvProjection", glm::inverse(projection));
            _programShade.GetUniforms().SetByName("u_LightDir", glm::vec3(5.f, 5.0f, 5.f));
            _programShade.GetUniforms().SetByName("u_LightColor", glm::vec3(0.12f, 0.56f, 1.0f));
            _programShade.GetUniforms().SetByName("u_DarkColor", glm::vec3(0.05f, 0.31f, 0.73f));

            auto const useProgram = _programShade.Use();
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, pResultTexture->Get());

            glBindVertexArray(_emptyVAO.Get());
            glDrawArrays(GL_TRIANGLES, 0, 3);
        }

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void Case3dSPHRendering::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void Case3dSPHRendering::ResetSystem() {
        _SPHSystem3d = SPHSystem3d(_SPHParams);
        _SPHSystem3d.positions.clear();
        _SPHSystem3d.velocities.clear();

        auto addParticle = [&](float x, float y, float z) {
            _SPHSystem3d.positions.push_back(glm::vec3(x, y, z));
            _SPHSystem3d.velocities.push_back(glm::vec3(0));
        };

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