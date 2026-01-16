#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Fluid Simulation/SPHSystem2d.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"

namespace VCX::Labs::Animation {
    class Case2dSPH : public Common::ICase {
    public:
        Case2dSPH();

        virtual std::string_view const GetName() override { return "Two Dimension SPH"; }
        
        virtual void OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void OnProcessInput(ImVec2 const & pos) override;
    
    private:
        Engine::GL::UniqueProgram               _program;
        Engine::GL::UniqueRenderFrame           _frame;
        Engine::Camera                          _camera        { .Eye = glm::vec3(0, 0, 8) };
        Common::OrbitCameraManager              _cameraManager;
        Engine::GL::UniqueRenderItem            _particlesItem;
        float                                   _particleSize  { 10 };
        bool                                    _stopped       { false };

        SPHParams   _SPHParams;
        SPHSystem2d _SPHSystem2d;
        std::pair<std::uint32_t, std::uint32_t> _viewportSize { 0, 0 };

        void ResetSystem();
    };
}
