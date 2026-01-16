#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Fluid Simulation/SPHSystem3d.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"

namespace VCX::Labs::Animation {
    class Case3dSPH : public Common::ICase {
    public:
        Case3dSPH();

        virtual std::string_view const GetName() override { return "Three Dimension SPH"; }
        
        virtual void OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void OnProcessInput(ImVec2 const & pos) override;
    
    private:
        Engine::GL::UniqueProgram               _program;
        Engine::GL::UniqueRenderFrame           _frame;
        Engine::Camera                          _camera        { .Eye = glm::vec3(0, 0, 2) };
        Common::OrbitCameraManager              _cameraManager;
        Engine::GL::UniqueRenderItem            _particlesItem;
        float                                   _particleSize  { 10 };
        bool                                    _stopped       { false };

        SPHParams3d _SPHParams;
        SPHSystem3d _SPHSystem3d;

        void ResetSystem();
    };
}
