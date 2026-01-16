#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Fluid Simulation/SPHSystem3d.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"

namespace VCX::Labs::Animation {

    using UniqueR16Frame = Engine::GL::UniqueFrame<Engine::GL::UniqueTexture2D, void, Engine::Formats::R32F>;

    class Case3dSPHRendering : public Common::ICase {
    public:
        Case3dSPHRendering();

        virtual std::string_view const GetName() override { return "Fluid Rendering"; }
        
        virtual void OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void OnProcessInput(ImVec2 const & pos) override;
    
    private:
        Engine::GL::UniqueProgram               _programDepth;
        Engine::GL::UniqueProgram               _programSmooth;
        Engine::GL::UniqueProgram               _programShade;

        Engine::GL::UniqueRenderFrame           _frame;
        Engine::GL::UniqueDepthFrame            _depthFrame;
        UniqueR16Frame                          _smoothFrame;
        UniqueR16Frame                          _sFrame;

        Engine::GL::UniqueVertexArray           _emptyVAO;

        Engine::Camera                          _camera        { .Eye = glm::vec3(0, 0, 3) };
        Common::OrbitCameraManager              _cameraManager;
        
        Engine::GL::UniqueRenderItem            _particlesItem;

        bool                                    _stopped       { false };

        SPHParams3d _SPHParams;
        SPHSystem3d _SPHSystem3d;   

        float _filterRadius { 10.0f };
        float _blurScale { 0.1f };
        float _blurDepthFalloff { 100.0f };
        int   _filterIterations { 20 };
        float _minDensity { 0.0f };

        void ResetSystem();
    };
}
