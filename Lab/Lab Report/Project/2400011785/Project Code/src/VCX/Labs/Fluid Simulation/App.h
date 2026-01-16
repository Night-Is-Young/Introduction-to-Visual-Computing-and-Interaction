#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/Fluid Simulation/Case2dSPH.h"
#include "Labs/Fluid Simulation/Case3dSPH.h"
#include "Labs/Fluid Simulation/Case3dSPHRendering.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::Animation {

    class App : public Engine::IApp {
    private:
        Common::UI             _ui;

        Case2dSPH          _case2dSPH;
        Case3dSPH          _case3dSPH;
        Case3dSPHRendering _case3dSPHRendering;

        std::size_t        _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = {
            _case2dSPH, _case3dSPH, _case3dSPHRendering,
        };

    public:
        App();
        void OnFrame() override;
    };
}
