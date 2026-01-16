#include "Assets/bundled.h"
#include "Labs/Fluid Simulation/App.h"

int main() {
    using namespace VCX;
    return Engine::RunApp<Labs::Animation::App>(Engine::AppContextOptions {
        .Title      = "VCX Project: Fluid Simulation",
        .WindowSize = { 1024, 576 },
        .FontSize   = 16,
        .IconFileNames = Assets::DefaultIcons,
        .FontFileNames = Assets::DefaultFonts,
    });
}
