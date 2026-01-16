#pragma once

#include <utility>
#include <vector>
#include <unordered_map>

#include <glm/glm.hpp>

namespace VCX::Labs::Animation {
    struct SPHParams {
        float     H                  = 0.1f;
        float     dt                 = 1.f/1000;
        float     Mass               = 4.f;
        float     RestDensity        = 1000.0f;
        float     GasK               = 50000.0f;
        float     GasGamma           = 7.0f;
        float     Viscosity          = 0.1f;
        float     Gravity            = 10.0f;
        float     BoundaryWidth      = 8.f;
        float     BoundaryHeight     = 8.f;
        float     BoundaryDamping    = -0.5f;
        float     BoundaryFriction   = 0.5f;
        int       InitParticleColNumber = 40;
        int       InitParticleRowNumber = 40;
        glm::vec2 InitPos            = { -3.9f, -3.9f };
        int       IterationsPerFrame    = 8;

        glm::vec3 ExternalForcePos { 0 };
        float     ExternalForceRadius = 1.0f;
        float     ExternalForceStrength = 10.0f;
        //glm::vec2 ScreenLeftTop, ScreenRightBottom;
    };

    class SPHSystem2d {
    public:
        std::vector<glm::vec3> positions;
        std::vector<glm::vec3> velocities;

        int numFluidParticles = 0;
        int numTotalParticles = 0;

        std::vector<float>     densities;
        std::vector<float>     pressures;
        std::vector<glm::vec3> forces;

        SPHParams params;

        SPHSystem2d();
        SPHSystem2d(const SPHParams & p);

        void Advance2dSPHSystem(const SPHParams & p);

    private:
        float cellSize;
        struct Cell {
            int x, y;
        };
        std::unordered_map<uint64_t, std::vector<int>> grid;

        float cubicspline_coef;
        float poly6_coef;
        float spiky_coef;
        float visc_lap_coef;

        void BuildGrid();
        void ComputeDensities();
        void ComputeForces();
        void IntegrateAndCollide(); 

        Cell     PositionToCell(const glm::vec3 & p) const;
        uint64_t HashCell(const Cell & c) const;

        glm::vec2 BoundMin, BoundMax;

        float     W_Poly6(const glm::vec3 & r) const;
        float     Cubic(const glm::vec3 & r) const;
        glm::vec3 GradW_Spiky(const glm::vec3 & r) const;
        glm::vec3 Cubic_Grad(const glm::vec3 & r) const;
        float     LapW_visc(const glm::vec3 & r) const;
    };
}
