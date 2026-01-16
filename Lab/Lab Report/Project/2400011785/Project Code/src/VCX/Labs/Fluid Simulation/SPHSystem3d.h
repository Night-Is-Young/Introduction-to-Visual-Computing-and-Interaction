#pragma once

#include <utility>
#include <vector>
#include <unordered_map>

#include <glm/glm.hpp>

namespace VCX::Labs::Animation {
    struct SPHParams3d {
        float     H                  = 0.05f;
        float     dt                 = 5.f/10000;
        float     Mass               = 0.03125f;
        float     RestDensity        = 1000.0f;
        float     GasK               = 50000.0f;
        float     GasGamma           = 7.0f;
        float     Viscosity          = 0.1f;
        float     Gravity            = 10.0f;
        float     BoundaryX          = 1.5f;
        float     BoundaryY          = 1.5f;
        float     BoundaryZ          = 1.5f;
        int       InitXNumber        = 20;
        int       InitYNumber        = 40;
        int       InitZNumber        = 20;
        glm::vec3 InitPos { -.7f, -.7f, -.7f };
    };

    class SPHSystem3d {
    public:
        std::vector<glm::vec3> positions;
        std::vector<glm::vec3> velocities;

        int numFluidParticles = 0;
        int numTotalParticles = 0;

        std::vector<float>     densities;
        std::vector<float>     pressures;
        std::vector<glm::vec3> forces;

        SPHParams3d params;

        SPHSystem3d();
        SPHSystem3d(const SPHParams3d & p);

        void Advance3dSPHSystem(const SPHParams3d & p);

    private:
        float cellSize;
        struct Cell {
            int x, y, z;
        };
        std::unordered_map<uint64_t, std::vector<int>> grid;

        float cubicspline_coef;
        float poly6_coef;
        float spiky_coef;

        void BuildGrid();
        void ComputeDensities();
        void ComputeForces();
        void IntegrateAndCollide(); 

        Cell     PositionToCell(const glm::vec3 & p) const;
        uint64_t HashCell(const Cell & c) const;

        float     W_Poly6(const glm::vec3 & r) const;
        float     Cubic(const glm::vec3 & r) const;
        glm::vec3 GradW_Spiky(const glm::vec3 & r) const;
        glm::vec3 Cubic_Grad(const glm::vec3 & r) const;
        float     LapW_visc(const glm::vec3 & r) const;
    };
}
