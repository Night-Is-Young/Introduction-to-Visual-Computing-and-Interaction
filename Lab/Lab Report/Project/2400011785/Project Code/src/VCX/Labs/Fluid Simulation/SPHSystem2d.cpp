#pragma once

#include <utility>
#include <vector>
#include <cmath>
#include <algorithm>
#include <omp.h>
#include <iostream>

#include <glm/glm.hpp>
#include "Labs/Fluid Simulation/SPHSystem2d.h"

namespace VCX::Labs::Animation {

    SPHSystem2d::SPHSystem2d() {
        
    }

    SPHSystem2d::SPHSystem2d(const SPHParams& p) {
        params = p;
        cellSize = p.H;

        cubicspline_coef = 40.0f / (7.0f * 3.14159265359f * std::pow(p.H, 2));
        poly6_coef       = 4.0f / (3.14159265359f * std::pow(p.H, 8));
        spiky_coef       = -30.0f / (3.14159265359f * std::pow(p.H, 5));

        BoundMin = glm::vec2 { -params.BoundaryWidth / 2, -params.BoundaryHeight / 2 };
        BoundMax = glm::vec2 { params.BoundaryWidth / 2, params.BoundaryHeight / 2 };
    }

    void SPHSystem2d::Advance2dSPHSystem(const SPHParams & p) {
        params = p;
        for (int _ = 0; _ < params.IterationsPerFrame; _++) {
            BuildGrid();
            ComputeDensities();
            ComputeForces();
            IntegrateAndCollide();
        }
    }

    SPHSystem2d::Cell SPHSystem2d::PositionToCell(const glm::vec3 & p) const {
        return Cell {
            static_cast<int>(std::floor(p.x / cellSize)),
            static_cast<int>(std::floor(p.y / cellSize))
        };
    }

    uint64_t SPHSystem2d::HashCell(const Cell & c) const {
        const uint64_t p1 = 73856093u;
        const uint64_t p2 = 19349663u;
        uint64_t       n  = (static_cast<uint64_t>(c.x) * p1) ^ (static_cast<uint64_t>(c.y) * p2);
        return n;
    }

    void SPHSystem2d::BuildGrid() {
        grid.clear();

        for (int i = 0; i < numTotalParticles; ++i) {
            Cell     c   = PositionToCell(positions[i]);
            uint64_t key = HashCell(c);
            grid[key].push_back(i);
        }
    }

    float SPHSystem2d::W_Poly6(const glm::vec3 & r) const {
        float r2 = r.x * r.x + r.y * r.y;
        float h2 = params.H * params.H;
        if (r2 >= 0.0f && r2 <= h2) {
            float diff = (h2 - r2);
            return poly6_coef * diff * diff * diff;
        } else {
            return 0.0f;
        }
    }

    float SPHSystem2d::Cubic(const glm::vec3 & r) const {
        float q = glm::length(r) / params.H;
        if (q < 0.5) {
            return cubicspline_coef * (6.0f * (q * q * q) - 6.0f * (q * q) + 1.0f);
        } else if (q <= 1.0f) {
            float diff = (1.0f - q);
            return cubicspline_coef * (2.0f * (1.0f - q) * (1.0f - q) * (1.0f - q));
        } else {
            return 0.0f;
        }
    }

    glm::vec3 SPHSystem2d::GradW_Spiky(const glm::vec3 & r) const {
        float rlen = glm::length(glm::vec2(r.x, r.y));
        if (rlen > 0.0f && rlen <= params.H) {
            float scalar = spiky_coef * (params.H - rlen) * (params.H - rlen) / rlen;
            return glm::vec3(scalar * r.x, scalar * r.y, 0.0f);
        } else {
            return glm::vec3(0.0f);
        }
    }

    glm::vec3 SPHSystem2d::Cubic_Grad(const glm::vec3 & r) const {
        float rlen = glm::length(glm::vec2(r.x, r.y));
        if (rlen > 0.0f && rlen <= params.H) {
            float q      = rlen / params.H;
            float factor = 0.0f;
            if (q < 0.5f) {
                factor = cubicspline_coef * (18.0f * q - 12.0f) / params.H;
            } else {
                factor = -cubicspline_coef * 6.0f * (1.0f - q) * (1.0f - q) / params.H;
            }
            factor /= rlen;
            return glm::vec3(factor * r.x, factor * r.y, 0.0f);
        } else {
            return glm::vec3(0.0f);
        }
    }

    float SPHSystem2d::LapW_visc(const glm::vec3 & r) const {
        float rlen = glm::length(glm::vec2(r.x, r.y));
        if (rlen >= 0.0f && rlen <= params.H) {
            return visc_lap_coef * (params.H - rlen);
        } else {
            return 0.0f;
        }
    }

    void SPHSystem2d::ComputeDensities() {
        float h2 = params.H * params.H;

        // Parallelize density computation
        // Use dynamic schedule because neighbor count varies per particle
        #pragma omp parallel for
        for (int i = 0; i < numTotalParticles; ++i) {
            float rho = 0.0f;

            Cell ci = PositionToCell(positions[i]);

            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    Cell     nc { ci.x + dx, ci.y + dy };
                    uint64_t key = HashCell(nc);
                    
                    auto     it  = grid.find(key);
                    if (it == grid.end()) continue;
                    float eps = 1e-5f;

                    const std::vector<int> & cellParticles = it->second;
                    for (int j : cellParticles) {
                        glm::vec3 rij = positions[i] - positions[j];
                        float r2 = rij.x * rij.x + rij.y * rij.y;
                        if (r2 <= h2) {
                            rho += params.Mass * W_Poly6(rij);
                        }
                    }
                }
            }
            densities[i] = (rho > 1e-6f) ? rho : 1e-6f;
        }
        #pragma omp parallel for
        for (int i = 0; i < numTotalParticles; ++i) {
            pressures[i] = std::max(params.GasK * (std::pow(densities[i] / params.RestDensity, params.GasGamma) - 1.f), .0f);
        }
    }

    void SPHSystem2d::ComputeForces() {

        #pragma omp parallel for
        for (int i = 0; i < numFluidParticles; ++i) {
            glm::vec3 f_pressure(0.0f);
            glm::vec3 f_visc(0.0f);
            glm::vec3 f_external(0.0f);

            Cell ci = PositionToCell(positions[i]);

            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    Cell     nc { ci.x + dx, ci.y + dy };
                    uint64_t key = HashCell(nc);
                    auto     it  = grid.find(key);
                    if (it == grid.end()) continue;

                    const std::vector<int> & cellParticles = it->second;
                    for (int j : cellParticles) {
                        if (j == i) continue;

                        glm::vec3 rij  = positions[i] - positions[j];
                        float     rlen = glm::length(glm::vec2(rij.x, rij.y));
                        if (rlen <= params.H && rlen > 1e-6f) {
                            glm::vec3 gradW        = GradW_Spiky(rij);
                            float     pressureTerm = (pressures[i] / (densities[i] * densities[i]) + 
                                                      pressures[j] / (densities[j] * densities[j]));
                            f_pressure += -params.Mass * pressureTerm * gradW;

                            glm::vec3 velDiff = velocities[j] - velocities[i];
                            
                            f_visc -= params.Viscosity * params.Mass * (velDiff / densities[j]) * glm::dot(Cubic_Grad(rij), rij) / (glm::length(rij) * glm::length(rij) + 0.01f * params.H * params.H);

                        }
                    }
                }
            }
            
            
            if (params.ExternalForceRadius != 0) {
                glm::vec3 v = params.ExternalForcePos - positions[i];
                f_external  = params.ExternalForceStrength * glm::normalize(v) * std::clamp(params.ExternalForceRadius / glm::length(v), 0.f, 5.f);
            }
            
            glm::vec3 f_gravity = glm::vec3(0.0f, -params.Gravity, 0.0f);

            forces[i] = f_pressure + f_visc + f_external + f_gravity;
        }
    }
    void SPHSystem2d::IntegrateAndCollide() {
        #pragma omp parallel for
        for (int i = 0; i < numFluidParticles; ++i) {
            velocities[i] += forces[i] * params.dt;
            positions[i] += velocities[i] * params.dt;
        }
    }
}
