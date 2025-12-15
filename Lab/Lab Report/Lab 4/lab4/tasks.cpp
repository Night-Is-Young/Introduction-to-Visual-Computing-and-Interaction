#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>
#include <iostream>
#include <cmath>
#include "Labs/4-Animation/tasks.h"
#include "IKSystem.h"
#include "CustomFunc.inl"


namespace VCX::Labs::Animation {
    void ForwardKinematics(IKSystem & ik, int StartIndex) {
        if (StartIndex == 0) {
            ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
            ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
            StartIndex                = 1;
        }
        
        for (int i = StartIndex; i < ik.JointLocalOffset.size(); i++) {
            // your code here: forward kinematics, update JointGlobalPosition and JointGlobalRotation
            ik.JointGlobalRotation[i] = ik.JointGlobalRotation[i - 1] * ik.JointLocalRotation[i];
            ik.JointGlobalPosition[i] = ik.JointGlobalPosition[i - 1] + ik.JointGlobalRotation[i - 1] * ik.JointLocalOffset[i];
        }
    }

    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        // These functions will be useful: glm::normalize, glm::rotation, glm::quat * glm::quat
        for (int CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; CCDIKIteration++) {
            // your code here: ccd ik
            for (int i = ik.NumJoints() - 2; i >= 0; i--) {
                glm::vec3 v_st = ik.JointGlobalPosition[i], v_ed = ik.JointGlobalPosition[ik.JointLocalOffset.size() - 1];
                glm::vec3 v = glm::normalize(v_ed - v_st), u = glm::normalize(EndPosition - v_st);
                glm::quat q = glm::rotation(v, u);
                ik.JointLocalRotation[i] = q * ik.JointLocalRotation[i];
                ForwardKinematics(ik, 0);
            }
        }
    }

    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        int nJoints = ik.NumJoints();
        std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
        for (int IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
            // task: fabr ik
            // backward update
            glm::vec3 next_position         = EndPosition;
            backward_positions[nJoints - 1] = EndPosition;

            for (int i = nJoints - 2; i >= 0; i--) {
                // your code here
                glm::vec3 dir         = glm::normalize(ik.JointGlobalPosition[i] - backward_positions[i + 1]);
                float     len         = glm::length(ik.JointOffsetLength[i + 1]);
                backward_positions[i] = backward_positions[i + 1] + len * dir;
            }

            // forward update
            glm::vec3 now_position = ik.JointGlobalPosition[0];
            forward_positions[0] = ik.JointGlobalPosition[0];
            for (int i = 0; i < nJoints - 1; i++) {
                // your code here
                glm::vec3 dir            = glm::normalize(backward_positions[i + 1] - forward_positions[i]);
                float     len            = glm::length(ik.JointOffsetLength[i + 1]);
                forward_positions[i + 1] = forward_positions[i] + len * dir;
            }
            ik.JointGlobalPosition = forward_positions; // copy forward positions to joint_positions
        }

        // Compute joint rotation by position here.
        for (int i = 0; i < nJoints - 1; i++) {
            ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i + 1]), glm::normalize(ik.JointGlobalPosition[i + 1] - ik.JointGlobalPosition[i]));
        }
        ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
        for (int i = 1; i < nJoints - 1; i++) {
            ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
        }
        ForwardKinematics(ik, 0);
    }

    IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
        // get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
        int nums = 500;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums));
        int index = 0;
        for (int i = 0; i < nums; i++) {
            float x_val = 0.2f * custom_x(9 * glm::pi<float>() * i / nums);
            float y_val = 0.2f * custom_y(9 * glm::pi<float>() * i / nums);
            (*custom)[index++] = glm::vec3(-x_val, 0.0f, y_val);
        }
        custom->resize(index);
        return custom;
    }

    static Eigen::VectorXf glm2eigen(std::vector<glm::vec3> const & glm_v) {
        Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf const, Eigen::Aligned>(reinterpret_cast<float const *>(glm_v.data()), static_cast<int>(glm_v.size() * 3));
        return v;
    }

    static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const & eigen_v) {
        return std::vector<glm::vec3>(
            reinterpret_cast<glm::vec3 const *>(eigen_v.data()),
            reinterpret_cast<glm::vec3 const *>(eigen_v.data() + eigen_v.size())
        );
    }

    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(n, n);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    // solve Ax = b and return x
    static Eigen::VectorXf ComputeSimplicialLLT(
        Eigen::SparseMatrix<float> const & A,
        Eigen::VectorXf const & b) {
        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(A);
        return solver.solve(b);
    }

    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
        // your code here: rewrite following code
        /*int const steps = 1000;
        float const ddt = dt / steps; 
        for (std::size_t s = 0; s < steps; s++) {
            std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
            for (auto const spring : system.Springs) {
                auto const p0 = spring.AdjIdx.first;
                auto const p1 = spring.AdjIdx.second;
                glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
                glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
                glm::vec3 const e01 = glm::normalize(x01);
                glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
                forces[p0] += f;
                forces[p1] -= f;
            }
            for (std::size_t i = 0; i < system.Positions.size(); i++) {
                if (system.Fixed[i]) continue;
                system.Velocities[i] += (glm::vec3(0, -system.Gravity, 0) + forces[i] / system.Mass) * ddt;
                system.Positions[i] += system.Velocities[i] * ddt;
            }
        }*/
        size_t nPoints = system.Positions.size();
        std::vector<Eigen::Triplet<float>> triplets_H_E;
        std::vector<glm::vec3>             vec_nabla_E(nPoints, glm::vec3(0.0f));
        for (auto const spring : system.Springs) {
            auto const i = spring.AdjIdx.first;
            auto const j = spring.AdjIdx.second;
            glm::vec3 const xi = system.Positions[i];
            glm::vec3 const xj = system.Positions[j];
            float           d  = glm::length(xi - xj);
            glm::vec3       f  = system.Stiffness * (glm::length(xi - xj) - spring.RestLength) * (xi - xj) / d;
            vec_nabla_E[i] += f;
            vec_nabla_E[j] -= f;
            glm::mat3       C  = glm::outerProduct(xi - xj, xi - xj) / (d * d);
            glm::mat3       I(1.0f);
            glm::mat3       H = system.Stiffness * (C + (1.0f - spring.RestLength / d) * (I - C));
            for (int col = 0; col < 3; ++col) {
                for (int row = 0; row < 3; ++row) {
                    float val = H[col][row];
                    triplets_H_E.emplace_back(3 * i + row, 3 * i + col, val);
                    triplets_H_E.emplace_back(3 * j + row, 3 * j + col, val);
                    triplets_H_E.emplace_back(3 * i + row, 3 * j + col, -val);
                    triplets_H_E.emplace_back(3 * j + row, 3 * i + col, -val);
                }
            }
        }
        auto H_E = CreateEigenSparseMatrix(3 * nPoints, triplets_H_E);
        auto nabla_E = glm2eigen(vec_nabla_E);

        std::vector<glm::vec3> vec_x_k(nPoints, glm::vec3(0.0f));
        std::vector<glm::vec3> vec_y_k(nPoints, glm::vec3(0.0f));
        for (size_t i = 0; i < nPoints; ++i) {
            vec_x_k[i] = system.Positions[i];
            vec_y_k[i] = system.Positions[i] + dt * system.Velocities[i] + dt * dt * glm::vec3(0, -system.Gravity, 0);
        }

        std::vector<Eigen::Triplet<float>> triplets_M;
        for (size_t i = 0; i < 3 * nPoints; ++i) {
            triplets_M.emplace_back(i, i, system.Mass);
        }
        auto M = CreateEigenSparseMatrix(3 * nPoints, triplets_M);

        auto x_k = glm2eigen(vec_x_k);
        auto y_k = glm2eigen(vec_y_k);

        auto nabla_g = system.Mass * (x_k - y_k) / (dt * dt) + nabla_E;
        auto H_g     = M / (dt * dt) + H_E;

        std::vector<glm::vec3> vec_x_res = eigen2glm(ComputeSimplicialLLT(H_g, -nabla_g));
        for (size_t i = 0; i < nPoints; ++i) {
            if (system.Fixed[i]) continue;
            system.Positions[i] += vec_x_res[i];
            system.Velocities[i] = vec_x_res[i] / dt;
        }
    }
}
