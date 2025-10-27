#include <unordered_map>

#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"

#include <cmath>

namespace VCX::Labs::GeometryProcessing {

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    /******************* 1. Mesh Subdivision *****************/
    void SubdivisionMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations) {
        Engine::SurfaceMesh curr_mesh = input;
        // We do subdivison iteratively.
        for (std::uint32_t it = 0; it < numIterations; ++it) {
            // During each iteration, we first move curr_mesh into prev_mesh.
            Engine::SurfaceMesh prev_mesh;
            prev_mesh.Swap(curr_mesh);
            // Then we create doubly connected edge list.
            DCEL G(prev_mesh);
            if (! G.IsManifold()) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Non-manifold mesh.");
                return;
            }
            // Note that here curr_mesh has already been empty.
            // We reserve memory first for efficiency.
            curr_mesh.Positions.reserve(prev_mesh.Positions.size() * 3 / 2);
            curr_mesh.Indices.reserve(prev_mesh.Indices.size() * 4);
            // Then we iteratively update currently existing vertices.
            for (std::size_t i = 0; i < prev_mesh.Positions.size(); ++i) {
                // Update the currently existing vertex v from prev_mesh.Positions.
                // Then add the updated vertex into curr_mesh.Positions.
                DCEL::VertexProxy const * v         = G.Vertex(i);
                std::vector<uint32_t>     neighbors = v->Neighbors();
                // your code here:
                glm::vec3   curr_v_pos { 0.0f };
                std::size_t n = neighbors.size();
                float       u = (n == 3) ? (3.0f / 16.0f) : (3.0f / (8.0f * n));
                curr_v_pos += (1.0f - u * n) * prev_mesh.Positions[i];
                for (std::size_t j = 0; j < n; ++j) curr_v_pos += u * prev_mesh.Positions[neighbors[j]];
                curr_mesh.Positions.push_back(curr_v_pos);
            }
            // We create an array to store indices of the newly generated vertices.
            // Note: newIndices[i][j] is the index of vertex generated on the "opposite edge" of j-th
            //       vertex in the i-th triangle.
            std::vector<std::array<std::uint32_t, 3U>> newIndices(prev_mesh.Indices.size() / 3, { ~0U, ~0U, ~0U });
            // Iteratively process each halfedge.
            for (auto e : G.Edges()) {
                // newIndices[face index][vertex index] = index of the newly generated vertex
                newIndices[G.IndexOf(e->Face())][e->EdgeLabel()] = curr_mesh.Positions.size();
                DCEL::HalfEdge const * eTwin                     = e->TwinEdgeOr(nullptr);
                // eTwin stores the twin halfedge.
                if (! eTwin) {
                    // When there is no twin halfedge (so, e is a boundary edge):
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    std::size_t start     = e->From();
                    std::size_t end       = e->To();
                    glm::vec3   new_v_pos = 0.5f * prev_mesh.Positions[start] + 0.5f * prev_mesh.Positions[end];
                    curr_mesh.Positions.push_back(new_v_pos);
                } else {
                    // When the twin halfedge exists, we should also record:
                    //     newIndices[face index][vertex index] = index of the newly generated vertex
                    // Because G.Edges() will only traverse once for two halfedges,
                    //     we have to record twice.
                    newIndices[G.IndexOf(eTwin->Face())][e->TwinEdge()->EdgeLabel()] = curr_mesh.Positions.size();
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    std::size_t start      = e->From();
                    std::size_t end        = e->To();
                    std::size_t opposite_1 = e->OppositeVertex();
                    std::size_t opposite_2 = eTwin->OppositeVertex();
                    glm::vec3   new_v_pos  = 0.375f * (prev_mesh.Positions[start] + prev_mesh.Positions[end])
                        + 0.125f * (prev_mesh.Positions[opposite_1] + prev_mesh.Positions[opposite_2]);
                    curr_mesh.Positions.push_back(new_v_pos);
                }
            }

            // Here we've already build all the vertices.
            // Next, it's time to reconstruct face indices.
            for (std::size_t i = 0; i < prev_mesh.Indices.size(); i += 3U) {
                // For each face F in prev_mesh, we should create 4 sub-faces.
                // v0,v1,v2 are indices of vertices in F.
                // m0,m1,m2 are generated vertices on the edges of F.
                auto v0           = prev_mesh.Indices[i + 0U];
                auto v1           = prev_mesh.Indices[i + 1U];
                auto v2           = prev_mesh.Indices[i + 2U];
                auto [m0, m1, m2] = newIndices[i / 3U];
                // Note: m0 is on the opposite edge (v1-v2) to v0.
                // Please keep the correct indices order (consistent with order v0-v1-v2)
                //     when inserting new face indices.
                // toInsert[i][j] stores the j-th vertex index of the i-th sub-face.
                std::uint32_t toInsert[4][3] = {
                    // your code here:
                    { v0, m2, m1 },
                    { v1, m0, m2 },
                    { v2, m1, m0 },
                    { m0, m1, m2 }
                };
                // Do insertion.
                curr_mesh.Indices.insert(
                    curr_mesh.Indices.end(),
                    reinterpret_cast<std::uint32_t *>(toInsert),
                    reinterpret_cast<std::uint32_t *>(toInsert) + 12U);
            }

            if (curr_mesh.Positions.size() == 0) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Empty mesh.");
                output = input;
                return;
            }
        }
        // Update output.
        output.Swap(curr_mesh);
    }

    /******************* 2. Mesh Parameterization *****************/
    void Parameterization(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, const std::uint32_t numIterations) {
        // Copy.
        output = input;
        // Reset output.TexCoords.
        output.TexCoords.resize(input.Positions.size(), glm::vec2 { 0 });

        // Build DCEL.
        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::Parameterization(..): non-manifold mesh.");
            return;
        }

        // Set boundary UVs for boundary vertices.
        // your code here: directly edit output.TexCoords
        std::size_t               start = 0;
        DCEL::VertexProxy const * v     = G.Vertex(start);
        while (! v->OnBoundary() && start < input.Positions.size()) {
            start++;
            DCEL::VertexProxy const * v = G.Vertex(start);
        }
        start--;
        std::vector<std::size_t> boundary, inside, inside_index(input.Positions.size(), 0);
        boundary.push_back(start);

        while (1) {
            DCEL::VertexProxy const *               v                    = G.Vertex(boundary.back());
            std::pair<std::uint32_t, std::uint32_t> boundary_v_neighbors = v->BoundaryNeighbors();
            if (boundary_v_neighbors.first == start) break;
            boundary.push_back(boundary_v_neighbors.first);
        }

        for (std::size_t i = 0; i < input.Positions.size(); ++i) {
            if (std::find(boundary.begin(), boundary.end(), i) == boundary.end()) {
                inside_index[i] = inside.size();
                inside.push_back(i);
            }
        }

        std::size_t        boundary_v_num = boundary.size(), inside_v_num = inside.size();
        float              perimeter = glm::length(input.Positions[boundary[0]] - input.Positions[boundary.back()]);
        std::vector<float> boundary_length;
        for (std::size_t i = 1; i < boundary_v_num; ++i) {
            boundary_length.push_back(glm::length(input.Positions[boundary[i]] - input.Positions[boundary[i - 1]]));
            perimeter += boundary_length.back();
        }
        float theta             = 1.1f;
        for (std::size_t i = 0; i < boundary_v_num; ++i) {
            output.TexCoords[boundary[i]] = {
                0.5f + 0.5f * std::cos(theta),
                0.5f + 0.5f * std::sin(theta),
            };
            theta += boundary_length[i] / perimeter * 2 * 3.14159265359;
        }
        std::vector<std::vector<float>> A(inside_v_num, std::vector<float>(inside_v_num, 0.0f));
        std::vector<glm::vec2>          x(inside_v_num, { 0.0f, 0.0f }), b(inside_v_num, { 0.0f, 0.0f });
        for (std::size_t i = 0; i < inside_v_num; ++i) {
            DCEL::VertexProxy const * v                  = G.Vertex(inside[i]);
            std::vector<uint32_t>     inside_v_neighbors = v->Neighbors();
            std::size_t               n                  = inside_v_neighbors.size();
            A[i][i]                                      = 1.0f;
            for (auto j : inside_v_neighbors) {
                float                     lambda_v = 1.0f / n;
                DCEL::VertexProxy const * u        = G.Vertex(j);
                if (u->OnBoundary()) {
                    b[i] += lambda_v * output.TexCoords[j];
                } else {
                    A[i][inside_index[j]] = -lambda_v;
                }
            }
        }

        // Solve equation via Gauss-Seidel Iterative Method.
        for (int k = 0; k < numIterations; ++k) {
            // your code here:
            for (std::size_t i = 0; i < inside_v_num; ++i) {
                glm::vec2 delta { 0.0f };
                for (std::size_t j = 0; j < inside_v_num; ++j) {
                    if (j != i) delta += A[i][j] * x[j];
                }
                x[i] = b[i] - delta;
            }
        }
        for (std::size_t i = 0; i < inside_v_num; ++i) {
            output.TexCoords[inside[i]] = x[i];
        }
    }

    /******************* 3. Mesh Simplification *****************/
    void SimplifyMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, float simplification_ratio) {
        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-watertight mesh.");
            return;
        }

        // Copy.
        output = input;

        // Compute Kp matrix of the face f.
        auto UpdateQ {
            [&G, &output](DCEL::Triangle const * f) -> glm::dmat4 {
                // your code here:
                glm::dvec3 v0 = output.Positions[f->VertexIndex(0)], v1 = output.Positions[f->VertexIndex(1)], v2 = output.Positions[f->VertexIndex(2)];
                glm::dvec3 e1 = v1 - v0, e2 = v2 - v0;
                glm::dvec3 n = glm::normalize(glm::cross(e1, e2));
                glm::dvec4 p = { n.x, n.y, n.z, -glm::dot(n, v0) };
                return glm::outerProduct(p, p);
            }
        };

        // The struct to record contraction info.
        struct ContractionPair {
            DCEL::HalfEdge const * edge;           // which edge to contract; if $edge == nullptr$, it means this pair is no longer valid
            glm::dvec4             targetPosition; // the targetPosition $v$ for vertex $edge->From()$ to move to
            double                 cost;           // the cost $v.T * Qbar * v$
        };

        // Given an edge (v1->v2), the positions of its two endpoints (p1, p2) and the Q matrix (Q1+Q2),
        //     return the ContractionPair struct.
        static constexpr auto MakePair {
            [](DCEL::HalfEdge const * edge,
               glm::dvec3 const &      p1,
               glm::dvec3 const &      p2,
               glm::dmat4 const &      Q) -> ContractionPair {
                // your code here:
                ContractionPair result {};
                result.edge = edge;
                glm::dmat4 q = Q;
                q[0][3]     = 0;
                q[1][3]     = 0;
                q[2][3]     = 0;
                q[3][3]     = 1;
                if (glm::determinant(q) >= 0.001 || glm::determinant(q) <= -0.001) {
                    result.targetPosition = glm::inverse(q) * (glm::dvec4 { 0, 0, 0, 1 });
                } else {
                    result.targetPosition = { 0.5 * (p1 + p2), 1.0 };
                }
                result.cost = glm::dot(result.targetPosition, Q * result.targetPosition);
                return result;
            }
        };

        // pair_map: map EdgeIdx to index of $pairs$
        // pairs:    store ContractionPair
        // Qv:       $Qv[idx]$ is the Q matrix of vertex with index $idx$
        // Kf:       $Kf[idx]$ is the Kp matrix of face with index $idx$
        std::unordered_map<DCEL::EdgeIdx, std::size_t> pair_map;
        std::vector<ContractionPair>                   pairs;
        std::vector<glm::dmat4>                        Qv(G.NumOfVertices(), glm::dmat4(0));
        std::vector<glm::dmat4>                        Kf(G.NumOfFaces(), glm::dmat4(0));

        // Initially, we compute Q matrix for each faces and it accumulates at each vertex.
        for (auto f : G.Faces()) {
            auto Q = UpdateQ(f);
            Qv[f->VertexIndex(0)] += Q;
            Qv[f->VertexIndex(1)] += Q;
            Qv[f->VertexIndex(2)] += Q;
            Kf[G.IndexOf(f)] = Q;
        }

        pair_map.reserve(G.NumOfFaces() * 3);
        pairs.reserve(G.NumOfFaces() * 3 / 2);

        // Initially, we make pairs from all the contractable edges.
        for (auto e : G.Edges()) {
            if (! G.IsContractable(e)) continue;
            auto v1                            = e->From();
            auto v2                            = e->To();
            auto pair                          = MakePair(e, input.Positions[v1], input.Positions[v2], Qv[v1] + Qv[v2]);
            pair_map[G.IndexOf(e)]             = pairs.size();
            pair_map[G.IndexOf(e->TwinEdge())] = pairs.size();
            pairs.emplace_back(pair);
        }

        // Loop until the number of vertices is less than $simplification_ratio * initial_size$.
        while (G.NumOfVertices() > simplification_ratio * Qv.size()) {
            // Find the contractable pair with minimal cost.
            std::size_t min_idx = ~0;
            for (std::size_t i = 1; i < pairs.size(); ++i) {
                if (! pairs[i].edge) continue;
                if (! ~min_idx || pairs[i].cost < pairs[min_idx].cost) {
                    if (G.IsContractable(pairs[i].edge)) min_idx = i;
                    else pairs[i].edge = nullptr;
                }
            }
            if (! ~min_idx) break;

            // top:    the contractable pair with minimal cost
            // v1:     the reserved vertex
            // v2:     the removed vertex
            // result: the contract result
            // ring:   the edge ring of vertex v1
            ContractionPair & top    = pairs[min_idx];
            auto              v1     = top.edge->From();
            auto              v2     = top.edge->To();
            auto              result = G.Contract(top.edge);
            auto              ring   = G.Vertex(v1)->Ring();

            top.edge             = nullptr;            // The contraction has already been done, so the pair is no longer valid. Mark it as invalid.
            output.Positions[v1] = top.targetPosition; // Update the positions.

            // We do something to repair $pair_map$ and $pairs$ because some edges and vertices no longer exist.
            for (int i = 0; i < 2; ++i) {
                DCEL::EdgeIdx removed           = G.IndexOf(result.removed_edges[i].first);
                DCEL::EdgeIdx collapsed         = G.IndexOf(result.collapsed_edges[i].second);
                pairs[pair_map[removed]].edge   = result.collapsed_edges[i].first;
                pairs[pair_map[collapsed]].edge = nullptr;
                pair_map[collapsed]             = pair_map[G.IndexOf(result.collapsed_edges[i].first)];
            }

            // For the two wing vertices, each of them lose one incident face.
            // So, we update the Q matrix.
            Qv[result.removed_faces[0].first] -= Kf[G.IndexOf(result.removed_faces[0].second)];
            Qv[result.removed_faces[1].first] -= Kf[G.IndexOf(result.removed_faces[1].second)];

            // For the vertex v1, Q matrix should be recomputed.
            // And as the position of v1 changed, all the vertices which are on the ring of v1 should update their Q matrix as well.
            Qv[v1] = glm::mat4(0);
            for (auto e : ring) {
                // your code here:
                //     1. Compute the new Kp matrix for $e->Face()$.
                //     2. According to the difference between the old Kp (in $Kf$) and the new Kp (computed in step 1),
                //        update Q matrix of each vertex on the ring (update $Qv$).
                //     3. Update Q matrix of vertex v1 as well (update $Qv$).
                //     4. Update $Kf$.
                auto f      = e->Face();
                auto Q      = UpdateQ(f);
                auto modify = Q - Kf[G.IndexOf(f)];
                Qv[e->From()] += modify;
                Qv[e->To()] += modify;
                Qv[v1] += Q;
                Kf[G.IndexOf(f)] = Q;
            }

            // Finally, as the Q matrix changed, we should update the relative $ContractionPair$ in $pairs$.
            // Any pair with the Q matrix of its endpoints changed, should be remade by $MakePair$.
            // your code here:
            for (auto e : ring) {
                auto v        = e->From();
                auto ring_out = (G.Vertex(v))->Ring();
                for (auto e_out : ring_out) {
                    auto v_out    = e_out->From();
                    auto e_modify = e_out->PrevEdge();
                    if (! G.IsContractable(e_modify)) {
                        pairs[pair_map[G.IndexOf(e_modify)]].edge = nullptr;
                    } else {
                        pairs[pair_map[G.IndexOf(e_modify)]] = MakePair(e_modify, output.Positions[v], output.Positions[v_out], Qv[v] + Qv[v_out]);
                    }
                }
            }
        }

        // In the end, we check if the result mesh is watertight and manifold.
        if (! G.DebugWatertightManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Result is not watertight manifold.");
        }

        auto exported = G.ExportMesh();
        output.Indices.swap(exported.Indices);
    }

    /******************* 4. Mesh Smoothing *****************/
    void SmoothMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations, float lambda, bool useUniformWeight) {
        // Define function to compute cotangent value of the angle v1-vAngle-v2
        static constexpr auto GetCotangent {
            [](glm::vec3 vAngle, glm::vec3 v1, glm::vec3 v2) -> float {
                // your code here:
                glm::vec3   u         = v1 - vAngle;
                glm::vec3   v         = v2 - vAngle;
                float       cross_len = glm::length(glm::cross(u, v));
                return std::clamp(glm::dot(u, v) / cross_len, 0.1f, 5.0f);
            }
        };

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-watertight mesh.");
            return;
        }

        Engine::SurfaceMesh prev_mesh;
        prev_mesh.Positions = input.Positions;
        for (std::uint32_t iter = 0; iter < numIterations; ++iter) {
            Engine::SurfaceMesh curr_mesh = prev_mesh;
            for (std::size_t i = 0; i < input.Positions.size(); ++i) {
                // your code here: curr_mesh.Positions[i] = ...
                auto v = G.Vertex(i);
                auto      neighbors = v->Neighbors();
                int       n         = neighbors.size();
                glm::vec3 sum { 0.0f };
                if (useUniformWeight) {
                    for (std::size_t j = 0; j < n; ++j) sum += prev_mesh.Positions[neighbors[j]];
                    sum = (1.0f / n) * sum;
                } else {
                    auto ring = v->Ring();
                    float sum_w = 0.0f;
                    for (auto e_ring : ring) {
                        auto e = e_ring->NextEdge();
                        std::vector<glm::vec3> pos(4, glm::vec3 { 0.0f });
                        pos[0] = prev_mesh.Positions[e->To()];
                        pos[1] = prev_mesh.Positions[e->From()];
                        pos[2] = prev_mesh.Positions[e->NextEdge()->To()];
                        pos[3] = prev_mesh.Positions[e->TwinEdge()->PrevEdge()->From()];
                        float w = GetCotangent(pos[2], pos[0], pos[1]) + GetCotangent(pos[3], pos[0], pos[1]);
                        sum_w += w;
                        sum += w * (pos[1]);
                    }
                    sum /= sum_w;
                }
                curr_mesh.Positions[i] = (1 - lambda) * prev_mesh.Positions[i] + lambda * sum;
            }
            // Move curr_mesh to prev_mesh.
            prev_mesh.Swap(curr_mesh);
        }
        // Move prev_mesh to output.
        output.Swap(prev_mesh);
        // Copy indices from input.
        output.Indices = input.Indices;
    }

    /******************* 5. Marching Cubes *****************/
    void MarchingCubes(Engine::SurfaceMesh & output, const std::function<float(const glm::vec3 &)> & sdf, const glm::vec3 & grid_min, const float dx, const int n) {
        // your code here:
        glm::mat3                                               unit = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        std::vector<std::vector<std::vector<std::vector<int>>>> grid(n+1, std::vector<std::vector<std::vector<int>>>(n+1, std::vector<std::vector<int>>(n+1, std::vector<int>(3, -1))));
        for (int nx = 0; nx < n; nx++) {
            for (int ny = 0; ny < n; ny++) {
                for (int nz = 0; nz < n; nz++) {
                    std::vector<glm::vec3> v_pos(8, glm::vec3 { 0.0f });
                    v_pos[0] = grid_min + glm::vec3 { nx * dx, ny * dx, nz * dx };
                    for (int i = 0; i < 8; ++i) {
                        v_pos[i] = v_pos[0] + dx * glm::vec3 { (i & 1), ((i >> 1) & 1), (i >> 2) };
                    }
                    int v_hash = 0;
                    for (int i = 7; i >= 0 ; --i) {
                        v_hash += (sdf(v_pos[i]) > 0) ? 1 : 0;
                        v_hash *= 2;
                    }
                    v_hash /= 2;
                    int e_hash = c_EdgeStateTable[v_hash];
                    std::vector<int> idx_e(12, -1);
                    for (int j = 0; j < 12; ++j) {
                        if (e_hash % 2 == 1) {
                            glm::vec3 v_start, v_end, v_new;
                            int       d[3]           = { 0, 0, 0 };
                            d[((j >> 2) + 1) % 3]    = j & 1;
                            d[((j >> 2) + 2) % 3]    = (j >> 1) & 1;
                            if (grid[nx + d[0]][ny + d[1]][nz + d[2]][j >> 2] == -1) {
                                v_start                                       = v_pos[0] + dx * glm::vec3 { (float) d[0], (float) d[1], (float) d[2] };
                                v_end                                         = v_start + dx * unit[j >> 2];
                                v_new                                         = (sdf(v_end) * v_start - sdf(v_start) * v_end) / (sdf(v_end) - sdf(v_start));
                                grid[nx + d[0]][ny + d[1]][nz + d[2]][j >> 2] = output.Positions.size();
                                output.Positions.push_back(v_new);
                            }
                            idx_e[j] = grid[nx + d[0]][ny + d[1]][nz + d[2]][j >> 2];
                        }
                        e_hash /= 2;
                    }
                    for (int k = 0; k < 5; ++k) {
                        int e0 = c_EdgeOrdsTable[v_hash][3 * k], e1 = c_EdgeOrdsTable[v_hash][3 * k + 1], e2 = c_EdgeOrdsTable[v_hash][3 * k + 2];
                        if (e0 == -1) break;
                        else {
                            output.Indices.push_back(idx_e[e2]);
                            output.Indices.push_back(idx_e[e1]);
                            output.Indices.push_back(idx_e[e0]);
                        }
                    }
                }
            }
        }
    }
} // namespace VCX::Labs::GeometryProcessing
