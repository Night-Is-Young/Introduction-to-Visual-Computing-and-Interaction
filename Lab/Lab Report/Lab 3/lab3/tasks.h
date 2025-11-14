#pragma once

#include <numeric>
#include <stack>
#include <spdlog/spdlog.h>

#include "Engine/Scene.h"
#include "Labs/3-Rendering/Ray.h"


namespace VCX::Labs::Rendering {

    constexpr float EPS1 = 1e-2f; // distance to prevent self-intersection
    constexpr float EPS2 = 1e-8f; // angle for parallel judgement
    constexpr float EPS3 = 1e-4f; // relative distance to enlarge kdtree

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord);

    glm::vec4 GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord);

    struct Intersection {
        float t, u, v; // ray parameter t, barycentric coordinates (u, v)
    };

    bool IntersectTriangle(Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3);

    struct RayHit {
        bool              IntersectState;
        Engine::BlendMode IntersectMode;
        glm::vec3         IntersectPosition;
        glm::vec3         IntersectNormal;
        glm::vec4         IntersectAlbedo;   // [Albedo   (vec3), Alpha     (float)]
        glm::vec4         IntersectMetaSpec; // [Specular (vec3), Shininess (float)]
    };

    struct TrivialRayIntersector {
        Engine::Scene const * InternalScene = nullptr;

        TrivialRayIntersector() = default;

        void InitScene(Engine::Scene const * scene) {
            InternalScene = scene;
        }

        RayHit IntersectRay(Ray const & ray) const {
            RayHit result;
            if (! InternalScene) {
                spdlog::warn("VCX::Labs::Rendering::RayIntersector::IntersectRay(..): uninitialized intersector.");
                result.IntersectState = false;
                return result;
            }
            int          modelIdx, meshIdx;
            Intersection its;
            float        tmin     = 1e7, umin, vmin;
            int          maxmodel = InternalScene->Models.size();
            for (int i = 0; i < maxmodel; ++i) {
                auto const & model  = InternalScene->Models[i];
                int          maxidx = model.Mesh.Indices.size();
                for (int j = 0; j < maxidx; j += 3) {
                    std::uint32_t const * face = model.Mesh.Indices.data() + j;
                    glm::vec3 const &     p1   = model.Mesh.Positions[face[0]];
                    glm::vec3 const &     p2   = model.Mesh.Positions[face[1]];
                    glm::vec3 const &     p3   = model.Mesh.Positions[face[2]];
                    if (! IntersectTriangle(its, ray, p1, p2, p3)) continue;
                    if (its.t < EPS1 || its.t > tmin) continue;
                    tmin = its.t, umin = its.u, vmin = its.v, modelIdx = i, meshIdx = j;
                }
            }
            if (tmin == 1e7) {
                result.IntersectState = false;
                return result;
            }
            auto const &          model     = InternalScene->Models[modelIdx];
            auto const &          normals   = model.Mesh.IsNormalAvailable() ? model.Mesh.Normals : model.Mesh.ComputeNormals();
            auto const &          texcoords = model.Mesh.IsTexCoordAvailable() ? model.Mesh.TexCoords : model.Mesh.GetEmptyTexCoords();
            std::uint32_t const * face      = model.Mesh.Indices.data() + meshIdx;
            glm::vec3 const &     p1        = model.Mesh.Positions[face[0]];
            glm::vec3 const &     p2        = model.Mesh.Positions[face[1]];
            glm::vec3 const &     p3        = model.Mesh.Positions[face[2]];
            glm::vec3 const &     n1        = normals[face[0]];
            glm::vec3 const &     n2        = normals[face[1]];
            glm::vec3 const &     n3        = normals[face[2]];
            glm::vec2 const &     uv1       = texcoords[face[0]];
            glm::vec2 const &     uv2       = texcoords[face[1]];
            glm::vec2 const &     uv3       = texcoords[face[2]];
            result.IntersectState           = true;
            auto const & material           = InternalScene->Materials[model.MaterialIndex];
            result.IntersectMode            = material.Blend;
            result.IntersectPosition        = (1.0f - umin - vmin) * p1 + umin * p2 + vmin * p3;
            result.IntersectNormal          = (1.0f - umin - vmin) * n1 + umin * n2 + vmin * n3;
            glm::vec2 uvCoord               = (1.0f - umin - vmin) * uv1 + umin * uv2 + vmin * uv3;
            result.IntersectAlbedo          = GetAlbedo(material, uvCoord);
            result.IntersectMetaSpec        = GetTexture(material.MetaSpec, uvCoord);

            return result;
        }
    };

    /* Optional: write your own accelerated intersector here */
    struct Triangle {
        glm::vec3 p1, p2, p3, centroid;
        int       modelIndex, triangleIndex;
    };
    struct AABB {
        glm::vec3 min;
        glm::vec3 max;
        AABB():
            min(FLT_MAX), max(-FLT_MAX) {}
        AABB(const glm::vec3 & min_, const glm::vec3 & max_):
            min(min_), max(max_) {}
        void expand(const Triangle & t) {
            min = glm::min(min, t.p1, t.p2, t.p3);
            max = glm::max(max, t.p1, t.p2, t.p3);
        }
    };
    struct BVHNode {
        AABB bound;
        int  left = -1, right = -1, start = 0, end = 0;
        bool isLeaf() const { return left == -1 && right == -1; }
    };
    inline bool IntersectAABB(Ray const & ray, AABB const & box) {
        float tmin = 0, tmax = 1e10f;
        for (int i = 0; i < 3; ++i) {
            float t1 = (box.min[i] - ray.Origin[i]) / ray.Direction[i];
            float t2 = (box.max[i] - ray.Origin[i]) / ray.Direction[i];
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
            if (tmin > tmax) return false;
        }
        return true;
    }
    struct BVHIntersector {
        Engine::Scene const * InternalScene = nullptr;
        BVHNode *             Root          = nullptr;
        std::vector<Triangle> Triangles;
        std::vector<BVHNode>  Nodes;
        BVHIntersector() = default;

        int BuildBVH(int start, int end) {
            BVHNode node;
            AABB    bound;
            for (int i = start; i < end; ++i) bound.expand(Triangles[i]);
            node.bound = bound;
            int n      = end - start;
            if (n <= 3) {
                node.start = start;
                node.end   = end;
                int idx    = Nodes.size();
                Nodes.push_back(node);
                return idx;
            }
            glm::vec3 ex   = bound.max - bound.min;
            int       axis = (ex.x > ex.y && ex.x > ex.z) ? 0 : ((ex.y > ex.z) ? 1 : 2);
            int midIdx = start + n / 2;
            std::nth_element(
                Triangles.begin() + start,
                Triangles.begin() + midIdx,
                Triangles.begin() + end,
                [axis](const Triangle & a, const Triangle & b) {
                    return a.centroid[axis] < b.centroid[axis];
                });
            int left  = BuildBVH(start, midIdx);
            int right  = BuildBVH(midIdx, end);
            node.left  = left;
            node.right = right;
            int idx    = Nodes.size();
            Nodes.push_back(node);
            return idx;
        }

        void InitScene(Engine::Scene const * scene) {
            InternalScene = scene;
            Triangles.clear();
            Nodes.clear();

            for (int i = 0; i < scene->Models.size(); ++i) {
                auto const & model  = InternalScene->Models[i];
                for (int j = 0; j < model.Mesh.Indices.size(); j += 3) {
                    std::uint32_t const * face = model.Mesh.Indices.data() + j;
                    Triangle              t {
                        model.Mesh.Positions[face[0]],
                        model.Mesh.Positions[face[1]],
                        model.Mesh.Positions[face[2]],
                        (model.Mesh.Positions[face[0]] + model.Mesh.Positions[face[1]] + model.Mesh.Positions[face[2]]) / 3.0f,
                        i,
                        j
                    };
                    Triangles.push_back(t);
                }
            }
            
            if (!Triangles.empty()) {
                int rootIdx = BuildBVH(0, (int) Triangles.size());
            }
        }

        RayHit IntersectRay(Ray const& ray) const {
            RayHit result;
            result.IntersectState = false;
            float tmin            = 1e7, umin, vmin;
            int   modelIdx, meshIdx;

            if (Nodes.empty()) return result;
            
            std::stack<int> boundStack;
            boundStack.push((int) Nodes.size() - 1);
            
            Intersection its;
            while (! boundStack.empty()) {
                int idx = boundStack.top();
                boundStack.pop();
                BVHNode const & node = Nodes[idx];
                
                if (! IntersectAABB(ray, node.bound)) continue;
                
                if (node.isLeaf()) {
                    for (int i = node.start; i < node.end; ++i) {
                        const Triangle & t = Triangles[i];
                        if (! IntersectTriangle(its, ray, t.p1, t.p2, t.p3)) continue;
                        if (its.t < EPS1 || its.t > tmin) continue;
                        tmin = its.t, umin = its.u, vmin = its.v;
                        modelIdx = t.modelIndex, meshIdx = t.triangleIndex;
                    }
                } else {
                    boundStack.push(node.left);
                    boundStack.push(node.right);
                }
            }
            if (tmin == 1e7) {
                result.IntersectState = false;
                return result;
            }
            auto const &          model     = InternalScene->Models[modelIdx];
            auto const &          normals   = model.Mesh.IsNormalAvailable() ? model.Mesh.Normals : model.Mesh.ComputeNormals();
            auto const &          texcoords = model.Mesh.IsTexCoordAvailable() ? model.Mesh.TexCoords : model.Mesh.GetEmptyTexCoords();
            std::uint32_t const * face      = model.Mesh.Indices.data() + meshIdx;
            glm::vec3 const &     p1        = model.Mesh.Positions[face[0]];
            glm::vec3 const &     p2        = model.Mesh.Positions[face[1]];
            glm::vec3 const &     p3        = model.Mesh.Positions[face[2]];
            glm::vec3 const &     n1        = normals[face[0]];
            glm::vec3 const &     n2        = normals[face[1]];
            glm::vec3 const &     n3        = normals[face[2]];
            glm::vec2 const &     uv1       = texcoords[face[0]];
            glm::vec2 const &     uv2       = texcoords[face[1]];
            glm::vec2 const &     uv3       = texcoords[face[2]];
            result.IntersectState           = true;
            auto const & material           = InternalScene->Materials[model.MaterialIndex];
            result.IntersectMode            = material.Blend;
            result.IntersectPosition        = (1.0f - umin - vmin) * p1 + umin * p2 + vmin * p3;
            result.IntersectNormal          = (1.0f - umin - vmin) * n1 + umin * n2 + vmin * n3;
            glm::vec2 uvCoord               = (1.0f - umin - vmin) * uv1 + umin * uv2 + vmin * uv3;
            result.IntersectAlbedo          = GetAlbedo(material, uvCoord);
            result.IntersectMetaSpec        = GetTexture(material.MetaSpec, uvCoord);
            return result;
        }
    };

    using RayIntersector = BVHIntersector;

    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow);

} // namespace VCX::Labs::Rendering
