#pragma once

#include "math/vector3.h"
#include "math/aabb.h"
#include "ray.h"
#include <cstdint>
#include <vector>


struct Triangle {
    math::Vector3<float> v0, v1, v2;
    math::Vector3<float> centroid;
    math::AABB<float> aabb;
};

struct BvhNode {
    math::AABB<float> aabb;
    uint32_t leftChild;
    uint32_t firstPrim;
    uint32_t primCount;

    bool isLeaf() const {
        return primCount > 0;
    }
};




namespace demo1 {

    class BVH {
    public:
        BVH() = default;
        
        void build(const std::vector<Triangle>& triangles);

        void intersectTriangle(Ray& ray, const Triangle& tri);

    private:
        std::vector<Triangle> m_triangles;
        math::AABB<float>     m_bounds;

    };
};


namespace demo2 {
    class BVH {
    public:
        BVH() = default;

        void createTriangles(uint32_t count);

        void build();

        void subdivide(BvhNode& node);

        void updateNodeBounds(BvhNode& node);

        void intersectTriangle(Ray& ray, const Triangle& tri);

        void intersectBVH(Ray& ray, const uint32_t nodeIdx);

        bool intersectAABB(Ray& ray, const math::AABB<float>& aabb);

    private:
        std::vector<Triangle> m_triangles;
        std::vector<BvhNode> m_nodes;
        std::vector<uint32_t> m_primitiveIndices;
        uint32_t currentNodeIdx = 0;
    };
};