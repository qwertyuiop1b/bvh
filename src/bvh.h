#pragma once

#include "math/vector3.h"
#include "math/aabb.h"
#include "ray.h"
#include <vector>


struct Triangle {
    math::Vector3<float> v0, v1, v2;
    math::Vector3<float> centroid;
    math::AABB<float> aabb;

    bool intersect(Ray& ray) const {
        return true;
    }
};


namespace demo1 {

    class BVH {
    public:
        BVH() = default;
        
        void build(const std::vector<Triangle>& triangles);

    private:
        std::vector<Triangle> m_triangles;
        math::AABB<float>     m_bounds;

    };
};