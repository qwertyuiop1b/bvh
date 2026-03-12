#include "bvh.h"


namespace demo1 {

    void BVH::build(const std::vector<Triangle>& triangles) {
        m_triangles = triangles;

        // Compute overall bounding box
        m_bounds = math::AABB<float>();
        for (const auto& tri : m_triangles) {
            m_bounds.expand(tri.aabb);
        }
    }

};