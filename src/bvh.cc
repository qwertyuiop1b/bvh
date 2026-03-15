#include "bvh.h"
#include "math/vector3.h"
#include "math/utils.h"
#include "ray.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>


namespace demo1 {

    void BVH::build(const std::vector<Triangle>& triangles) {
        m_triangles = triangles;

        // Compute overall bounding box
        m_bounds = math::AABB<float>();
        for (const auto& tri : m_triangles) {
            m_bounds.expand(tri.aabb);
        }
    }


    void BVH::intersectTriangle(Ray& ray, const Triangle& tri) {
        const auto edge1 = tri.v1 - tri.v0;
        const auto edge2 = tri.v2 - tri.v0;
        const auto h = math::cross(ray.direction, edge2);
        const auto a = math::dot(edge1, h);
        if (a > -0.0001f && a < 0.0001f) {
            return; // This ray is parallel to this triangle.
        }

        const float f = 1.0f / a;
        const auto s = ray.origin - tri.v0;
        const float u = f * math::dot(s, h);
        if (u < 0.0f || u > 1.0f) {
            return;
        }

        const auto q = math::cross(s, edge1);
        const float v = f * math::dot(ray.direction, q);
        if (v < 0.0f || u + v > 1.0f) {
            return;
        }   

        // At this stage we can compute t to find out where the intersection point is on the line.
        const float t = f * math::dot(edge2, q);
        if (t > 0.0001f) { // ray intersection
            ray.t = std::min(ray.t, t);
        }
    }
};


namespace demo2 {
    void BVH::createTriangles(uint32_t count) {
        m_triangles.reserve(count);
        for (int i = 0; i < count; i++) {
            Triangle triangle;
            auto v0 = math::Vector3f(math::random(0.0f, 1.0f), math::random(0.0f, 1.0f), math::random(0.0f, 1.0f));
            auto v1 = math::Vector3f(math::random(0.0f, 1.0f), math::random(0.0f, 1.0f), math::random(0.0f, 1.0f));
            auto v2 = math::Vector3f(math::random(0.0f, 1.0f), math::random(0.0f, 1.0f), math::random(0.0f, 1.0f));
            
            triangle.v0 = v0 * 10.f - math::Vector3f(5.0f);
            triangle.v1 = triangle.v0 + v1;
            triangle.v2 = triangle.v0 + v2;
            triangle.centroid = (triangle.v0 + triangle.v1 + triangle.v2) / 3.0f;
            triangle.aabb = math::AABB<float>(triangle.v0, triangle.v1, triangle.v2);
            m_triangles.push_back(triangle);
        }
    }


    void BVH::build() {
        m_nodes.reserve(m_triangles.size() * 2 - 1);
        m_primitiveIndices.reserve(m_triangles.size());
        for (uint32_t i = 0; i < m_triangles.size(); i++) {
            m_primitiveIndices.push_back(i);
        }

        BvhNode rootNode;
        rootNode.leftChild = 0;
        rootNode.firstPrim = 0;
        rootNode.primCount = m_primitiveIndices.size();
        updateNodeBounds(rootNode);
        m_nodes.push_back(rootNode);
        currentNodeIdx++;

        subdivide(m_nodes[0]);
    }

    void BVH::updateNodeBounds(BvhNode& node) {
        math::AABB<float> bounds;
        for (uint32_t i = 0; i < node.primCount; i++) {
            bounds.expand(m_triangles[m_primitiveIndices[node.firstPrim + i]].aabb);
        }
        node.aabb = bounds;
    }

    void BVH::subdivide(BvhNode& node) {
        if (node.primCount <= 2) {
            return; // Leaf node
        }
        math::Vector3f extent = node.aabb.max - node.aabb.min;
        int axis = 0;
        if (extent.y > extent.x) axis = 1;
        if (extent.z > extent[axis]) axis = 2;
        float splitPos = node.aabb.min[axis] + extent[axis] * 0.5f;

        int i = node.firstPrim;
        int j = node.firstPrim + node.primCount - 1;
        while(i <= j) {
            if (m_triangles[m_primitiveIndices[i]].centroid[axis] < splitPos) {
                i++;
            } else {
                std::swap(m_primitiveIndices[i], m_primitiveIndices[j]);
                j--;
            }
        }

        uint32_t leftCount = i - node.firstPrim;
        if (leftCount == 0 || leftCount == node.primCount) {
            return; // Can't split, all primitives are on one side
        }

        uint32_t nodeIdx = &node - m_nodes.data();
        m_nodes[nodeIdx].leftChild = currentNodeIdx;

        BvhNode leftChild;
        leftChild.firstPrim = m_nodes[nodeIdx].firstPrim;
        leftChild.primCount = leftCount;
        leftChild.leftChild = 0;
        m_nodes.push_back(leftChild);
        uint32_t leftIdx = currentNodeIdx++;

        BvhNode rightChild;
        rightChild.firstPrim = i;
        rightChild.primCount = m_nodes[nodeIdx].primCount - leftCount;
        rightChild.leftChild = 0;
        m_nodes.push_back(rightChild);
        uint32_t rightIdx = currentNodeIdx++;

        m_nodes[nodeIdx].primCount = 0; // This node is now an internal node

        updateNodeBounds(m_nodes[leftIdx]);
        updateNodeBounds(m_nodes[rightIdx]);

        subdivide(m_nodes[leftIdx]);
        subdivide(m_nodes[rightIdx]);

    }

    void BVH::intersectBVH(Ray& ray, const uint32_t nodeIdx) {
        auto& node = m_nodes[nodeIdx];
        if (!intersectAABB(ray, node.aabb)) return;

        if (node.isLeaf()) {
            for (uint32_t i = 0; i < node.primCount; i++) {
                intersectTriangle(ray, m_triangles[m_primitiveIndices[node.firstPrim + i]]);
            }
        } else {
            intersectBVH(ray, node.leftChild);
            intersectBVH(ray, node.leftChild + 1);
        }
        
    }

    bool BVH::intersectAABB(Ray& ray, const math::AABB<float>& aabb) {
        float tmin = -1e30f, tmax = 1e30f;

        for (int i = 0; i < 3; ++i) {
            if (std::abs(ray.direction[i]) < 1e-8f) {
                // Ray is parallel to this axis
                if (ray.origin[i] < aabb.min[i] || ray.origin[i] > aabb.max[i]) {
                    return false;
                }
            } else {
                float invD = 1.0f / ray.direction[i];
                float t0 = (aabb.min[i] - ray.origin[i]) * invD;
                float t1 = (aabb.max[i] - ray.origin[i]) * invD;

                if (invD < 0.0f) std::swap(t0, t1);

                tmin = std::max(tmin, t0);
                tmax = std::min(tmax, t1);

                if (tmax <= tmin) return false;
            }
        }

        return tmin < ray.t && tmax > 0.0f;
    }

    void BVH::intersectTriangle(Ray& ray, const Triangle& tri) {
        const auto edge1 = tri.v1 - tri.v0;
        const auto edge2 = tri.v2 - tri.v0;
        const auto h = math::cross(ray.direction, edge2);
        const auto a = math::dot(edge1, h);
        if (a > -0.0001f && a < 0.0001f) {
            return; // This ray is parallel to this triangle.
        }

        const float f = 1.0f / a;
        const auto s = ray.origin - tri.v0;
        const float u = f * math::dot(s, h);
        if (u < 0.0f || u > 1.0f) {
            return;
        }

        const auto q = math::cross(s, edge1);
        const float v = f * math::dot(ray.direction, q);
        if (v < 0.0f || u + v > 1.0f) {
            return;
        }   

        // At this stage we can compute t to find out where the intersection point is on the line.
        const float t = f * math::dot(edge2, q);
        if (t > 0.0001f) { // ray intersection
            ray.t = std::min(ray.t, t);
        }
    }
};
