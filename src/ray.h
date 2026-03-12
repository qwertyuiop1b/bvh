#pragma once

#include "math/vector3.h"

struct Ray {
    math::Vector3f origin;
    math::Vector3f direction;
    float t = 1e30f;

    Ray() = default;

    Ray(const math::Vector3f& origin, const math::Vector3f& direction): origin(origin), direction(math::normalize(direction)) {}

    math::Vector3f at(float t) const {
        return origin + direction * t;
    }
};