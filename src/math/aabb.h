#pragma once

#include <algorithm>
#include <limits>

#include "vector3.h"

namespace math {

    template<typename T>
    struct AABB {
        Vector3<T> min;
        Vector3<T> max;

        AABB()
            : min(Vector3<T>(std::numeric_limits<T>::max()))
            , max(Vector3<T>(std::numeric_limits<T>::min()))
        {}
        AABB(const Vector3<T>& min, const Vector3<T>& max): min(min), max(max) {}
        AABB(const Vector3<T>& v0, const Vector3<T>& v1, const Vector3<T>& v2)
            : min(Vector3<T>(std::min(v0.x, std::min(v1.x, v2.x)), std::min(v0.y, std::min(v1.y, v2.y)), std::min(v0.z, std::min(v1.z, v2.z))))
            , max(Vector3<T>(std::max(v0.x, std::max(v1.x, v2.x)), std::max(v0.y, std::max(v1.y, v2.y)), std::max(v0.z, std::max(v1.z, v2.z)))) 
            {}

        void expand(const Vector3<T>& point) {
            min = Vector3<T>(std::min(min.x, point.x), std::min(min.y, point.y), std::min(min.z, point.z));
            max = Vector3<T>(std::max(max.x, point.x), std::max(max.y, point.y), std::max(max.z, point.z));
        }

        void expand(const AABB<T>& other) {
            expand(other.min);
            expand(other.max);
        }
    };
}