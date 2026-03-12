#pragma once
#include <cmath>

namespace math {
    template<typename T>
    struct Vector3 {
        T x, y, z;
        Vector3(): x(0.0), y(0.0), z(0.0) {}
        Vector3(T x, T y, T z): x(x), y(y), z(z) {}
        explicit Vector3(T scalar): x(scalar), y(scalar), z(scalar) {}

        Vector3 operator+(const Vector3& other) const {
            return Vector3(x + other.x, y + other.y, z + other.z);
        }

        Vector3 operator-(const Vector3& other) const {
            return Vector3(x - other.x, y - other.y, z - other.z);
        }

        Vector3 operator*(const T& scalar) const {
            return Vector3(x * scalar, y * scalar, z * scalar);
        }

        Vector3 operator/(const T& scalar) const {
            return Vector3(x / scalar, y / scalar, z / scalar);
        }

        Vector3 operator-() const {
            return Vector3(-x, -y, -z);
        }

        Vector3& operator+=(const Vector3& other) {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }

        Vector3& operator-=(const Vector3& other) {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }

        Vector3& operator*=(const T& scalar) {
            x *= scalar;
            y *= scalar;
            z *= scalar;
            return *this;
        }

        Vector3& operator/=(const T& scalar) {
            x /= scalar;
            y /= scalar;
            z /= scalar;
            return *this;
        }

        const T& operator[](int index) const {
            return (&x)[index];
        }

        T& operator[](int index) {
            return (&x)[index];
        }

        bool operator==(const Vector3& other) const {
            return x == other.x && y == other.y && z == other.z;
        }

        bool operator!=(const Vector3& other) const {
            return !(*this == other);
        }

    };

    template<typename T>
    inline T dot(const Vector3<T>& a, const Vector3<T>& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    template<typename T>
    inline Vector3<T> cross(const Vector3<T>& a, const Vector3<T>& b) {
        return Vector3<T>(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
    }

    template<typename T>
    inline T length(const Vector3<T>& v) {
        return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    template<typename T>
    inline Vector3<T> normalize(const Vector3<T>& v) {
        return v / length(v);
    }

    using Vector3f = Vector3<float>;
    using Vector3d = Vector3<double>;
};