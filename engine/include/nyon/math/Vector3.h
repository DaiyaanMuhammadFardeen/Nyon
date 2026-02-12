#pragma once

#include <cmath>

namespace Nyon::Math
{
    struct Vector3
    {
        float x, y, z;

        Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
        Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

        // Basic operations
        Vector3 operator+(const Vector3& other) const { return Vector3(x + other.x, y + other.y, z + other.z); }
        Vector3 operator-(const Vector3& other) const { return Vector3(x - other.x, y - other.y, z - other.z); }
        Vector3 operator*(float scalar) const { return Vector3(x * scalar, y * scalar, z * scalar); }
        Vector3 operator/(float scalar) const { return Vector3(x / scalar, y / scalar, z / scalar); }

        Vector3& operator+=(const Vector3& other) { x += other.x; y += other.y; z += other.z; return *this; }
        Vector3& operator-=(const Vector3& other) { x -= other.x; y -= other.y; z -= other.z; return *this; }
        Vector3& operator*=(float scalar) { x *= scalar; y *= scalar; z *= scalar; return *this; }
        Vector3& operator/=(float scalar) { x /= scalar; y /= scalar; z /= scalar; return *this; }

        // Unary minus
        Vector3 operator-() const { return Vector3(-x, -y, -z); }

        float Length() const { return sqrt(x * x + y * y + z * z); }
        float LengthSquared() const { return x * x + y * y + z * z; }
        Vector3 Normalize() const 
        { 
            float len = Length();
            if (len > 0.0f) return Vector3(x / len, y / len, z / len);
            return Vector3(0.0f, 0.0f, 0.0f);
        }

        // Cross product
        Vector3 Cross(const Vector3& other) const
        {
            return Vector3(
                y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x
            );
        }

        // Dot product
        float Dot(const Vector3& other) const
        {
            return x * other.x + y * other.y + z * other.z;
        }
    };
}