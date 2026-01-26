#pragma once

#include <cmath>

namespace Nyon::Math
{
    struct Vector2
    {
        float x, y;

        Vector2() : x(0.0f), y(0.0f) {}
        Vector2(float x, float y) : x(x), y(y) {}
        Vector2(float x, float y, float z) : x(x), y(y) {} // For compatibility with color vectors

        // Basic operations
        Vector2 operator+(const Vector2& other) const { return Vector2(x + other.x, y + other.y); }
        Vector2 operator-(const Vector2& other) const { return Vector2(x - other.x, y - other.y); }
        Vector2 operator*(float scalar) const { return Vector2(x * scalar, y * scalar); }
        Vector2 operator/(float scalar) const { return Vector2(x / scalar, y / scalar); }

        Vector2& operator+=(const Vector2& other) { x += other.x; y += other.y; return *this; }
        Vector2& operator-=(const Vector2& other) { x -= other.x; y -= other.y; return *this; }
        Vector2& operator*=(float scalar) { x *= scalar; y *= scalar; return *this; }
        Vector2& operator/=(float scalar) { x /= scalar; y /= scalar; return *this; }

        float Length() const { return sqrt(x * x + y * y); }
        float LengthSquared() const { return x * x + y * y; }
        Vector2 Normalize() const 
        { 
            float len = Length();
            if (len > 0.0f) return Vector2(x / len, y / len);
            return Vector2(0.0f, 0.0f);
        }
    };
}