#pragma once
#include <cmath>
namespace Nyon::Math {
    struct Vector2 {
        float x, y;
        Vector2() : x(0.0f), y(0.0f) {}
        Vector2(float x, float y) : x(x), y(y) {}
        Vector2 operator+(const Vector2& other) const { return Vector2(x + other.x, y + other.y); }
        Vector2 operator-(const Vector2& other) const { return Vector2(x - other.x, y - other.y); }
        Vector2 operator-() const { return Vector2(-x, -y); }   
        Vector2 operator*(float scalar) const { return Vector2(x * scalar, y * scalar); }
        Vector2 operator/(float scalar) const { return Vector2(x / scalar, y / scalar); }
        Vector2& operator+=(const Vector2& other) { x += other.x; y += other.y; return *this; }
        Vector2& operator-=(const Vector2& other) { x -= other.x; y -= other.y; return *this; }
        Vector2& operator*=(float scalar) { x *= scalar; y *= scalar; return *this; }
        Vector2& operator/=(float scalar) { x /= scalar; y /= scalar; return *this; }
        void Set(float newX, float newY) { x = newX; y = newY; }
        [[nodiscard]] float Length() const { return std::sqrt(x * x + y * y); }
        [[nodiscard]] float LengthSquared() const { return x * x + y * y; }
        [[nodiscard]] Vector2 Normalize() const { 
            float len = Length();
            if (len > 0.0f) return Vector2(x / len, y / len);
            return Vector2(0.0f, 0.0f); }
        [[nodiscard]] static float Dot(const Vector2& a, const Vector2& b) {
            return a.x * b.x + a.y * b.y; }
        [[nodiscard]] static float Cross(const Vector2& a, const Vector2& b) {
            return a.x * b.y - a.y * b.x; }
        [[nodiscard]] static Vector2 Cross(float s, const Vector2& v) {
            return Vector2(-s * v.y, s * v.x); }
        [[nodiscard]] static Vector2 Cross(const Vector2& v, float s) {
            return Vector2(s * v.y, -s * v.x); }
        [[nodiscard]] static float DistanceSquared(const Vector2& a, const Vector2& b) {
            float dx = a.x - b.x;
            float dy = a.y - b.y;
            return dx * dx + dy * dy; } };
    struct Rotation2D {
        float c, s;  
        Rotation2D() : c(1.0f), s(0.0f) {}
        explicit Rotation2D(float angle) : c(std::cos(angle)), s(std::sin(angle)) {}
        Rotation2D(float cos_val, float sin_val) : c(cos_val), s(sin_val) {}
        [[nodiscard]] Vector2 operator*(const Vector2& v) const {
            return {c * v.x - s * v.y, s * v.x + c * v.y}; }
        [[nodiscard]] Rotation2D Inverse() const {
            return {c, -s}; } }; }