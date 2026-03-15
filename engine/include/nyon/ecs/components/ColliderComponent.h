#pragma once

#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include <vector>
#include <variant>
#include <string>
#include <cstdint>
#include <algorithm>

namespace Nyon::ECS
{
    /**
     * @brief Enhanced collider component with full Box2D-inspired shape support.
     * 
     * Supports all major collision shapes with advanced features like sensors,
     * filtering, and material properties. Designed for high-performance collision detection.
     */
    struct ColliderComponent
    {
        // === SHAPE TYPES ===
        enum class ShapeType 
        { 
            Circle, 
            Polygon, 
            Capsule, 
            Segment,
            Chain,
            Composite 
        };
        
        // === SHAPE DEFINITIONS ===
        struct CircleShape
        {
            Math::Vector2 center = {0.0f, 0.0f};
            float radius = 16.0f;
        };
        
        struct PolygonShape
        {
            std::vector<Math::Vector2> vertices;
            std::vector<Math::Vector2> normals;  // Face normals for SAT
            Math::Vector2 centroid = {0.0f, 0.0f}; // Geometric center
            float radius = 0.0f; // Rounded corners radius
            
            PolygonShape() = default;
            PolygonShape(const std::vector<Math::Vector2>& verts) : vertices(verts) 
            {
                CalculateProperties();
            }
            
            void CalculateProperties()
            {
                if (vertices.empty()) return;
                
                // Validate and enforce counter-clockwise winding order
                // This ensures face normals point outward consistently
                if (!IsCounterClockwise()) {
                    // Reverse vertex order to make it counter-clockwise
                    std::reverse(vertices.begin(), vertices.end());
                }
                
                // Calculate true polygon centroid using area-weighted formula (shoelace method)
                // This fixes angular dynamics errors for irregular polygons
                centroid = {0.0f, 0.0f};
                float twoArea = 0.0f;
                
                size_t n = vertices.size();
                for (size_t i = 0; i < n; ++i) {
                    size_t j = (i + 1) % n;
                    float cross = vertices[i].x * vertices[j].y - vertices[j].x * vertices[i].y;
                    twoArea += cross;
                    centroid.x += (vertices[i].x + vertices[j].x) * cross;
                    centroid.y += (vertices[i].y + vertices[j].y) * cross;
                }
                
                if (std::abs(twoArea) > 0.0002f) {
                    centroid = centroid * (1.0f / (3.0f * twoArea)); // Direct formula using 2*area
                } else {
                    // Fallback to vertex average for degenerate cases
                    centroid = {0.0f, 0.0f};
                    for (const auto& vertex : vertices) {
                        centroid = centroid + vertex;
                    }
                    centroid = centroid * (1.0f / vertices.size());
                }
                
                // Calculate normals with proper outward direction enforcement
                normals.clear();
                for (size_t i = 0; i < vertices.size(); ++i) {
                    size_t next = (i + 1) % vertices.size();
                    Math::Vector2 edge = vertices[next] - vertices[i];
                    Math::Vector2 normal = {edge.y, -edge.x}; // Perpendicular CW for outward normal in Y-up CCW polygon
                    float length = sqrt(normal.x * normal.x + normal.y * normal.y);
                    if (length > 0.0001f) {
                        normal = normal * (1.0f / length);
                        normals.push_back(normal);
                    }
                }
            }
            
            // Helper method to check winding order
            bool IsCounterClockwise() const
            {
                if (vertices.size() < 3) return true;
                
                // Calculate signed area using shoelace formula
                float area = 0.0f;
                size_t n = vertices.size();
                for (size_t i = 0; i < n; ++i) {
                    size_t j = (i + 1) % n;
                    area += vertices[i].x * vertices[j].y - vertices[j].x * vertices[i].y;
                }
                
                // Positive area = counter-clockwise, negative = clockwise
                return area > 0.0f;
            }
        };
        
        struct CapsuleShape
        {
            Math::Vector2 center1 = {0.0f, 0.0f};
            Math::Vector2 center2 = {0.0f, 32.0f};
            float radius = 8.0f;
        };
        
        struct SegmentShape
        {
            Math::Vector2 point1 = {0.0f, 0.0f};
            Math::Vector2 point2 = {32.0f, 0.0f};
            float radius = 0.0f; // Thickness
        };
        
        struct ChainShape
        {
            std::vector<Math::Vector2> vertices;
            bool isLoop = false; // Closed loop vs open chain
            float radius = 0.0f;
        };
        
        struct CompositeShape
        {
            std::vector<std::variant<CircleShape, PolygonShape, CapsuleShape, SegmentShape>> subShapes;
        };
        
        // === CORE PROPERTIES ===
        ShapeType type = ShapeType::Polygon;
        Math::Vector3 color = {1.0f, 1.0f, 1.0f}; // Visual debugging color
        float density = 1.0f; // Mass per unit area
        
        // Variant to hold different shape types
        std::variant<CircleShape, PolygonShape, CapsuleShape, SegmentShape, ChainShape, CompositeShape> shape;
        
        // === FILTERING SYSTEM ===
        struct Filter
        {
            uint16_t categoryBits = 0x0001;  // Categories this shape belongs to
            uint16_t maskBits = 0xFFFF;      // Categories this shape can collide with
            int16_t groupIndex = 0;          // Group for fine-grained filtering
            
            // Default collision filtering
            bool ShouldCollide(const Filter& other) const
            {
                if (groupIndex != 0 && groupIndex == other.groupIndex) {
                    return groupIndex > 0; // Positive = must collide, negative = must not collide
                }
                
                bool collideA = (categoryBits & other.maskBits) != 0;
                bool collideB = (other.categoryBits & maskBits) != 0;
                return collideA && collideB;
            }
        };
        
        Filter filter;
        
        // === SENSOR SYSTEM ===
        bool isSensor = false;           // Trigger volume (no collision response)
        bool enableContactEvents = true; // Generate contact events
        bool enableSensorEvents = true;  // Generate sensor overlap events
        bool enableHitEvents = false;    // Generate hit events for high-speed impacts
        
        // === MATERIAL PROPERTIES ===
        struct Material
        {
            float friction = 0.2f;       // Friction coefficient (0-1)
            float restitution = 0.0f;    // Restitution coefficient (0-1)
            float density = 1.0f;        // Override shape density
            uint64_t userData = 0;       // Custom material identifier
            std::string name = "default"; // Material name for debugging
        };
        
        Material material;
        
        // === BROAD PHASE DATA ===
        int proxyId = -1;                // Broad phase proxy identifier
        bool forceUpdate = false;        // Force broad phase update next frame
        
        // === CONSTRUCTORS ===
        ColliderComponent() 
        {
            shape = PolygonShape({{0, 0}, {32, 0}, {32, 32}, {0, 32}});
        }
        
        ColliderComponent(const PolygonShape& poly) : type(ShapeType::Polygon)
        {
            shape = poly;
        }
        
        ColliderComponent(float radius) : type(ShapeType::Circle)
        {
            shape = CircleShape{{0, 0}, radius};
        }
        
        ColliderComponent(const CircleShape& circle) : type(ShapeType::Circle)
        {
            shape = circle;
        }
        
        ColliderComponent(const CapsuleShape& capsule) : type(ShapeType::Capsule)
        {
            shape = capsule;
        }
        
        ColliderComponent(const SegmentShape& segment) : type(ShapeType::Segment)
        {
            shape = segment;
        }
        
        // === SHAPE ACCESSORS ===
        template<typename T>
        T& GetShape() { return std::get<T>(shape); }
        
        template<typename T>
        const T& GetShape() const { return std::get<T>(shape); }
        
        PolygonShape& GetPolygon() { return std::get<PolygonShape>(shape); }
        const PolygonShape& GetPolygon() const { return std::get<PolygonShape>(shape); }
        
        CircleShape& GetCircle() { return std::get<CircleShape>(shape); }
        const CircleShape& GetCircle() const { return std::get<CircleShape>(shape); }
        
        CapsuleShape& GetCapsule() { return std::get<CapsuleShape>(shape); }
        const CapsuleShape& GetCapsule() const { return std::get<CapsuleShape>(shape); }
        
        SegmentShape& GetSegment() { return std::get<SegmentShape>(shape); }
        const SegmentShape& GetSegment() const { return std::get<SegmentShape>(shape); }
        
        // === GEOMETRY CALCULATIONS ===
        void CalculateAABB(const Math::Vector2& position, float rotation, Math::Vector2& outMin, Math::Vector2& outMax) const
        {
            const float speculativeDistance = 0.1f; // Extra padding for movement
            
            switch (type)
            {
                case ShapeType::Circle:
                {
                    const auto& circle = GetCircle();
                    Math::Vector2 worldCenter = circle.center + position;
                    float radius = circle.radius + speculativeDistance;
                    outMin = {worldCenter.x - radius, worldCenter.y - radius};
                    outMax = {worldCenter.x + radius, worldCenter.y + radius};
                    break;
                }
                
                case ShapeType::Polygon:
                {
                    const auto& polygon = GetPolygon();
                    if (polygon.vertices.empty()) {
                        outMin = position;
                        outMax = position;
                        return;
                    }
                    
                    // Apply rotation to vertices if there's rotation
                    if (std::abs(rotation) > 1e-6f)
                    {
                        float cosTheta = std::cos(rotation);
                        float sinTheta = std::sin(rotation);
                        
                        // Rotate first vertex
                        Math::Vector2 rotatedVertex = {
                            polygon.vertices[0].x * cosTheta - polygon.vertices[0].y * sinTheta,
                            polygon.vertices[0].x * sinTheta + polygon.vertices[0].y * cosTheta
                        };
                        outMin = rotatedVertex + position;
                        outMax = outMin;
                        
                        // Rotate and check remaining vertices
                        for (size_t i = 1; i < polygon.vertices.size(); ++i)
                        {
                            rotatedVertex = {
                                polygon.vertices[i].x * cosTheta - polygon.vertices[i].y * sinTheta,
                                polygon.vertices[i].x * sinTheta + polygon.vertices[i].y * cosTheta
                            };
                            Math::Vector2 worldVertex = rotatedVertex + position;
                            outMin.x = std::min(outMin.x, worldVertex.x);
                            outMin.y = std::min(outMin.y, worldVertex.y);
                            outMax.x = std::max(outMax.x, worldVertex.x);
                            outMax.y = std::max(outMax.y, worldVertex.y);
                        }
                    }
                    else
                    {
                        // No rotation - simple position offset
                        outMin = polygon.vertices[0] + position;
                        outMax = outMin;
                        
                        for (const auto& vertex : polygon.vertices) {
                            Math::Vector2 worldVertex = vertex + position;
                            outMin.x = std::min(outMin.x, worldVertex.x);
                            outMin.y = std::min(outMin.y, worldVertex.y);
                            outMax.x = std::max(outMax.x, worldVertex.x);
                            outMax.y = std::max(outMax.y, worldVertex.y);
                        }
                    }
                    
                    // Add speculative distance
                    outMin.x -= speculativeDistance;
                    outMin.y -= speculativeDistance;
                    outMax.x += speculativeDistance;
                    outMax.y += speculativeDistance;
                    break;
                }
                
                case ShapeType::Capsule:
                {
                    const auto& capsule = GetCapsule();
                    
                    // Apply rotation to capsule endpoints
                    float cosR = std::cos(rotation);
                    float sinR = std::sin(rotation);
                    auto rotate2D = [&](Math::Vector2 v) -> Math::Vector2 {
                        return { v.x * cosR - v.y * sinR, v.x * sinR + v.y * cosR };
                    };
                    
                    Math::Vector2 center1 = rotate2D(capsule.center1) + position;
                    Math::Vector2 center2 = rotate2D(capsule.center2) + position;
                    
                    Math::Vector2 min = {
                        std::min(center1.x, center2.x) - capsule.radius,
                        std::min(center1.y, center2.y) - capsule.radius
                    };
                    Math::Vector2 max = {
                        std::max(center1.x, center2.x) + capsule.radius,
                        std::max(center1.y, center2.y) + capsule.radius
                    };
                    
                    outMin = min;
                    outMax = max;
                    break;
                }
                
                case ShapeType::Segment:
                {
                    const auto& segment = GetSegment();
                    
                    // Apply rotation to segment endpoints
                    float cosR = std::cos(rotation);
                    float sinR = std::sin(rotation);
                    auto rotate2D = [&](Math::Vector2 v) -> Math::Vector2 {
                        return { v.x * cosR - v.y * sinR, v.x * sinR + v.y * cosR };
                    };
                    
                    Math::Vector2 p1 = rotate2D(segment.point1) + position;
                    Math::Vector2 p2 = rotate2D(segment.point2) + position;
                    
                    outMin = {
                        std::min(p1.x, p2.x) - segment.radius,
                        std::min(p1.y, p2.y) - segment.radius
                    };
                    outMax = {
                        std::max(p1.x, p2.x) + segment.radius,
                        std::max(p1.y, p2.y) + segment.radius
                    };
                    break;
                }
                
                default:
                    outMin = position;
                    outMax = position;
                    break;
            }
        }
        
        float CalculateArea() const
        {
            switch (type)
            {
                case ShapeType::Circle:
                {
                    const auto& circle = GetCircle();
                    return 3.14159f * circle.radius * circle.radius;
                }
                
                case ShapeType::Polygon:
                {
                    const auto& polygon = GetPolygon();
                    if (polygon.vertices.size() < 3) return 0.0f;
                    
                    // Shoelace formula
                    float area = 0.0f;
                    size_t n = polygon.vertices.size();
                    for (size_t i = 0; i < n; ++i) {
                        size_t j = (i + 1) % n;
                        area += polygon.vertices[i].x * polygon.vertices[j].y;
                        area -= polygon.vertices[j].x * polygon.vertices[i].y;
                    }
                    return std::abs(area) * 0.5f;
                }
                
                case ShapeType::Capsule:
                {
                    const auto& capsule = GetCapsule();
                    Math::Vector2 diff = capsule.center2 - capsule.center1;
                    float length = sqrt(diff.x * diff.x + diff.y * diff.y);
                    return 3.14159f * capsule.radius * capsule.radius + length * 2 * capsule.radius;
                }
                
                default:
                    return 0.0f;
            }
        }
        
        // Approximate moment of inertia about the local origin for a unit-density body.
        // The caller should multiply by actual mass if needed.
        float CalculateInertiaForUnitDensity() const
        {
            switch (type)
            {
                case ShapeType::Circle:
                {
                    const auto& circle = GetCircle();
                    // I/m = r²/2 for solid disk (per-unit-mass inertia)
                    // Caller multiplies by mass to get actual inertia: I = m * (r²/2)
                    return 0.5f * circle.radius * circle.radius;
                }
                
                case ShapeType::Polygon:
                {
                    // Correct polygon inertia using shoelace-based formula
                    const auto& polygon = GetPolygon();
                    if (polygon.vertices.size() < 3) return 0.0f;
                    
                    const auto& verts = polygon.vertices;
                    size_t n = verts.size();
                    float numerator = 0.0f;
                    float twoArea = 0.0f;
                    
                    for (size_t i = 0; i < n; ++i) {
                        const Math::Vector2& p0 = verts[i];
                        const Math::Vector2& p1 = verts[(i + 1) % n];
                        float cross = std::abs(Math::Vector2::Cross(p0, p1));
                        
                        // Accumulate terms for moment of inertia
                        numerator += cross * (
                            p0.x * p0.x + p0.x * p1.x + p1.x * p1.x +
                            p0.y * p0.y + p0.y * p1.y + p1.y * p1.y
                        );
                        
                        // Accumulate 2 * area
                        twoArea += cross;
                    }
                    
                    // I = (density * area * numerator) / (6 * 2*area) = density * numerator / 12
                    // For unit density: I = numerator / 12
                    return numerator / 12.0f;
                }
                
                case ShapeType::Capsule:
                {
                    const auto& capsule = GetCapsule();
                    Math::Vector2 diff = capsule.center2 - capsule.center1;
                    float h = std::sqrt(diff.x * diff.x + diff.y * diff.y);
                    float r = capsule.radius;
                    // Rough approximation: treat as rectangle (2r x h) plus two circles.
                    float rectArea = 2.0f * r * h;
                    float rectInertia = rectArea * (h * h + (2.0f * r) * (2.0f * r)) / 12.0f;
                    float circleArea = 3.14159f * r * r;
                    float circleInertia = 0.5f * 3.14159f * r * r * r * r * 2.0f; // two circles
                    return rectInertia + circleInertia;
                }
                
                default:
                    return 0.0f;
            }
        }
        
        float CalculateMass(float densityOverride = -1.0f) const
        {
            float actualDensity = (densityOverride > 0.0f) ? densityOverride : material.density;
            return CalculateArea() * actualDensity;
        }
        
        // === UTILITY METHODS ===
        ShapeType GetType() const { return type; }
        bool IsSensor() const { return isSensor; }
        void SetSensor(bool sensor) { isSensor = sensor; }
        void SetFilter(const Filter& newFilter) { filter = newFilter; }
        Filter GetFilter() const { return filter; }
        
        // Backwards compatibility method - DEPRECATED, use CalculateAABB(position, rotation, min, max) instead
        void GetBounds(const Math::Vector2& position, Math::Vector2& outMin, Math::Vector2& outMax) const
        {
            CalculateAABB(position, 0.0f, outMin, outMax);
        }
    };
}
