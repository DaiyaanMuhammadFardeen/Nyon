#pragma once

#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include <vector>
#include <variant>

namespace Nyon::ECS
{
    /**
     * @brief Collider component supporting multiple shape types.
     * 
     * Supports various collision shapes for flexible physics interactions.
     */
    struct ColliderComponent
    {
        enum class ShapeType 
        { 
            Polygon, 
            Circle, 
            Capsule, 
            Composite 
        };
        
        struct CircleShape
        {
            Math::Vector2 center = {0.0f, 0.0f};
            float radius = 16.0f;
        };
        
        struct CapsuleShape
        {
            Math::Vector2 start = {0.0f, 0.0f};
            Math::Vector2 end = {0.0f, 32.0f};
            float radius = 8.0f;
        };
        
        struct CompositeShape
        {
            std::vector<std::vector<Math::Vector2>> subShapes;
        };
        
        // Using std::vector for polygon to maintain compatibility with existing code
        using PolygonShape = std::vector<Math::Vector2>;
        
        ShapeType type = ShapeType::Polygon;
        Math::Vector3 color = {1.0f, 1.0f, 1.0f}; // Visual color
        float restitution = 0.0f; // Bounciness (0-1)
        
        // Variant to hold different shape types
        std::variant<PolygonShape, CircleShape, CapsuleShape, CompositeShape> shape;
        
        // Physics material properties
        struct PhysicsMaterial
        {
            float friction = 0.1f;
            float restitution = 0.0f;
            float density = 1.0f;
            std::string surfaceType = "default";
        };
        
        PhysicsMaterial material;
        
        ColliderComponent() 
        {
            shape = PolygonShape{{0, 0}, {32, 0}, {32, 32}, {0, 32}};
        }
        
        ColliderComponent(const PolygonShape& poly) : type(ShapeType::Polygon)
        {
            shape = poly;
        }
        
        ColliderComponent(float radius) : type(ShapeType::Circle)
        {
            shape = CircleShape{{0, 0}, radius};
        }
        
        // Helper methods to access shape data safely
        PolygonShape& GetPolygon() 
        {
            return std::get<PolygonShape>(shape);
        }
        
        const PolygonShape& GetPolygon() const
        {
            return std::get<PolygonShape>(shape);
        }
        
        CircleShape& GetCircle()
        {
            return std::get<CircleShape>(shape);
        }
        
        const CircleShape& GetCircle() const
        {
            return std::get<CircleShape>(shape);
        }
        
        // Calculate AABB for the collider
        void CalculateAABB(const Math::Vector2& position, Math::Vector2& outMin, Math::Vector2& outMax) const
        {
            switch (type)
            {
                case ShapeType::Polygon:
                {
                    const auto& polygon = GetPolygon();
                    if (polygon.empty()) {
                        outMin = position;
                        outMax = position;
                        return;
                    }
                    
                    outMin = polygon[0] + position;
                    outMax = outMin;
                    
                    for (const auto& vertex : polygon) {
                        Math::Vector2 worldVertex = vertex + position;
                        if (worldVertex.x < outMin.x) outMin.x = worldVertex.x;
                        if (worldVertex.y < outMin.y) outMin.y = worldVertex.y;
                        if (worldVertex.x > outMax.x) outMax.x = worldVertex.x;
                        if (worldVertex.y > outMax.y) outMax.y = worldVertex.y;
                    }
                    break;
                }
                
                case ShapeType::Circle:
                {
                    const auto& circle = GetCircle();
                    Math::Vector2 worldCenter = circle.center + position;
                    outMin = {worldCenter.x - circle.radius, worldCenter.y - circle.radius};
                    outMax = {worldCenter.x + circle.radius, worldCenter.y + circle.radius};
                    break;
                }
                
                // TODO: Implement for other shapes
                case ShapeType::Capsule:
                case ShapeType::Composite:
                    outMin = position;
                    outMax = position;
                    break;
            }
        }
    };
}