#pragma once

#include "nyon/math/Vector2.h"
#include <vector>

namespace Nyon::Utils
{
    class Physics
    {
    public:
        static const float Gravity;
        
        struct Body
        {
            Math::Vector2 position;
            Math::Vector2 velocity;
            Math::Vector2 acceleration;
            float mass;
            bool isStatic;
            
            Body() : position(0.0f, 0.0f), velocity(0.0f, 0.0f), acceleration(0.0f, 0.0f), mass(1.0f), isStatic(false) {}
        };
        
        // Define a polygon as a collection of vertices
        using Polygon = std::vector<Math::Vector2>;
        
        static void ApplyGravity(Body& body);
        static void UpdateBody(Body& body, float deltaTime);
        static bool CheckCollision(const Body& body1, const Math::Vector2& size1, 
                                  const Body& body2, const Math::Vector2& size2);
        // New SAT-based collision detection functions
        static bool CheckPolygonCollision(const Polygon& poly1, const Math::Vector2& pos1,
                                         const Polygon& poly2, const Math::Vector2& pos2);
        
    private:
        // Helper functions for SAT algorithm
        static Math::Vector2 GetEdgeNormal(const Math::Vector2& edge);
        static float DotProduct(const Math::Vector2& a, const Math::Vector2& b);
        static std::pair<float, float> ProjectPolygonOntoAxis(const Polygon& polygon, 
                                                             const Math::Vector2& pos,
                                                             const Math::Vector2& axis);
        static bool CheckOverlap(float min1, float max1, float min2, float max2);
    };
}