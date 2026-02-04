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
        
        // Result of collision detection with MTV (Minimum Translation Vector)
        struct CollisionResult {
            bool collided;
            Math::Vector2 overlapAxis;  // Direction to push object to resolve collision
            float overlapAmount;        // Distance to push object to resolve collision
            
            CollisionResult() : collided(false), overlapAxis(0.0f, 0.0f), overlapAmount(0.0f) {}
            CollisionResult(bool coll, const Math::Vector2& axis, float amount) 
                : collided(coll), overlapAxis(axis), overlapAmount(amount) {}
        };
        
        // Deprecated: Use UpdateBody instead which handles gravity internally
        static void ApplyGravity(Body& body);
        // Physics integration function that applies gravity and other forces
        // NOTE: This function resets acceleration to (0,0) at the end of the update
        static void UpdateBody(Body& body, float deltaTime, bool isGrounded = false);
        static bool CheckCollision(const Body& body1, const Math::Vector2& size1, 
                                  const Body& body2, const Math::Vector2& size2);
        // Broad-phase collision check before SAT
        static bool CheckAABBCollision(const Math::Vector2& pos1, const Math::Vector2& size1,
                                      const Math::Vector2& pos2, const Math::Vector2& size2);
        // New SAT-based collision detection functions
        static CollisionResult CheckPolygonCollision(const Polygon& poly1, const Math::Vector2& pos1,
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