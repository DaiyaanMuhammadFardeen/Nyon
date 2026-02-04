#pragma once

#include "nyon/math/Vector2.h"
#include <vector>
#include <utility>

namespace Nyon::Utils
{
    /**
     * @brief Physics class providing collision detection and physics simulation utilities.
     * 
     * Uses a y-positive-down coordinate system with pixels as units. Polygons must be convex.
     */
    class Physics
    {
    public:
        /// Gravity constant in pixels/s^2 (y-positive-down coordinate system)
        static const float Gravity;
        
        /**
         * @brief Represents a physics body with position, velocity, and physical properties.
         * 
         * Coordinates use y-positive-down system (standard for screen coordinates).
         * Polygons must be convex and wound consistently.
         */
        struct Body
        {
            Math::Vector2 position;   ///< Position in world space
            Math::Vector2 velocity;   ///< Velocity in pixels/second
            Math::Vector2 acceleration; ///< Acceleration in pixels/second^2
            float mass;               ///< Mass of the body (used in future calculations)
            float friction;           ///< Friction coefficient when grounded (0.0 = no friction, higher = more friction)
            float drag;               ///< Drag coefficient for air resistance (0.0 = no air resistance)
            float maxSpeed;           ///< Maximum speed limit to prevent extreme velocities
            bool isStatic;            ///< Whether the body is static (immovable)
            
            Body() : position(0.0f, 0.0f), velocity(0.0f, 0.0f), acceleration(0.0f, 0.0f), 
                   mass(1.0f), friction(0.1f), drag(0.0f), maxSpeed(1000.0f), isStatic(false) {}
        };
        
        /// Define a polygon as a collection of vertices in local space
        using Polygon = std::vector<Math::Vector2>;
        
        /**
         * @brief Result of collision detection with MTV (Minimum Translation Vector).
         * 
         * Contains information needed to resolve a collision.
         */
        struct CollisionResult {
            bool collided;              ///< True if collision occurred
            Math::Vector2 overlapAxis;  ///< Direction vector to push object to resolve collision
            float overlapAmount;        ///< Distance to push object to resolve collision
            
            CollisionResult() : collided(false), overlapAxis(0.0f, 0.0f), overlapAmount(0.0f) {}
            CollisionResult(bool coll, const Math::Vector2& axis, float amount) 
                : collided(coll), overlapAxis(axis), overlapAmount(amount) {}
        };
        
        /**
         * @brief Updates the physics body with gravity, friction, and other forces.
         * 
         * Uses sub-stepping to prevent tunneling with large delta times.
         * Automatically applies gravity unless the body is grounded.
         * Applies friction when grounded and drag for air resistance.
         * Clamps velocity to prevent extreme speeds.
         * 
         * @param body The physics body to update
         * @param deltaTime Time elapsed since last update (seconds)
         * @param isGrounded Whether the body is in contact with a surface
         */
        static void UpdateBody(Body& body, float deltaTime, bool isGrounded = false);
        
        /**
         * @brief Checks collision between two rectangular bodies using AABB collision detection.
         * 
         * @param body1 First body
         * @param size1 Size of first body
         * @param body2 Second body
         * @param size2 Size of second body
         * @return True if bodies are colliding
         */
        static bool CheckCollision(const Body& body1, const Math::Vector2& size1, 
                                  const Body& body2, const Math::Vector2& size2);
        
        /**
         * @brief Broad-phase collision check using Axis-Aligned Bounding Boxes (AABB).
         * 
         * @param pos1 Position of first box
         * @param size1 Size of first box
         * @param pos2 Position of second box
         * @param size2 Size of second box
         * @return True if AABBs are overlapping
         */
        static bool CheckAABBCollision(const Math::Vector2& pos1, const Math::Vector2& size1,
                                      const Math::Vector2& pos2, const Math::Vector2& size2);
        
        /**
         * @brief SAT-based collision detection between two convex polygons.
         * 
         * Uses Separating Axis Theorem (SAT) to detect collisions.
         * Returns the Minimum Translation Vector (MTV) to resolve collision.
         * Polygons must be convex and wound consistently.
         * 
         * @param poly1 First polygon (vertices in local space)
         * @param pos1 World position of first polygon
         * @param poly2 Second polygon (vertices in local space)
         * @param pos2 World position of second polygon
         * @return CollisionResult with collision information and MTV
         */
        static CollisionResult CheckPolygonCollision(const Polygon& poly1, const Math::Vector2& pos1,
                                                   const Polygon& poly2, const Math::Vector2& pos2);
        
        /**
         * @brief Resolve collision between two bodies using the MTV from collision detection.
         * 
         * Moves bodies apart according to their masses. Non-static bodies are moved, static
         * bodies remain stationary. The resolution accounts for mass ratios.
         * 
         * @param body1 First body involved in collision
         * @param body2 Second body involved in collision
         * @param result Collision result containing MTV from collision detection
         */
        static void ResolveCollision(Body& body1, Body& body2, const CollisionResult& result);
        
        /**
         * @brief Determines if a body is grounded based on collision with surfaces below it.
         * 
         * A body is considered grounded if there's a collision with a surface whose normal
         * has a significant y-component (pointing up). Threshold controls sensitivity.
         * 
         * @param body The body to check for grounded state
         * @param collisionResult Result from collision detection
         * @param threshold Minimum y-component of normal to consider as ground (default 0.7)
         * @return True if the body is in contact with a surface below it
         */
        static bool IsBodyGrounded(const Body& body, const CollisionResult& collisionResult, float threshold = 0.7f);
        
    private:
        // Internal function for sub-stepped physics updates
        static void InternalUpdateBody(Body& body, float deltaTime, bool isGrounded);
        // Helper functions for SAT algorithm
        static Math::Vector2 GetEdgeNormal(const Math::Vector2& edge);
        static float DotProduct(const Math::Vector2& a, const Math::Vector2& b);
        static std::pair<float, float> ProjectPolygonOntoAxis(const Polygon& polygon, 
                                                             const Math::Vector2& pos,
                                                             const Math::Vector2& axis);
        static bool CheckOverlap(float min1, float max1, float min2, float max2);
        static Math::Vector2 CalculatePolygonCenter(const Polygon& polygon, const Math::Vector2& pos);
    };
}