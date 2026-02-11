#pragma once

#include "nyon/math/Vector2.h"
#include <vector>
#include <utility>

namespace Nyon::Utils
{
    /**
     * @brief Physics namespace providing shared physics types and constants.
     * 
     * This class serves as a namespace for shared physics types and constants.
     * Specific functionality has been moved to specialized classes:
     * - GravityPhysics: gravity and general physics updates
     * - CollisionPhysics: collision detection and resolution
     * - MovementPhysics: movement and kinematics
     */
    class Physics
    {
    public:
        
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
    private:
        // Private constructor to prevent instantiation
        Physics() = delete;
        ~Physics() = delete;
    };
}