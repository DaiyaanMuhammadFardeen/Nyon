#pragma once

#include "nyon/math/Vector2.h"
#include <vector>
#include <cstdint>

namespace Nyon::ECS
{
    /**
     * @brief Contact point data structure for collision information.
     * 
     * Stores detailed information about contact points between bodies.
     */
    struct ContactPoint
    {
        Math::Vector2 position = {0.0f, 0.0f};          // World position of contact point
        Math::Vector2 normal = {0.0f, 0.0f};            // Contact normal (points from A to B)
        float separation = 0.0f;                        // Separation distance (negative = penetration)
        float normalImpulse = 0.0f;                     // Normal impulse applied
        float tangentImpulse = 0.0f;                    // Tangent impulse applied
        float normalMass = 0.0f;                        // Normal constraint mass
        float tangentMass = 0.0f;                       // Tangent constraint mass
        float velocityBias = 0.0f;                      // Velocity bias for restitution
        uint32_t featureId = 0;                         // Feature identifier for persistence
        bool persisted = false;                         // Whether this point persisted from previous step
    };
    
    /**
     * @brief Contact manifold representing collision between two shapes.
     * 
     * Contains all contact points and geometric information for a collision.
     */
    struct ContactManifold
    {
        std::vector<ContactPoint> points;           // Contact points (0-2 typically)
        Math::Vector2 normal = {0.0f, 0.0f};        // Contact normal
        Math::Vector2 localNormal = {0.0f, 0.0f};   // Normal in local coordinates
        Math::Vector2 localPoint = {0.0f, 0.0f};    // Reference point in local coordinates
        float friction = 0.0f;                      // Combined friction coefficient
        float restitution = 0.0f;                   // Combined restitution coefficient
        float tangentSpeed = 0.0f;                  // Tangent speed for friction
        uint32_t entityIdA = 0;                     // First entity ID
        uint32_t entityIdB = 0;                     // Second entity ID
        uint32_t shapeIdA = 0;                      // First shape ID
        uint32_t shapeIdB = 0;                      // Second shape ID
        bool touching = false;                      // Whether shapes are touching
        bool persisted = false;                     // Whether this contact persisted from previous frame
    };
}
