#pragma once

#include "nyon/math/Vector2.h"
#include "nyon/utils/Physics.h"

namespace Nyon::Utils
{
    /**
     * @brief GravityPhysics class providing gravitational physics simulation utilities.
     * 
     * Uses a y-positive-down coordinate system with pixels as units.
     */
    class GravityPhysics
    {
    public:
        /// Gravity constant in pixels/s^2 (y-positive-down coordinate system)
        static const float Gravity;
        
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
        static void UpdateBody(Physics::Body& body, float deltaTime, bool isGrounded = false);

    private:
        // Internal function for sub-stepped physics updates
        static void InternalUpdateBody(Physics::Body& body, float deltaTime, bool isGrounded);
    };
}