#pragma once

#include "nyon/math/Vector2.h"
#include "nyon/utils/Physics.h"

namespace Nyon::Utils
{
    /**
     * @brief MovementPhysics class providing movement-related physics utilities.
     * 
     * Handles general movement calculations and kinematics.
     */
    class MovementPhysics
    {
    public:
        /**
         * @brief Applies a force to a physics body.
         * 
         * Adds the specified force to the body's acceleration.
         * 
         * @param body The physics body to apply force to
         * @param force The force vector to apply
         */
        static void ApplyForce(Physics::Body& body, const Math::Vector2& force);
        
        /**
         * @brief Applies an impulse to a physics body.
         * 
         * Directly modifies the body's velocity by the impulse amount.
         * 
         * @param body The physics body to apply impulse to
         * @param impulse The impulse vector to apply
         */
        static void ApplyImpulse(Physics::Body& body, const Math::Vector2& impulse);
        
        /**
         * @brief Sets the velocity of a physics body.
         * 
         * Directly sets the body's velocity to the specified value.
         * 
         * @param body The physics body to set velocity for
         * @param velocity The new velocity vector
         */
        static void SetVelocity(Physics::Body& body, const Math::Vector2& velocity);
        
        /**
         * @brief Gets the current speed of a physics body.
         * 
         * Returns the magnitude of the body's velocity vector.
         * 
         * @param body The physics body to get speed for
         * @return The current speed (magnitude of velocity)
         */
        static float GetSpeed(const Physics::Body& body);
        
        /**
         * @brief Gets the current velocity angle of a physics body.
         * 
         * Returns the angle of the velocity vector in radians.
         * 
         * @param body The physics body to get velocity angle for
         * @return The angle of the velocity vector in radians
         */
        static float GetVelocityAngle(const Physics::Body& body);
        
        /**
         * @brief Limits the speed of a physics body.
         * 
         * Clamps the body's velocity magnitude to the specified maximum speed.
         * 
         * @param body The physics body to limit speed for
         * @param maxSpeed The maximum allowed speed
         */
        static void LimitSpeed(Physics::Body& body, float maxSpeed);
    };
}