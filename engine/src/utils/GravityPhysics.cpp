#include "nyon/utils/GravityPhysics.h"
#include <algorithm>
#include <cmath>

namespace Nyon::Utils
{
    const float GravityPhysics::Gravity = 980.0f; // pixels/s^2
    
    void GravityPhysics::UpdateBody(Physics::Body& body, float deltaTime, bool isGrounded)
    {
        if (!body.isStatic)
        {
            // Sub-stepping to prevent tunneling with large delta times
            const float maxStep = 1.0f / 60.0f;  // Maximum time step (60 FPS equivalent)
            float remainingTime = deltaTime;
            
            while (remainingTime > maxStep) {
                InternalUpdateBody(body, maxStep, isGrounded);
                remainingTime -= maxStep;
            }
            // Update with remaining time (could be 0 if deltaTime < maxStep)
            InternalUpdateBody(body, remainingTime, isGrounded);
        }
    }
    
    void GravityPhysics::InternalUpdateBody(Physics::Body& body, float deltaTime, bool isGrounded)
    {
        if (deltaTime > 0.0f)
        {
            if (body.isStatic) {
                // Static bodies have zero velocity and acceleration
                body.velocity.x = 0.0f;
                body.velocity.y = 0.0f;
                body.acceleration.x = 0.0f;
                body.acceleration.y = 0.0f;
                return; // Static bodies don't need further updates
            }
            
            // Apply other accelerations to velocity
            body.velocity.x += body.acceleration.x * deltaTime;
            body.velocity.y += body.acceleration.y * deltaTime;
            
            // Apply gravity as acceleration
            body.acceleration.y += Gravity;
            
            // Apply all accumulated acceleration to velocity
            body.velocity.x += body.acceleration.x * deltaTime;
            body.velocity.y += body.acceleration.y * deltaTime;
            
            // Handle grounded state after velocity update
            if (isGrounded) {
                // Clamp vertical velocity if moving downward into the ground
                if (body.velocity.y > 0.0f) {
                    body.velocity.y = 0.0f;
                }
            }
            
            // Apply friction if grounded
            if (isGrounded) {
                body.velocity.x *= (1.0f - std::min(body.friction * deltaTime, 1.0f));
            }
            // Apply drag (air resistance)
            body.velocity *= (1.0f - std::min(body.drag * deltaTime, 1.0f));
            
            // Apply velocity to position (Symplectic Euler)
            body.position.x += body.velocity.x * deltaTime;
            body.position.y += body.velocity.y * deltaTime;
            
            // Clamp velocity to prevent extreme speeds
            const float maxVel = body.maxSpeed;
            if (body.velocity.x > maxVel) body.velocity.x = maxVel;
            if (body.velocity.x < -maxVel) body.velocity.x = -maxVel;
            if (body.velocity.y > maxVel) body.velocity.y = maxVel;
            if (body.velocity.y < -maxVel) body.velocity.y = -maxVel;
            
            // CRITICAL: Reset acceleration for the next frame
            body.acceleration.x = 0.0f;
            body.acceleration.y = 0.0f;
        }
    }
}