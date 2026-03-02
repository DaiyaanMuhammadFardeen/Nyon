#pragma once

#include "nyon/math/Vector2.h"

namespace Nyon::ECS
{
    /**
     * @brief Physics body component for dynamic physics simulation.
     * 
     * Contains all physical properties needed for motion and collision response.
     * Uses y-positive-down coordinate system consistent with rendering.
     */
    struct PhysicsBodyComponent
    {
        Math::Vector2 velocity = {0.0f, 0.0f};      // Velocity in pixels/second
        Math::Vector2 acceleration = {0.0f, 0.0f};  // Acceleration in pixels/second^2
        float mass = 1.0f;                          // Mass of the body
        float friction = 0.1f;                      // Friction coefficient when grounded
        float drag = 0.0f;                          // Drag coefficient for air resistance
        float maxSpeed = 1000.0f;                   // Maximum speed limit
        bool isStatic = false;                      // Whether body is immovable
        bool isGrounded = false;                    // Current grounded state
        
        // For stable grounded detection
        int groundedFrames = 0;                     // Consecutive frames grounded
        static constexpr int GROUNDED_THRESHOLD = 2; // Minimum frames to be considered grounded
        
        PhysicsBodyComponent() = default;
        PhysicsBodyComponent(float m) : mass(m) {}
        PhysicsBodyComponent(float m, bool stat) : mass(m), isStatic(stat) {}
        
        // Stable grounded state getter
        bool IsStablyGrounded() const { return groundedFrames >= GROUNDED_THRESHOLD; }
        
        // Update grounded frame counter
        void UpdateGroundedState(bool currentlyGrounded)
        {
            if (currentlyGrounded) {
                groundedFrames++;
            } else {
                groundedFrames = 0;
            }
            isGrounded = IsStablyGrounded();
        }
    };
}