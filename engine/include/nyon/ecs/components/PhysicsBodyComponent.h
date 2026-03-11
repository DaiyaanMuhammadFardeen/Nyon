#pragma once

#include "nyon/math/Vector2.h"

namespace Nyon::ECS
{
    /**
     * @brief Enhanced physics body component inspired by Box2D.
     * 
     * Contains all physical properties needed for rigid body dynamics simulation.
     * Supports both 2D translational and rotational physics.
     * Uses y-positive-down coordinate system consistent with rendering.
     */
    struct PhysicsBodyComponent
    {
        // === TRANSLATIONAL PHYSICS ===
        Math::Vector2 velocity = {0.0f, 0.0f};      // Linear velocity in pixels/second
        Math::Vector2 acceleration = {0.0f, 0.0f};  // Linear acceleration in pixels/second^2
        Math::Vector2 force = {0.0f, 0.0f};         // Accumulated forces for this frame
        
        // === ROTATIONAL PHYSICS ===
        float angularVelocity = 0.0f;               // Angular velocity in radians/second
        float angularAcceleration = 0.0f;           // Angular acceleration in radians/second^2
        float torque = 0.0f;                        // Accumulated torques for this frame
        
        // === MASS PROPERTIES ===
        float mass = 1.0f;                          // Mass of the body (0 = infinite mass/static)
        float inverseMass = 1.0f;                   // 1/mass (cached for performance)
        float inertia = 1.0f;                       // Moment of inertia
        float inverseInertia = 1.0f;                // 1/inertia (cached for performance)
        Math::Vector2 centerOfMass = {0.0f, 0.0f};  // Local center of mass
        
        // === MATERIAL PROPERTIES ===
        float friction = 0.1f;                      // Friction coefficient (0-1)
        float restitution = 0.0f;                   // Restitution/bounciness (0-1)
        float angularDamping = 0.0f;                // Angular velocity decay (0-1)
        
        // === MOVEMENT PROPERTIES ===
        float drag = 0.0f;                          // Air resistance coefficient (physically correct v² model)
        
        // === CONSTRAINTS ===
        float maxLinearSpeed = 1000.0f;             // Maximum linear speed limit
        float maxAngularSpeed = 100.0f;             // Maximum angular speed limit (radians/sec)
        
        // === BODY TYPE FLAGS ===
        bool isStatic = false;                      // Immovable body (infinite mass)
        bool isKinematic = false;                   // Controlled by user, affects dynamic bodies
        bool isBullet = false;                      // Enable continuous collision detection
        
        // === SLEEP MECHANISM ===
        bool isAwake = true;                        // Active simulation state
        bool allowSleep = true;                     // Whether body can fall asleep
        float sleepTimer = 0.0f;                    // Time accumulated while stationary
        static constexpr float SLEEP_THRESHOLD = 0.5f; // Seconds of inactivity to sleep
        static constexpr float LINEAR_SLEEP_TOLERANCE = 0.01f; // Velocity threshold for sleep
        static constexpr float ANGULAR_SLEEP_TOLERANCE = 0.01f; // Angular vel threshold for sleep
        
        // === BACKWARD COMPATIBILITY ===
        bool isGrounded = false;                    // Legacy grounded state
        int groundedFrames = 0;                     // Legacy frame counter
        static constexpr int GROUNDED_THRESHOLD = 2; // Legacy threshold
        
        // === MOTION LOCKS ===
        struct MotionLocks
        {
            bool lockTranslationX = false;
            bool lockTranslationY = false;
            bool lockRotation = false;
        } motionLocks;
        
        PhysicsBodyComponent() = default;
        PhysicsBodyComponent(float m) : mass(m) { UpdateMassProperties(); }
        PhysicsBodyComponent(float m, bool stat) : mass(m), isStatic(stat) { UpdateMassProperties(); }
        
        // === MASS PROPERTY MANAGEMENT ===
        void SetMass(float newMass)
        {
            mass = newMass;
            UpdateMassProperties();
        }
        
        // For now this only handles body type flags and scalar mass.
        // Shape-specific inertia is supplied when the collider is initialized.
        void UpdateMassProperties()
        {
            // Static and kinematic bodies have infinite mass in the solver
            if (isStatic || isKinematic || mass <= 0.0f)
            {
                mass = 0.0f;
                inverseMass = 0.0f;
                inertia = 0.0f;
                inverseInertia = 0.0f;
            }
            else
            {
                inverseMass = 1.0f / mass;
                // inertia is expected to be set from shape info; keep any existing value
                if (inertia > 0.0f)
                {
                    inverseInertia = 1.0f / inertia;
                }
                else
                {
                    // Fallback: treat as point mass (no rotation)
                    inertia = 0.0f;
                    inverseInertia = 0.0f;
                }
            }
        }
        
        // === SLEEP MANAGEMENT ===
        void SetAwake(bool awake)
        {
            if (isStatic) return;
            
            isAwake = awake;
            if (awake)
            {
                sleepTimer = 0.0f;
            }
        }
        
        void AllowSleep(bool enable)
        {
            allowSleep = enable;
            if (!enable)
            {
                SetAwake(true);
            }
        }
        
        // === FORCE AND TORQUE APPLICATION ===
        void ApplyForce(const Math::Vector2& forceVec)
        {
            if (isStatic) return;
            force = force + forceVec;
            SetAwake(true);
        }
        
        void ApplyForceAtPoint(const Math::Vector2& forceVec, const Math::Vector2& point)
        {
            if (isStatic) return;
            
            force = force + forceVec;
            
            // Calculate torque from force applied at offset point
            Math::Vector2 offset = point - centerOfMass;
            torque += offset.x * forceVec.y - offset.y * forceVec.x;
            
            SetAwake(true);
        }
        
        void ApplyTorque(float torqueAmount)
        {
            if (isStatic) return;
            torque += torqueAmount;
            SetAwake(true);
        }
        
        void ApplyLinearImpulse(const Math::Vector2& impulse)
        {
            if (isStatic) return;
            velocity = velocity + impulse * inverseMass;
            SetAwake(true);
        }
        
        void ApplyAngularImpulse(float impulse)
        {
            if (isStatic) return;
            angularVelocity += impulse * inverseInertia;
            SetAwake(true);
        }
        
        // === CLEAR FORCES ===
        void ClearForces()
        {
            force = {0.0f, 0.0f};
            torque = 0.0f;
        }
        
        // === BACKWARD COMPATIBILITY METHODS ===
        bool IsStablyGrounded() const { return groundedFrames >= GROUNDED_THRESHOLD; }
        
        void UpdateGroundedState(bool currentlyGrounded)
        {
            if (currentlyGrounded) {
                groundedFrames++;
            } else {
                groundedFrames = 0;
            }
            isGrounded = IsStablyGrounded();
        }
        
        // === UTILITY METHODS ===
        bool IsStatic() const { return isStatic; }
        bool IsKinematic() const { return isKinematic; }
        bool IsDynamic() const { return !isStatic && !isKinematic; }
        bool ShouldCollide() const { return isAwake || isKinematic; }
        float GetMass() const { return mass; }
        float GetInertia() const { return inertia; }
    };
}
