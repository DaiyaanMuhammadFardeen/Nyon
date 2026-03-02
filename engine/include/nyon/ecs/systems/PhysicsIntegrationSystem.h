#pragma once

#include "nyon/ecs/System.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"

namespace Nyon::ECS
{
    /**
     * @brief Physics integration system that handles falling body physics simulation.
     * 
     * Implements velocity and position integration following Box2D's approach:
     * - Applies gravity only to non-grounded dynamic bodies
     * - Integrates forces to update velocities
     * - Integrates velocities to update positions
     * - Handles damping and constraints
     * - Uses contact-aware sleep system to prevent perpetual sinking
     * 
     * Gravity is selectively applied to prevent perpetual sinking of grounded objects.
     * Sleep system considers contact support to ensure stable resting objects can sleep.
     */
    class PhysicsIntegrationSystem : public System
    {
    public:
        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override
        {
            System::Initialize(entityManager, componentStore);
        }
        
        void Update(float deltaTime) override
        {
            if (!m_EntityManager || !m_ComponentStore) return;
            
            // Get physics world component for gravity and settings
            const auto& worldEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsWorldComponent>();
            if (worldEntities.empty()) return;
            
            const auto& world = m_ComponentStore->GetComponent<PhysicsWorldComponent>(worldEntities[0]);
            
            // Get all physics bodies
            const auto& physicsEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsBodyComponent>();
            
            // Prepare transform components for interpolation by storing previous state
            for (EntityID entity : physicsEntities)
            {
                if (m_ComponentStore->HasComponent<TransformComponent>(entity))
                {
                    auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entity);
                    transform.PrepareForUpdate();
                }
            }
            
            // First pass: Apply forces and integrate velocities
            for (EntityID entity : physicsEntities)
            {
                if (!m_ComponentStore->HasComponent<TransformComponent>(entity)) continue;
                
                auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entity);
                auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entity);
                
                // Skip static and sleeping bodies
                if (body.isStatic || !body.isAwake) continue;
                
                IntegrateVelocity(body, transform, world.gravity, deltaTime);
            }
            
            // Second pass: Integrate positions
            for (EntityID entity : physicsEntities)
            {
                if (!m_ComponentStore->HasComponent<TransformComponent>(entity)) continue;
                
                auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entity);
                auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entity);
                
                // Skip static and sleeping bodies
                if (body.isStatic || !body.isAwake) continue;
                
                IntegratePosition(body, transform, deltaTime);
            }
        }
        
        void Shutdown() override
        {
            // Cleanup if needed
        }
        
    private:
        /**
         * @brief Integrate velocity using explicit Euler integration
         * Following Box2D's approach: v = v0 + (F/m) * dt
         */
        void IntegrateVelocity(PhysicsBodyComponent& body, TransformComponent& transform,
                              const Math::Vector2& gravity, float deltaTime)
        {
            // Apply gravity only to non-grounded bodies
            // This prevents perpetual sinking of grounded objects
            Math::Vector2 acceleration = {0.0f, 0.0f};
            
            // Only apply gravity if body is not stably grounded
            if (!body.IsStablyGrounded())
            {
                acceleration = gravity;
            }
            
            // Add force contribution (F/m)
            if (body.inverseMass > 0.0f)
            {
                acceleration = acceleration + body.force * body.inverseMass;
            }
            
            // Apply aerodynamic drag (physically correct: F = -drag * v², a = -drag * v²/mass)
            // Using proper physics: drag force = -drag_coefficient * v² * v_direction
            // drag_coefficient has units of mass/length (making force have correct units)
            if (body.drag > 0.0f && body.velocity.LengthSquared() > 0.0f)
            {
                // Physically correct drag force calculation
                float speed = body.velocity.Length();
                Math::Vector2 dragDirection = body.velocity.Normalize();
                float dragForceMagnitude = body.drag * speed * speed; // F = c * v²
                Math::Vector2 dragForce = dragDirection * (-dragForceMagnitude);
                
                // Convert force to acceleration: a = F/m
                Math::Vector2 dragAcceleration = dragForce * body.inverseMass;
                acceleration = acceleration + dragAcceleration;
            }
            
            // Update linear velocity: v = v0 + a * dt
            body.velocity = body.velocity + acceleration * deltaTime;
            
            // Clamp to maximum linear speed
            float speedSq = body.velocity.LengthSquared();
            if (speedSq > body.maxLinearSpeed * body.maxLinearSpeed)
            {
                body.velocity = body.velocity.Normalize() * body.maxLinearSpeed;
            }
            
            // Update angular velocity: ω = ω0 + (τ/I) * dt
            float angularAcceleration = body.torque * body.inverseInertia;
            
            // Apply angular damping
            if (body.angularDamping > 0.0f)
            {
                angularAcceleration -= body.angularVelocity * body.angularDamping;
            }
            
            body.angularVelocity += angularAcceleration * deltaTime;
            
            // Clamp angular velocity
            if (body.angularVelocity > body.maxAngularSpeed)
                body.angularVelocity = body.maxAngularSpeed;
            else if (body.angularVelocity < -body.maxAngularSpeed)
                body.angularVelocity = -body.maxAngularSpeed;
            
            // Clear accumulated forces for next frame
            body.ClearForces();
            
            // Update sleep state based on velocity
            UpdateSleepState(body, deltaTime);
        }
        
        /**
         * @brief Integrate position using updated velocities
         * Following Box2D's approach: x = x0 + v * dt
         */
        void IntegratePosition(PhysicsBodyComponent& body, TransformComponent& transform, float deltaTime)
        {
            // Update position: x = x0 + v * dt
            transform.position = transform.position + body.velocity * deltaTime;
            
            // Apply boundary constraints to keep objects within screen bounds
            ApplyBoundaryConstraints(body, transform);
            
            // Update rotation: θ = θ0 + ω * dt
            transform.rotation += body.angularVelocity * deltaTime;
            
            // Keep rotation in reasonable bounds
            if (transform.rotation > 6.28318530718f) // 2π
                transform.rotation -= 6.28318530718f;
            else if (transform.rotation < -6.28318530718f)
                transform.rotation += 6.28318530718f;
        }
        
        /**
         * @brief Update body sleep state based on activity level and contact support
         * Following Box2D's contact-aware sleep mechanism
         */
        void UpdateSleepState(PhysicsBodyComponent& body, float deltaTime)
        {
            if (!body.allowSleep) return;
            
            // Check if body is moving slowly enough to sleep
            float linearSpeedSq = body.velocity.LengthSquared();
            float angularSpeed = fabs(body.angularVelocity);
            
            // Additional check: verify body is properly supported by contacts
            // that oppose gravity before allowing sleep
            bool isProperlySupported = IsBodyProperlySupported(body);
            
            if (linearSpeedSq > body.LINEAR_SLEEP_TOLERANCE * body.LINEAR_SLEEP_TOLERANCE ||
                angularSpeed > body.ANGULAR_SLEEP_TOLERANCE ||
                !isProperlySupported)
            {
                // Body is still active or not properly supported
                body.sleepTimer = 0.0f;
            }
            else
            {
                // Body is nearly stationary and properly supported, accumulate sleep time
                body.sleepTimer += deltaTime;
                
                // Put to sleep if inactive long enough
                if (body.sleepTimer >= body.SLEEP_THRESHOLD)
                {
                    body.SetAwake(false);
                    body.velocity = {0.0f, 0.0f};
                    body.angularVelocity = 0.0f;
                }
            }
        }
        
        /**
         * @brief Apply screen boundary constraints to prevent objects from leaving play area
         * 
         * Enforces screen boundaries similar to the original CollisionSystem implementation.
         * Also contributes to grounded state when objects hit bottom boundary.
         */
        void ApplyBoundaryConstraints(PhysicsBodyComponent& body, TransformComponent& transform)
        {
            // Screen boundaries (matches original CollisionSystem values)
            const float SCREEN_WIDTH = 1280.0f;
            const float SCREEN_HEIGHT = 720.0f;
            const float PLAYER_SIZE = 32.0f; // Approximate size for boundary calculations
            
            // Left boundary
            if (transform.position.x < 0) {
                transform.position.x = 0;
                body.velocity.x = std::max(0.0f, body.velocity.x);
            }
            
            // Right boundary
            if (transform.position.x > SCREEN_WIDTH - PLAYER_SIZE) {
                transform.position.x = SCREEN_WIDTH - PLAYER_SIZE;
                body.velocity.x = std::min(0.0f, body.velocity.x);
            }
            
            // Bottom boundary (ground) - also contributes to grounded state
            if (transform.position.y > SCREEN_HEIGHT - PLAYER_SIZE) {
                transform.position.y = SCREEN_HEIGHT - PLAYER_SIZE;
                body.velocity.y = std::min(0.0f, body.velocity.y);
                // Contribute to grounded frames (similar to CollisionSystem)
                body.groundedFrames++;
            }
            
            // Top boundary
            if (transform.position.y < 0) {
                transform.position.y = 0;
                body.velocity.y = std::abs(body.velocity.y) * 0.5f; // Bounce with energy loss
            }
        }
        
        /**
         * @brief Check if body is properly supported by contacts opposing gravity
         * 
         * A body is considered properly supported if all active contacts have
         * normals that oppose the gravity direction (dot product < 0).
         */
        bool IsBodyProperlySupported(const PhysicsBodyComponent& body) const
        {
            // For now, use the existing grounded state as a proxy
            // In a full implementation, this would check actual contact normals
            // against the gravity vector
            return body.IsStablyGrounded();
        }
    };
}