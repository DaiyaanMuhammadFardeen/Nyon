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
     * - Applies gravity to dynamic bodies
     * - Integrates forces to update velocities
     * - Integrates velocities to update positions
     * - Handles damping and constraints
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
            // Apply gravity (F = m * g, so a = g)
            Math::Vector2 acceleration = gravity;
            
            // Add force contribution (F/m)
            if (body.inverseMass > 0.0f)
            {
                acceleration = acceleration + body.force * body.inverseMass;
            }
            
            // Apply linear damping (air resistance)
            if (body.linearDamping > 0.0f)
            {
                acceleration = acceleration - body.velocity * body.linearDamping;
            }
            
            // Apply drag
            if (body.drag > 0.0f && body.velocity.LengthSquared() > 0.0f)
            {
                Math::Vector2 dragForce = body.velocity.Normalize() * (-body.drag * body.velocity.LengthSquared());
                acceleration = acceleration + dragForce * body.inverseMass;
            }
            
            // Update linear velocity: v = v0 + a * dt
            body.velocity = body.velocity + acceleration * deltaTime;
            
            // Clamp to maximum speed
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
            
            // Update rotation: θ = θ0 + ω * dt
            transform.rotation += body.angularVelocity * deltaTime;
            
            // Keep rotation in reasonable bounds
            if (transform.rotation > 6.28318530718f) // 2π
                transform.rotation -= 6.28318530718f;
            else if (transform.rotation < -6.28318530718f)
                transform.rotation += 6.28318530718f;
        }
        
        /**
         * @brief Update body sleep state based on activity level
         * Following Box2D's sleep mechanism
         */
        void UpdateSleepState(PhysicsBodyComponent& body, float deltaTime)
        {
            if (!body.allowSleep) return;
            
            // Check if body is moving slowly enough to sleep
            float linearSpeedSq = body.velocity.LengthSquared();
            float angularSpeed = fabs(body.angularVelocity);
            
            if (linearSpeedSq > body.LINEAR_SLEEP_TOLERANCE * body.LINEAR_SLEEP_TOLERANCE ||
                angularSpeed > body.ANGULAR_SLEEP_TOLERANCE)
            {
                // Body is still active
                body.sleepTimer = 0.0f;
            }
            else
            {
                // Body is nearly stationary, accumulate sleep time
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
    };
}