#pragma once
#include "nyon/ecs/System.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
namespace Nyon::ECS {
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
    class PhysicsIntegrationSystem : public System {
    public:
        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override {
            System::Initialize(entityManager, componentStore);
        }
        void Update(float deltaTime) override {
            if (!m_EntityManager || !m_ComponentStore) return;
            const auto& worldEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsWorldComponent>();
            if (worldEntities.empty()) return;
            const auto& world = m_ComponentStore->GetComponent<PhysicsWorldComponent>(worldEntities[0]);
            const auto& physicsEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsBodyComponent>();
            for (EntityID entity : physicsEntities)
            {
                auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entity);
                body.UpdateGroundedState(false);
                if (m_ComponentStore->HasComponent<TransformComponent>(entity))
                {
                    auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entity);
                    transform.PrepareForUpdate();
                }
            }
            for (EntityID entity : physicsEntities)
            {
                if (!m_ComponentStore->HasComponent<TransformComponent>(entity)) continue;
                auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entity);
                auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entity);
                if (body.isStatic || !body.isAwake) continue;
                IntegrateVelocity(body, transform, world.gravity, deltaTime);
            }
            for (EntityID entity : physicsEntities)
            {
                if (!m_ComponentStore->HasComponent<TransformComponent>(entity)) continue;
                auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entity);
                auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entity);
                if (body.isStatic || !body.isAwake) continue;
                IntegratePosition(body, transform, deltaTime);
            }
        }
        void Shutdown() override {
        }
    private:
        /**
         * @brief Integrate velocity using explicit Euler integration
         * Following Box2D's approach: v = v0 + (F/m) * dt
         */
        void IntegrateVelocity(PhysicsBodyComponent& body, TransformComponent& transform,
                              const Math::Vector2& gravity, float deltaTime)
        {
            Math::Vector2 acceleration = gravity;
            if (body.inverseMass > 0.0f)
            {
                acceleration = acceleration + body.force * body.inverseMass;
            }
            if (body.drag > 0.0f && body.velocity.LengthSquared() > 0.0f)
            {
                float speed = body.velocity.Length();
                Math::Vector2 dragDirection = body.velocity.Normalize();
                float dragForceMagnitude = body.drag * speed * speed;  
                Math::Vector2 dragForce = dragDirection * (-dragForceMagnitude);
                Math::Vector2 dragAcceleration = dragForce * body.inverseMass;
                acceleration = acceleration + dragAcceleration;
            }
            Math::Vector2 oldVelocity = body.velocity;
            body.velocity = body.velocity + acceleration * deltaTime;
            float speedSq = body.velocity.LengthSquared();
            if (speedSq > body.maxLinearSpeed * body.maxLinearSpeed)
            {
                body.velocity = body.velocity.Normalize() * body.maxLinearSpeed;
            }
            float angularAcceleration = body.torque * body.inverseInertia;
            if (body.angularDamping > 0.0f)
            {
                angularAcceleration -= body.angularVelocity * body.angularDamping;
            }
            body.angularVelocity += angularAcceleration * deltaTime;
            if (body.angularVelocity > body.maxAngularSpeed)
                body.angularVelocity = body.maxAngularSpeed;
            else if (body.angularVelocity < -body.maxAngularSpeed)
                body.angularVelocity = -body.maxAngularSpeed;
            body.ClearForces();
            UpdateSleepState(body, deltaTime);
        }
        /**
         * @brief Integrate position using updated velocities
         * Following Box2D's approach: x = x0 + v * dt
         */
        void IntegratePosition(PhysicsBodyComponent& body, TransformComponent& transform, float deltaTime)
        {
            transform.position = transform.position + body.velocity * deltaTime;
            transform.rotation += body.angularVelocity * deltaTime;
        }
        /**
         * @brief Update body sleep state based on activity level and contact support
         * Following Box2D's contact-aware sleep mechanism
         */
        void UpdateSleepState(PhysicsBodyComponent& body, float deltaTime)
        {
            if (!body.allowSleep) return;
            float linearSpeedSq = body.velocity.LengthSquared();
            float angularSpeed = fabs(body.angularVelocity);
            bool isProperlySupported = IsBodyProperlySupported(body);
            if (linearSpeedSq > body.LINEAR_SLEEP_TOLERANCE * body.LINEAR_SLEEP_TOLERANCE ||
                angularSpeed > body.ANGULAR_SLEEP_TOLERANCE ||
                !isProperlySupported)
            {
                body.sleepTimer = 0.0f;
            }
            else {
                body.sleepTimer += deltaTime;
                if (body.sleepTimer >= body.TIME_TO_SLEEP)
                {
                    body.SetAwake(false);
                    body.velocity = {0.0f, 0.0f};
                    body.angularVelocity = 0.0f;
                }
            }
        }
        /**
         * @brief Check if body is properly supported by contacts opposing gravity
         * 
         * A body is considered properly supported if all active contacts have
         * normals that oppose the gravity direction (dot product < 0).
         */
        bool IsBodyProperlySupported(const PhysicsBodyComponent& body) const {
            return body.IsStablyGrounded();
        }
    };
}