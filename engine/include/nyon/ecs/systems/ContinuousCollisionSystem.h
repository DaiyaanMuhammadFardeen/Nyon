#pragma once

#include "nyon/ecs/System.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/BehaviorComponent.h"
#include "nyon/physics/CollisionDetectionStrategy.h"
#include <vector>
#include <iostream>

namespace Nyon::ECS
{
    /**
     * @brief Enhanced Continuous Collision Detection system with Box2D-inspired algorithms.
     * 
     * Implements swept collision detection to prevent tunneling through
     * thin obstacles and fast-moving objects.
     */
    class ContinuousCollisionSystem : public System
    {
    public:
        struct SweepResult
        {
            bool collided = false;
            float timeOfImpact = -1.0f;
            Math::Vector2 contactPoint;
            Math::Vector2 normal;
            uint32_t entityIdA = 0;
            uint32_t entityIdB = 0;
        };

        void Update(float deltaTime) override
        {
            if (!m_ComponentStore)
                return;

            // Get all physics bodies
            const auto& bodyEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsBodyComponent>();
            
            // Check each pair for potential CCD
            for (size_t i = 0; i < bodyEntities.size(); ++i)
            {
                for (size_t j = i + 1; j < bodyEntities.size(); ++j)
                {
                    EntityID entityA = bodyEntities[i];
                    EntityID entityB = bodyEntities[j];
                    
                    if (!m_ComponentStore->HasComponent<TransformComponent>(entityA) ||
                        !m_ComponentStore->HasComponent<TransformComponent>(entityB) ||
                        !m_ComponentStore->HasComponent<ColliderComponent>(entityA) ||
                        !m_ComponentStore->HasComponent<ColliderComponent>(entityB))
                    {
                        continue;
                    }
                    
                    auto& bodyA = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityA);
                    auto& bodyB = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityB);
                    auto& transformA = m_ComponentStore->GetComponent<TransformComponent>(entityA);
                    auto& transformB = m_ComponentStore->GetComponent<TransformComponent>(entityB);
                    auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(entityA);
                    auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(entityB);
                    
                    // Skip static-static pairs
                    if (bodyA.isStatic && bodyB.isStatic)
                    {
                        continue;
                    }
                    
                    // Determine if CCD is needed
                    Physics::CollisionDetectionStrategy::DetectionType strategy = 
                        Physics::CollisionDetectionStrategy::DetermineStrategy(bodyA, bodyB, deltaTime);
                    
                    if (strategy == Physics::CollisionDetectionStrategy::DetectionType::CCD_REQUIRED)
                    {
                        PerformCCD(transformA, transformB, bodyA, bodyB, 
                                 colliderA, colliderB, entityA, entityB, deltaTime);
                    }
                }
            }
        }

    private:
        void PerformCCD(
            TransformComponent& transformA, TransformComponent& transformB,
            PhysicsBodyComponent& bodyA, PhysicsBodyComponent& bodyB,
            ColliderComponent& colliderA, ColliderComponent& colliderB,
            EntityID entityA, EntityID entityB, float deltaTime);
        
        /**
         * @brief Resolves CCD impulse using consistent geometry.
         * @deprecated Use ResolveCCDCollision instead for unified approach.
         */
        void ResolveCCDImpulse(
            TransformComponent& transformA, TransformComponent& transformB,
            PhysicsBodyComponent& bodyA, PhysicsBodyComponent& bodyB,
            EntityID entityA, EntityID entityB);
        
        /**
         * @brief Applies positional correction for CCD.
         * @deprecated Use ResolveCCDCollision instead for unified approach.
         */
        void ResolveCCDPositionalCorrection(
            TransformComponent& transformA, TransformComponent& transformB,
            PhysicsBodyComponent& bodyA, PhysicsBodyComponent& bodyB,
            EntityID entityA, EntityID entityB);
        
        /**
         * @brief Unified CCD collision resolution with consistent geometry and standardized normal convention.
         * 
         * This method provides a coherent approach to CCD by:
         * 1. Using swept AABB TOI calculation for consistent timing
         * 2. Calculating collision normal at exact TOI moment with standardized convention (FROM B TO A)
         * 3. Applying impulse and positional correction in single pass
         * 4. Maintaining geometric consistency between detection and resolution
         * 
         * Fixes CB-02: Inconsistent geometry between TOI (sphere model) and resolution (AABB model)
         * Fixes CB-03: Inconsistent normal convention between positional correction and impulse resolution
         * Fixes CB-04: Unconditional impulse application and incorrect penetration evaluation timing
         * Fixes CB-05: Velocity clamping in collision resolver destroying physics realism
         * Fixes CB-06: Inconsistent ground contact detection using wrong normal direction
         * Fixes CB-07: Dead code elimination - ResolveCCDCollision is now the active collision resolution path
         */
        void ResolveCCDCollision(
            TransformComponent& transformA, TransformComponent& transformB,
            PhysicsBodyComponent& bodyA, PhysicsBodyComponent& bodyB,
            const Math::Vector2& startPosA, const Math::Vector2& startPosB,
            const Math::Vector2& endPosA, const Math::Vector2& endPosB,
            float timeOfImpact,
            EntityID entityA, EntityID entityB);
        
        void NotifyCollisionEvent(EntityID entityA, EntityID entityB);
    };
}