#pragma once

#include "nyon/ecs/System.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/utils/CollisionPhysics.h"
#include <vector>

namespace Nyon::ECS
{
    /**
     * @brief Collision detection and resolution system.
     * 
     * Handles collision detection and response for all collidable entities.
     * Uses frame-stable grounded detection to prevent flickering.
     */
    class CollisionSystem : public System
    {
    public:
        void Update(float deltaTime) override
        {
            if (!m_EntityManager || !m_ComponentStore) return;
            
            // Reset grounded frame counters for all dynamic bodies
            ResetGroundedCounters();
            
            // Process all collisions
            ProcessCollisions(deltaTime);
            
            // Apply boundary constraints
            ApplyBoundaryConstraints();
            
            // Update final grounded states
            UpdateFinalGroundedStates();
        }
        
    private:
        void ResetGroundedCounters()
        {
            const auto& physicsEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsBodyComponent>();
            for (EntityID entity : physicsEntities)
            {
                if (!m_ComponentStore->HasComponent<PhysicsBodyComponent>(entity)) continue;
                auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entity);
                if (!body.isStatic) {
                    body.groundedFrames = 0; // Reset counter, will be incremented if grounded this frame
                }
            }
        }
        
        void ProcessCollisions(float deltaTime)
        {
            const auto& physicsEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsBodyComponent>();
            
            for (EntityID entity : physicsEntities)
            {
                if (!m_ComponentStore->HasComponent<TransformComponent>(entity) ||
                    !m_ComponentStore->HasComponent<ColliderComponent>(entity))
                    continue;
                
                auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entity);
                if (body.isStatic) continue;
                
                auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entity);
                auto& collider = m_ComponentStore->GetComponent<ColliderComponent>(entity);
                
                // Check collisions with all other entities
                const auto& allEntities = m_ComponentStore->GetEntitiesWithComponent<ColliderComponent>();
                
                for (EntityID otherEntity : allEntities)
                {
                    if (otherEntity == entity) continue;
                    if (!m_ComponentStore->HasComponent<TransformComponent>(otherEntity)) continue;
                    if (!m_ComponentStore->HasComponent<PhysicsBodyComponent>(otherEntity)) continue;
                    
                    auto& otherBody = m_ComponentStore->GetComponent<PhysicsBodyComponent>(otherEntity);
                    auto& otherTransform = m_ComponentStore->GetComponent<TransformComponent>(otherEntity);
                    auto& otherCollider = m_ComponentStore->GetComponent<ColliderComponent>(otherEntity);
                    
                    // Broad-phase AABB check
                    Math::Vector2 minA, maxA, minB, maxB;
                    collider.CalculateAABB(transform.position, minA, maxA);
                    otherCollider.CalculateAABB(otherTransform.position, minB, maxB);
                    
                    Math::Vector2 sizeA = maxA - minA;
                    Math::Vector2 sizeB = maxB - minB;
                    
                    if (Utils::CollisionPhysics::CheckAABBCollision(minA, sizeA, minB, sizeB))
                    {
                        // Narrow-phase collision detection
                        auto collisionResult = Utils::CollisionPhysics::CheckPolygonCollision(
                            collider.GetPolygon(),
                            transform.position,
                            otherCollider.GetPolygon(),
                            otherTransform.position
                        );
                        
                        if (collisionResult.collided)
                        {
                            // Resolve collision
                            ResolveCollision(entity, otherEntity, collisionResult, deltaTime);
                            
                            // Check if this collision contributes to grounded state
                            CheckGroundedContribution(entity, otherEntity, collisionResult);
                            
                            // Notify behavior components
                            NotifyCollision(entity, otherEntity);
                            NotifyCollision(otherEntity, entity);
                        }
                    }
                }
            }
        }
        
        void ResolveCollision(EntityID entityA, EntityID entityB, 
                            const Utils::CollisionPhysics::CollisionResult& collision,
                            float deltaTime)
        {
            // Get physics bodies
            auto& bodyA = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityA);
            auto& bodyB = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityB);
            
            // Use existing collision resolution
            Utils::Physics::Body tempBodyA, tempBodyB;
            tempBodyA.position = m_ComponentStore->GetComponent<TransformComponent>(entityA).position;
            tempBodyA.velocity = bodyA.velocity;
            tempBodyA.mass = bodyA.mass;
            tempBodyA.isStatic = bodyA.isStatic;
            
            tempBodyB.position = m_ComponentStore->GetComponent<TransformComponent>(entityB).position;
            tempBodyB.velocity = bodyB.velocity;
            tempBodyB.mass = bodyB.mass;
            tempBodyB.isStatic = bodyB.isStatic;
            
            Utils::CollisionPhysics::ResolveCollision(tempBodyA, tempBodyB, collision);
            
            // Update back to ECS components
            if (!bodyA.isStatic) {
                bodyA.velocity = tempBodyA.velocity;
                m_ComponentStore->GetComponent<TransformComponent>(entityA).position = tempBodyA.position;
            }
            
            if (!bodyB.isStatic) {
                bodyB.velocity = tempBodyB.velocity;
                m_ComponentStore->GetComponent<TransformComponent>(entityB).position = tempBodyB.position;
            }
        }
        
        void CheckGroundedContribution(EntityID entity, EntityID otherEntity,
                                     const Utils::CollisionPhysics::CollisionResult& collision)
        {
            auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entity);
            auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entity);
            auto& otherTransform = m_ComponentStore->GetComponent<TransformComponent>(otherEntity);
            auto& otherBody = m_ComponentStore->GetComponent<PhysicsBodyComponent>(otherEntity);
            
            // Check if other entity is below this one (potential ground)
            if (otherTransform.position.y > transform.position.y + 10.0f)
            {
                // Check collision normal - if it's pushing us upward, it's ground
                const float GROUND_THRESHOLD = -0.3f; // Relatively vertical normal
                if (collision.overlapAxis.y < GROUND_THRESHOLD)
                {
                    body.groundedFrames++; // Increment frame counter
                }
            }
        }
        
        void ApplyBoundaryConstraints()
        {
            const auto& physicsEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsBodyComponent>();
            
            for (EntityID entity : physicsEntities)
            {
                if (!m_ComponentStore->HasComponent<TransformComponent>(entity)) continue;
                
                auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entity);
                if (body.isStatic) continue;
                
                auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entity);
                
                // Screen boundaries
                const float SCREEN_WIDTH = 1280.0f;
                const float SCREEN_HEIGHT = 720.0f;
                const float PLAYER_SIZE = 32.0f;
                
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
                
                // Bottom boundary (ground)
                if (transform.position.y > SCREEN_HEIGHT - PLAYER_SIZE) {
                    transform.position.y = SCREEN_HEIGHT - PLAYER_SIZE;
                    body.velocity.y = std::min(0.0f, body.velocity.y);
                    body.groundedFrames++; // Contribute to grounded state
                }
                
                // Top boundary
                if (transform.position.y < 0) {
                    transform.position.y = 0;
                    body.velocity.y = std::abs(body.velocity.y) * 0.5f;
                }
            }
        }
        
        void UpdateFinalGroundedStates()
        {
            const auto& physicsEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsBodyComponent>();
            for (EntityID entity : physicsEntities)
            {
                if (!m_ComponentStore->HasComponent<PhysicsBodyComponent>(entity)) continue;
                auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entity);
                if (!body.isStatic) {
                    body.UpdateGroundedState(body.groundedFrames > 0);
                }
            }
        }
        
        void NotifyCollision(EntityID entityA, EntityID entityB)
        {
            if (m_ComponentStore->HasComponent<BehaviorComponent>(entityA))
            {
                auto& behavior = m_ComponentStore->GetComponent<BehaviorComponent>(entityA);
                behavior.OnCollision(entityA, entityB);
            }
        }
    };
}