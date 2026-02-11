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
     * Handles broad-phase culling, narrow-phase collision detection,
     * and collision response for all collidable entities.
     */
    class CollisionSystem : public System
    {
    public:
        void Update(float deltaTime) override
        {
            if (!m_EntityManager || !m_ComponentStore) return;
            
            // Get all entities with colliders
            const auto& colliderEntities = m_ComponentStore->GetEntitiesWithComponent<ColliderComponent>();
            
            // Simple broad-phase: check all pairs (TODO: implement spatial partitioning)
            for (size_t i = 0; i < colliderEntities.size(); ++i)
            {
                EntityID entityA = colliderEntities[i];
                if (!m_ComponentStore->HasComponent<TransformComponent>(entityA)) continue;
                
                auto& transformA = m_ComponentStore->GetComponent<TransformComponent>(entityA);
                auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(entityA);
                
                for (size_t j = i + 1; j < colliderEntities.size(); ++j)
                {
                    EntityID entityB = colliderEntities[j];
                    if (!m_ComponentStore->HasComponent<TransformComponent>(entityB)) continue;
                    
                    auto& transformB = m_ComponentStore->GetComponent<TransformComponent>(entityB);
                    auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(entityB);
                    
                    // Broad-phase AABB check
                    Math::Vector2 minA, maxA, minB, maxB;
                    colliderA.CalculateAABB(transformA.position, minA, maxA);
                    colliderB.CalculateAABB(transformB.position, minB, maxB);
                    
                    Math::Vector2 sizeA = maxA - minA;
                    Math::Vector2 sizeB = maxB - minB;
                    
                    if (Utils::CollisionPhysics::CheckAABBCollision(minA, sizeA, minB, sizeB))
                    {
                        // Narrow-phase collision detection
                        auto collisionResult = Utils::CollisionPhysics::CheckPolygonCollision(
                            colliderA.GetPolygon(),
                            transformA.position,
                            colliderB.GetPolygon(),
                            transformB.position
                        );
                        
                        if (collisionResult.collided)
                        {
                            // Resolve collision
                            ResolveCollision(entityA, entityB, collisionResult, deltaTime);
                            
                            // Notify behavior components if they exist
                            NotifyCollision(entityA, entityB);
                            NotifyCollision(entityB, entityA);
                        }
                    }
                }
            }
            
            // Update grounded states
            UpdateGroundedStates(deltaTime);
        }
        
    private:
        void ResolveCollision(EntityID entityA, EntityID entityB, 
                            const Utils::CollisionPhysics::CollisionResult& collision,
                            float deltaTime)
        {
            // Get physics bodies if they exist
            bool hasBodyA = m_ComponentStore->HasComponent<PhysicsBodyComponent>(entityA);
            bool hasBodyB = m_ComponentStore->HasComponent<PhysicsBodyComponent>(entityB);
            
            if (hasBodyA && hasBodyB)
            {
                auto& bodyA = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityA);
                auto& bodyB = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityB);
                
                // Use existing collision resolution (adapted for ECS)
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
            else if (hasBodyA)
            {
                // Only A has physics - resolve against static B
                auto& bodyA = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityA);
                if (!bodyA.isStatic) {
                    auto& transformA = m_ComponentStore->GetComponent<TransformComponent>(entityA);
                    Utils::Physics::Body tempBody;
                    tempBody.position = transformA.position;
                    tempBody.velocity = bodyA.velocity;
                    tempBody.isStatic = false;
                    
                    Utils::CollisionPhysics::ResolveCCDCollision(tempBody, 
                        Utils::CollisionPhysics::CCDResult(true, 0.0f, tempBody.position, collision),
                        deltaTime);
                    
                    bodyA.velocity = tempBody.velocity;
                    transformA.position = tempBody.position;
                }
            }
            else if (hasBodyB)
            {
                // Only B has physics - resolve against static A
                auto& bodyB = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityB);
                if (!bodyB.isStatic) {
                    auto& transformB = m_ComponentStore->GetComponent<TransformComponent>(entityB);
                    Utils::Physics::Body tempBody;
                    tempBody.position = transformB.position;
                    tempBody.velocity = bodyB.velocity;
                    tempBody.isStatic = false;
                    
                    Utils::CollisionPhysics::ResolveCCDCollision(tempBody,
                        Utils::CollisionPhysics::CCDResult(true, 0.0f, tempBody.position, collision),
                        deltaTime);
                    
                    bodyB.velocity = tempBody.velocity;
                    transformB.position = tempBody.position;
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
        
        void UpdateGroundedStates(float deltaTime)
        {
            const auto& physicsEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsBodyComponent>();
            
            for (EntityID entity : physicsEntities)
            {
                if (!m_ComponentStore->HasComponent<TransformComponent>(entity) ||
                    !m_ComponentStore->HasComponent<ColliderComponent>(entity))
                    continue;
                
                auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entity);
                auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entity);
                auto& collider = m_ComponentStore->GetComponent<ColliderComponent>(entity);
                
                // Simple ground check - look for collisions with surfaces below
                body.isGrounded = false;
                
                const auto& allColliders = m_ComponentStore->GetEntitiesWithComponent<ColliderComponent>();
                for (EntityID otherEntity : allColliders)
                {
                    if (otherEntity == entity) continue;
                    if (!m_ComponentStore->HasComponent<TransformComponent>(otherEntity)) continue;
                    
                    auto& otherTransform = m_ComponentStore->GetComponent<TransformComponent>(otherEntity);
                    auto& otherCollider = m_ComponentStore->GetComponent<ColliderComponent>(otherEntity);
                    
                    // Check if other entity is below this one
                    if (otherTransform.position.y > transform.position.y + 10.0f)
                    {
                        // Broad-phase check
                        Math::Vector2 minA, maxA, minB, maxB;
                        collider.CalculateAABB(transform.position, minA, maxA);
                        otherCollider.CalculateAABB(otherTransform.position, minB, maxB);
                        
                        Math::Vector2 sizeA = maxA - minA;
                        Math::Vector2 sizeB = maxB - minB;
                        
                        if (Utils::CollisionPhysics::CheckAABBCollision(minA, sizeA, minB, sizeB))
                        {
                            body.isGrounded = true;
                            break;
                        }
                    }
                }
            }
        }
    };
}