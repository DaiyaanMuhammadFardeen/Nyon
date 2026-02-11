#pragma once

#include "nyon/ecs/System.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/utils/GravityPhysics.h"
#include "nyon/utils/MovementPhysics.h"
#include "nyon/utils/Physics.h"

namespace Nyon::ECS
{
    /**
     * @brief Physics system that updates all entities with PhysicsBodyComponent.
     * 
     * Integrates forces, applies gravity, handles friction and drag.
     * Works with both static and dynamic bodies.
     */
    class PhysicsSystem : public System
    {
    public:
        void Update(float deltaTime) override
        {
            if (!m_EntityManager || !m_ComponentStore) return;
            
            // Get all entities with physics bodies
            const auto& entities = m_ComponentStore->GetEntitiesWithComponent<PhysicsBodyComponent>();
            
            for (EntityID entity : entities)
            {
                auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entity);
                
                // Skip static bodies - they don't move
                if (body.isStatic) continue;
                
                // Get transform for position updates
                if (m_ComponentStore->HasComponent<TransformComponent>(entity))
                {
                    auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entity);
                    
                    // Convert ECS PhysicsBodyComponent to Utils::Physics::Body for processing
                    Utils::Physics::Body physicsBody;
                    physicsBody.position = transform.position;
                    physicsBody.velocity = body.velocity;
                    physicsBody.acceleration = body.acceleration;
                    physicsBody.mass = body.mass;
                    physicsBody.friction = body.friction;
                    physicsBody.drag = body.drag;
                    physicsBody.maxSpeed = body.maxSpeed;
                    physicsBody.isStatic = body.isStatic;
                    
                    // Update physics with current grounded state
                    Utils::GravityPhysics::UpdateBody(physicsBody, deltaTime, body.isGrounded);
                    
                    // Update ECS components from processed physics body
                    body.velocity = physicsBody.velocity;
                    body.acceleration = physicsBody.acceleration;
                    transform.position = physicsBody.position;
                }
                else
                {
                    // Fallback: update body directly if no transform component
                    Utils::Physics::Body physicsBody;
                    physicsBody.velocity = body.velocity;
                    physicsBody.acceleration = body.acceleration;
                    physicsBody.mass = body.mass;
                    physicsBody.friction = body.friction;
                    physicsBody.drag = body.drag;
                    physicsBody.maxSpeed = body.maxSpeed;
                    physicsBody.isStatic = body.isStatic;
                    
                    Utils::GravityPhysics::UpdateBody(physicsBody, deltaTime, body.isGrounded);
                    
                    body.velocity = physicsBody.velocity;
                    body.acceleration = physicsBody.acceleration;
                }
                
                // Reset acceleration for next frame
                body.acceleration = {0.0f, 0.0f};
            }
        }
    };
}