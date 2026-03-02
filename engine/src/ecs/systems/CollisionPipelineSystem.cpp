#include "nyon/ecs/systems/CollisionPipelineSystem.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/components/TransformComponent.h"

namespace Nyon::ECS
{
    void CollisionPipelineSystem::Initialize(EntityManager& entityManager, ComponentStore& componentStore)
    {
        // Store reference to component store for later use
        m_ComponentStore = &componentStore;
        
        // Get physics world component
        const auto& worldEntities = componentStore.GetEntitiesWithComponent<PhysicsWorldComponent>();
        if (!worldEntities.empty())
        {
            m_PhysicsWorld = &componentStore.GetComponent<PhysicsWorldComponent>(worldEntities[0]);
        }
        
        // Get all entities with both physics body and collider components
        const auto& bodyEntities = componentStore.GetEntitiesWithComponent<PhysicsBodyComponent>();
        for (auto entityId : bodyEntities)
        {
            if (componentStore.HasComponent<ColliderComponent>(entityId))
            {
                PhysicsBodyComponent* body = &componentStore.GetComponent<PhysicsBodyComponent>(entityId);
                ColliderComponent* collider = &componentStore.GetComponent<ColliderComponent>(entityId);
                
                if (body && collider && body->ShouldCollide())
                {
                    m_Colliders.emplace_back(entityId, body, collider);
                }
            }
        }
    }
    
    void CollisionPipelineSystem::Update(float deltaTime)
    {
        if (!m_PhysicsWorld)
            return;
            
        // Fixed timestep accumulation
        m_Accumulator += deltaTime;
        
        // Process fixed steps
        while (m_Accumulator >= m_PhysicsWorld->timeStep)
        {
            // Update broad phase
            UpdateBroadPhase();
            
            // Process narrow phase
            ProcessNarrowPhase();
            
            // Update contacts
            UpdateContacts();
            
            m_Accumulator -= m_PhysicsWorld->timeStep;
        }
    }
    
    void CollisionPipelineSystem::UpdateBroadPhase()
    {
        // Update all shape AABBs in the broad phase tree
        for (const auto& [entityId, body, collider] : m_Colliders)
        {
            // Get position from TransformComponent
            Math::Vector2 position = {0.0f, 0.0f};
            if (m_ComponentStore->HasComponent<TransformComponent>(entityId))
            {
                const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
                position = transform.position;
            }
            
            // Update shape AABB in broad phase
            UpdateShapeAABB(entityId, 0, collider, position);
        }
        
        // Handle moved proxies
        for (const auto& [shapeId, proxyId] : m_ShapeProxyMap)
        {
            if (m_BroadPhaseTree.WasMoved(proxyId))
            {
                m_BroadPhaseTree.ClearMoved(proxyId);
                // Mark contacts for re-filtering
            }
        }
    }
    
    void CollisionPipelineSystem::ProcessNarrowPhase()
    {
        m_ActivePairs.clear();
        
        // For each shape, query broad phase for potential collisions
        for (const auto& [entityIdA, bodyA, colliderA] : m_Colliders)
        {
            // Skip static bodies for performance
            if (bodyA->isStatic && !bodyA->isKinematic)
                continue;
                
            // Get current position
            Math::Vector2 positionA = {0.0f, 0.0f};
            if (m_ComponentStore->HasComponent<TransformComponent>(entityIdA))
            {
                const auto& transformA = m_ComponentStore->GetComponent<TransformComponent>(entityIdA);
                positionA = transformA.position;
            }
                
            // Get current AABB
            Math::Vector2 min, max;
            colliderA->CalculateAABB(positionA, min, max);
            Physics::AABB queryAABB(min, max);
            
            // Query broad phase
            BroadPhaseCallback callback;
            callback.system = this;
            callback.entityId = entityIdA;
            callback.shapeId = 0;
            
            m_BroadPhaseTree.Query(queryAABB, &callback);
        }
    }
    
    void CollisionPipelineSystem::UpdateContacts()
    {
        // Process active contact pairs
        for (const auto& pair : m_ActivePairs)
        {
            // Get components for both entities
            if (!m_ComponentStore->HasComponent<TransformComponent>(pair.entityIdA) ||
                !m_ComponentStore->HasComponent<TransformComponent>(pair.entityIdB) ||
                !m_ComponentStore->HasComponent<PhysicsBodyComponent>(pair.entityIdA) ||
                !m_ComponentStore->HasComponent<PhysicsBodyComponent>(pair.entityIdB) ||
                !m_ComponentStore->HasComponent<ColliderComponent>(pair.entityIdA) ||
                !m_ComponentStore->HasComponent<ColliderComponent>(pair.entityIdB))
            {
                continue;
            }
            
            // Get const references for reading
            const auto& transformA = m_ComponentStore->GetComponent<TransformComponent>(pair.entityIdA);
            const auto& transformB = m_ComponentStore->GetComponent<TransformComponent>(pair.entityIdB);
            const auto& bodyA = m_ComponentStore->GetComponent<PhysicsBodyComponent>(pair.entityIdA);
            const auto& bodyB = m_ComponentStore->GetComponent<PhysicsBodyComponent>(pair.entityIdB);
            const auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(pair.entityIdA);
            const auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(pair.entityIdB);
            
            // Simple AABB intersection test for basic collision detection
            Math::Vector2 minA, maxA, minB, maxB;
            colliderA.CalculateAABB(transformA.position, minA, maxA);
            colliderB.CalculateAABB(transformB.position, minB, maxB);
            
            // Check for AABB overlap
            if (minA.x < maxB.x && maxA.x > minB.x && 
                minA.y < maxB.y && maxA.y > minB.y)
            {
                // COLLISION DETECTED - but don't resolve here
                // Let the ContinuousCollisionSystem handle resolution
                // This prevents conflicts between collision systems
                
                // For debugging: print when collisions are detected
                // std::cout << "[COLLISION DETECTED] Between entities " << pair.entityIdA 
                //           << " and " << pair.entityIdB << std::endl;
                
                // Create contact constraint
                ContactPair contactKey{pair.entityIdA, pair.entityIdB, pair.shapeIdA, pair.shapeIdB};
                
                if (m_ContactMap.find(contactKey) == m_ContactMap.end())
                {
                    // New contact - create contact constraint
                    uint32_t contactId = static_cast<uint32_t>(m_ContactMap.size());
                    m_ContactMap[contactKey] = contactId;
                    
                    // Call begin contact callback
                    if (m_PhysicsWorld->callbacks.beginContact)
                    {
                        m_PhysicsWorld->callbacks.beginContact(pair.entityIdA, pair.entityIdB);
                    }
                }
            }
        }
        
        // Destroy inactive contacts
        DestroyInactiveContacts();
    }
    
    void CollisionPipelineSystem::DestroyInactiveContacts()
    {
        // Remove contacts that are no longer active
        auto it = m_ContactMap.begin();
        while (it != m_ContactMap.end())
        {
            const ContactPair& pair = it->first;
            bool isActive = false;
            
            // Check if pair is still in active pairs
            for (const auto& activePair : m_ActivePairs)
            {
                if (activePair.entityIdA == pair.entityIdA && 
                    activePair.entityIdB == pair.entityIdB &&
                    activePair.shapeIdA == pair.shapeIdA && 
                    activePair.shapeIdB == pair.shapeIdB)
                {
                    isActive = true;
                    break;
                }
            }
            
            if (!isActive)
            {
                // Call end contact callback
                if (m_PhysicsWorld->callbacks.endContact)
                {
                    m_PhysicsWorld->callbacks.endContact(pair.entityIdA, pair.entityIdB);
                }
                
                it = m_ContactMap.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }
    
    void CollisionPipelineSystem::UpdateShapeAABB(uint32_t entityId, uint32_t shapeId,
                                                ColliderComponent* collider,
                                                const Math::Vector2& position)
    {
        // Calculate new AABB
        Math::Vector2 min, max;
        collider->CalculateAABB(position, min, max);
        Physics::AABB newAABB(min, max);
        
        auto proxyIt = m_ShapeProxyMap.find(shapeId);
        if (proxyIt != m_ShapeProxyMap.end())
        {
            // Update existing proxy
            uint32_t proxyId = proxyIt->second;
            // For now, use zero displacement - we'll improve this later
            Math::Vector2 displacement = {0.0f, 0.0f};
            m_BroadPhaseTree.MoveProxy(proxyId, newAABB, displacement);
        }
        else
        {
            // Create new proxy
            uint32_t proxyId = m_BroadPhaseTree.CreateProxy(newAABB, entityId);
            m_ShapeProxyMap[shapeId] = proxyId;
        }
    }
    
    bool CollisionPipelineSystem::BroadPhaseCallback::QueryCallback(uint32_t nodeId, uint32_t userData)
    {
        uint32_t otherEntityId = userData;
        
        // Don't collide with self
        if (entityId == otherEntityId)
            return true;
            
        // Create contact pair
        ContactPair pair{entityId, otherEntityId, shapeId, 0};
        system->m_ActivePairs.push_back(pair);
        
        return true; // Continue querying
    }
}
