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
        
        // No pointer caching - query fresh each Update() call
        // This prevents dangling pointers when new entities are added
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
        // Query fresh entities each frame to avoid stale pointers
        const auto& bodyEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsBodyComponent>();
        
        // Clean up proxies for destroyed entities first
        CleanupDestroyedProxies(bodyEntities);
        
        // Update all shape AABBs in the broad phase tree
        for (auto entityId : bodyEntities)
        {
            if (!m_ComponentStore->HasComponent<ColliderComponent>(entityId))
                continue;
                
            const auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityId);
            const auto& collider = m_ComponentStore->GetComponent<ColliderComponent>(entityId);
            
            if (!body.ShouldCollide())
                continue;
            
            // Get position from TransformComponent
            Math::Vector2 position = {0.0f, 0.0f};
            if (m_ComponentStore->HasComponent<TransformComponent>(entityId))
            {
                const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
                position = transform.position;
            }
            
            // Update shape AABB in broad phase
            // Use entityId as unique identifier since each entity has one shape
            UpdateShapeAABB(entityId, entityId, const_cast<ColliderComponent*>(&collider), position);
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
        
        // Query fresh entities each frame to avoid stale pointers
        const auto& bodyEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsBodyComponent>();
        
        // For each shape, query broad phase for potential collisions
        for (auto entityIdA : bodyEntities)
        {
            if (!m_ComponentStore->HasComponent<ColliderComponent>(entityIdA))
                continue;
                
            const auto& bodyA = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityIdA);
            const auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(entityIdA);
            
            // Skip static bodies for performance
            if (bodyA.isStatic && !bodyA.isKinematic)
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
            colliderA.CalculateAABB(positionA, min, max);
            Physics::AABB queryAABB(min, max);
            
            // Query broad phase
            BroadPhaseCallback callback;
            callback.system = this;
            callback.entityId = entityIdA;
            callback.shapeId = entityIdA;
            
            m_BroadPhaseTree.Query(queryAABB, &callback);
        }
    }
    
    void CollisionPipelineSystem::UpdateContacts()
    {
        // Process active contact pairs: detect overlap and resolve it immediately.
        for (const auto& pair : m_ActivePairs)
        {
            // Require all necessary components
            if (!m_ComponentStore->HasComponent<TransformComponent>(pair.entityIdA) ||
                !m_ComponentStore->HasComponent<TransformComponent>(pair.entityIdB) ||
                !m_ComponentStore->HasComponent<PhysicsBodyComponent>(pair.entityIdA) ||
                !m_ComponentStore->HasComponent<PhysicsBodyComponent>(pair.entityIdB) ||
                !m_ComponentStore->HasComponent<ColliderComponent>(pair.entityIdA) ||
                !m_ComponentStore->HasComponent<ColliderComponent>(pair.entityIdB))
            {
                continue;
            }
            
            // Get mutable references so we can resolve the collision
            auto& transformA = m_ComponentStore->GetComponent<TransformComponent>(pair.entityIdA);
            auto& transformB = m_ComponentStore->GetComponent<TransformComponent>(pair.entityIdB);
            auto& bodyA = m_ComponentStore->GetComponent<PhysicsBodyComponent>(pair.entityIdA);
            auto& bodyB = m_ComponentStore->GetComponent<PhysicsBodyComponent>(pair.entityIdB);
            const auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(pair.entityIdA);
            const auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(pair.entityIdB);
            
            // Skip if neither body should participate
            if (!bodyA.ShouldCollide() && !bodyB.ShouldCollide())
                continue;
            
            // Simple AABB intersection test
            Math::Vector2 minA, maxA, minB, maxB;
            colliderA.CalculateAABB(transformA.position, minA, maxA);
            colliderB.CalculateAABB(transformB.position, minB, maxB);
            
            if (!(minA.x < maxB.x && maxA.x > minB.x && 
                  minA.y < maxB.y && maxA.y > minB.y))
            {
                continue;
            }
            
            // Compute overlap and normal using minimum-penetration axis
            Math::Vector2 centerA = (minA + maxA) * 0.5f;
            Math::Vector2 centerB = (minB + maxB) * 0.5f;
            Math::Vector2 delta = centerB - centerA;
            
            Math::Vector2 halfSizeA = (maxA - minA) * 0.5f;
            Math::Vector2 halfSizeB = (maxB - minB) * 0.5f;
            
            float overlapX = (halfSizeA.x + halfSizeB.x) - std::abs(delta.x);
            float overlapY = (halfSizeA.y + halfSizeB.y) - std::abs(delta.y);
            
            if (overlapX <= 0.0f || overlapY <= 0.0f)
                continue;
            
            Math::Vector2 normal;
            float penetration;
            if (overlapX < overlapY)
            {
                normal = (delta.x > 0.0f) ? Math::Vector2{1.0f, 0.0f} : Math::Vector2{-1.0f, 0.0f};
                penetration = overlapX;
            }
            else
            {
                normal = (delta.y > 0.0f) ? Math::Vector2{0.0f, 1.0f} : Math::Vector2{0.0f, -1.0f};
                penetration = overlapY;
            }
            
            // Positional correction using inverse masses
            float invMassA = bodyA.inverseMass;
            float invMassB = bodyB.inverseMass;
            float totalInvMass = invMassA + invMassB;
            if (totalInvMass > 0.0f)
            {
                const float baumgarte = 0.6f;
                Math::Vector2 correction = normal * (penetration * baumgarte / totalInvMass);
                
                if (!bodyA.isStatic)
                {
                    transformA.position = transformA.position - correction * invMassA;
                }
                if (!bodyB.isStatic)
                {
                    transformB.position = transformB.position + correction * invMassB;
                }
            }
            
            // Velocity resolution (impulse-based)
            Math::Vector2 relativeVel = bodyB.velocity - bodyA.velocity;
            float velAlongNormal = Math::Vector2::Dot(relativeVel, normal);
            
            // If velocities are separating, skip impulse but keep positional fix
            if (velAlongNormal < 0.0f)
            {
                float restitution = std::sqrt(bodyA.restitution * bodyB.restitution);
                float j = -(1.0f + restitution) * velAlongNormal;
                float massTerm = invMassA + invMassB;
                if (massTerm > 0.0f)
                {
                    j /= massTerm;
                    Math::Vector2 impulse = normal * j;
                    
                    if (!bodyA.isStatic)
                    {
                        bodyA.velocity = bodyA.velocity - impulse * invMassA;
                    }
                    if (!bodyB.isStatic)
                    {
                        bodyB.velocity = bodyB.velocity + impulse * invMassB;
                    }
                }
            }
            
            // Grounded state: if dynamic body resting on static ground with upward normal (y-down coords)
            auto markGroundedIfApplicable = [&](PhysicsBodyComponent& dynBody,
                                                 const PhysicsBodyComponent& otherBody,
                                                 const Math::Vector2& n)
            {
                if (!dynBody.isStatic && otherBody.isStatic && n.y < -0.7f)
                {
                    dynBody.UpdateGroundedState(true);
                }
            };
            
            markGroundedIfApplicable(bodyA, bodyB, normal);
            markGroundedIfApplicable(bodyB, bodyA, -normal);
            
            // Manage contact callbacks
            ContactPair contactKey{pair.entityIdA, pair.entityIdB, pair.shapeIdA, pair.shapeIdB};
            if (m_ContactMap.find(contactKey) == m_ContactMap.end())
            {
                uint32_t contactId = static_cast<uint32_t>(m_ContactMap.size());
                m_ContactMap[contactKey] = contactId;
                
                if (m_PhysicsWorld->callbacks.beginContact)
                {
                    m_PhysicsWorld->callbacks.beginContact(pair.entityIdA, pair.entityIdB);
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
    
    void CollisionPipelineSystem::CleanupDestroyedProxies(const std::vector<EntityID>& activeEntities)
    {
        // Create a set of active entity IDs for fast lookup
        std::unordered_set<EntityID> activeSet(activeEntities.begin(), activeEntities.end());
        
        // Find and remove proxies for destroyed entities
        auto it = m_ShapeProxyMap.begin();
        while (it != m_ShapeProxyMap.end())
        {
            EntityID entityId = it->first;
            uint32_t proxyId = it->second;
            
            // Check if entity is still active
            if (activeSet.find(entityId) == activeSet.end())
            {
                // Entity has been destroyed - remove its proxy
                m_BroadPhaseTree.DestroyProxy(proxyId);
                it = m_ShapeProxyMap.erase(it);
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
        
        // Use entityId as unique shape identifier to avoid collisions
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
            
        // Check if both entities have collider components
        if (!system->m_ComponentStore->HasComponent<ColliderComponent>(entityId) ||
            !system->m_ComponentStore->HasComponent<ColliderComponent>(otherEntityId))
        {
            return true; // Continue querying
        }
        
        // Get collider components for filtering
        const auto& colliderA = system->m_ComponentStore->GetComponent<ColliderComponent>(entityId);
        const auto& colliderB = system->m_ComponentStore->GetComponent<ColliderComponent>(otherEntityId);
        
        // Apply broad-phase filtering using ColliderComponent::Filter
        if (!colliderA.filter.ShouldCollide(colliderB.filter))
        {
            return true; // Skip this pair due to filtering
        }
            
        // Create contact pair
        ContactPair pair{entityId, otherEntityId, shapeId, otherEntityId};
        system->m_ActivePairs.push_back(pair);
        
        return true; // Continue querying
    }
}
