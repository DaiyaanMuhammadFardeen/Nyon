#include "nyon/physics/Island.h"
#include "nyon/ecs/components/JointComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include <algorithm>

// Debug logging macro - only output in debug builds
#ifdef _DEBUG
#define NYON_DEBUG_LOG(x) std::cerr << "[Island] " << x << std::endl
#else
#define NYON_DEBUG_LOG(x)
#endif

namespace Nyon::Physics
{
    IslandManager::IslandManager(ECS::ComponentStore& componentStore)
        : m_ComponentStore(componentStore)
    {
    }

    void IslandManager::UpdateIslands(float deltaTime, const std::vector<ECS::EntityID>& activeEntities)
    {
        NYON_DEBUG_LOG("Updating islands for " << activeEntities.size() << " active entities");
        
        // Build connection graphs
        BuildContactGraph(activeEntities);
        BuildJointGraph();
        
        // Find all islands
        FindIslands();
        
        // Update sleep timers and sleeping state
        UpdateSleepTimers(deltaTime);
        PutIslandsToSleep();
        WakeSleepingIslands();
        
        NYON_DEBUG_LOG("Island update complete: " << m_AwakeIslands.size() << " awake, " 
                     << m_SleepingIslands.size() << " sleeping islands");
    }

    void IslandManager::BuildContactGraph(const std::vector<ECS::EntityID>& activeEntities)
    {
        m_ContactGraph.clear();
        
        // Build graph from active contact manifolds, not all possible body pairs
        // This ensures only bodies actually in contact are connected in the graph
        const auto& worldEntities = m_ComponentStore.GetEntitiesWithComponent<ECS::PhysicsWorldComponent>();
        if (!worldEntities.empty())
        {
            const auto& physicsWorld = m_ComponentStore.GetComponent<ECS::PhysicsWorldComponent>(worldEntities[0]);
            for (const auto& manifold : physicsWorld.contactManifolds)
            {
                if (!manifold.touching)
                    continue;

                ECS::EntityID a = manifold.entityIdA;
                ECS::EntityID b = manifold.entityIdB;

                // Add bidirectional connections for touching contacts
                m_ContactGraph[a].push_back(b);
                m_ContactGraph[b].push_back(a);
            }
        }
        
        NYON_DEBUG_LOG("Built contact graph with " << m_ContactGraph.size() << " bodies from active manifolds");
    }

    void IslandManager::BuildJointGraph()
    {
        m_JointGraph.clear();
        
        // Look for joint components connecting bodies
        m_ComponentStore.ForEachComponent<ECS::JointComponent>([&](ECS::EntityID entityId, const ECS::JointComponent& joint) {
            // For now, we'll check common joint types
            if (joint.type == ECS::JointComponent::Type::Distance ||
                joint.type == ECS::JointComponent::Type::Revolute ||
                joint.type == ECS::JointComponent::Type::Prismatic)
            {
                // Use the generic joint endpoints stored on the component
                ECS::EntityID bodyA = joint.entityIdA;
                ECS::EntityID bodyB = joint.entityIdB;
                
                if (m_ComponentStore.HasComponent<ECS::PhysicsBodyComponent>(bodyA) &&
                    m_ComponentStore.HasComponent<ECS::PhysicsBodyComponent>(bodyB))
                {
                    const auto& bodyCompA = m_ComponentStore.GetComponent<ECS::PhysicsBodyComponent>(bodyA);
                    const auto& bodyCompB = m_ComponentStore.GetComponent<ECS::PhysicsBodyComponent>(bodyB);
                    
                    // Only connect dynamic bodies (static bodies don't form islands)
                    if (!bodyCompA.isStatic || !bodyCompB.isStatic)
                    {
                        m_JointGraph[bodyA].push_back(bodyB);
                        m_JointGraph[bodyB].push_back(bodyA);
                    }
                }
            }
        });
        
        NYON_DEBUG_LOG("Built joint graph with " << m_JointGraph.size() << " connected bodies");
    }

    void IslandManager::FindIslands()
    {
        // Save per-body sleep state from existing islands before clearing.
        // This preserves sleep timers and awake state across frames so islands
        // can eventually reach the TIME_TO_SLEEP threshold.
        m_BodySleepState.clear();
        for (const auto& island : m_AllIslands)
        {
            for (auto bodyId : island.bodyIds)
            {
                m_BodySleepState[bodyId] = {island.sleepTimer, island.isAwake};
            }
        }

        m_AllIslands.clear();
        m_VisitedBodies.clear();
        m_BodyIslandMap.clear();
        
        // Combine contact and joint graphs
        m_ConnectionGraph.clear();
        
        // Add contact connections
        for (const auto& [bodyId, connections] : m_ContactGraph)
        {
            m_ConnectionGraph[bodyId] = connections;
        }
        
        // Add joint connections
        for (const auto& [bodyId, connections] : m_JointGraph)
        {
            for (const auto& connectedBody : connections)
            {
                // Add if not already present
                if (std::find(m_ConnectionGraph[bodyId].begin(), m_ConnectionGraph[bodyId].end(), connectedBody) 
                    == m_ConnectionGraph[bodyId].end())
                {
                    m_ConnectionGraph[bodyId].push_back(connectedBody);
                }
            }
        }
        
        NYON_DEBUG_LOG("Combined graph has " << m_ConnectionGraph.size() << " bodies with connections");
        
        // Find all connected components (islands)
        for (const auto& [bodyId, connections] : m_ConnectionGraph)
        {
            if (m_VisitedBodies.find(bodyId) == m_VisitedBodies.end())
            {
                Island newIsland;
                FloodFill(bodyId, newIsland);
                
                if (!newIsland.bodyIds.empty())
                {
                    // Restore sleep state from previous frame
                    RestoreIslandSleepState(newIsland);
                    m_AllIslands.push_back(std::move(newIsland));
                }
            }
        }
        
        // Also add isolated bodies (bodies with no connections) as individual islands
        m_ComponentStore.ForEachComponent<ECS::PhysicsBodyComponent>([&](ECS::EntityID entityId, const ECS::PhysicsBodyComponent& body) {
            if (body.isStatic)
                return; // Static bodies don't form islands
                
            if (m_VisitedBodies.find(entityId) == m_VisitedBodies.end())
            {
                Island isolatedIsland;
                isolatedIsland.bodyIds.push_back(entityId);
                isolatedIsland.canSleep = IsBodyEligibleForSleeping(entityId);
                
                // Restore sleep state from previous frame
                RestoreIslandSleepState(isolatedIsland);
                
                // Register isolated body in island map BEFORE adding to array
                size_t islandIndex = m_AllIslands.size();
                m_BodyIslandMap[entityId] = islandIndex;
                
                m_AllIslands.push_back(std::move(isolatedIsland));
                m_VisitedBodies.insert(entityId);
            }
        });
        
        NYON_DEBUG_LOG("Found " << m_AllIslands.size() << " total islands");
    }

    void IslandManager::FloodFill(ECS::EntityID startBody, Island& island)
    {
        std::queue<ECS::EntityID> queue;
        queue.push(startBody);
        m_VisitedBodies.insert(startBody);
        island.bodyIds.push_back(startBody);
        
        // Capture island index explicitly BEFORE push_back to avoid stale reference
        size_t islandIndex = m_AllIslands.size();
        m_BodyIslandMap[startBody] = islandIndex;
        
        while (!queue.empty())
        {
            ECS::EntityID currentBody = queue.front();
            queue.pop();
            
            // Check connections in combined graph
            auto it = m_ConnectionGraph.find(currentBody);
            if (it != m_ConnectionGraph.end())
            {
                for (const auto& connectedBody : it->second)
                {
                    if (m_VisitedBodies.find(connectedBody) == m_VisitedBodies.end())
                    {
                        queue.push(connectedBody);
                        m_VisitedBodies.insert(connectedBody);
                        island.bodyIds.push_back(connectedBody);
                        m_BodyIslandMap[connectedBody] = islandIndex; // Use captured index
                    }
                }
            }
        }
        
        // Determine if island can sleep
        island.canSleep = true;
        for (const auto& bodyId : island.bodyIds)
        {
            if (!IsBodyEligibleForSleeping(bodyId))
            {
                island.canSleep = false;
                break;
            }
        }
    }

    void IslandManager::UpdateSleepTimers(float deltaTime)
    {
        for (auto& island : m_AllIslands)
        {
            if (!island.canSleep || !island.isAwake)
                continue;
                
            bool belowThreshold = true;
            float maxVelocitySq = 0.0f;
            
            for (const auto& bodyId : island.bodyIds)
            {
                float velocitySq = GetBodySleepVelocity(bodyId);
                maxVelocitySq = std::max(maxVelocitySq, velocitySq);
                
                if (velocitySq > SLEEP_THRESHOLD * SLEEP_THRESHOLD)
                {
                    belowThreshold = false;
                    break;
                }
            }
            
            if (belowThreshold)
            {
                island.sleepTimer += deltaTime;
                NYON_DEBUG_LOG("Island sleep timer: " << island.sleepTimer << " (max velocity sq: " << maxVelocitySq << ")");
            }
            else
            {
                island.sleepTimer = 0.0f;
            }
        }
    }

    void IslandManager::PutIslandsToSleep()
    {
        for (auto& island : m_AllIslands)
        {
            if (island.isAwake && island.canSleep && island.sleepTimer >= TIME_TO_SLEEP)
            {
                if (ShouldIslandSleep(island))
                {
                    island.isAwake = false;
                    NYON_DEBUG_LOG("Putting island to sleep with " << island.bodyIds.size() << " bodies");
                }
            }
        }
    }

    void IslandManager::WakeSleepingIslands()
    {
        for (auto& island : m_AllIslands)
        {
            if (!island.isAwake)
            {
                // Check if any body in this island has active contacts or was disturbed
                bool shouldWake = false;
                for (const auto& bodyId : island.bodyIds)
                {
                    // Wake if body has active contacts this frame
                    if (m_ContactGraph.find(bodyId) != m_ContactGraph.end())
                    {
                        shouldWake = true;
                        break;
                    }
                    // Wake if body's previous state has been cleared (body newly created or state unknown)
                    if (m_BodySleepState.find(bodyId) == m_BodySleepState.end())
                    {
                        shouldWake = true;
                        break;
                    }
                }
                
                if (shouldWake)
                {
                    island.isAwake = true;
                    island.sleepTimer = 0.0f;
                }
            }
        }
    }

    void IslandManager::RestoreIslandSleepState(Island& island)
    {
        // Restore sleep timer and awake state from previous frame's per-body state.
        // Take the maximum sleep timer from all bodies in the island (conservative),
        // and wake if any body was awake.
        float maxSleepTimer = 0.0f;
        bool anyAwake = false;
        bool anyUnknown = false;
        
        for (const auto& bodyId : island.bodyIds)
        {
            auto it = m_BodySleepState.find(bodyId);
            if (it != m_BodySleepState.end())
            {
                maxSleepTimer = std::max(maxSleepTimer, it->second.first);
                if (it->second.second)
                    anyAwake = true;
            }
            else
            {
                anyUnknown = true;
            }
        }
        
        // If any body was awake or is newly created, the island is awake
        if (anyAwake || anyUnknown)
        {
            island.isAwake = true;
            island.sleepTimer = 0.0f;
        }
        else
        {
            island.isAwake = false;
            island.sleepTimer = maxSleepTimer;
        }
    }

    bool IslandManager::ShouldIslandSleep(const Island& island) const
    {
        // An island should sleep if all bodies are below sleep thresholds
        for (const auto& bodyId : island.bodyIds)
        {
            // Check combined linear + angular velocity (squared)
            if (GetBodySleepVelocity(bodyId) > SLEEP_THRESHOLD * SLEEP_THRESHOLD)
            {
                return false;
            }
        }
        return true;
    }

    void IslandManager::WakeIslandContaining(ECS::EntityID bodyId)
    {
        auto it = m_BodyIslandMap.find(bodyId);
        if (it != m_BodyIslandMap.end())
        {
            size_t islandIndex = it->second;
            if (islandIndex < m_AllIslands.size())
            {
                m_AllIslands[islandIndex].isAwake = true;
                m_AllIslands[islandIndex].sleepTimer = 0.0f;
                NYON_DEBUG_LOG("Waking island containing body " << bodyId);
            }
        }
    }

    bool IslandManager::IsBodyAwake(ECS::EntityID bodyId) const
    {
        auto it = m_BodyIslandMap.find(bodyId);
        if (it != m_BodyIslandMap.end())
        {
            size_t islandIndex = it->second;
            if (islandIndex < m_AllIslands.size())
            {
                return m_AllIslands[islandIndex].isAwake;
            }
        }
        return true; // Default to awake if not found
    }

    bool IslandManager::AreBodiesConnected(ECS::EntityID bodyA, ECS::EntityID bodyB) const
    {
        // Check if bodies are connected through contacts or joints
        auto contactIt = m_ContactGraph.find(bodyA);
        if (contactIt != m_ContactGraph.end())
        {
            if (std::find(contactIt->second.begin(), contactIt->second.end(), bodyB) != contactIt->second.end())
                return true;
        }
        
        auto jointIt = m_JointGraph.find(bodyA);
        if (jointIt != m_JointGraph.end())
        {
            if (std::find(jointIt->second.begin(), jointIt->second.end(), bodyB) != jointIt->second.end())
                return true;
        }
        
        return false;
    }

    float IslandManager::GetBodySleepVelocity(ECS::EntityID bodyId) const
    {
        if (!m_ComponentStore.HasComponent<ECS::PhysicsBodyComponent>(bodyId))
            return 0.0f;
            
        const auto& body = m_ComponentStore.GetComponent<ECS::PhysicsBodyComponent>(bodyId);
        
        // Compute body's half-diagonal extent for angular velocity scaling
        // Use bounding box diagonal as approximation
        float extent = 1.0f; // Default extent
        if (m_ComponentStore.HasComponent<ECS::ColliderComponent>(bodyId))
        {
            const auto& collider = m_ComponentStore.GetComponent<ECS::ColliderComponent>(bodyId);
            // Calculate extent from collider AABB half-diagonal
            Math::Vector2 aabbMin, aabbMax;
            collider.CalculateAABB(Math::Vector2{0.0f, 0.0f}, 0.0f, aabbMin, aabbMax);
            Math::Vector2 diag = aabbMax - aabbMin;
            extent = std::max(diag.x, diag.y) * 0.5f;
            if (extent < 0.1f) extent = 1.0f;
        }
        
        // Combined linear and angular velocity metric
        // Angular velocity contribution scaled by body extent (tangential velocity at extremity)
        float linearVelSq = body.velocity.LengthSquared();
        float angularVel = std::abs(body.angularVelocity);
        float tangentialVel = angularVel * extent;
        
        // Return squared magnitude to avoid sqrt in comparisons
        return linearVelSq + tangentialVel * tangentialVel;
    }

    bool IslandManager::IsBodyEligibleForSleeping(ECS::EntityID bodyId) const
    {
        if (!m_ComponentStore.HasComponent<ECS::PhysicsBodyComponent>(bodyId))
            return false;
            
        const auto& body = m_ComponentStore.GetComponent<ECS::PhysicsBodyComponent>(bodyId);
        
        // Static bodies don't sleep (they're always "asleep")
        if (body.isStatic)
            return false;
            
        // Bodies with infinite mass (kinematic) don't sleep
        if (body.inverseMass == 0.0f)
            return false;
            
        return true;
    }

    IslandManager::Statistics IslandManager::GetStatistics() const
    {
        Statistics stats;
        stats.totalIslands = m_AllIslands.size();
        
        for (const auto& island : m_AllIslands)
        {
            stats.totalBodies += island.bodyIds.size();
            if (island.isAwake)
                stats.awakeBodies += island.bodyIds.size();
            else
                stats.sleepingBodies += island.bodyIds.size();
        }
        
        return stats;
    }
}