#include "nyon/physics/Island.h"
#include "nyon/ecs/components/JointComponent.h"
#include <algorithm>
#include <iostream>

#ifdef _DEBUG
#define NYON_DEBUG_LOG(x) std::cout << "[Island] " << x << std::endl
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
        
        // For simplicity, we'll assume all active dynamic bodies can potentially contact each other
        // In a real implementation, this would come from the collision detection system
        for (size_t i = 0; i < activeEntities.size(); ++i)
        {
            if (!m_ComponentStore.HasComponent<ECS::PhysicsBodyComponent>(activeEntities[i]))
                continue;
                
            const auto& bodyA = m_ComponentStore.GetComponent<ECS::PhysicsBodyComponent>(activeEntities[i]);
            if (bodyA.isStatic)
                continue;
                
            for (size_t j = i + 1; j < activeEntities.size(); ++j)
            {
                if (!m_ComponentStore.HasComponent<ECS::PhysicsBodyComponent>(activeEntities[j]))
                    continue;
                    
                const auto& bodyB = m_ComponentStore.GetComponent<ECS::PhysicsBodyComponent>(activeEntities[j]);
                if (bodyB.isStatic)
                    continue;
                
                // Add bidirectional connections (potential contact)
                m_ContactGraph[activeEntities[i]].push_back(activeEntities[j]);
                m_ContactGraph[activeEntities[j]].push_back(activeEntities[i]);
            }
        }
        
        NYON_DEBUG_LOG("Built contact graph with " << m_ContactGraph.size() << " bodies");
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
        m_AllIslands.clear();
        m_AwakeIslands.clear();
        m_SleepingIslands.clear();
        m_BodyIslandMap.clear();
        m_VisitedBodies.clear();
        
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
                m_AllIslands.push_back(std::move(isolatedIsland));
                m_VisitedBodies.insert(entityId);
            }
        });
        
        // Separate awake and sleeping islands
        for (auto& island : m_AllIslands)
        {
            if (island.isAwake)
            {
                m_AwakeIslands.push_back(std::move(island));
            }
            else
            {
                m_SleepingIslands.push_back(std::move(island));
            }
        }
        
        NYON_DEBUG_LOG("Found " << m_AllIslands.size() << " total islands (" 
                     << m_AwakeIslands.size() << " awake, " << m_SleepingIslands.size() << " sleeping)");
    }

    void IslandManager::FloodFill(ECS::EntityID startBody, Island& island)
    {
        std::queue<ECS::EntityID> queue;
        queue.push(startBody);
        m_VisitedBodies.insert(startBody);
        island.bodyIds.push_back(startBody);
        m_BodyIslandMap[startBody] = m_AllIslands.size(); // Index of current island being built
        
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
                        m_BodyIslandMap[connectedBody] = m_AllIslands.size();
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
            float maxVelocity = 0.0f;
            
            for (const auto& bodyId : island.bodyIds)
            {
                float velocity = GetBodySleepVelocity(bodyId);
                maxVelocity = std::max(maxVelocity, velocity);
                
                if (velocity > SLEEP_THRESHOLD)
                {
                    belowThreshold = false;
                    break;
                }
            }
            
            if (belowThreshold)
            {
                island.sleepTimer += deltaTime;
                NYON_DEBUG_LOG("Island sleep timer: " << island.sleepTimer << " (max velocity: " << maxVelocity << ")");
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
        // Move islands that should be awake from sleeping to awake list
        auto it = m_SleepingIslands.begin();
        while (it != m_SleepingIslands.end())
        {
            if (it->isAwake)
            {
                m_AwakeIslands.push_back(std::move(*it));
                it = m_SleepingIslands.erase(it);
                NYON_DEBUG_LOG("Waking up island with " << m_AwakeIslands.back().bodyIds.size() << " bodies");
            }
            else
            {
                ++it;
            }
        }
    }

    bool IslandManager::ShouldIslandSleep(const Island& island) const
    {
        // An island should sleep if all bodies are below sleep thresholds
        for (const auto& bodyId : island.bodyIds)
        {
            if (GetBodySleepVelocity(bodyId) > SLEEP_THRESHOLD)
            {
                return false;
            }
            
            // Check angular velocity too
            if (m_ComponentStore.HasComponent<ECS::PhysicsBodyComponent>(bodyId))
            {
                const auto& body = m_ComponentStore.GetComponent<ECS::PhysicsBodyComponent>(bodyId);
                if (std::abs(body.angularVelocity) > ANGULAR_SLEEP_THRESHOLD)
                {
                    return false;
                }
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
        return body.velocity.Length();
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
        stats.awakeIslands = m_AwakeIslands.size();
        stats.sleepingIslands = m_SleepingIslands.size();
        
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