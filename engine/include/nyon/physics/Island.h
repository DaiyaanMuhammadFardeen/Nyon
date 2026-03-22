#pragma once
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include <vector>
#include <unordered_set>
#include <queue>
namespace Nyon::Physics {
    struct Island {
        std::vector<ECS::EntityID> bodyIds;       
        std::vector<std::pair<ECS::EntityID, ECS::EntityID>> contactPairs;  
        std::vector<std::pair<ECS::EntityID, ECS::EntityID>> jointPairs;    
        bool isAwake = true;                      
        float sleepTimer = 0.0f;                  
        bool canSleep = true;                     
        void Reset() {
            bodyIds.clear();
            contactPairs.clear();
            jointPairs.clear();
            isAwake = true;
            sleepTimer = 0.0f; } };
    class IslandManager {
    public:
        explicit IslandManager(ECS::ComponentStore& componentStore);
        void UpdateIslands(float deltaTime, const std::vector<ECS::EntityID>& activeEntities);
        const std::vector<Island>& GetAwakeIslands() const { return m_AwakeIslands; }
        const std::vector<Island>& GetSleepingIslands() const { return m_SleepingIslands; }
        void WakeIslandContaining(ECS::EntityID bodyId);
        bool IsBodyAwake(ECS::EntityID bodyId) const;
        struct Statistics {
            size_t totalIslands = 0;
            size_t awakeIslands = 0;
            size_t sleepingIslands = 0;
            size_t totalBodies = 0;
            size_t awakeBodies = 0;
            size_t sleepingBodies = 0; };
        Statistics GetStatistics() const;
    private:
        void BuildContactGraph(const std::vector<ECS::EntityID>& activeEntities);
        void BuildJointGraph();
        void FindIslands();
        void FloodFill(ECS::EntityID startBody, Island& island);
        void UpdateSleepTimers(float deltaTime);
        void PutIslandsToSleep();
        void WakeSleepingIslands();
        bool ShouldIslandSleep(const Island& island) const;
        bool AreBodiesConnected(ECS::EntityID bodyA, ECS::EntityID bodyB) const;
        float GetBodySleepVelocity(ECS::EntityID bodyId) const;
        bool IsBodyEligibleForSleeping(ECS::EntityID bodyId) const;
        ECS::ComponentStore& m_ComponentStore;
        std::unordered_map<ECS::EntityID, std::vector<ECS::EntityID>> m_ContactGraph;
        std::unordered_map<ECS::EntityID, std::vector<ECS::EntityID>> m_JointGraph;
        std::unordered_map<ECS::EntityID, std::vector<ECS::EntityID>> m_ConnectionGraph;  
        std::vector<Island> m_AllIslands;
        std::vector<Island> m_AwakeIslands;
        std::vector<Island> m_SleepingIslands;
        std::unordered_set<ECS::EntityID> m_VisitedBodies;
        std::unordered_map<ECS::EntityID, size_t> m_BodyIslandMap;  
        static constexpr float SLEEP_THRESHOLD = 2.0f;         
        static constexpr float ANGULAR_SLEEP_THRESHOLD = 0.2f;  
        static constexpr float TIME_TO_SLEEP = 0.5f;           
        static constexpr float LINEAR_WAKE_THRESHOLD = 2.0f;    
        static constexpr float ANGULAR_WAKE_THRESHOLD = 0.5f;    }; }