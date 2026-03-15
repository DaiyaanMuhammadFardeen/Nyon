#pragma once
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include <vector>
#include <unordered_set>
#include <queue>
namespace Nyon::Physics {
    /**
     * @brief Represents a group of connected bodies that can interact with each other
     * 
     * Islands are groups of bodies connected by contacts or joints that can be solved
     * together. Bodies in separate islands don't interact and can be processed independently
     * or put to sleep when stable.
     */
    struct Island {
        std::vector<ECS::EntityID> bodyIds;       
        std::vector<std::pair<ECS::EntityID, ECS::EntityID>> contactPairs;  
        std::vector<std::pair<ECS::EntityID, ECS::EntityID>> jointPairs;    
        bool isAwake = true;                      
        float sleepTimer = 0.0f;                  
        bool canSleep = true;                     
        void Reset()
        {
            bodyIds.clear();
            contactPairs.clear();
            jointPairs.clear();
            isAwake = true;
            sleepTimer = 0.0f;
        }
    };
    /**
     * @brief Manages island detection and sleeping optimization for physics simulation
     * 
     * Implements broad-phase island detection by grouping connected bodies and joints,
     * then puts stable islands to sleep to improve performance. Inspired by Box2D's
     * island-based sleeping system.
     */
    class IslandManager {
    public:
        explicit IslandManager(ECS::ComponentStore& componentStore);
        /**
         * @brief Detect islands and update sleeping state
         * @param deltaTime Time step for integration
         * @param activeEntities List of currently active physics entities
         */
        void UpdateIslands(float deltaTime, const std::vector<ECS::EntityID>& activeEntities);
        /**
         * @brief Get awake islands for processing
         * @return Vector of awake islands
         */
        const std::vector<Island>& GetAwakeIslands() const { return m_AwakeIslands; }
        /**
         * @brief Get sleeping islands
         * @return Vector of sleeping islands
         */
        const std::vector<Island>& GetSleepingIslands() const { return m_SleepingIslands; }
        /**
         * @brief Wake up an island containing a specific body
         * @param bodyId Body that triggered wake-up
         */
        void WakeIslandContaining(ECS::EntityID bodyId);
        /**
         * @brief Check if a body is in an awake island
         * @param bodyId Body to check
         * @return True if body is in an awake island
         */
        bool IsBodyAwake(ECS::EntityID bodyId) const;
        /**
         * @brief Get statistics about island distribution
         */
        struct Statistics {
            size_t totalIslands = 0;
            size_t awakeIslands = 0;
            size_t sleepingIslands = 0;
            size_t totalBodies = 0;
            size_t awakeBodies = 0;
            size_t sleepingBodies = 0;
        };
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
        static constexpr float SLEEP_THRESHOLD = 0.5f;       
        static constexpr float ANGULAR_SLEEP_THRESHOLD = 0.1f;  
        static constexpr float TIME_TO_SLEEP = 0.5f;         
        static constexpr float LINEAR_WAKE_THRESHOLD = 2.0f;  
        static constexpr float ANGULAR_WAKE_THRESHOLD = 0.5f;  
    };
}