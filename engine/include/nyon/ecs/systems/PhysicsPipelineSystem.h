#pragma once

#include "nyon/ecs/System.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/physics/Island.h"
#include "nyon/physics/DynamicTree.h"
#include <vector>
#include <unordered_map>

namespace Nyon::ECS
{
    /**
     * @brief Unified physics pipeline system implementing coherent collision detection and response
     * 
     * This system unifies the entire physics pipeline into a single cohesive system that handles:
     * 1. Broad-phase collision detection using DynamicTree
     * 2. Narrow-phase collision detection and manifold generation
     * 3. Island detection and sleeping optimization
     * 4. Constraint solving and integration
     * 5. Positional correction and stabilization
     * 
     * Inspired by Box2D's unified physics pipeline approach.
     */
    class PhysicsPipelineSystem : public System
    {
    public:
        void Update(float deltaTime) override;
        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override;
        virtual ~PhysicsPipelineSystem() = default;
        
        // Pipeline configuration
        struct Config
        {
            int velocityIterations = 8;      // Number of velocity constraint iterations
            int positionIterations = 3;      // Number of position constraint iterations
            float baumgarte = 0.2f;          // Baumgarte stabilization factor
            float linearSlop = 0.005f;       // Linear slop for position correction
            float maxLinearCorrection = 0.2f; // Maximum linear position correction
            bool warmStarting = true;        // Enable warm starting of constraints
            bool useIslandSleeping = true;   // Enable island-based sleeping optimization
        };
        
        void SetConfig(const Config& config) { m_Config = config; }
        const Config& GetConfig() const { return m_Config; }
        
        // Pipeline statistics
        struct Statistics
        {
            size_t broadPhasePairs = 0;
            size_t narrowPhaseContacts = 0;
            size_t activeConstraints = 0;
            size_t awakeBodies = 0;
            size_t sleepingBodies = 0;
            float updateTime = 0.0f; // Time spent in last update (milliseconds)
            Physics::IslandManager::Statistics islandStats;
        };
        
        const Statistics& GetStatistics() const { return m_Stats; }
        
    private:
        // Contact and constraint structures
        struct ContactPoint
        {
            Math::Vector2 position;          // World contact position
            Math::Vector2 normal;            // Contact normal (from A to B)
            float separation;                // Separation distance (negative = penetration)
            float normalImpulse;             // Accumulated normal impulse
            float tangentImpulse;            // Accumulated tangent impulse
            float normalMass;                // Normal constraint mass
            float tangentMass;               // Tangent constraint mass
            float velocityBias;              // Velocity bias for restitution
            uint32_t featureId;              // Feature identifier for persistence
        };
        
        struct ContactManifold
        {
            std::vector<ContactPoint> points; // Contact points
            Math::Vector2 normal;             // Manifold normal
            uint32_t entityIdA;               // First entity
            uint32_t entityIdB;               // Second entity
            bool persisted = false;           // Whether this contact persisted from previous frame
        };
        
        struct VelocityConstraint
        {
            Math::Vector2 normal;                           // Contact normal
            Math::Vector2 tangent;                          // Contact tangent
            std::vector<ContactPoint> points;               // Contact points with constraint data
            uint32_t indexA;                                // Body A index in solver arrays
            uint32_t indexB;                                // Body B index in solver arrays
            float friction;                                 // Combined friction
            float restitution;                              // Combined restitution
            float invMassA, invMassB;                       // Inverse masses
            float invIA, invIB;                             // Inverse inertias
        };
        
        // Solver body structure
        struct SolverBody
        {
            Math::Vector2 position;                         // Current position
            float angle;                                    // Current angle
            Math::Vector2 velocity;                         // Linear velocity
            float angularVelocity;                          // Angular velocity
            Math::Vector2 prevPosition;                     // Previous position for interpolation
            float prevAngle;                                // Previous angle for interpolation
            Math::Vector2 force;                            // Accumulated force
            float torque;                                   // Accumulated torque
            float invMass;                                  // Inverse mass
            float invInertia;                               // Inverse inertia
            Math::Vector2 localCenter;                      // Local center of mass
            bool isStatic;                                  // Whether body is static
            bool isAwake;                                   // Whether body is awake
            ECS::EntityID entityId;                         // Associated entity ID
        };
        
        // Pipeline phases
        void BroadPhaseDetection();
        void NarrowPhaseDetection();
        void IslandDetection();
        void ConstraintInitialization();
        void VelocitySolving();
        void PositionSolving();
        void Integration();
        void StoreImpulses();
        void UpdateSleeping();
        
        // Broad phase helpers
        struct BroadPhaseCallback : public Physics::ITreeQueryCallback
        {
            PhysicsPipelineSystem* system;
            uint32_t entityId;
            
            bool QueryCallback(uint32_t nodeId, uint32_t userData) override;
        };
        
        void UpdateShapeAABB(uint32_t entityId, ColliderComponent* collider, 
                           const Math::Vector2& position, float angle);
        
        // Collision detection helpers
        bool TestCollision(uint32_t entityIdA, uint32_t entityIdB);
        ContactManifold GenerateManifold(uint32_t entityIdA, uint32_t entityIdB);
        Math::Vector2 ComputeClosestPoint(const Math::Vector2& point, 
                                        const Math::Vector2& min, const Math::Vector2& max);
        
        // Impulse caching
        uint64_t MakeImpulseCacheKey(uint32_t entityIdA, uint32_t entityIdB, uint32_t featureId) const;
        
        // Constraint solving helpers
        void InitializeVelocityConstraints();
        void SolveVelocityConstraints();
        void SolvePositionConstraints();
        void WarmStartConstraints();
        void IntegrateVelocities(float dt);
        void IntegratePositions(float dt);
        
        // Utility methods
        void PrepareBodiesForUpdate();
        void UpdateTransformsFromSolver();
        void ClearPersistentContacts();
        
        // Component references
        ComponentStore* m_ComponentStore = nullptr;
        PhysicsWorldComponent* m_PhysicsWorld = nullptr;
        
        // Pipeline data
        Config m_Config;
        Statistics m_Stats;
        
        // Broad phase
        Physics::DynamicTree m_BroadPhaseTree;
        std::unordered_map<uint32_t, uint32_t> m_ShapeProxyMap;
        std::vector<std::pair<uint32_t, uint32_t>> m_BroadPhasePairs;
        
        // Contact management
        std::vector<ContactManifold> m_ContactManifolds;
        std::unordered_map<uint64_t, size_t> m_ContactMap; // entityId pair -> manifold index
        std::vector<bool> m_ContactPersisted; // Tracks which contacts persisted
        
        // Impulse cache for warm starting (keyed by entity pair + feature ID)
        struct ImpulseData
        {
            float normalImpulse = 0.0f;
            float tangentImpulse = 0.0f;
        };
        std::unordered_map<uint64_t, ImpulseData> m_ImpulseCache;
        
        // Island management
        std::unique_ptr<Physics::IslandManager> m_IslandManager;
        std::vector<uint32_t> m_ActiveEntities;
        
        // Solver data
        std::vector<SolverBody> m_SolverBodies;
        std::unordered_map<uint32_t, size_t> m_EntityToSolverIndex;
        std::vector<VelocityConstraint> m_VelocityConstraints;
        
        // Timing
        float m_Accumulator = 0.0f;
        
        // Constants
        static constexpr float FIXED_TIMESTEP = 1.0f / 60.0f; // 60 FPS physics
        static constexpr float MAX_TIMESTEP = 0.25f;          // Maximum time step
    };
}