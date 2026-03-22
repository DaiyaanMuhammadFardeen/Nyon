#pragma once
#include "nyon/ecs/System.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/physics/Island.h"
#include "nyon/physics/DynamicTree.h"
#include "nyon/physics/ContactTypes.h"
#include "nyon/utils/ThreadPool.h"
#include "nyon/EngineConstants.h"
#include <vector>
#include <unordered_map>
#include <future>
#include <atomic>
namespace Nyon::ECS {
    class PhysicsPipelineSystem : public System {
    public:
        void Update(float deltaTime) override;
        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override;
        virtual ~PhysicsPipelineSystem() = default;
        struct Config {
            int velocityIterations = 8;       
            int positionIterations = 3;       
            float baumgarte = 0.2f;           
            float linearSlop = 0.5f;          
            float maxLinearCorrection = 20.0f;  
            bool warmStarting = true;         
            bool useIslandSleeping = true;     };
        void SetConfig(const Config& config) { m_Config = config; }
        const Config& GetConfig() const { return m_Config; }
        struct Statistics {
            size_t broadPhasePairs = 0;
            size_t narrowPhaseContacts = 0;
            size_t activeConstraints = 0;
            size_t awakeBodies = 0;
            size_t sleepingBodies = 0;
            float updateTime = 0.0f;  
            Physics::IslandManager::Statistics islandStats; };
        const Statistics& GetStatistics() const { return m_Stats; }
    private:
        struct ContactPointConstraint {
            Math::Vector2 position;           
            Math::Vector2 normal;             
            float separation;                 
            float normalImpulse;              
            float tangentImpulse;             
            float normalMass;                 
            float tangentMass;                
            float velocityBias;               
            uint32_t featureId;                };
        struct VelocityConstraint {
            Math::Vector2 normal;                            
            Math::Vector2 tangent;                           
            std::vector<ContactPointConstraint> points;      
            uint32_t indexA;                                 
            uint32_t indexB;                                 
            float friction;                                  
            float restitution;                               
            float invMassA, invMassB;                        
            float invIA, invIB;                               };
        struct SolverBody {
            Math::Vector2 position;                          
            float angle;                                     
            Math::Vector2 velocity;                          
            float angularVelocity;                           
            Math::Vector2 prevPosition;                      
            float prevAngle;                                 
            Math::Vector2 force;                             
            float torque;                                    
            float invMass;                                   
            float invInertia;                                
            Math::Vector2 localCenter;                       
            bool isStatic;                                   
            bool isAwake;                                    
            ECS::EntityID entityId;                          
            float linearDamping;                             
            float angularDamping;                             };
        void BroadPhaseDetection();
        void NarrowPhaseDetection();
        void IslandDetection();
        void ConstraintInitialization();
        void VelocitySolving();
        void PositionSolving();
        void Integration();
        void StoreImpulses();
        void UpdateSleeping();
        void ParallelBroadPhase();
        void ParallelNarrowPhase();
        void ParallelVelocitySolving(float subStepDt);
        void ParallelPositionSolving(float subStepDt);
        struct BroadPhaseCallback : public Physics::ITreeQueryCallback {
            PhysicsPipelineSystem* system;
            uint32_t entityId;
            std::vector<std::pair<uint32_t, uint32_t>>* localPairs = nullptr;
            bool QueryCallback(uint32_t nodeId, uint32_t userData) override; };
        void UpdateShapeAABB(uint32_t entityId, ColliderComponent* collider, 
                           const Math::Vector2& position, float angle);
        bool TestCollision(uint32_t entityIdA, uint32_t entityIdB);
        ECS::ContactManifold GenerateManifold(uint32_t entityIdA, uint32_t entityIdB);
        Math::Vector2 ComputeClosestPoint(const Math::Vector2& point, 
                                        const Math::Vector2& min, const Math::Vector2& max);
        uint64_t MakeImpulseCacheKey(uint32_t entityIdA, uint32_t entityIdB, uint32_t featureId) const;
        void InitializeVelocityConstraints();
        void SolveVelocityConstraints();
        void SolvePositionConstraints();
        void WarmStartConstraints();
        void IntegrateVelocities(float dt);
        void IntegrateVelocities(float dt, size_t start, size_t end);   
        void IntegratePositions(float dt);
        void PrepareBodiesForUpdate();
        void UpdateTransformsFromSolver();
        void ClearPersistentContacts();
        ComponentStore* m_ComponentStore = nullptr;
        EntityID m_PhysicsWorldEntity = INVALID_ENTITY;
        Config m_Config;
        Statistics m_Stats;
        Physics::DynamicTree m_BroadPhaseTree;
        std::unordered_map<uint32_t, uint32_t> m_ShapeProxyMap;
        std::vector<std::pair<uint32_t, uint32_t>> m_BroadPhasePairs;
        std::vector<ECS::ContactManifold> m_ContactManifolds;
        std::unordered_map<uint64_t, size_t> m_ContactMap;  
        struct ImpulseData {
            float normalImpulse = 0.0f;
            float tangentImpulse = 0.0f; };
        std::unordered_map<uint64_t, ImpulseData> m_ImpulseCache;
        std::unique_ptr<Physics::IslandManager> m_IslandManager;
        std::vector<uint32_t> m_ActiveEntities;
        std::vector<SolverBody> m_SolverBodies;
        std::unordered_map<uint32_t, size_t> m_EntityToSolverIndex;
        std::vector<VelocityConstraint> m_VelocityConstraints;
        bool m_UseMultiThreading = true;
        size_t m_NumThreads = 0; }; }