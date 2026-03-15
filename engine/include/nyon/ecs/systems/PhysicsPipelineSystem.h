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
namespace Nyon::ECS {
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
    class PhysicsPipelineSystem : public System {
    public:
        void Update(float deltaTime) override;
        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override;
        virtual ~PhysicsPipelineSystem() = default;
        struct Config {
            int velocityIterations = 8;       
            int positionIterations = 3;       
            float baumgarte = 0.2f;           
            float linearSlop = 0.005f;        
            float maxLinearCorrection = 0.2f;  
            bool warmStarting = true;         
            bool useIslandSleeping = true;    
        };
        void SetConfig(const Config& config) { m_Config = config; }
        const Config& GetConfig() const { return m_Config; }
        struct Statistics {
            size_t broadPhasePairs = 0;
            size_t narrowPhaseContacts = 0;
            size_t activeConstraints = 0;
            size_t awakeBodies = 0;
            size_t sleepingBodies = 0;
            float updateTime = 0.0f;  
            Physics::IslandManager::Statistics islandStats;
        };
        const Statistics& GetStatistics() const { return m_Stats; }
    private:
        struct ContactPoint {
            Math::Vector2 position;           
            Math::Vector2 normal;             
            float separation;                 
            float normalImpulse;              
            float tangentImpulse;             
            float normalMass;                 
            float tangentMass;                
            float velocityBias;               
        };
        struct ContactManifold {
            std::vector<ContactPoint> points;  
            Math::Vector2 normal;              
            uint32_t entityIdA;                
            uint32_t entityIdB;                
            bool persisted = false;            
        };
        struct VelocityConstraint {
            Math::Vector2 normal;                            
            Math::Vector2 tangent;                           
            std::vector<ContactPoint> points;                
            uint32_t indexA;                                 
            uint32_t indexB;                                 
            float friction;                                  
            float restitution;                               
            float invMassA, invMassB;                        
            float invIA, invIB;                              
        };
        struct PositionConstraint {
            Math::Vector2 localPoints[2];                    
            Math::Vector2 localNormal;                       
            Math::Vector2 localPoint;                        
            uint32_t indexA;                                 
            uint32_t indexB;                                 
            float invMassA, invMassB;                        
            Math::Vector2 localCenterA;                      
            Math::Vector2 localCenterB;                      
            float invIA, invIB;                              
            int pointCount;                                  
        };
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
        };
        void BroadPhaseDetection();
        void NarrowPhaseDetection();
        void IslandDetection();
        void ConstraintInitialization();
        void VelocitySolving();
        void PositionSolving();
        void Integration();
        void StoreImpulses();
        void UpdateSleeping();
        struct BroadPhaseCallback : public Physics::ITreeQueryCallback {
            PhysicsPipelineSystem* system;
            uint32_t entityId;
            bool QueryCallback(uint32_t nodeId, uint32_t userData) override;
        };
        void UpdateShapeAABB(uint32_t entityId, ColliderComponent* collider, 
                           const Math::Vector2& position, float angle);
        bool TestCollision(uint32_t entityIdA, uint32_t entityIdB);
        ContactManifold GenerateManifold(uint32_t entityIdA, uint32_t entityIdB);
        Math::Vector2 ComputeClosestPoint(const Math::Vector2& point, 
                                        const Math::Vector2& min, const Math::Vector2& max);
        void InitializeVelocityConstraints();
        void SolveVelocityConstraints();
        void SolvePositionConstraints();
        void WarmStartConstraints();
        void IntegrateVelocities(float dt);
        void IntegratePositions(float dt);
        void PrepareBodiesForUpdate();
        void UpdateTransformsFromSolver();
        void ClearPersistentContacts();
        ComponentStore* m_ComponentStore = nullptr;
        PhysicsWorldComponent* m_PhysicsWorld = nullptr;
        Config m_Config;
        Statistics m_Stats;
        Physics::DynamicTree m_BroadPhaseTree;
        std::unordered_map<uint32_t, uint32_t> m_ShapeProxyMap;
        std::vector<std::pair<uint32_t, uint32_t>> m_BroadPhasePairs;
        std::vector<ContactManifold> m_ContactManifolds;
        std::unordered_map<uint64_t, size_t> m_ContactMap;  
        std::vector<bool> m_ContactPersisted;  
        std::unique_ptr<Physics::IslandManager> m_IslandManager;
        std::vector<uint32_t> m_ActiveEntities;
        std::vector<SolverBody> m_SolverBodies;
        std::unordered_map<uint32_t, size_t> m_EntityToSolverIndex;
        std::vector<VelocityConstraint> m_VelocityConstraints;
        std::vector<PositionConstraint> m_PositionConstraints;
        float m_Accumulator = 0.0f;
        static constexpr float FIXED_TIMESTEP = 1.0f / 60.0f;  
        static constexpr float MAX_TIMESTEP = 0.25f;           
    };
}