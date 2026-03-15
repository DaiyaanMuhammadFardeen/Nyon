#pragma once
#include "nyon/math/Vector2.h"
#include <functional>
#include <vector>
#include <unordered_map>
#include <memory>
namespace Nyon::ECS {
    /**
     * @brief Contact point data structure for collision information.
     * 
     * Stores detailed information about contact points between bodies.
     */
    struct ContactPoint {
        Math::Vector2 position = {0.0f, 0.0f};           
        Math::Vector2 normal = {0.0f, 0.0f};             
        float separation = 0.0f;                         
        float normalImpulse = 0.0f;                      
        float tangentImpulse = 0.0f;                     
        float normalMass = 0.0f;                         
        float tangentMass = 0.0f;                        
        uint32_t featureId = 0;                          
        bool persisted = false;                          
    };
    /**
     * @brief Contact manifold representing collision between two shapes.
     * 
     * Contains all contact points and geometric information for a collision.
     */
    struct ContactManifold {
        std::vector<ContactPoint> points;            
        Math::Vector2 normal = {0.0f, 0.0f};         
        Math::Vector2 localNormal = {0.0f, 0.0f};    
        Math::Vector2 localPoint = {0.0f, 0.0f};     
        float friction = 0.0f;                       
        float restitution = 0.0f;                    
        float tangentSpeed = 0.0f;                   
        uint32_t entityIdA = 0;                      
        uint32_t entityIdB = 0;                      
        uint32_t shapeIdA = 0;                       
        uint32_t shapeIdB = 0;                       
        bool touching = false;                       
    };
    /**
     * @brief Core physics world component inspired by Box2D's b2World.
     * 
     * Manages all physics simulation aspects including time stepping,
     * constraint solving, collision detection, and broad phase management.
     * Designed as a singleton component attached to a world entity.
     */
    struct PhysicsWorldComponent {
        Math::Vector2 gravity = {0.0f, 980.0f};       
        float timeStep = 1.0f / 60.0f;               
        int velocityIterations = 8;                  
        int positionIterations = 3;                  
        int subStepCount = 4;                        
        float baumgarteBeta = 0.2f;                  
        float linearSlop = 0.005f;                   
        float maxLinearCorrection = 0.2f;            
        float maxAngularCorrection = 0.1f;           
        float maxPenetrationCorrection = 50.0f;      
        float sleepLinearThreshold = 0.01f;          
        float sleepAngularThreshold = 0.01f;         
        float sleepTimeThreshold = 0.5f;             
        float restitutionThreshold = 100.0f;         
        float maxLinearSpeed = 10000.0f;             
        float maxAngularSpeed = 100.0f;              
        bool enableSleep = true;                     
        bool enableWarmStarting = true;              
        bool enableContinuous = true;                
        bool enableSpeculative = true;               
        float contactHertz = 30.0f;                  
        float contactDampingRatio = 1.0f;            
        float contactPushSpeed = 10.0f;              
        struct Profile {
            float broadPhaseTime = 0.0f;             
            float narrowPhaseTime = 0.0f;            
            float solverTime = 0.0f;                 
            float islandTime = 0.0f;                 
            float totalTime = 0.0f;                  
        } profile;
        struct Counters {
            int bodyCount = 0;                       
            int awakeBodyCount = 0;                  
            int contactCount = 0;                    
            int jointCount = 0;                      
            int islandCount = 0;                     
        } counters;
        bool drawShapes = false;                     
        bool drawJoints = false;                     
        bool drawAABBs = false;                      
        bool drawContacts = false;                   
        bool drawIslands = false;                    
        std::vector<ContactManifold> contactManifolds;
        struct Callbacks {
            std::function<void(uint32_t entityIdA, uint32_t entityIdB)> beginContact;
            std::function<void(uint32_t entityIdA, uint32_t entityIdB)> endContact;
            std::function<void(uint32_t entityIdA, uint32_t entityIdB, float impulse)> preSolve;
            std::function<void(uint32_t entityIdA, uint32_t entityIdB, float impulse)> postSolve;
            std::function<void(uint32_t jointId, float force, float torque)> jointBreak;
            std::function<void(uint32_t sensorId, uint32_t entityId)> sensorBegin;
            std::function<void(uint32_t sensorId, uint32_t entityId)> sensorEnd;
        } callbacks;
        PhysicsWorldComponent() = default;
        void SetGravity(const Math::Vector2& newGravity) { gravity = newGravity; }
        void SetTimeStep(float dt) { timeStep = dt; }
        void SetIterations(int velocityIters, int positionIters) 
        { 
            velocityIterations = velocityIters;
            positionIterations = positionIters;
        }
        void SetSubSteps(int subSteps) { subStepCount = subSteps; }
        void EnableSleeping(bool enable) { enableSleep = enable; }
        void EnableWarmStarting(bool enable) { enableWarmStarting = enable; }
        void EnableContinuous(bool enable) { enableContinuous = enable; }
        void SetContactTuning(float hertz, float dampingRatio, float pushSpeed)
        {
            contactHertz = hertz;
            contactDampingRatio = dampingRatio;
            contactPushSpeed = pushSpeed;
        }
        const Profile& GetProfile() const { return profile; }
        const Counters& GetCounters() const { return counters; }
        float GetInvTimeStep() const { return (timeStep > 0.0f) ? 1.0f / timeStep : 0.0f; }
        bool IsSleepingEnabled() const { return enableSleep; }
        bool IsContinuousEnabled() const { return enableContinuous; }
        bool IsWarmStartingEnabled() const { return enableWarmStarting; }
        void SetDebugDraw(bool shapes, bool joints, bool aabbs, bool contacts, bool islands)
        {
            drawShapes = shapes;
            drawJoints = joints;
            drawAABBs = aabbs;
            drawContacts = contacts;
            drawIslands = islands;
        }
        void SetBeginContactCallback(std::function<void(uint32_t, uint32_t)> callback) 
        { 
            callbacks.beginContact = callback; 
        }
        void SetEndContactCallback(std::function<void(uint32_t, uint32_t)> callback) 
        { 
            callbacks.endContact = callback; 
        }
        void SetPreSolveCallback(std::function<void(uint32_t, uint32_t, float)> callback) 
        { 
            callbacks.preSolve = callback; 
        }
        void SetPostSolveCallback(std::function<void(uint32_t, uint32_t, float)> callback) 
        { 
            callbacks.postSolve = callback; 
        }
        void SetJointBreakCallback(std::function<void(uint32_t, float, float)> callback) 
        { 
            callbacks.jointBreak = callback; 
        }
        void SetSensorBeginCallback(std::function<void(uint32_t, uint32_t)> callback) 
        { 
            callbacks.sensorBegin = callback; 
        }
        void SetSensorEndCallback(std::function<void(uint32_t, uint32_t)> callback) 
        { 
            callbacks.sensorEnd = callback; 
        }
    };
    /**
     * @brief Joint edge connecting two bodies through a constraint.
     * 
     * Represents the connection between bodies in the constraint graph.
     */
    struct JointEdge {
        uint32_t otherEntityId = 0;                  
        uint32_t jointId = 0;                        
        JointEdge* prev = nullptr;                   
        JointEdge* next = nullptr;                   
    };
}
