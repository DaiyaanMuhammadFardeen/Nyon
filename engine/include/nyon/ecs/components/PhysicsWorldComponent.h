#pragma once

#include "nyon/math/Vector2.h"
#include <functional>
#include <vector>
#include <unordered_map>
#include <memory>

namespace Nyon::ECS
{
    /**
     * @brief Contact point data structure for collision information.
     * 
     * Stores detailed information about contact points between bodies.
     */
    struct ContactPoint
    {
        Math::Vector2 position = {0.0f, 0.0f};          // World position of contact point
        Math::Vector2 normal = {0.0f, 0.0f};            // Contact normal (points from A to B)
        float separation = 0.0f;                        // Separation distance (negative = penetration)
        float normalImpulse = 0.0f;                     // Normal impulse applied
        float tangentImpulse = 0.0f;                    // Tangent impulse applied
        float normalMass = 0.0f;                        // Normal constraint mass
        float tangentMass = 0.0f;                       // Tangent constraint mass
        uint32_t featureId = 0;                         // Feature identifier for persistence
        bool persisted = false;                         // Whether this point persisted from previous step
    };
    
    /**
     * @brief Contact manifold representing collision between two shapes.
     * 
     * Contains all contact points and geometric information for a collision.
     */
    struct ContactManifold
    {
        std::vector<ContactPoint> points;           // Contact points (0-2 typically)
        Math::Vector2 normal = {0.0f, 0.0f};        // Contact normal
        Math::Vector2 localNormal = {0.0f, 0.0f};   // Normal in local coordinates
        Math::Vector2 localPoint = {0.0f, 0.0f};    // Reference point in local coordinates
        float friction = 0.0f;                      // Combined friction coefficient
        float restitution = 0.0f;                   // Combined restitution coefficient
        float tangentSpeed = 0.0f;                  // Tangent speed for friction
        uint32_t entityIdA = 0;                     // First entity ID
        uint32_t entityIdB = 0;                     // Second entity ID
        uint32_t shapeIdA = 0;                      // First shape ID
        uint32_t shapeIdB = 0;                      // Second shape ID
        bool touching = false;                      // Whether shapes are touching
    };
    
    /**
     * @brief Core physics world component inspired by Box2D's b2World.
     * 
     * Manages all physics simulation aspects including time stepping,
     * constraint solving, collision detection, and broad phase management.
     * Designed as a singleton component attached to a world entity.
     */
    struct PhysicsWorldComponent
    {
        // === SIMULATION SETTINGS ===
        Math::Vector2 gravity = {0.0f, -980.0f};     // Gravity vector (pixels/sec²) - downward in Y-up screen space
        float timeStep = 1.0f / 60.0f;              // Fixed time step (seconds)
        int velocityIterations = 8;                 // Velocity constraint solver iterations
        int positionIterations = 3;                 // Position constraint solver iterations
        int subStepCount = 4;                       // Sub-stepping for stability
        
        // === POSITION CORRECTION PARAMETERS ===
        float baumgarteBeta = 0.2f;                 // Baumgarte stabilization factor
        float linearSlop = 0.5f;                    // Linear slop for position correction (half a pixel for pixel-unit worlds)
        float maxLinearCorrection = 1000.0f;        // Very large linear position correction for impenetrable solid bodies
        float maxAngularCorrection = 0.5f;          // Maximum angular position correction
        float maxPenetrationCorrection = 1000.0f;   // Very large penetration depth correction
        
        // === SLEEP PARAMETERS ===
        float sleepLinearThreshold = 0.01f;         // Linear velocity threshold for sleeping
        float sleepAngularThreshold = 0.01f;        // Angular velocity threshold for sleeping
        float sleepTimeThreshold = 0.5f;            // Time duration before allowing sleep
        
        // === DAMPING PARAMETERS ===
        // Note: For frame-rate-independent damping, use formula: 1.0f / (1.0f + dt * damping)
        // Default values approximate air resistance
        
        // === GLOBAL PARAMETERS ===
        float restitutionThreshold = 1.0f;          // Velocity threshold for restitution (lower = less bounce)
        float maxLinearSpeed = 10000.0f;            // Maximum body linear speed
        float maxAngularSpeed = 100.0f;             // Maximum body angular speed (rad/sec)
        bool enableSleep = true;                    // Global sleep enable/disable
        bool enableWarmStarting = true;             // Enable constraint warm starting
        bool enableContinuous = true;               // Continuous collision detection
        bool enableSpeculative = true;              // Speculative contacts
        
        // === CONTACT TUNING ===
        float contactHertz = 30.0f;                 // Contact stiffness frequency
        float contactDampingRatio = 1.0f;           // Contact damping ratio
        float contactPushSpeed = 10.0f;             // Maximum contact push-out speed
        
        // === PERFORMANCE COUNTING ===
        struct Profile
        {
            float broadPhaseTime = 0.0f;            // Broad phase update time
            float narrowPhaseTime = 0.0f;           // Narrow phase collision time
            float solverTime = 0.0f;                // Constraint solver time
            float islandTime = 0.0f;                // Island building time
            float totalTime = 0.0f;                 // Total step time
        } profile;
        
        struct Counters
        {
            int bodyCount = 0;                      // Total bodies
            int awakeBodyCount = 0;                 // Awake bodies
            int contactCount = 0;                   // Active contacts
            int jointCount = 0;                     // Active joints
            int islandCount = 0;                    // Active islands
        } counters;
        
        // === DEBUGGING ===
        bool drawShapes = false;                    // Visualize shapes
        bool drawJoints = false;                    // Visualize joints
        bool drawAABBs = false;                     // Visualize bounding boxes
        bool drawContacts = false;                  // Visualize contact points
        bool drawIslands = false;                   // Visualize islands
        
        // === NARROW-PHASE CONTACT MANIFOLDS ===
        // Populated by the collision pipeline each physics step and consumed by the constraint solver.
        std::vector<ContactManifold> contactManifolds;
        
        // === EVENT CALLBACKS ===
        struct Callbacks
        {
            // Contact events
            std::function<void(uint32_t entityIdA, uint32_t entityIdB)> beginContact;
            std::function<void(uint32_t entityIdA, uint32_t entityIdB)> endContact;
            std::function<void(uint32_t entityIdA, uint32_t entityIdB, float impulse)> preSolve;
            std::function<void(uint32_t entityIdA, uint32_t entityIdB, float impulse)> postSolve;
            
            // Joint events
            std::function<void(uint32_t jointId, float force, float torque)> jointBreak;
            
            // Sensor events
            std::function<void(uint32_t sensorId, uint32_t entityId)> sensorBegin;
            std::function<void(uint32_t sensorId, uint32_t entityId)> sensorEnd;
        } callbacks;
        
        // === CONSTRUCTOR ===
        PhysicsWorldComponent() = default;
        
        // === CONFIGURATION METHODS ===
        void SetGravity(const Math::Vector2& newGravity) { gravity = newGravity; }
        void SetTimeStep(float dt) { timeStep = dt; }
        void SetIterations(int velocityIters, int positionIters) 
        { 
            velocityIterations = velocityIters;
            positionIterations = positionIters;
        }
        void SetSubSteps(int subSteps) { subStepCount = subSteps; }
        
        // === GLOBAL CONTROL ===
        void EnableSleeping(bool enable) { enableSleep = enable; }
        void EnableWarmStarting(bool enable) { enableWarmStarting = enable; }
        void EnableContinuous(bool enable) { enableContinuous = enable; }
        
        // === CONTACT PARAMETERS ===
        void SetContactTuning(float hertz, float dampingRatio, float pushSpeed)
        {
            contactHertz = hertz;
            contactDampingRatio = dampingRatio;
            contactPushSpeed = pushSpeed;
        }
        
        // === PERFORMANCE ACCESSORS ===
        const Profile& GetProfile() const { return profile; }
        const Counters& GetCounters() const { return counters; }
        
        // === UTILITY METHODS ===
        float GetInvTimeStep() const { return (timeStep > 0.0f) ? 1.0f / timeStep : 0.0f; }
        bool IsSleepingEnabled() const { return enableSleep; }
        bool IsContinuousEnabled() const { return enableContinuous; }
        bool IsWarmStartingEnabled() const { return enableWarmStarting; }
        
        // === DEBUG CONTROL ===
        void SetDebugDraw(bool shapes, bool joints, bool aabbs, bool contacts, bool islands)
        {
            drawShapes = shapes;
            drawJoints = joints;
            drawAABBs = aabbs;
            drawContacts = contacts;
            drawIslands = islands;
        }
        
        // === EVENT REGISTRATION ===
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
    struct JointEdge
    {
        uint32_t otherEntityId = 0;                 // Other body entity ID
        uint32_t jointId = 0;                       // Joint connecting the bodies
        JointEdge* prev = nullptr;                  // Previous edge in body's joint list
        JointEdge* next = nullptr;                  // Next edge in body's joint list
    };
}
