#pragma once

#include "nyon/ecs/System.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/math/Vector2.h"
#include <vector>

namespace Nyon::ECS
{
    /**
     * @brief Constraint solver system implementing sequential impulse solver.
     * 
     * Solves velocity and position constraints using iterative sequential impulses.
     * Inspired by Box2D's constraint solver implementation.
     */
    class ConstraintSolverSystem : public System
    {
    public:
        void Update(float deltaTime) override;
        
        // Initialize system with required components
        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override;
        
    private:
        // Constraint structures
        struct VelocityConstraintPoint
        {
            Math::Vector2 rA;           // Vector from body A center of mass to contact point
            Math::Vector2 rB;           // Vector from body B center of mass to contact point
            float normalImpulse;        // Normal impulse accumulated
            float tangentImpulse;       // Tangent impulse accumulated
            float normalMass;           // Normal constraint mass
            float tangentMass;          // Tangent constraint mass
            float velocityBias;         // Velocity bias for restitution
        };
        
        struct ContactVelocityConstraint
        {
            Math::Vector2 normal;                       // Contact normal (from A to B)
            Math::Vector2 tangent;                      // Contact tangent
            VelocityConstraintPoint points[2];          // Up to 2 contact points
            uint32_t indexA;                            // Body A index in solver arrays
            uint32_t indexB;                            // Body B index in solver arrays
            float friction;                             // Combined friction
            float restitution;                          // Combined restitution
            float invMassA, invMassB;                   // Inverse masses
            float invIA, invIB;                         // Inverse inertias
            int pointCount;                             // Number of contact points (1 or 2)
        };
        
        struct ContactPositionConstraint
        {
            Math::Vector2 localPoints[2];               // Contact points in local coordinates
            Math::Vector2 localNormal;                  // Normal in local coordinates
            Math::Vector2 localPoint;                   // Reference point in local coordinates
            uint32_t indexA;                            // Body A index
            uint32_t indexB;                            // Body B index
            float invMassA, invMassB;                   // Inverse masses
            float localCenterA, localCenterB;           // Local centers of mass
            float invIA, invIB;                         // Inverse inertias
            int pointCount;                             // Number of contact points
        };
        
        // Solver data structures
        struct SolverBody
        {
            Math::Vector2 position;                     // World position
            float angle;                                // World angle
            Math::Vector2 velocity;                     // Linear velocity
            float angularVelocity;                      // Angular velocity
            float invMass;                              // Inverse mass
            float invInertia;                           // Inverse inertia
            Math::Vector2 localCenter;                  // Local center of mass
            bool isStatic;                              // Whether body is static
        };
        
        // Internal solver methods
        void InitializeConstraints();
        void SolveVelocityConstraints();
        void SolvePositionConstraints();
        void StoreImpulses();
        
        // Utility methods
        void WarmStart();
        void IntegrateVelocities(float dt);
        void IntegratePositions(float dt);
        
        // Component references
        ComponentStore* m_ComponentStore = nullptr;
        PhysicsWorldComponent* m_PhysicsWorld;
        std::vector<PhysicsBodyComponent*> m_BodyComponents;
        std::vector<uint32_t> m_BodyEntityIds;
        
        // Solver data
        std::vector<SolverBody> m_SolverBodies;
        std::vector<ContactVelocityConstraint> m_VelocityConstraints;
        std::vector<ContactPositionConstraint> m_PositionConstraints;
        
        // Constants
        static constexpr float BAIUMGARTE = 0.2f;           // Baumgarte stabilization factor
        static constexpr float LINEAR_SLOP = 0.005f;        // Linear slop for position correction
        static constexpr float MAX_LINEAR_CORRECTION = 0.2f; // Maximum linear position correction
        static constexpr float MAX_ANGULAR_CORRECTION = 0.1f; // Maximum angular position correction
        static constexpr float VELOCITY_THRESHOLD = 1.0f;   // Velocity threshold for restitution
    };
}
