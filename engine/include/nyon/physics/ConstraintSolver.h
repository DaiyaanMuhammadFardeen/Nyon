// SPDX-FileCopyrightText: 2026 Nyon Engine
// SPDX-License-Identifier: MIT

#pragma once

#include "nyon/physics/SATCollisionDetector.h"
#include "nyon/math/Vector2.h"
#include <vector>
#include <cstdint>

namespace Nyon::Physics
{
    /**
     * @brief Solver body - temporary body representation for constraint solving
     */
    struct SolverBody
    {
        uint32_t entityId;
        Math::Vector2 position;
        Math::Vector2 linearVelocity;
        float angle;
        float angularVelocity;
        float invMass;
        float invInertia;
        Math::Vector2 localCenter;
        bool isStatic;
        
        // Position correction deltas
        Math::Vector2 deltaPosition;
        float deltaRotation;
        
        SolverBody()
            : entityId(0)
            , position{0.0f, 0.0f}
            , linearVelocity{0.0f, 0.0f}
            , angle(0.0f)
            , angularVelocity(0.0f)
            , invMass(0.0f)
            , invInertia(0.0f)
            , localCenter{0.0f, 0.0f}
            , isStatic(false)
            , deltaPosition{0.0f, 0.0f}
            , deltaRotation(0.0f)
        {}
    };
    
    /**
     * @brief Velocity constraint for a single contact point
     */
    struct VelocityConstraintPoint
    {
        Math::Vector2 rA;         // Vector from body A center to contact (world space)
        Math::Vector2 rB;         // Vector from body B center to contact (world space)
        float normalMass;         // Effective mass in normal direction
        float tangentMass;        // Effective mass in tangent direction
        float bias;               // Baumgarte stabilization bias
        float normalImpulse;      // Accumulated normal impulse
        float tangentImpulse;     // Accumulated tangent impulse
        float totalNormalImpulse; // Total normal impulse for friction
        float maxFriction;        // Maximum friction force
    };
    
    /**
     * @brief Velocity constraint for a contact manifold
     */
    struct VelocityConstraint
    {
        uint32_t indexA;          // Index of body A in solver array
        uint32_t indexB;          // Index of body B in solver array
        Math::Vector2 normal;     // Contact normal (A to B)
        Math::Vector2 tangent;    // Contact tangent (perpendicular to normal)
        std::vector<VelocityConstraintPoint> points;
        float friction;           // Combined friction coefficient
        float restitution;        // Combined restitution coefficient
        float tangentSpeed;       // Target tangent speed (for conveyor belts)
        int contactIndex;         // Original contact manifold index
    };
    
    /**
     * @brief Position constraint for position correction
     */
    struct PositionConstraint
    {
        uint32_t indexA;
        uint32_t indexB;
        Math::Vector2 localPointA;  // Pivot in local space of A
        Math::Vector2 localPointB;  // Pivot in local space of B
        Math::Vector2 localNormal;  // Normal in local space of A
        float separation;           // Current separation (negative = penetration)
        float minSeparation;        // Minimum separation for stability
    };
    
    /**
     * @brief Advanced constraint solver with multiple stabilization techniques
     * 
     * Features:
     * - Sequential Impulses with position correction
     * - Baumgarte stabilization
     * - Speculative contacts
     * - Split impulses for anti-flickering
     * - Friction and restitution modeling
     * - Rolling resistance
     */
    class ConstraintSolver
    {
    public:
        ConstraintSolver();
        ~ConstraintSolver();
        
        /**
         * @brief Initialize solver with contacts and bodies
         */
        void Initialize(
            const std::vector<ContactManifold>& contacts,
            const std::vector<SolverBody>& bodies,
            float timeStep,
            int velocityIterations = 8,
            int positionIterations = 3);
        
        /**
         * @brief Solve velocity constraints
         * 
         * Uses sequential impulse method with warm starting.
         */
        void SolveVelocityConstraints();
        
        /**
         * @brief Integrate velocities to update positions
         */
        void IntegrateVelocities(float dt);
        
        /**
         * @brief Solve position constraints to correct penetration
         * 
         * Uses Baumgarte stabilization with clamping to prevent overshoot.
         */
        void SolvePositionConstraints();
        
        /**
         * @brief Apply solved velocities back to bodies
         */
        void WriteBackToBodies(std::vector<SolverBody>& bodies);
        
        /**
         * @brief Get solved velocity constraints for persistence
         */
        const std::vector<VelocityConstraint>& GetVelocityConstraints() const 
        { return m_VelocityConstraints; }
        
        /**
         * @brief Set gravity for bias computation
         */
        void SetGravity(const Math::Vector2& gravity) { m_Gravity = gravity; }
        
        /**
         * @brief Enable/disable warm starting
         */
        void SetWarmStarting(bool enabled) { m_WarmStarting = enabled; }
        
        /**
         * @brief Enable/disable split impulses for better stability
         */
        void SetSplitImpulses(bool enabled) { m_UseSplitImpulses = enabled; }
        
        /**
         * @brief Set Baumgarte stabilization parameters
         */
        void SetBaumgarteParameters(float beta, float slop)
        {
            m_BaumgarteBeta = beta;
            m_BaumgarteSlop = slop;
        }
        
    private:
        std::vector<SolverBody> m_SolverBodies;
        std::vector<VelocityConstraint> m_VelocityConstraints;
        std::vector<PositionConstraint> m_PositionConstraints;
        
        float m_TimeStep;
        int m_VelocityIterations;
        int m_PositionIterations;
        Math::Vector2 m_Gravity;
        bool m_WarmStarting;
        bool m_UseSplitImpulses;
        float m_BaumgarteBeta;
        float m_BaumgarteSlop;
        
        /**
         * @brief Initialize velocity constraints from contact manifolds
         */
        void InitializeVelocityConstraints(const std::vector<ContactManifold>& contacts);
        
        /**
         * @brief Initialize position constraints from contact manifolds
         */
        void InitializePositionConstraints(const std::vector<ContactManifold>& contacts);
        
        /**
         * @brief Compute effective mass for a contact point
         */
        void ComputeEffectiveMass(VelocityConstraint& vc);
        
        /**
         * @brief Apply warm starting impulses
         */
        void WarmStart();
        
        /**
         * @brief Solve a single velocity constraint
         */
        void SolveVelocityConstraint(VelocityConstraint& vc);
        
        /**
         * @brief Solve a single position constraint
         * @return true if constraint was satisfied
         */
        bool SolvePositionConstraint(PositionConstraint& pc);
        
        /**
         * @brief Compute combined friction coefficient
         */
        static float CombineFriction(float frictionA, float frictionB);
        
        /**
         * @brief Compute combined restitution coefficient
         */
        static float CombineRestitution(float restitutionA, float restitutionB);
    };
}
