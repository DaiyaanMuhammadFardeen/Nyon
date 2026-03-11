// SPDX-FileCopyrightText: 2026 Nyon Engine
// SPDX-License-Identifier: MIT

#include "nyon/physics/ConstraintSolver.h"
#include <algorithm>
#include <cmath>
#include <cassert>

namespace Nyon::Physics
{
    ConstraintSolver::ConstraintSolver()
        : m_TimeStep(1.0f / 60.0f)
        , m_VelocityIterations(8)
        , m_PositionIterations(3)
        , m_Gravity{0.0f, -9.81f}
        , m_WarmStarting(true)
        , m_UseSplitImpulses(true)
        , m_BaumgarteBeta(0.2f)
        , m_BaumgarteSlop(0.005f)
    {
    }
    
    ConstraintSolver::~ConstraintSolver() = default;
    
    void ConstraintSolver::Initialize(
        const std::vector<ContactManifold>& contacts,
        const std::vector<SolverBody>& bodies,
        float timeStep,
        int velocityIterations,
        int positionIterations)
    {
        m_SolverBodies = bodies;
        m_TimeStep = timeStep;
        m_VelocityIterations = velocityIterations;
        m_PositionIterations = positionIterations;
        
        InitializeVelocityConstraints(contacts);
        InitializePositionConstraints(contacts);
    }
    
    void ConstraintSolver::InitializeVelocityConstraints(const std::vector<ContactManifold>& contacts)
    {
        m_VelocityConstraints.clear();
        m_VelocityConstraints.reserve(contacts.size());
        
        for (size_t i = 0; i < contacts.size(); ++i)
        {
            const auto& manifold = contacts[i];
            
            if (!manifold.touching || manifold.points.empty())
                continue;
            
            // Find solver body indices
            uint32_t indexA = UINT32_MAX, indexB = UINT32_MAX;
            for (uint32_t j = 0; j < m_SolverBodies.size(); ++j)
            {
                if (m_SolverBodies[j].entityId == manifold.entityIdA)
                    indexA = j;
                else if (m_SolverBodies[j].entityId == manifold.entityIdB)
                    indexB = j;
            }
            
            if (indexA == UINT32_MAX || indexB == UINT32_MAX)
                continue;
            
            VelocityConstraint vc;
            vc.indexA = indexA;
            vc.indexB = indexB;
            vc.normal = manifold.normal;
            vc.tangent = {-manifold.normal.y, manifold.normal.x};  // Perpendicular
            vc.contactIndex = static_cast<int>(i);
            vc.friction = 0.3f;  // Default friction
            vc.restitution = 0.0f;  // Default restitution
            vc.tangentSpeed = 0.0f;
            
            // Create velocity constraint points
            for (const auto& cp : manifold.points)
            {
                VelocityConstraintPoint vcp;
                vcp.rA = cp.position - (m_SolverBodies[indexA].position + m_SolverBodies[indexA].localCenter);
                vcp.rB = cp.position - (m_SolverBodies[indexB].position + m_SolverBodies[indexB].localCenter);
                vcp.normalImpulse = 0.0f;
                vcp.tangentImpulse = 0.0f;
                vcp.totalNormalImpulse = 0.0f;
                vcp.bias = 0.0f;
                vcp.maxFriction = 0.0f;
                
                vc.points.push_back(vcp);
            }
            
            m_VelocityConstraints.push_back(vc);
        }
    }
    
    void ConstraintSolver::InitializePositionConstraints(const std::vector<ContactManifold>& contacts)
    {
        m_PositionConstraints.clear();
        m_PositionConstraints.reserve(contacts.size());
        
        for (const auto& manifold : contacts)
        {
            if (!manifold.touching)
                continue;
            
            uint32_t indexA = UINT32_MAX, indexB = UINT32_MAX;
            for (uint32_t j = 0; j < m_SolverBodies.size(); ++j)
            {
                if (m_SolverBodies[j].entityId == manifold.entityIdA)
                    indexA = j;
                else if (m_SolverBodies[j].entityId == manifold.entityIdB)
                    indexB = j;
            }
            
            if (indexA == UINT32_MAX || indexB == UINT32_MAX)
                continue;
            
            PositionConstraint pc;
            pc.indexA = indexA;
            pc.indexB = indexB;
            pc.localPointA = manifold.localPoint;
            pc.localPointB = manifold.localPoint;  // Simplified
            pc.localNormal = manifold.localNormal;
            pc.separation = manifold.points.empty() ? 0.0f : manifold.points[0].separation;
            pc.minSeparation = -0.02f;  // Allow small penetration for stability
            
            m_PositionConstraints.push_back(pc);
        }
    }
    
    void ConstraintSolver::SolveVelocityConstraints()
    {
        // Warm starting
        if (m_WarmStarting)
        {
            WarmStart();
        }
        
        // Solve constraints iteratively
        for (int iter = 0; iter < m_VelocityIterations; ++iter)
        {
            for (auto& vc : m_VelocityConstraints)
            {
                SolveVelocityConstraint(vc);
            }
        }
    }
    
    void ConstraintSolver::WarmStart()
    {
        // Scale cached impulses by dt ratio (not implemented in this simplified version)
        // In a full implementation, you would cache impulses from previous frame
    }
    
    void ConstraintSolver::SolveVelocityConstraint(VelocityConstraint& vc)
    {
        SolverBody& bodyA = m_SolverBodies[vc.indexA];
        SolverBody& bodyB = m_SolverBodies[vc.indexB];
        
        Math::Vector2 vA = bodyA.linearVelocity;
        float wA = bodyA.angularVelocity;
        Math::Vector2 vB = bodyB.linearVelocity;
        float wB = bodyB.angularVelocity;
        
        for (auto& vcp : vc.points)
        {
            // Compute relative velocity at contact
            Math::Vector2 dvA = Math::Vector2{-wA * vcp.rA.y, wA * vcp.rA.x};
            Math::Vector2 dvB = Math::Vector2{-wB * vcp.rB.y, wB * vcp.rB.x};
            Math::Vector2 relativeVelocity = (vB + dvB) - (vA + dvA);
            
            // Normal velocity
            float vn = Math::Vector2::Dot(relativeVelocity, vc.normal);
            
            // Apply position correction bias (Baumgarte stabilization)
            float C = std::max(vcp.separation + m_BaumgarteSlop, 0.0f);
            vcp.bias = -m_BaumgarteBeta * C / m_TimeStep;
            
            // Compute normal impulse
            float lambda = -vcp.normalMass * (vn + vcp.bias);
            
            // Clamp accumulated impulse
            float newImpulse = std::max(vcp.normalImpulse + lambda, 0.0f);
            lambda = newImpulse - vcp.normalImpulse;
            vcp.normalImpulse = newImpulse;
            vcp.totalNormalImpulse += lambda;
            
            // Apply normal impulse
            Math::Vector2 P = vc.normal * lambda;
            if (!bodyA.isStatic)
            {
                vA = vA - P * bodyA.invMass;
                wA -= bodyA.invInertia * (vcp.rA.x * P.y - vcp.rA.y * P.x);
            }
            if (!bodyB.isStatic)
            {
                vB = vB + P * bodyB.invMass;
                wB += bodyB.invInertia * (vcp.rB.x * P.y - vcp.rB.y * P.x);
            }
            
            // Tangential velocity (friction)
            Math::Vector2 tangentVel = relativeVelocity - vc.normal * vn;
            float vt = Math::Vector2::Dot(tangentVel, vc.tangent) - vc.tangentSpeed;
            
            // Compute friction impulse
            lambda = -vcp.tangentMass * vt;
            
            // Coulomb friction clamp
            vcp.maxFriction = 0.3f * vcp.totalNormalImpulse;  // mu * normal force
            float maxTangentImpulse = vcp.maxFriction * m_TimeStep;
            
            float oldTangentImpulse = vcp.tangentImpulse;
            vcp.tangentImpulse = std::clamp(vcp.tangentImpulse + lambda, -maxTangentImpulse, maxTangentImpulse);
            lambda = vcp.tangentImpulse - oldTangentImpulse;
            
            // Apply friction impulse
            Math::Vector2 tangentP = vc.tangent * lambda;
            if (!bodyA.isStatic)
            {
                vA = vA - tangentP * bodyA.invMass;
                wA -= bodyA.invInertia * (vcp.rA.x * tangentP.y - vcp.rA.y * tangentP.x);
            }
            if (!bodyB.isStatic)
            {
                vB = vB + tangentP * bodyB.invMass;
                wB += bodyB.invInertia * (vcp.rB.x * tangentP.y - vcp.rB.y * tangentP.x);
            }
        }
        
        // Write back velocities
        bodyA.linearVelocity = vA;
        bodyA.angularVelocity = wA;
        bodyB.linearVelocity = vB;
        bodyB.angularVelocity = wB;
    }
    
    void ConstraintSolver::IntegrateVelocities(float dt)
    {
        for (auto& body : m_SolverBodies)
        {
            if (body.isStatic)
                continue;
            
            // Apply gravity
            if (body.invMass > 0.0f)
            {
                body.linearVelocity = body.linearVelocity + m_Gravity * dt;
            }
            
            // Integrate position
            body.deltaPosition = body.linearVelocity * dt;
            body.deltaRotation = body.angularVelocity * dt;
        }
    }
    
    void ConstraintSolver::SolvePositionConstraints()
    {
        const float linearSlop = m_BaumgarteSlop;
        const float maxCorrection = 0.2f;
        const float angularSlop = 0.01f;
        const float maxAngularCorrection = 0.1f;
        
        for (int iter = 0; iter < m_PositionIterations; ++iter)
        {
            bool allOK = true;
            
            for (auto& pc : m_PositionConstraints)
            {
                SolverBody& bodyA = m_SolverBodies[pc.indexA];
                SolverBody& bodyB = m_SolverBodies[pc.indexB];
                
                // Transform local points to world space
                // Simplified: assuming no rotation for now
                Math::Vector2 rA = pc.localPointA;
                Math::Vector2 rB = pc.localPointB;
                
                Math::Vector2 pA = bodyA.position + bodyA.localCenter + rA;
                Math::Vector2 pB = bodyB.position + bodyB.localCenter + rB;
                
                Math::Vector2 d = pB - pA;
                float separation = Math::Vector2::Dot(d, pc.localNormal);
                
                // Only correct significant penetration
                float C = std::min(separation + linearSlop, 0.0f);
                if (C == 0.0f)
                    continue;
                
                allOK = false;
                
                // Compute position correction
                float invMassA = bodyA.invMass;
                float invMassB = bodyB.invMass;
                
                if (invMassA == 0.0f && invMassB == 0.0f)
                    continue;
                
                // Clamp correction to prevent overshoot
                C = std::max(C, -maxCorrection);
                
                Math::Vector2 correction = pc.localNormal * C;
                
                // Apply position correction
                if (invMassA > 0.0f)
                {
                    bodyA.deltaPosition = bodyA.deltaPosition - correction * invMassA / (invMassA + invMassB);
                }
                if (invMassB > 0.0f)
                {
                    bodyB.deltaPosition = bodyB.deltaPosition + correction * invMassB / (invMassA + invMassB);
                }
            }
            
            if (allOK)
                break;
        }
    }
    
    void ConstraintSolver::WriteBackToBodies(std::vector<SolverBody>& bodies)
    {
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            if (m_SolverBodies[i].isStatic)
                continue;
            
            // Update positions with deltas
            bodies[i].position = bodies[i].position + m_SolverBodies[i].deltaPosition;
            bodies[i].angle = bodies[i].angle + m_SolverBodies[i].deltaRotation;
            
            // Keep velocities updated
            bodies[i].linearVelocity = m_SolverBodies[i].linearVelocity;
            bodies[i].angularVelocity = m_SolverBodies[i].angularVelocity;
        }
    }
    
    void ConstraintSolver::ComputeEffectiveMass(VelocityConstraint& vc)
    {
        SolverBody& bodyA = m_SolverBodies[vc.indexA];
        SolverBody& bodyB = m_SolverBodies[vc.indexB];
        
        for (auto& vcp : vc.points)
        {
            // Normal effective mass
            float rnA = vcp.rA.x * vc.normal.y - vcp.rA.y * vc.normal.x;
            float rnB = vcp.rB.x * vc.normal.y - vcp.rB.y * vc.normal.x;
            
            float invMass = bodyA.invMass + bodyA.invInertia * rnA * rnA +
                           bodyB.invMass + bodyB.invInertia * rnB * rnB;
            
            vcp.normalMass = invMass > 0.0f ? 1.0f / invMass : 0.0f;
            
            // Tangent effective mass
            float rtA = vcp.rA.x * vc.tangent.y - vcp.rA.y * vc.tangent.x;
            float rtB = vcp.rB.x * vc.tangent.y - vcp.rB.y * vc.tangent.x;
            
            invMass = bodyA.invMass + bodyA.invInertia * rtA * rtA +
                     bodyB.invMass + bodyB.invInertia * rtB * rtB;
            
            vcp.tangentMass = invMass > 0.0f ? 1.0f / invMass : 0.0f;
        }
    }
}
