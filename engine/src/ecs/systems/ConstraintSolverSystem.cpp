#include "nyon/ecs/systems/ConstraintSolverSystem.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/components/TransformComponent.h"
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <chrono>

namespace Nyon::ECS
{
    void ConstraintSolverSystem::Initialize(EntityManager& entityManager, ComponentStore& componentStore)
    {
        // Store reference to component store for later use
        m_ComponentStore = &componentStore;
        
        // Get physics world component
        const auto& worldEntities = componentStore.GetEntitiesWithComponent<PhysicsWorldComponent>();
        if (!worldEntities.empty())
        {
            m_PhysicsWorld = &componentStore.GetComponent<PhysicsWorldComponent>(worldEntities[0]);
        }
        
        // No pointer caching - query fresh each Update() call
        // This prevents dangling pointers when new entities are added
    }
    
    void ConstraintSolverSystem::Update(float deltaTime)
    {
        if (!m_PhysicsWorld)
            return;
            
        // Query fresh entities each frame to avoid stale pointers
        const auto& bodyEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsBodyComponent>();
        
        // Filter active bodies
        std::vector<const PhysicsBodyComponent*> activeBodies;
        std::vector<uint32_t> activeEntityIds;
        
        for (auto entityId : bodyEntities)
        {
            const auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityId);
            if (body.ShouldCollide())
            {
                activeBodies.push_back(&body);
                activeEntityIds.push_back(entityId);
            }
        }
        
        if (activeBodies.empty())
            return;
            
        // Prepare solver bodies
        m_SolverBodies.resize(activeBodies.size());
        for (size_t i = 0; i < activeBodies.size(); ++i)
        {
            const auto* body = activeBodies[i];
            auto& solverBody = m_SolverBodies[i];
            uint32_t entityId = activeEntityIds[i];
            
            // Read transform for initial position/rotation if available.
            if (m_ComponentStore->HasComponent<TransformComponent>(entityId))
            {
                const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
                solverBody.position = transform.position;
                solverBody.angle = transform.rotation;
            }
            else
            {
                solverBody.position = {0.0f, 0.0f};
                solverBody.angle = 0.0f;
            }
            solverBody.velocity = body->velocity;
            solverBody.angularVelocity = body->angularVelocity;
            solverBody.invMass = body->inverseMass;
            solverBody.invInertia = body->inverseInertia;
            solverBody.localCenter = body->centerOfMass;
            solverBody.isStatic = body->isStatic;
            solverBody.isKinematic = body->isKinematic;
            solverBody.entityId = entityId;
        }
        
        // Initialize constraints
        InitializeConstraints();
        
        // Warm start if enabled
        if (m_PhysicsWorld->enableWarmStarting)
        {
            WarmStart();
        }
        
        // Integrate velocities
        IntegrateVelocities(deltaTime);
        
        // Solve velocity constraints
        SolveVelocityConstraints();
        
        // Integrate positions
        IntegratePositions(deltaTime);
        
        // Solve position constraints
        SolvePositionConstraints();
        
        // Store results back to components
        for (size_t i = 0; i < activeBodies.size(); ++i)
        {
            auto entityId = activeEntityIds[i];
            auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityId);
            const auto& solverBody = m_SolverBodies[i];
            
            body.velocity = solverBody.velocity;
            body.angularVelocity = solverBody.angularVelocity;
            
            // Clamp speeds
            float linearSpeedSq = body.velocity.x * body.velocity.x + body.velocity.y * body.velocity.y;
            if (linearSpeedSq > body.maxLinearSpeed * body.maxLinearSpeed)
            {
                float scale = body.maxLinearSpeed / sqrt(linearSpeedSq);
                body.velocity = body.velocity * scale;
            }
            
            if (std::abs(body.angularVelocity) > body.maxAngularSpeed)
            {
                body.angularVelocity = (body.angularVelocity > 0) ? 
                    body.maxAngularSpeed : -body.maxAngularSpeed;
            }
        }
    }
    
    void ConstraintSolverSystem::InitializeConstraints()
    {
        // Clear existing constraints
        m_VelocityConstraints.clear();
        m_PositionConstraints.clear();
        
        if (!m_PhysicsWorld)
            return;

        const auto& manifolds = m_PhysicsWorld->contactManifolds;
        if (manifolds.empty())
            return;

        // Map entity IDs to solver body indices.
        std::unordered_map<uint32_t, uint32_t> entityToIndex;
        entityToIndex.reserve(m_SolverBodies.size());
        for (uint32_t i = 0; i < m_SolverBodies.size(); ++i)
        {
            entityToIndex[m_SolverBodies[i].entityId] = i;
        }

        m_VelocityConstraints.reserve(manifolds.size());
        m_PositionConstraints.reserve(manifolds.size());

        for (const auto& manifold : manifolds)
        {
            if (!manifold.touching || manifold.points.empty())
                continue;

            auto itA = entityToIndex.find(manifold.entityIdA);
            auto itB = entityToIndex.find(manifold.entityIdB);
            if (itA == entityToIndex.end() || itB == entityToIndex.end())
                continue;

            uint32_t indexA = itA->second;
            uint32_t indexB = itB->second;

            auto& bodyA = m_SolverBodies[indexA];
            auto& bodyB = m_SolverBodies[indexB];

            ContactVelocityConstraint vc{};
            vc.indexA = indexA;
            vc.indexB = indexB;
            vc.normal = manifold.normal;
            vc.tangent = Math::Vector2(-manifold.normal.y, manifold.normal.x);
            vc.invMassA = bodyA.invMass;
            vc.invMassB = bodyB.invMass;
            vc.invIA = bodyA.invInertia;
            vc.invIB = bodyB.invInertia;
            vc.friction = manifold.friction;
            vc.restitution = manifold.restitution;

            ContactPositionConstraint pc{};
            pc.indexA = indexA;
            pc.indexB = indexB;
            pc.localNormal = manifold.localNormal;
            pc.localPoint = manifold.localPoint;
            pc.invMassA = bodyA.invMass;
            pc.invMassB = bodyB.invMass;
            pc.localCenterA = bodyA.localCenter;
            pc.localCenterB = bodyB.localCenter;
            pc.invIA = bodyA.invInertia;
            pc.invIB = bodyB.invInertia;

            const int pointCount = static_cast<int>(std::min<size_t>(manifold.points.size(), 2));
            vc.pointCount = pointCount;
            pc.pointCount = pointCount;

            for (int j = 0; j < pointCount; ++j)
            {
                const auto& mp = manifold.points[j];
                auto& vcp = vc.points[j];

                // Relative contact position from centers of mass.
                Math::Vector2 rA = mp.position - (bodyA.position + bodyA.localCenter);
                Math::Vector2 rB = mp.position - (bodyB.position + bodyB.localCenter);

                vcp.rA = rA;
                vcp.rB = rB;
                vcp.normalImpulse = mp.normalImpulse;
                vcp.tangentImpulse = mp.tangentImpulse;

                // Effective mass for normal axis.
                float rnA = Math::Vector2::Cross(rA, vc.normal);
                float rnB = Math::Vector2::Cross(rB, vc.normal);
                float kNormal = vc.invMassA + vc.invMassB + vc.invIA * rnA * rnA + vc.invIB * rnB * rnB;
                vcp.normalMass = (kNormal > 0.0f) ? 1.0f / kNormal : 0.0f;

                // Effective mass for tangent axis (friction).
                float rtA = Math::Vector2::Cross(rA, vc.tangent);
                float rtB = Math::Vector2::Cross(rB, vc.tangent);
                float kTangent = vc.invMassA + vc.invMassB + vc.invIA * rtA * rtA + vc.invIB * rtB * rtB;
                vcp.tangentMass = (kTangent > 0.0f) ? 1.0f / kTangent : 0.0f;

                // Compute velocity bias for restitution (bouncing)
                // Only apply restitution if collision is energetic enough
                vcp.velocityBias = 0.0f;
                
                // Store local-space contact point for position constraints.
                pc.localPoints[j] = mp.position;
            }

            m_VelocityConstraints.push_back(vc);
            m_PositionConstraints.push_back(pc);
        }
    }
    
    void ConstraintSolverSystem::WarmStart()
    {
        // Apply previous impulses to warm start the solver
        for (auto& vc : m_VelocityConstraints)
        {
            auto& bodyA = m_SolverBodies[vc.indexA];
            auto& bodyB = m_SolverBodies[vc.indexB];
            
            for (int j = 0; j < vc.pointCount; ++j)
            {
                auto& vcp = vc.points[j];
                Math::Vector2 P = vc.normal * vcp.normalImpulse + vc.tangent * vcp.tangentImpulse;
                
                if (!bodyA.isStatic)
                {
                    bodyA.velocity = bodyA.velocity - P * vc.invMassA;
                    bodyA.angularVelocity = bodyA.angularVelocity - vc.invIA * (vcp.rA.x * P.y - vcp.rA.y * P.x);
                }
                
                if (!bodyB.isStatic)
                {
                    bodyB.velocity = bodyB.velocity + P * vc.invMassB;
                    bodyB.angularVelocity = bodyB.angularVelocity + vc.invIB * (vcp.rB.x * P.y - vcp.rB.y * P.x);
                }
            }
        }
    }
    
    void ConstraintSolverSystem::IntegrateVelocities(float dt)
    {
        float inv_dt = (dt > 0.0f) ? 1.0f / dt : 0.0f;
        
        for (auto& body : m_SolverBodies)
        {
            if (body.isStatic || body.isKinematic) continue;
            
            // Apply forces accumulated in PhysicsIntegrationSystem
            // F = ma, so a = F/m
            Math::Vector2 acceleration = {0.0f, 0.0f};
            
            // Gravity is applied through force accumulation in PhysicsIntegrationSystem
            // Here we only apply velocity-level damping for stability
            
            // Linear damping (air resistance)
            float linearDamping = 0.999f; // Small damping for numerical stability
            body.velocity = body.velocity * linearDamping;
            
            // Angular damping (rotational air resistance)
            float angularDamping = 0.995f;
            body.angularVelocity = body.angularVelocity * angularDamping;
        }
    }
    
    void ConstraintSolverSystem::SolveVelocityConstraints()
    {
        // Sequential impulse solver with friction and angular momentum
        for (int i = 0; i < m_PhysicsWorld->velocityIterations; ++i)
        {
            for (auto& vc : m_VelocityConstraints)
            {
                auto& bodyA = m_SolverBodies[vc.indexA];
                auto& bodyB = m_SolverBodies[vc.indexB];
                
                Math::Vector2 vA = bodyA.velocity;
                float wA = bodyA.angularVelocity;
                Math::Vector2 vB = bodyB.velocity;
                float wB = bodyB.angularVelocity;
                
                float mA = vc.invMassA;
                float mB = vc.invMassB;
                float iA = vc.invIA;
                float iB = vc.invIB;
                
                // Solve each contact point
                for (int j = 0; j < vc.pointCount; ++j)
                {
                    auto& vcp = vc.points[j];
                    
                    // Relative velocity at contact point (including angular components)
                    Math::Vector2 dv = vB + Math::Vector2{-wB * vcp.rB.y, wB * vcp.rB.x} -
                                      vA - Math::Vector2{-wA * vcp.rA.y, wA * vcp.rA.x};
                    
                    // === NORMAL IMPULSE ===
                    float vn = dv.x * vc.normal.x + dv.y * vc.normal.y;
                    float normalImpulse = -vcp.normalMass * (vn - vcp.velocityBias);
                    
                    // Clamp normal impulse (only pushing, not pulling)
                    float oldNormalImpulse = vcp.normalImpulse;
                    vcp.normalImpulse = std::max(oldNormalImpulse + normalImpulse, 0.0f);
                    normalImpulse = vcp.normalImpulse - oldNormalImpulse;
                    
                    // Apply normal impulse
                    Math::Vector2 normalP = vc.normal * normalImpulse;
                    
                    if (!bodyA.isStatic)
                    {
                        vA = vA - normalP * mA;
                        wA = wA - iA * Math::Vector2::Cross(vcp.rA, normalP);
                    }
                    
                    if (!bodyB.isStatic)
                    {
                        vB = vB + normalP * mB;
                        wB = wB + iB * Math::Vector2::Cross(vcp.rB, normalP);
                    }
                    
                    // === TANGENT/FRICTION IMPULSE ===
                    // Compute relative velocity in tangent direction
                    float vt = dv.x * vc.tangent.x + dv.y * vc.tangent.y;
                    
                    // Compute friction impulse using Coulomb friction model
                    float tangentImpulse = -vcp.tangentMass * vt;
                    
                    // Apply Coulomb friction cone constraint
                    float maxFriction = vc.friction * vcp.normalImpulse;
                    tangentImpulse = std::clamp(tangentImpulse, -maxFriction, maxFriction);
                    
                    // Store accumulated tangent impulse
                    float oldTangentImpulse = vcp.tangentImpulse;
                    vcp.tangentImpulse = oldTangentImpulse + tangentImpulse;
                    tangentImpulse = vcp.tangentImpulse - oldTangentImpulse;
                    
                    // Apply friction impulse
                    Math::Vector2 tangentP = vc.tangent * tangentImpulse;
                    
                    if (!bodyA.isStatic)
                    {
                        vA = vA - tangentP * mA;
                        wA = wA - iA * Math::Vector2::Cross(vcp.rA, tangentP);
                    }
                    
                    if (!bodyB.isStatic)
                    {
                        vB = vB + tangentP * mB;
                        wB = wB + iB * Math::Vector2::Cross(vcp.rB, tangentP);
                    }
                }
                
                bodyA.velocity = vA;
                bodyA.angularVelocity = wA;
                bodyB.velocity = vB;
                bodyB.angularVelocity = wB;
            }
        }
    }
    
    void ConstraintSolverSystem::IntegratePositions(float dt)
    {
        for (auto& body : m_SolverBodies)
        {
            if (body.isStatic) continue;
            
            body.position = body.position + body.velocity * dt;
            body.angle = body.angle + body.angularVelocity * dt;
        }
    }
    
    void ConstraintSolverSystem::SolvePositionConstraints()
    {
        // Baumgarte stabilization with rotational correction
        if (m_VelocityConstraints.empty())
            return;

        const float linearSlop = 0.005f;
        const float maxCorrection = 0.2f;
        const float angularSlop = 0.01f; // ~0.5 degrees
        const float maxAngularCorrection = 0.1f; // ~5.7 degrees

        for (int it = 0; it < m_PhysicsWorld->positionIterations; ++it)
        {
            bool allOK = true;

            for (auto& vc : m_VelocityConstraints)
            {
                auto& bodyA = m_SolverBodies[vc.indexA];
                auto& bodyB = m_SolverBodies[vc.indexB];

                for (int j = 0; j < vc.pointCount; ++j)
                {
                    auto& vcp = vc.points[j];

                    // World-space contact points using current positions
                    Math::Vector2 pA = bodyA.position + bodyA.localCenter + vcp.rA;
                    Math::Vector2 pB = bodyB.position + bodyB.localCenter + vcp.rB;
                    Math::Vector2 d = pB - pA;
                    
                    // Separation distance (negative = penetration)
                    float separation = Math::Vector2::Dot(d, vc.normal);

                    // Only correct when there is noticeable penetration
                    float C = std::min(separation + linearSlop, 0.0f);
                    if (C == 0.0f)
                        continue;

                    allOK = false;

                    // Compute positional correction magnitude
                    float correctionMag = std::clamp(-C, 0.0f, maxCorrection);
                    Math::Vector2 correction = vc.normal * correctionMag;

                    // Apply linear position correction
                    float totalInvMass = vc.invMassA + vc.invMassB;
                    if (totalInvMass > 0.0f)
                    {
                        if (!bodyA.isStatic)
                        {
                            bodyA.position = bodyA.position - correction * (vc.invMassA / totalInvMass);
                        }
                        if (!bodyB.isStatic)
                        {
                            bodyB.position = bodyB.position + correction * (vc.invMassB / totalInvMass);
                        }
                    }

                    // Apply rotational correction to reduce angular penetration
                    // This helps objects rotate away from each other when they're interpenetrating
                    if (!bodyA.isStatic && vc.invIA > 0.0f)
                    {
                        float angularCorrection = Math::Vector2::Cross(vcp.rA, correction) * vc.invIA * 0.1f;
                        angularCorrection = std::clamp(angularCorrection, -maxAngularCorrection, maxAngularCorrection);
                        bodyA.angle -= angularCorrection;
                    }
                    
                    if (!bodyB.isStatic && vc.invIB > 0.0f)
                    {
                        float angularCorrection = Math::Vector2::Cross(vcp.rB, correction) * vc.invIB * 0.1f;
                        angularCorrection = std::clamp(angularCorrection, -maxAngularCorrection, maxAngularCorrection);
                        bodyB.angle += angularCorrection;
                    }
                }
            }

            if (allOK)
                break;
        }
    }
    
    void ConstraintSolverSystem::StoreImpulses()
    {
        // Store computed impulses back to manifolds for warm starting next frame
        if (!m_PhysicsWorld)
            return;
        
        // Update contact manifolds with solved impulses
        for (auto& vc : m_VelocityConstraints)
        {
            // Find corresponding manifold
            auto& bodyA = m_SolverBodies[vc.indexA];
            auto& bodyB = m_SolverBodies[vc.indexB];
            
            for (auto& manifold : m_PhysicsWorld->contactManifolds)
            {
                if (manifold.entityIdA == bodyA.entityId && 
                    manifold.entityIdB == bodyB.entityId)
                {
                    // Store impulses back to contact points
                    for (int j = 0; j < vc.pointCount && j < static_cast<int>(manifold.points.size()); ++j)
                    {
                        manifold.points[j].normalImpulse = vc.points[j].normalImpulse;
                        manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse;
                    }
                    break;
                }
            }
        }
    }
}
