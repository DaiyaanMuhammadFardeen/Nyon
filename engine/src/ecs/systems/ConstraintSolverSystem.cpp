#include "nyon/ecs/systems/ConstraintSolverSystem.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"

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
            
            solverBody.position = {0.0f, 0.0f}; // Get from TransformComponent
            solverBody.angle = 0.0f;            // Get from TransformComponent
            solverBody.velocity = body->velocity;
            solverBody.angularVelocity = body->angularVelocity;
            solverBody.invMass = body->inverseMass;
            solverBody.invInertia = body->inverseInertia;
            solverBody.localCenter = body->centerOfMass;
            solverBody.isStatic = body->isStatic;
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
        
        // For now, leave constraint lists empty as placeholder
        // In a complete implementation, this would:
        // 1. Query active contacts from collision detection
        // 2. Create velocity and position constraints for each contact
        // 3. Initialize constraint parameters (mass, bias, etc.)
        
        // TODO: Implement proper contact constraint initialization
        // This requires access to contact manifolds and contact points
        // from the collision detection system
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
            if (body.isStatic) continue;
            
            // Gravity is already applied in PhysicsIntegrationSystem
            // Only apply damping for stability
            body.velocity = body.velocity * 0.999f; // Small damping for stability
        }
    }
    
    void ConstraintSolverSystem::SolveVelocityConstraints()
    {
        // Sequential impulse solver iterations
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
                
                // Solve normal constraints
                for (int j = 0; j < vc.pointCount; ++j)
                {
                    auto& vcp = vc.points[j];
                    
                    // Relative velocity at contact point
                    Math::Vector2 dv = vB + Math::Vector2{-wB * vcp.rB.y, wB * vcp.rB.x} -
                                      vA - Math::Vector2{-wA * vcp.rA.y, wA * vcp.rA.x};
                    
                    // Compute normal impulse
                    float vn = dv.x * vc.normal.x + dv.y * vc.normal.y;
                    float impulse = -vcp.normalMass * (vn - vcp.velocityBias);
                    
                    // Clamp impulse
                    float oldImpulse = vcp.normalImpulse;
                    vcp.normalImpulse = std::max(oldImpulse + impulse, 0.0f);
                    impulse = vcp.normalImpulse - oldImpulse;
                    
                    // Apply impulse
                    Math::Vector2 P = vc.normal * impulse;
                    
                    if (!bodyA.isStatic)
                    {
                        vA = vA - P * mA;
                        wA = wA - iA * (vcp.rA.x * P.y - vcp.rA.y * P.x);
                    }
                    
                    if (!bodyB.isStatic)
                    {
                        vB = vB + P * mB;
                        wB = wB + iB * (vcp.rB.x * P.y - vcp.rB.y * P.x);
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
        float k_tolerance = 0.01f;
        bool positionsOkay = true;
        
        for (int i = 0; i < m_PhysicsWorld->positionIterations; ++i)
        {
            float minSeparation = 0.0f;
            
            for (const auto& pc : m_PositionConstraints)
            {
                // Position constraint solving would go here
                // This is a placeholder implementation
                minSeparation = std::min(minSeparation, -0.001f);
            }
            
            if (minSeparation >= -k_tolerance)
            {
                positionsOkay = true;
                break;
            }
        }
    }
    
    void ConstraintSolverSystem::StoreImpulses()
    {
        // Store computed impulses for warm starting next frame
        // This will be implemented when we have the full contact system
    }
}
