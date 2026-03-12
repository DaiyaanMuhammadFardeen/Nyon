#include "nyon/ecs/systems/PhysicsPipelineSystem.h"
#include "nyon/ecs/components/JointComponent.h"
#include "nyon/physics/ManifoldGenerator.h"
#include <chrono>
#include <algorithm>

// Debug logging macro - only output in debug builds
#ifdef _DEBUG
#define NYON_DEBUG_LOG(x) std::cerr << "[PhysicsPipeline] " << x << std::endl
#else
#define NYON_DEBUG_LOG(x)
#endif

namespace Nyon::ECS
{
void PhysicsPipelineSystem::Initialize(EntityManager& entityManager, ComponentStore& componentStore)
{
    m_ComponentStore = &componentStore;

    // Find physics world component
    m_ComponentStore->ForEachComponent<PhysicsWorldComponent>([&](EntityID entityId, PhysicsWorldComponent& world) {
        m_PhysicsWorld = &world;
        NYON_DEBUG_LOG("Found PhysicsWorldComponent at entity ID: " << entityId);
    });

    if (!m_PhysicsWorld)
    {
        NYON_DEBUG_LOG("Warning: No PhysicsWorldComponent found!");
        return;
    }

    // Initialize island manager
    m_IslandManager = std::make_unique<Physics::IslandManager>(*m_ComponentStore);

    NYON_DEBUG_LOG("PhysicsPipelineSystem initialized");
}

    void PhysicsPipelineSystem::Update(float deltaTime)
    {
        if (!m_PhysicsWorld || !m_ComponentStore)
            return;
            
        auto startTime = std::chrono::high_resolution_clock::now();
        
        // Fixed timestep accumulation
        m_Accumulator += deltaTime;
        m_Accumulator = std::min(m_Accumulator, MAX_TIMESTEP);
        
        // Process fixed timesteps
        while (m_Accumulator >= FIXED_TIMESTEP)
        {
            // Collect active entities
            m_ActiveEntities.clear();
            m_ComponentStore->ForEachComponent<PhysicsBodyComponent>([&](EntityID entityId, const PhysicsBodyComponent& body) {
                if (!body.isStatic) // Only process non-static bodies
                {
                    m_ActiveEntities.push_back(entityId);
                }
            });
            
            NYON_DEBUG_LOG("Processing " << m_ActiveEntities.size() << " active entities");
            
            // Execute pipeline phases
            PrepareBodiesForUpdate();
            BroadPhaseDetection();
            NarrowPhaseDetection();
            IslandDetection();
            ConstraintInitialization();
            VelocitySolving();
            PositionSolving();
            Integration();
            StoreImpulses();
            UpdateSleeping();
            UpdateTransformsFromSolver();
            
            m_Accumulator -= FIXED_TIMESTEP;
        }
        
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<float, std::milli>(endTime - startTime);
        m_Stats.updateTime = duration.count();
        
        NYON_DEBUG_LOG("Physics update completed in " << m_Stats.updateTime << " ms");
    }

    void PhysicsPipelineSystem::PrepareBodiesForUpdate()
    {
        m_SolverBodies.clear();
        m_EntityToSolverIndex.clear();
        
        size_t solverIndex = 0;
        m_ComponentStore->ForEachComponent<PhysicsBodyComponent>([&](EntityID entityId, PhysicsBodyComponent& body) {
            if (!m_IslandManager->IsBodyAwake(entityId) && !body.isStatic)
            {
                return; // Skip sleeping bodies
            }
            
            SolverBody solverBody;
            solverBody.entityId = entityId;
            solverBody.isStatic = body.isStatic;
            solverBody.isAwake = body.isStatic || m_IslandManager->IsBodyAwake(entityId);
            solverBody.invMass = body.inverseMass;
            solverBody.invInertia = body.inverseInertia;
            solverBody.localCenter = body.centerOfMass;
            
            // Get transform
            if (m_ComponentStore->HasComponent<TransformComponent>(entityId))
            {
                const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
                solverBody.position = transform.position;
                solverBody.angle = transform.rotation;
                solverBody.prevPosition = transform.previousPosition;
                solverBody.prevAngle = transform.previousRotation;
            }
            
            // Get current velocities
            solverBody.velocity = body.velocity;
            solverBody.angularVelocity = body.angularVelocity;
            
            // Initialize forces
            solverBody.force = Math::Vector2{0.0f, 0.0f};
            solverBody.torque = 0.0f;
            
            m_SolverBodies.push_back(solverBody);
            m_EntityToSolverIndex[entityId] = solverIndex++;
        });
        
        NYON_DEBUG_LOG("Prepared " << m_SolverBodies.size() << " solver bodies");
    }

    void PhysicsPipelineSystem::BroadPhaseDetection()
    {
        m_BroadPhasePairs.clear();
        m_ShapeProxyMap.clear();
        
        // Update broad phase tree and collect potential pairs
        m_ComponentStore->ForEachComponent<ColliderComponent>([&](EntityID entityId, ColliderComponent& collider) {
            if (!m_ComponentStore->HasComponent<TransformComponent>(entityId))
                return;
                
            const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
            
            // Update shape AABB in broad phase tree
            UpdateShapeAABB(entityId, &collider, transform.position, transform.rotation);
        });
        
        // Query broad phase for overlapping pairs
        BroadPhaseCallback callback;
        callback.system = this;
        
        for (const auto& [entityId, proxyId] : m_ShapeProxyMap)
        {
            callback.entityId = entityId;
            const auto& aabb = m_BroadPhaseTree.GetFatAABB(proxyId);
            m_BroadPhaseTree.Query(aabb, &callback);
        }
        
        m_Stats.broadPhasePairs = m_BroadPhasePairs.size();
        NYON_DEBUG_LOG("Broad phase found " << m_BroadPhasePairs.size() << " potential pairs");
    }

    void PhysicsPipelineSystem::NarrowPhaseDetection()
    {
        m_ContactManifolds.clear();
        m_ContactMap.clear();
        
        // Test each broad phase pair for actual collision
        for (const auto& [entityIdA, entityIdB] : m_BroadPhasePairs)
        {
            if (TestCollision(entityIdA, entityIdB))
            {
                ContactManifold manifold = GenerateManifold(entityIdA, entityIdB);
                if (!manifold.points.empty())
                {
                    // Store contact for constraint solving
                    uint64_t key = (static_cast<uint64_t>(std::min(entityIdA, entityIdB)) << 32) | 
                                  static_cast<uint64_t>(std::max(entityIdA, entityIdB));
                    m_ContactMap[key] = m_ContactManifolds.size();
                    m_ContactManifolds.push_back(std::move(manifold));
                }
            }
        }
        
        m_Stats.narrowPhaseContacts = m_ContactManifolds.size();
        NYON_DEBUG_LOG("Narrow phase generated " << m_ContactManifolds.size() << " contact manifolds");
    }

    void PhysicsPipelineSystem::IslandDetection()
    {
        if (m_Config.useIslandSleeping)
        {
            m_IslandManager->UpdateIslands(FIXED_TIMESTEP, m_ActiveEntities);
            m_Stats.islandStats = m_IslandManager->GetStatistics();
        }
    }

    void PhysicsPipelineSystem::ConstraintInitialization()
    {
        m_VelocityConstraints.clear();
        m_PositionConstraints.clear();
        
        // Create velocity constraints from contact manifolds
        for (const auto& manifold : m_ContactManifolds)
        {
            VelocityConstraint vc;
            vc.normal = manifold.normal;
            vc.tangent = Math::Vector2{-manifold.normal.y, manifold.normal.x};
            vc.points = manifold.points;
            vc.friction = 0.3f; // Default friction
            vc.restitution = 0.2f; // Default restitution
            
            // Get solver indices
            auto itA = m_EntityToSolverIndex.find(manifold.entityIdA);
            auto itB = m_EntityToSolverIndex.find(manifold.entityIdB);
            
            if (itA != m_EntityToSolverIndex.end() && itB != m_EntityToSolverIndex.end())
            {
                vc.indexA = itA->second;
                vc.indexB = itB->second;
                
                const auto& bodyA = m_SolverBodies[vc.indexA];
                const auto& bodyB = m_SolverBodies[vc.indexB];
                
                vc.invMassA = bodyA.invMass;
                vc.invMassB = bodyB.invMass;
                vc.invIA = bodyA.invInertia;
                vc.invIB = bodyB.invInertia;
                
                m_VelocityConstraints.push_back(vc);
            }
        }
        
        m_Stats.activeConstraints = m_VelocityConstraints.size();
        NYON_DEBUG_LOG("Initialized " << m_VelocityConstraints.size() << " velocity constraints");
    }

    void PhysicsPipelineSystem::VelocitySolving()
    {
        if (m_Config.warmStarting)
        {
            WarmStartConstraints();
        }
        
        // Apply gravity and other forces
        for (auto& body : m_SolverBodies)
        {
            if (!body.isStatic && body.isAwake && m_PhysicsWorld)
            {
                // Apply gravity as force: F = m * g
                float mass = (body.invMass > 0.0f) ? 1.0f / body.invMass : 0.0f;
                body.force += m_PhysicsWorld->gravity * mass;
            }
        }
        
        IntegrateVelocities(FIXED_TIMESTEP);
        
        // Solve velocity constraints iteratively
        for (int i = 0; i < m_Config.velocityIterations; ++i)
        {
            SolveVelocityConstraints();
        }
    }

    void PhysicsPipelineSystem::PositionSolving()
    {
        IntegratePositions(FIXED_TIMESTEP);
        
        // Solve position constraints for stabilization
        for (int i = 0; i < m_Config.positionIterations; ++i)
        {
            SolvePositionConstraints();
        }
    }

    void PhysicsPipelineSystem::Integration()
    {
        // Update body components with solved velocities and positions
        for (const auto& solverBody : m_SolverBodies)
        {
            if (m_ComponentStore->HasComponent<PhysicsBodyComponent>(solverBody.entityId))
            {
                auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(solverBody.entityId);
                body.velocity = solverBody.velocity;
                body.angularVelocity = solverBody.angularVelocity;
                // Damping is already applied in ConstraintSolverSystem - no additional damping here
            }
        }
    }

    void PhysicsPipelineSystem::StoreImpulses()
    {
        // Store accumulated impulses for warm starting next frame
        for (auto& constraint : m_VelocityConstraints)
        {
            for (auto& point : constraint.points)
            {
                // Preserve impulses at full value for effective warm starting
                // No decay - impulses should be maintained across frames
            }
        }
    }

    void PhysicsPipelineSystem::UpdateSleeping()
    {
        if (!m_Config.useIslandSleeping)
            return;
            
        // Update body sleeping states based on island manager
        m_ComponentStore->ForEachComponent<PhysicsBodyComponent>([&](EntityID entityId, PhysicsBodyComponent& body) {
            if (!body.isStatic)
            {
                body.isAwake = m_IslandManager->IsBodyAwake(entityId);
            }
        });
        
        m_Stats.awakeBodies = m_Stats.islandStats.awakeBodies;
        m_Stats.sleepingBodies = m_Stats.islandStats.sleepingBodies;
    }

    void PhysicsPipelineSystem::UpdateTransformsFromSolver()
    {
        // Update transform components from solver results
        for (const auto& solverBody : m_SolverBodies)
        {
            if (m_ComponentStore->HasComponent<TransformComponent>(solverBody.entityId))
            {
                auto& transform = m_ComponentStore->GetComponent<TransformComponent>(solverBody.entityId);
                transform.position = solverBody.position;
                transform.rotation = solverBody.angle;
            }
        }
    }

    // BroadPhaseCallback implementation
    bool PhysicsPipelineSystem::BroadPhaseCallback::QueryCallback(uint32_t nodeId, uint32_t userData)
    {
        uint32_t otherEntityId = userData;
        
        // Avoid self-collision and ensure canonical pair ordering
        if (otherEntityId <= entityId)
            return true;
            
        // Check if both entities have colliders and are not filtered
        if (system->m_ComponentStore->HasComponent<ColliderComponent>(entityId) &&
            system->m_ComponentStore->HasComponent<ColliderComponent>(otherEntityId))
        {
            const auto& colliderA = system->m_ComponentStore->GetComponent<ColliderComponent>(entityId);
            const auto& colliderB = system->m_ComponentStore->GetComponent<ColliderComponent>(otherEntityId);
            
            // Check filter settings
            if (colliderA.filter.ShouldCollide(colliderB.filter))
            {
                system->m_BroadPhasePairs.emplace_back(entityId, otherEntityId);
            }
        }
        
        return true; // Continue querying
    }

    void PhysicsPipelineSystem::UpdateShapeAABB(uint32_t entityId, ColliderComponent* collider, 
                                              const Math::Vector2& position, float angle)
    {
        Math::Vector2 min, max;
        collider->CalculateAABB(position, angle, min, max);
        
        Physics::AABB aabb;
        aabb.lowerBound = {min.x, min.y};
        aabb.upperBound = {max.x, max.y};
        
        // Expand AABB slightly for better broad phase performance
        Physics::AABB fatAABB = aabb;
        fatAABB.lowerBound.x -= 0.1f;
        fatAABB.lowerBound.y -= 0.1f;
        fatAABB.upperBound.x += 0.1f;
        fatAABB.upperBound.y += 0.1f;
        
        auto it = m_ShapeProxyMap.find(entityId);
        if (it != m_ShapeProxyMap.end())
        {
            // Update existing proxy with velocity-based displacement hint
            Math::Vector2 displacement = {0.0f, 0.0f};
            if (m_ComponentStore->HasComponent<PhysicsBodyComponent>(entityId)) {
                const auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityId);
                displacement = body.velocity * FIXED_TIMESTEP;
            }
            m_BroadPhaseTree.MoveProxy(it->second, fatAABB, displacement);
        }
        else
        {
            // Create new proxy
            uint32_t proxyId = m_BroadPhaseTree.CreateProxy(fatAABB, entityId);
            m_ShapeProxyMap[entityId] = proxyId;
        }
    }

    bool PhysicsPipelineSystem::TestCollision(uint32_t entityIdA, uint32_t entityIdB)
    {
        // Simple AABB overlap test for now
        if (!m_ComponentStore->HasComponent<ColliderComponent>(entityIdA) ||
            !m_ComponentStore->HasComponent<ColliderComponent>(entityIdB) ||
            !m_ComponentStore->HasComponent<TransformComponent>(entityIdA) ||
            !m_ComponentStore->HasComponent<TransformComponent>(entityIdB))
        {
            return false;
        }
        
        const auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(entityIdA);
        const auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(entityIdB);
        const auto& transformA = m_ComponentStore->GetComponent<TransformComponent>(entityIdA);
        const auto& transformB = m_ComponentStore->GetComponent<TransformComponent>(entityIdB);
        
        Math::Vector2 minA, maxA, minB, maxB;
        colliderA.CalculateAABB(transformA.position, transformA.rotation, minA, maxA);
        colliderB.CalculateAABB(transformB.position, transformB.rotation, minB, maxB);
        
        // AABB overlap test
        return !(minA.x >= maxB.x || maxA.x <= minB.x || minA.y >= maxB.y || maxA.y <= minB.y);
    }

    PhysicsPipelineSystem::ContactManifold PhysicsPipelineSystem::GenerateManifold(uint32_t entityIdA, uint32_t entityIdB)
    {
        ContactManifold manifold;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        
        if (!m_ComponentStore->HasComponent<ColliderComponent>(entityIdA) ||
            !m_ComponentStore->HasComponent<ColliderComponent>(entityIdB) ||
            !m_ComponentStore->HasComponent<TransformComponent>(entityIdA) ||
            !m_ComponentStore->HasComponent<TransformComponent>(entityIdB))
        {
            return manifold;
        }
        
        const auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(entityIdA);
        const auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(entityIdB);
        const auto& transformA = m_ComponentStore->GetComponent<TransformComponent>(entityIdA);
        const auto& transformB = m_ComponentStore->GetComponent<TransformComponent>(entityIdB);
        
        // Use proper ManifoldGenerator for accurate contact generation
        // Get first shape from each entity (assuming single-shape entities)
        uint32_t shapeIdA = 0;
        uint32_t shapeIdB = 0;
        
        ECS::ContactManifold generatedManifold = Physics::ManifoldGenerator::GenerateManifold(
            entityIdA, entityIdB,
            shapeIdA, shapeIdB,
            colliderA, colliderB,
            transformA, transformB
        );
        
        // Convert to internal ContactManifold format
        manifold.points.reserve(generatedManifold.points.size());
        manifold.normal = generatedManifold.normal;
        
        for (const auto& point : generatedManifold.points)
        {
            ContactPoint contactPoint;
            contactPoint.position = point.position;
            contactPoint.normal = point.normal;
            contactPoint.separation = point.separation;
            contactPoint.normalImpulse = 0.0f;
            contactPoint.tangentImpulse = 0.0f;
            manifold.points.push_back(contactPoint);
        }
        
        return manifold;
    }

    void PhysicsPipelineSystem::WarmStartConstraints()
    {
        for (auto& constraint : m_VelocityConstraints)
        {
            const auto& bodyA = m_SolverBodies[constraint.indexA];
            const auto& bodyB = m_SolverBodies[constraint.indexB];
            
            for (auto& point : constraint.points)
            {
                // Apply previous impulses
                Math::Vector2 P = point.normal * point.normalImpulse + 
                                constraint.tangent * point.tangentImpulse;
                
                if (!bodyA.isStatic)
                {
                    m_SolverBodies[constraint.indexA].velocity -= P * constraint.invMassA;
                    m_SolverBodies[constraint.indexA].angularVelocity -= 
                        constraint.invIA * Math::Vector2::Cross(point.position - bodyA.position, P);
                }
                
                if (!bodyB.isStatic)
                {
                    m_SolverBodies[constraint.indexB].velocity += P * constraint.invMassB;
                    m_SolverBodies[constraint.indexB].angularVelocity += 
                        constraint.invIB * Math::Vector2::Cross(point.position - bodyB.position, P);
                }
            }
        }
    }

    void PhysicsPipelineSystem::SolveVelocityConstraints()
    {
        for (auto& constraint : m_VelocityConstraints)
        {
            auto& bodyA = m_SolverBodies[constraint.indexA];
            auto& bodyB = m_SolverBodies[constraint.indexB];
            
            Math::Vector2 vA = bodyA.velocity;
            Math::Vector2 vB = bodyB.velocity;
            float wA = bodyA.angularVelocity;
            float wB = bodyB.angularVelocity;
            
            for (auto& point : constraint.points)
            {
                // Relative velocity at contact point
                Math::Vector2 rA = point.position - bodyA.position;
                Math::Vector2 rB = point.position - bodyB.position;
                
                Math::Vector2 dv = vB + Math::Vector2::Cross(wB, rB) - vA - Math::Vector2::Cross(wA, rA);
                
                // Normal impulse
                float vn = Math::Vector2::Dot(dv, constraint.normal);
                float impulse = -point.normalMass * (vn - point.velocityBias);
                
                // Clamp impulse
                float oldImpulse = point.normalImpulse;
                point.normalImpulse = std::max(oldImpulse + impulse, 0.0f);
                impulse = point.normalImpulse - oldImpulse;
                
                // Apply impulse
                Math::Vector2 P = constraint.normal * impulse;
                
                if (!bodyA.isStatic)
                {
                    bodyA.velocity -= P * constraint.invMassA;
                    bodyA.angularVelocity -= constraint.invIA * Math::Vector2::Cross(rA, P);
                }
                
                if (!bodyB.isStatic)
                {
                    bodyB.velocity += P * constraint.invMassB;
                    bodyB.angularVelocity += constraint.invIB * Math::Vector2::Cross(rB, P);
                }
            }
        }
    }

    void PhysicsPipelineSystem::SolvePositionConstraints()
    {
        for (const auto& constraint : m_VelocityConstraints)
        {
            auto& bodyA = m_SolverBodies[constraint.indexA];
            auto& bodyB = m_SolverBodies[constraint.indexB];
            
            for (const auto& point : constraint.points)
            {
                if (point.separation < -m_Config.linearSlop)
                {
                    Math::Vector2 rA = point.position - bodyA.position;
                    Math::Vector2 rB = point.position - bodyB.position;
                    
                    // Position correction
                    float C = m_Config.baumgarte * (-point.separation - m_Config.linearSlop);
                    C = std::min(C, m_Config.maxLinearCorrection);
                    
                    float mass = constraint.invMassA + constraint.invMassB;
                    if (mass > 0.0f)
                    {
                        mass = 1.0f / mass;
                    }
                    
                    Math::Vector2 P = constraint.normal * (C * mass);
                    
                    if (!bodyA.isStatic)
                    {
                        bodyA.position -= P * constraint.invMassA;
                    }
                    
                    if (!bodyB.isStatic)
                    {
                        bodyB.position += P * constraint.invMassB;
                    }
                }
            }
        }
    }

    void PhysicsPipelineSystem::IntegrateVelocities(float dt)
    {
        for (auto& body : m_SolverBodies)
        {
            if (body.isStatic || !body.isAwake)
                continue;
                
            // Integrate linear velocity
            body.velocity += (body.force * body.invMass) * dt;
            
            // Integrate angular velocity
            body.angularVelocity += (body.torque * body.invInertia) * dt;
            
            // Clear forces for next step
            body.force = Math::Vector2{0.0f, 0.0f};
            body.torque = 0.0f;
        }
    }

    void PhysicsPipelineSystem::IntegratePositions(float dt)
    {
        for (auto& body : m_SolverBodies)
        {
            if (body.isStatic || !body.isAwake)
                continue;
                
            // Store previous position for interpolation
            body.prevPosition = body.position;
            body.prevAngle = body.angle;
            
            // Integrate position
            body.position += body.velocity * dt;
            
            // Integrate angle
            body.angle += body.angularVelocity * dt;
        }
    }

    void PhysicsPipelineSystem::ClearPersistentContacts()
    {
        // Mark all contacts as non-persistent for next frame
        for (auto& manifold : m_ContactManifolds)
        {
            manifold.persisted = false;
        }
    }
}