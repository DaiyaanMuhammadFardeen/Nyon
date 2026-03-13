#include "nyon/ecs/systems/PhysicsPipelineSystem.h"
#include "nyon/ecs/components/JointComponent.h"
#include "nyon/physics/ManifoldGenerator.h"
#include <chrono>
#include <algorithm>
#include <iostream>

// ALWAYS enabled debug logging (not dependent on _DEBUG)
#define NYON_DEBUG_LOG(x) std::cerr << "[PHYSICS] " << x << std::endl

// Rate-limited debug logging (outputs every N frames to avoid flooding)
static int s_DebugFrameCounter = 0;
static constexpr int DEBUG_OUTPUT_INTERVAL = 25; // Output every 25 frames (~417ms at 60fps)
#define NYON_DEBUG_LOG_EVERY(x) \
    do { \
        if (++s_DebugFrameCounter >= DEBUG_OUTPUT_INTERVAL) { \
            s_DebugFrameCounter = 0; \
            std::cerr << "[PHYSICS@" << s_DebugFrameCounter << "] " << x << std::endl; \
        } \
    } while(0)

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
        // Lazy initialization - find PhysicsWorldComponent if not already found
        if (!m_PhysicsWorld && m_ComponentStore)
        {
            m_ComponentStore->ForEachComponent<PhysicsWorldComponent>([&](EntityID entityId, PhysicsWorldComponent& world) {
                m_PhysicsWorld = &world;
                NYON_DEBUG_LOG("Found PhysicsWorldComponent at entity ID: " << entityId);
                
                // Initialize island manager now that we have a physics world
                if (!m_IslandManager)
                {
                    m_IslandManager = std::make_unique<Physics::IslandManager>(*m_ComponentStore);
                    NYON_DEBUG_LOG("IslandManager initialized via lazy init");
                }
            });
        }
        
        NYON_DEBUG_LOG("[PHYSICS] Update() called with deltaTime=" << deltaTime);
        
        if (!m_PhysicsWorld || !m_ComponentStore)
        {
            NYON_DEBUG_LOG_EVERY("[PHYSICS] Early exit - m_PhysicsWorld=" << (m_PhysicsWorld ? "valid" : "null") 
                          << ", m_ComponentStore=" << (m_ComponentStore ? "valid" : "null"));
            return;
        }
        
        // Safety check: ensure IslandManager is initialized
        if (!m_IslandManager)
        {
            NYON_DEBUG_LOG("Initializing IslandManager...");
            m_IslandManager = std::make_unique<Physics::IslandManager>(*m_ComponentStore);
        }
            
        auto startTime = std::chrono::high_resolution_clock::now();
        
        // Fixed timestep accumulation
        m_Accumulator += deltaTime;
        m_Accumulator = std::min(m_Accumulator, MAX_TIMESTEP);
        
        NYON_DEBUG_LOG("[PHYSICS] Accumulator=" << m_Accumulator << ", FIXED_TIMESTEP=" << FIXED_TIMESTEP);
        
        // Process fixed timesteps
        int stepCount = 0;
        while (m_Accumulator >= FIXED_TIMESTEP)
        {
            stepCount++;
            NYON_DEBUG_LOG("[PHYSICS] Processing physics step #" << stepCount);
            
            // Collect active entities
            m_ActiveEntities.clear();
            m_ComponentStore->ForEachComponent<PhysicsBodyComponent>([&](EntityID entityId, const PhysicsBodyComponent& body) {
                if (!body.isStatic) // Only process non-static bodies
                {
                    m_ActiveEntities.push_back(entityId);
                }
            });
            
            NYON_DEBUG_LOG("[PHYSICS] Found " << m_ActiveEntities.size() << " active entities");
            
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
        
        NYON_DEBUG_LOG("[PHYSICS] Physics update completed in " << m_Stats.updateTime << " ms, processed " << stepCount << " steps");
    }

    void PhysicsPipelineSystem::PrepareBodiesForUpdate()
    {
        m_SolverBodies.clear();
        m_EntityToSolverIndex.clear();
        
        size_t solverIndex = 0;
        m_ComponentStore->ForEachComponent<PhysicsBodyComponent>([&](EntityID entityId, PhysicsBodyComponent& body) {
            // Determine if body should be included in solver
            bool shouldInclude = false;
            
            if (body.isStatic)
            {
                // Static bodies are always included
                shouldInclude = true;
            }
            else
            {
                // For dynamic bodies, check both island manager and component's own awake state
                // This ensures newly created bodies (not yet in island manager) are still processed
                bool islandAwake = m_IslandManager->IsBodyAwake(entityId);
                bool componentAwake = body.isAwake;
                
                // Include if either says awake (treat unknown bodies as awake)
                shouldInclude = islandAwake || componentAwake;
            }
            
            if (!shouldInclude)
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
        
        NYON_DEBUG_LOG_EVERY("Prepared " << m_SolverBodies.size() << " solver bodies:");
        for (size_t i = 0; i < std::min(m_SolverBodies.size(), (size_t)5); ++i)
        {
            const auto& body = m_SolverBodies[i];
            NYON_DEBUG_LOG_EVERY("  Body[" << i << "] entity=" << body.entityId 
                              << " pos=(" << body.position.x << "," << body.position.y 
                              << ") vel=(" << body.velocity.x << "," << body.velocity.y 
                              << ") invMass=" << body.invMass 
                              << " static=" << body.isStatic 
                              << " awake=" << body.isAwake);
        }
    }

    void PhysicsPipelineSystem::BroadPhaseDetection()
    {
        m_BroadPhasePairs.clear();
        
        // DON'T clear m_ShapeProxyMap - we need to preserve proxy IDs across frames
        // Only remove proxies for entities that no longer have colliders
        std::vector<uint32_t> entitiesToRemove;
        for (const auto& [entityId, proxyId] : m_ShapeProxyMap)
        {
            if (!m_ComponentStore->HasComponent<ColliderComponent>(entityId))
            {
                entitiesToRemove.push_back(entityId);
            }
        }
        
        for (uint32_t entityId : entitiesToRemove)
        {
            uint32_t proxyId = m_ShapeProxyMap[entityId];
            m_BroadPhaseTree.DestroyProxy(proxyId);
            m_ShapeProxyMap.erase(entityId);
        }
        
        NYON_DEBUG_LOG_EVERY("BroadPhaseDetection: " << m_ShapeProxyMap.size() << " proxies in map, " 
                          << m_ActiveEntities.size() << " active entities");
        
        // Update broad phase tree and collect potential pairs
        m_ComponentStore->ForEachComponent<ColliderComponent>([&](EntityID entityId, ColliderComponent& collider) {
            if (!m_ComponentStore->HasComponent<TransformComponent>(entityId))
                return;
                
            const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
            
            // Debug: Log entity position
            NYON_DEBUG_LOG_EVERY("  Entity " << entityId 
                              << " pos=(" << transform.position.x << "," << transform.position.y << ")"
                              << " colliderType=" << (int)collider.type);
            
            // Update shape AABB in broad phase tree
            UpdateShapeAABB(entityId, &collider, transform.position, transform.rotation);
        });
        
        // Query broad phase for overlapping pairs
        BroadPhaseCallback callback;
        callback.system = this;
        
        // Collect all proxy-entity pairs first to avoid iterator invalidation
        std::vector<std::pair<uint32_t, uint32_t>> proxyPairs(m_ShapeProxyMap.begin(), m_ShapeProxyMap.end());
        
        // MANUAL OVERLAP TEST - brute force check all pairs
        for (size_t i = 0; i < proxyPairs.size(); ++i)
        {
            for (size_t j = i + 1; j < proxyPairs.size(); ++j)
            {
                const auto& [entityA, proxyA] = proxyPairs[i];
                const auto& [entityB, proxyB] = proxyPairs[j];
                    
                const auto& aabbA = m_BroadPhaseTree.GetFatAABB(proxyA);
                const auto& aabbB = m_BroadPhaseTree.GetFatAABB(proxyB);
                    
                bool overlaps = !(aabbA.upperBound.x <= aabbB.lowerBound.x || 
                                aabbA.lowerBound.x >= aabbB.upperBound.x ||
                                aabbA.upperBound.y <= aabbB.lowerBound.y || 
                                aabbA.lowerBound.y >= aabbB.upperBound.y);
                    
                if (overlaps)
                {
                    m_BroadPhasePairs.emplace_back(entityA, entityB);
                    std::cerr << "[PHYSICS] COLLISION DETECTED: entities " << entityA << " and " << entityB << std::endl;
                }
            }
        }
        
        m_Stats.broadPhasePairs = m_BroadPhasePairs.size();
        NYON_DEBUG_LOG_EVERY("Broad phase found " << m_BroadPhasePairs.size() << " potential pairs");
        if (!m_BroadPhasePairs.empty())
        {
            for (size_t i = 0; i < std::min(m_BroadPhasePairs.size(), (size_t)5); ++i)
            {
                const auto& [entityA, entityB] = m_BroadPhasePairs[i];
                NYON_DEBUG_LOG_EVERY("  Pair[" << i << "] entityA=" << entityA << " entityB=" << entityB);
            }
        }
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
        NYON_DEBUG_LOG_EVERY("Narrow phase generated " << m_ContactManifolds.size() << " contact manifolds");
        if (!m_ContactManifolds.empty())
        {
            for (size_t i = 0; i < std::min(m_ContactManifolds.size(), (size_t)5); ++i)
            {
                const auto& manifold = m_ContactManifolds[i];
                NYON_DEBUG_LOG_EVERY("  Contact[" << i << "] entityA=" << manifold.entityIdA 
                                  << " entityB=" << manifold.entityIdB 
                                  << " points=" << manifold.points.size()
                                  << " normal=(" << manifold.normal.x << "," << manifold.normal.y << ")");
            }
        }
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
                
                // Compute effective mass for each contact point
                for (auto& point : vc.points)
                {
                    // Moment arms from body centers to contact point
                    Math::Vector2 rA = point.position - bodyA.position;
                    Math::Vector2 rB = point.position - bodyB.position;

                    // Cross products with normal (scalar, since 2D)
                    float rAcrossN = Math::Vector2::Cross(rA, vc.normal);
                    float rBcrossN = Math::Vector2::Cross(rB, vc.normal);

                    // Effective mass = sum of translational and rotational contributions
                    float kNormal = vc.invMassA + vc.invMassB
                                  + vc.invIA * rAcrossN * rAcrossN
                                  + vc.invIB * rBcrossN * rBcrossN;

                    point.normalMass = (kNormal > 1e-6f) ? (1.0f / kNormal) : 0.0f;

                    // Tangent mass (for friction)
                    float rAcrossT = Math::Vector2::Cross(rA, vc.tangent);
                    float rBcrossT = Math::Vector2::Cross(rB, vc.tangent);
                    float kTangent = vc.invMassA + vc.invMassB
                                   + vc.invIA * rAcrossT * rAcrossT
                                   + vc.invIB * rBcrossT * rBcrossT;
                    point.tangentMass = (kTangent > 1e-6f) ? (1.0f / kTangent) : 0.0f;

                    // Compute velocity bias for restitution (bounce)
                    Math::Vector2 vA = bodyA.velocity;
                    Math::Vector2 vB = bodyB.velocity;
                    float wA = bodyA.angularVelocity;
                    float wB = bodyB.angularVelocity;

                    // Relative velocity at contact point
                    Math::Vector2 relVel = vB + Math::Vector2::Cross(wB, rB)
                                         - vA - Math::Vector2::Cross(wA, rA);
                    float vRel = Math::Vector2::Dot(relVel, vc.normal);

                    // Only apply restitution for separating contacts above threshold
                    const float RESTITUTION_VELOCITY_THRESHOLD = 1.0f;
                    if (vRel < -RESTITUTION_VELOCITY_THRESHOLD)
                    {
                        point.velocityBias = -vc.restitution * vRel;
                    }
                    else
                    {
                        point.velocityBias = 0.0f;
                    }
                }
                
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
        NYON_DEBUG_LOG_EVERY("Position solving: starting with " << m_VelocityConstraints.size() << " constraints");
        
        IntegratePositions(FIXED_TIMESTEP);
        
        // Solve position constraints for stabilization
        for (int i = 0; i < m_Config.positionIterations; ++i)
        {
            NYON_DEBUG_LOG_EVERY("  Position iteration " << i);
            SolvePositionConstraints();
        }
        
        // Debug: Log corrected positions
        NYON_DEBUG_LOG_EVERY("Position solving completed. Sample body positions:");
        for (size_t i = 0; i < std::min(m_SolverBodies.size(), (size_t)3); ++i)
        {
            const auto& body = m_SolverBodies[i];
            NYON_DEBUG_LOG_EVERY("  Body[" << i << "] pos=(" << body.position.x << "," << body.position.y 
                              << ") angle=" << body.angle);
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
        // ONLY update dynamic bodies - static bodies should NOT be overwritten!
        for (const auto& solverBody : m_SolverBodies)
        {
            if (solverBody.isStatic)
                continue;  // Skip static bodies - their transforms are set at creation time
                
            if (m_ComponentStore->HasComponent<TransformComponent>(solverBody.entityId))
            {
                auto& transform = m_ComponentStore->GetComponent<TransformComponent>(solverBody.entityId);

                // ✅ Write back the pre-integration position for interpolation
                transform.previousPosition = solverBody.prevPosition;
                transform.previousRotation = solverBody.prevAngle;

                // Current (post-integration) state
                transform.position = solverBody.position;
                transform.rotation = solverBody.angle;
            }
        }
        
        // Debug: Log positions every N frames
        NYON_DEBUG_LOG_EVERY("Updated " << m_SolverBodies.size() << " bodies:");
        for (size_t i = 0; i < std::min(m_SolverBodies.size(), (size_t)3); ++i)
        {
            const auto& body = m_SolverBodies[i];
            NYON_DEBUG_LOG_EVERY("  Body[" << i << "] entity=" << body.entityId 
                              << " pos=(" << body.position.x << "," << body.position.y 
                              << ") vel=(" << body.velocity.x << "," << body.velocity.y 
                              << ") static=" << body.isStatic);
        }
    }

    // BroadPhaseCallback implementation
    bool PhysicsPipelineSystem::BroadPhaseCallback::QueryCallback(uint32_t nodeId, uint32_t userData)
    {
        uint32_t otherEntityId = userData;
        
        // Debug logging using system pointer
        if (++s_DebugFrameCounter >= DEBUG_OUTPUT_INTERVAL) {
            s_DebugFrameCounter = 0;
            std::cerr << "[PHYSICS@0]   QueryCallback: querying entity=" << entityId 
                      << " found node=" << nodeId << " otherEntity=" << otherEntityId << std::endl;
        }
        
        // Avoid self-collision and ensure canonical pair ordering
        if (otherEntityId <= entityId)
        {
            if (++s_DebugFrameCounter >= DEBUG_OUTPUT_INTERVAL) {
                s_DebugFrameCounter = 0;
                std::cerr << "[PHYSICS@0]     Skipping: otherEntityId <= entityId" << std::endl;
            }
            return true;
        }
            
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
                if (++s_DebugFrameCounter >= DEBUG_OUTPUT_INTERVAL) {
                    s_DebugFrameCounter = 0;
                    std::cerr << "[PHYSICS@0]     Added pair: (" << entityId << "," << otherEntityId << ")" << std::endl;
                }
            }
            else
            {
                if (++s_DebugFrameCounter >= DEBUG_OUTPUT_INTERVAL) {
                    s_DebugFrameCounter = 0;
                    std::cerr << "[PHYSICS@0]     Filtered out by collision filter" << std::endl;
                }
            }
        }
        else
        {
            if (++s_DebugFrameCounter >= DEBUG_OUTPUT_INTERVAL) {
                s_DebugFrameCounter = 0;
                std::cerr << "[PHYSICS@0]     One or both entities missing ColliderComponent" << std::endl;
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
        
        NYON_DEBUG_LOG_EVERY("  AABB entity=" << entityId 
                          << " center=(" << position.x << "," << position.y << ")"
                          << " bounds=[(" << fatAABB.lowerBound.x << "," << fatAABB.lowerBound.y << ")-"
                          << "(" << fatAABB.upperBound.x << "," << fatAABB.upperBound.y << ")]");
        
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
        int correctionsApplied = 0;
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
                    
                    NYON_DEBUG_LOG_EVERY("  Contact correction: separation=" << point.separation 
                                      << " normal=(" << constraint.normal.x << "," << constraint.normal.y << ")"
                                      << " correction=(" << P.x << "," << P.y << ")");
                    
                    // ONLY move dynamic bodies - NEVER move static bodies!
                    // Move each body based on its inverse mass (lighter bodies move more)
                    if (!bodyA.isStatic && !bodyB.isStatic)
                    {
                        // Both dynamic - move both apart
                        bodyA.position -= P * constraint.invMassA;
                        bodyB.position += P * constraint.invMassB;
                        correctionsApplied++;
                    }
                    else if (!bodyA.isStatic)
                    {
                        // B is static - only move A away from B
                        bodyA.position -= P * constraint.invMassA;
                        correctionsApplied++;
                    }
                    else if (!bodyB.isStatic)
                    {
                        // A is static - only move B away from A
                        bodyB.position += P * constraint.invMassB;
                        correctionsApplied++;
                    }
                    // If both are static, don't move anything (shouldn't happen)
                }
            }
        }
        NYON_DEBUG_LOG_EVERY("Position constraints solved with " << correctionsApplied << " corrections applied");
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