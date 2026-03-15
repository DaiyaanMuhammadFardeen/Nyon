#include "nyon/ecs/systems/PhysicsPipelineSystem.h"
#include "nyon/physics/ManifoldGenerator.h"
#include <chrono>
#include <algorithm>
#include <iostream>

namespace Nyon::ECS
{
    void PhysicsPipelineSystem::Initialize(EntityManager& entityManager, ComponentStore& componentStore)
    {
        m_ComponentStore = &componentStore;

        // Find physics world component and store its entity ID
        m_ComponentStore->ForEachComponent<PhysicsWorldComponent>([&](EntityID entityId, PhysicsWorldComponent& world) {
                m_PhysicsWorldEntity = entityId;
                });

        if (m_PhysicsWorldEntity == INVALID_ENTITY)
        {
            return;
        }

        // Initialize island manager
        m_IslandManager = std::make_unique<Physics::IslandManager>(*m_ComponentStore);
    }

    void PhysicsPipelineSystem::Update(float deltaTime)
    {
        // Lazy initialization - find PhysicsWorldComponent if not already found
        if (m_PhysicsWorldEntity == INVALID_ENTITY && m_ComponentStore)
        {
            m_ComponentStore->ForEachComponent<PhysicsWorldComponent>([&](EntityID entityId, PhysicsWorldComponent& world) {
                    m_PhysicsWorldEntity = entityId;

                    // Initialize island manager now that we have a physics world
                    if (!m_IslandManager)
                    {
                        m_IslandManager = std::make_unique<Physics::IslandManager>(*m_ComponentStore);
                    }
                    });
        }

        if (m_PhysicsWorldEntity == INVALID_ENTITY || !m_ComponentStore)
        {
            return;
        }

        // Safety check: ensure IslandManager is initialized
        if (!m_IslandManager)
        {
            m_IslandManager = std::make_unique<Physics::IslandManager>(*m_ComponentStore);
        }

        auto startTime = std::chrono::high_resolution_clock::now();

        // deltaTime is guaranteed == FIXED_TIMESTEP by Application::Run()
        // No inner accumulator needed - just execute one physics step

        // Collect active entities
        m_ActiveEntities.clear();
        m_ComponentStore->ForEachComponent<PhysicsBodyComponent>([&](EntityID entityId, const PhysicsBodyComponent& body) {
                if (!body.isStatic) // Only process non-static bodies
                {
                m_ActiveEntities.push_back(entityId);
                }
                });

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

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<float, std::milli>(endTime - startTime);
        m_Stats.updateTime = duration.count();
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

                // === COMPUTE MASS PROPERTIES FROM COLLIDER SHAPE ===
                // This ensures inertia is correctly computed from shape geometry
                if (!body.isStatic && m_ComponentStore->HasComponent<ColliderComponent>(entityId))
                {
                    const auto& collider = m_ComponentStore->GetComponent<ColliderComponent>(entityId);

                    // Calculate mass from area and density
                    float area = collider.CalculateArea();
                    float density = collider.material.density;
                    float calculatedMass = area * density;

                    // Only update mass if it hasn't been explicitly set (i.e., still default 1.0)
                    // or if the calculated mass is significantly different
                    if (body.mass <= 0.0f || std::abs(body.mass - calculatedMass) > 0.01f)
                    {
                        body.SetMass(calculatedMass);
                    }

                    // Calculate and set inertia from shape geometry
                    float unitInertia = collider.CalculateInertiaForUnitDensity();
                    float calculatedInertia = unitInertia * density;

                    // Only update inertia if it hasn't been explicitly set (i.e., still default 1.0)
                    // or if the calculated inertia is significantly different
                    if (body.inertia <= 0.0f || std::abs(body.inertia - calculatedInertia) > 0.01f)
                    {
                        body.SetInertia(calculatedInertia);
                    }
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

                // Initialize forces from ECS component (user-applied forces, gravity, etc.)
                solverBody.force = body.force;
                solverBody.torque = body.torque;

                // Enforce motion locks early in the solver so collisions do not cause unwanted rotation.
                // This keeps the body stable when lockRotation is enabled (e.g., player character).
                if (body.motionLocks.lockRotation)
                {
                    solverBody.angularVelocity = 0.0f;
                    solverBody.torque = 0.0f;
                    solverBody.invInertia = 0.0f;
                }

                // Clear ECS-side forces so they don't accumulate across frames
                body.ClearForces();

                m_SolverBodies.push_back(solverBody);
                m_EntityToSolverIndex[entityId] = solverIndex++;
        });
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

        // Update broad phase tree and collect potential pairs
        m_ComponentStore->ForEachComponent<ColliderComponent>([&](EntityID entityId, ColliderComponent& collider) {
                if (!m_ComponentStore->HasComponent<TransformComponent>(entityId))
                return;

                const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);

                // Update shape AABB in broad phase tree
                UpdateShapeAABB(entityId, &collider, transform.position, transform.rotation);
                });

        // Query broad phase for overlapping pairs using DynamicTree
        // This is O(n log n) instead of O(n²) brute force
        for (const auto& [entityId, proxyId] : m_ShapeProxyMap)
        {
            // Only query for dynamic bodies (static bodies don't initiate collision checks)
            if (!m_ComponentStore->HasComponent<PhysicsBodyComponent>(entityId))
                continue;

            const auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityId);
            if (body.isStatic)
                continue;

            // Get the fat AABB for this proxy
            const auto& fatAABB = m_BroadPhaseTree.GetFatAABB(proxyId);

            // Query the tree for overlapping proxies
            BroadPhaseCallback callback;
            callback.system = this;
            callback.entityId = entityId;
            m_BroadPhaseTree.Query(fatAABB, &callback);
        }

        m_Stats.broadPhasePairs = m_BroadPhasePairs.size();
        
#ifdef _DEBUG
        if (!m_BroadPhasePairs.empty()) {
            std::cerr << "[PHYSICS] Broad phase found " << m_BroadPhasePairs.size() << " potential collision pairs\n";
        }
#endif
    }

    void PhysicsPipelineSystem::NarrowPhaseDetection()
    {
        m_ContactManifolds.clear();
        m_ContactMap.clear();

        // Clear world contacts for this frame
        if (m_PhysicsWorldEntity != INVALID_ENTITY && m_ComponentStore) {
            auto& world = m_ComponentStore->GetComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity);
            world.contactManifolds.clear();
        }


        // Test each broad phase pair for actual collision
        for (const auto& [entityIdA, entityIdB] : m_BroadPhasePairs)
        {
            if (TestCollision(entityIdA, entityIdB))
            {
                ContactManifold manifold = GenerateManifold(entityIdA, entityIdB);
                if (!manifold.points.empty())
                {
#ifdef _DEBUG
                    std::cerr << "[PHYSICS] Collision detected between entities " 
                              << entityIdA << " and " << entityIdB 
                              << " with " << manifold.points.size() << " contact points\n";
#endif

                    // Store contact for constraint solving
                    uint64_t key = (static_cast<uint64_t>(std::min(entityIdA, entityIdB)) << 32) |
                        static_cast<uint64_t>(std::max(entityIdA, entityIdB));
                    m_ContactMap[key] = m_ContactManifolds.size();
                    m_ContactManifolds.push_back(std::move(manifold));

                    // Also populate the world component for island manager and debug rendering
                    if (m_PhysicsWorldEntity != INVALID_ENTITY && m_ComponentStore) {
                        auto& world = m_ComponentStore->GetComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity);
                        ECS::ContactManifold worldManifold;
                        worldManifold.entityIdA = manifold.entityIdA;
                        worldManifold.entityIdB = manifold.entityIdB;
                        worldManifold.normal = manifold.normal;
                        worldManifold.touching = true;

                        for (const auto& pt : manifold.points) {
                            ECS::ContactPoint cp;
                            cp.position = pt.position;
                            cp.normal = pt.normal;
                            cp.separation = pt.separation;
                            cp.normalImpulse = pt.normalImpulse;
                            cp.tangentImpulse = pt.tangentImpulse;
                            worldManifold.points.push_back(std::move(cp));
                        }

                        world.contactManifolds.push_back(std::move(worldManifold));
                    }
                }
                else
                {
#ifdef _DEBUG
                    std::cerr << "[PHYSICS] Collision detected but manifold generation failed for entities " 
                              << entityIdA << " and " << entityIdB << "\n";
#endif
                }
            }
        }

        m_Stats.narrowPhaseContacts = m_ContactManifolds.size();

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

        // Create velocity constraints from contact manifolds
        for (const auto& manifold : m_ContactManifolds)
        {
            VelocityConstraint vc;
            vc.normal = manifold.normal;
            vc.tangent = Math::Vector2{-manifold.normal.y, manifold.normal.x};
            vc.points = manifold.points;

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

                // === COMPUTE FRICTION AND RESTITUTION FROM MATERIALS ===
                // Look up both colliders to get material properties
                const auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(manifold.entityIdA);
                const auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(manifold.entityIdB);

                // Mix friction using geometric mean (standard approach)
                vc.friction = std::sqrt(colliderA.material.friction * colliderB.material.friction);

                // Mix restitution using maximum (standard approach)
                vc.restitution = std::max(colliderA.material.restitution, colliderB.material.restitution);

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

                    // Use world restitution threshold instead of hardcoded value
                    float restitutionThreshold = 0.0f;
                    if (m_PhysicsWorldEntity != INVALID_ENTITY && m_ComponentStore)
                    {
                        restitutionThreshold = m_ComponentStore->GetComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity).restitutionThreshold;
                    }

                    if (vRel < -restitutionThreshold)
                    {
                        point.velocityBias = -vc.restitution * vRel;
                    }
                    else
                    {
                        point.velocityBias = 0.0f;
                    }

                    // Restore cached impulses for warm starting
                    uint64_t cacheKey = MakeImpulseCacheKey(manifold.entityIdA, manifold.entityIdB, point.featureId);
                    auto cacheIt = m_ImpulseCache.find(cacheKey);
                    if (cacheIt != m_ImpulseCache.end())
                    {
                        point.normalImpulse = cacheIt->second.normalImpulse;
                        point.tangentImpulse = cacheIt->second.tangentImpulse;
                    }
                }

                m_VelocityConstraints.push_back(vc);
            }
        }

        m_Stats.activeConstraints = m_VelocityConstraints.size();
    }

    void PhysicsPipelineSystem::VelocitySolving()
    {
        // 1. Apply gravity and other external forces
        for (auto& body : m_SolverBodies)
        {
            if (!body.isStatic && body.isAwake && m_PhysicsWorldEntity != INVALID_ENTITY && m_ComponentStore)
            {
                // Apply gravity as force: F = m * g
                float mass = (body.invMass > 0.0f) ? 1.0f / body.invMass : 0.0f;
                const auto& world = m_ComponentStore->GetComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity);
                body.force += world.gravity * mass;
            }
        }
        
        // 2. Integrate velocities from forces
        IntegrateVelocities(FIXED_TIMESTEP);
        
        // 3. Warm start with this frame's post-integration velocities
        if (m_Config.warmStarting)
        {
            WarmStartConstraints();
        }
        
        // 4. Solve velocity constraints iteratively
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

        // Debug: Log corrected positions

        for (size_t i = 0; i < std::min(m_SolverBodies.size(), (size_t)3); ++i)
        {
            const auto& body = m_SolverBodies[i];

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

                // === ENFORCE MOTION LOCKS ===
                Math::Vector2 lockedVelocity = solverBody.velocity;
                float lockedAngularVelocity = solverBody.angularVelocity;

                if (body.motionLocks.lockTranslationX)
                    lockedVelocity.x = 0.0f;
                if (body.motionLocks.lockTranslationY)
                    lockedVelocity.y = 0.0f;
                if (body.motionLocks.lockRotation)
                    lockedAngularVelocity = 0.0f;

                body.velocity = lockedVelocity;
                body.angularVelocity = lockedAngularVelocity;

                // Damping is already applied in ConstraintSolverSystem - no additional damping here
            }
        }
    }

    void PhysicsPipelineSystem::StoreImpulses()
    {
        // Store accumulated impulses for warm starting next frame
        std::unordered_map<uint64_t, bool> activeKeys; // Track which contacts are still active

        for (const auto& constraint : m_VelocityConstraints)
        {
            uint32_t entityIdA = m_SolverBodies[constraint.indexA].entityId;
            uint32_t entityIdB = m_SolverBodies[constraint.indexB].entityId;

            for (const auto& point : constraint.points)
            {
                // Create cache key from entity pair + feature ID
                uint64_t cacheKey = MakeImpulseCacheKey(entityIdA, entityIdB, point.featureId);

                // Store impulses
                m_ImpulseCache[cacheKey] = {
                    point.normalImpulse,
                    point.tangentImpulse
                };

                // Mark this key as active
                activeKeys[cacheKey] = true;
            }
        }

        // Evict stale cache entries (contacts that no longer exist)
        // Only evict if the contact is not in the active set
        std::vector<uint64_t> keysToRemove;
        for (const auto& [key, impulse] : m_ImpulseCache)
        {
            if (activeKeys.find(key) == activeKeys.end())
            {
                keysToRemove.push_back(key);
            }
        }

        for (uint64_t key : keysToRemove)
        {
            m_ImpulseCache.erase(key);
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

                // Get motion locks from physics body if it exists
                bool hasLocks = false;
                PhysicsBodyComponent::MotionLocks locks{};
                if (m_ComponentStore->HasComponent<PhysicsBodyComponent>(solverBody.entityId))
                {
                    const auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(solverBody.entityId);
                    locks = body.motionLocks;
                    hasLocks = true;
                }

                // ✅ Write back the pre-integration position for interpolation
                transform.previousPosition = solverBody.prevPosition;
                transform.previousRotation = solverBody.prevAngle;

                // Current (post-integration) state with motion locks enforced
                Math::Vector2 lockedPosition = solverBody.position;
                float lockedAngle = solverBody.angle;

                if (hasLocks)
                {
                    if (locks.lockTranslationX)
                        lockedPosition.x = transform.position.x; // Keep previous X
                    if (locks.lockTranslationY)
                        lockedPosition.y = transform.position.y; // Keep previous Y
                    if (locks.lockRotation)
                        lockedAngle = transform.rotation; // Keep previous rotation
                }

                transform.position = lockedPosition;
                transform.rotation = lockedAngle;
            }
        }

        // Debug: Log positions every N frames

        for (size_t i = 0; i < std::min(m_SolverBodies.size(), (size_t)3); ++i)
        {
            const auto& body = m_SolverBodies[i];

        }
    }

    // BroadPhaseCallback implementation
    bool PhysicsPipelineSystem::BroadPhaseCallback::QueryCallback(uint32_t nodeId, uint32_t userData)
    {
        uint32_t otherEntityId = userData;

        // Avoid self-collision (only skip when both entities would generate the same pair)
        if (otherEntityId == entityId)
        {
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

        // No manual padding - MoveProxy/CreateProxy will apply AABB_EXTENSION internally


        auto it = m_ShapeProxyMap.find(entityId);
        if (it != m_ShapeProxyMap.end())
        {
            // Update existing proxy with velocity-based displacement hint
            Math::Vector2 displacement = {0.0f, 0.0f};
            if (m_ComponentStore->HasComponent<PhysicsBodyComponent>(entityId)) {
                const auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityId);
                displacement = body.velocity * FIXED_TIMESTEP;
            }
            m_BroadPhaseTree.MoveProxy(it->second, aabb, displacement);
        }
        else
        {
            // Create new proxy
            uint32_t proxyId = m_BroadPhaseTree.CreateProxy(aabb, entityId);
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

        for (size_t i = 0; i < generatedManifold.points.size(); ++i)
        {
            const auto& point = generatedManifold.points[i];
            ContactPoint contactPoint;
            contactPoint.position = point.position;
            contactPoint.normal = point.normal;
            contactPoint.separation = point.separation;
            contactPoint.normalImpulse = 0.0f;
            contactPoint.tangentImpulse = 0.0f;
            // Use feature ID from ECS manifold if available, otherwise generate from point index
            contactPoint.featureId = point.featureId != 0 ? point.featureId : (i + 1);
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

    uint64_t PhysicsPipelineSystem::MakeImpulseCacheKey(uint32_t entityIdA, uint32_t entityIdB, uint32_t featureId) const
    {
        // Create a unique key from entity pair (order-independent) + feature ID
        uint32_t minEntity = std::min(entityIdA, entityIdB);
        uint32_t maxEntity = std::max(entityIdA, entityIdB);

        // Combine entity pair into 64-bit key, then XOR with feature ID
        uint64_t pairKey = (static_cast<uint64_t>(minEntity) << 32) | static_cast<uint64_t>(maxEntity);
        return pairKey ^ (static_cast<uint64_t>(featureId) << 32);
    }

    void PhysicsPipelineSystem::SolveVelocityConstraints()
    {
        for (auto& constraint : m_VelocityConstraints)
        {
            auto& bodyA = m_SolverBodies[constraint.indexA];
            auto& bodyB = m_SolverBodies[constraint.indexB];

            for (auto& point : constraint.points)
            {
                // Recompute rA, rB from LIVE body positions each iteration:
                Math::Vector2 rA = point.position - bodyA.position;
                Math::Vector2 rB = point.position - bodyB.position;

                // Use LIVE body velocities (not the stale snapshot):
                Math::Vector2 dv = bodyB.velocity + Math::Vector2::Cross(bodyB.angularVelocity, rB)
                    - bodyA.velocity - Math::Vector2::Cross(bodyA.angularVelocity, rA);

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

                // --- Tangent (friction) solve ---
                // Recompute relative velocity for tangent direction
                Math::Vector2 dvTangent = bodyB.velocity + Math::Vector2::Cross(bodyB.angularVelocity, rB)
                    - bodyA.velocity - Math::Vector2::Cross(bodyA.angularVelocity, rA);
                float vt = Math::Vector2::Dot(dvTangent, constraint.tangent);
                float tangentImpulse = -point.tangentMass * vt;

                // Friction cone clamp (Coulomb friction): |tangentImpulse| <= friction * normalImpulse
                float maxFriction = constraint.friction * point.normalImpulse;
                float oldTangentImpulse = point.tangentImpulse;
                point.tangentImpulse = std::clamp(oldTangentImpulse + tangentImpulse,
                        -maxFriction, maxFriction);
                tangentImpulse = point.tangentImpulse - oldTangentImpulse;

                // Apply tangent impulse
                Math::Vector2 Pt = constraint.tangent * tangentImpulse;
                if (!bodyA.isStatic)
                {
                    bodyA.velocity -= Pt * constraint.invMassA;
                    bodyA.angularVelocity -= constraint.invIA * Math::Vector2::Cross(rA, Pt);
                }

                if (!bodyB.isStatic)
                {
                    bodyB.velocity += Pt * constraint.invMassB;
                    bodyB.angularVelocity += constraint.invIB * Math::Vector2::Cross(rB, Pt);
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

                    // Calculate full effective mass including rotational inertia
                    float rAcrossN = Math::Vector2::Cross(rA, constraint.normal);
                    float rBcrossN = Math::Vector2::Cross(rB, constraint.normal);

                    float kNormal = constraint.invMassA + constraint.invMassB
                        + constraint.invIA * rAcrossN * rAcrossN
                        + constraint.invIB * rBcrossN * rBcrossN;

                    float mass = (kNormal > 1e-6f) ? 1.0f / kNormal : 0.0f;

                    Math::Vector2 P = constraint.normal * (C * mass);



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

    }

    void PhysicsPipelineSystem::IntegrateVelocities(float dt)
    {
        // Respect global/world speed limits if available
        float maxLinearSpeed = std::numeric_limits<float>::infinity();
        float maxAngularSpeed = std::numeric_limits<float>::infinity();
        if (m_PhysicsWorldEntity != INVALID_ENTITY && m_ComponentStore)
        {
            const auto& world = m_ComponentStore->GetComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity);
            maxLinearSpeed = world.maxLinearSpeed;
            maxAngularSpeed = world.maxAngularSpeed;
        }

        for (auto& body : m_SolverBodies)
        {
            if (body.isStatic || !body.isAwake)
                continue;

            // Integrate linear velocity
            body.velocity += (body.force * body.invMass) * dt;

            // Clamp velocity to prevent excessive tunneling
            float speed = body.velocity.Length();
            if (speed > maxLinearSpeed && speed > 0.0f)
            {
                body.velocity = body.velocity * (maxLinearSpeed / speed);
            }

            // Integrate angular velocity
            body.angularVelocity += (body.torque * body.invInertia) * dt;

            // Clamp angular velocity
            body.angularVelocity = std::clamp(body.angularVelocity, -maxAngularSpeed, maxAngularSpeed);

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
