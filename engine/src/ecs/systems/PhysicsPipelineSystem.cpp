#include "nyon/ecs/systems/PhysicsPipelineSystem.h"
#include "nyon/physics/ManifoldGenerator.h"
#include <chrono>
#include <algorithm>
#include <iostream>
#include <mutex>
namespace Nyon::ECS {
    void PhysicsPipelineSystem::Initialize(EntityManager& entityManager, ComponentStore& componentStore) {
        m_ComponentStore = &componentStore;
        m_ComponentStore->ForEachComponent<PhysicsWorldComponent>([&](EntityID entityId, PhysicsWorldComponent& world) {
                m_PhysicsWorldEntity = entityId; });
        if (m_PhysicsWorldEntity == INVALID_ENTITY) {
            return; }
        m_IslandManager = std::make_unique<Physics::IslandManager>(*m_ComponentStore);
        Utils::ThreadPool::Initialize();
        m_NumThreads = Utils::ThreadPool::Instance().GetThreadCount();
        std::cerr << "[PHYSICS] Multi-threaded physics initialized with " << m_NumThreads << " threads\n"; }
    void PhysicsPipelineSystem::Update(float deltaTime) {
        if (m_PhysicsWorldEntity == INVALID_ENTITY && m_ComponentStore) {
            m_ComponentStore->ForEachComponent<PhysicsWorldComponent>([&](EntityID entityId, PhysicsWorldComponent& world) {
                    m_PhysicsWorldEntity = entityId;
                    if (!m_IslandManager) {
                        m_IslandManager = std::make_unique<Physics::IslandManager>(*m_ComponentStore); } }); }
        if (m_PhysicsWorldEntity == INVALID_ENTITY || !m_ComponentStore) {
            return; }
        if (!m_IslandManager) {
            m_IslandManager = std::make_unique<Physics::IslandManager>(*m_ComponentStore); }
        auto startTime = std::chrono::high_resolution_clock::now();
        float maxSpeedSquared = 0.0f;
        m_ComponentStore->ForEachComponent<PhysicsBodyComponent>([&](EntityID entityId, const PhysicsBodyComponent& body) {
                if (!body.isStatic) {
                    float speedSq = body.velocity.LengthSquared();
                    if (speedSq > maxSpeedSquared) {
                        maxSpeedSquared = speedSq; } } });
        constexpr float SUBSTEP_SPEED_THRESHOLD = 400.0f;
        int numSubSteps = 1;
        if (std::sqrt(maxSpeedSquared) > SUBSTEP_SPEED_THRESHOLD) {
            numSubSteps = 2;    }
        float subStepDt = deltaTime / numSubSteps;
        for (int step = 0; step < numSubSteps; ++step) {
            m_ActiveEntities.clear();
            m_ComponentStore->ForEachComponent<PhysicsBodyComponent>([&](EntityID entityId, const PhysicsBodyComponent& body) {
                    if (!body.isStatic) {
                        m_ActiveEntities.push_back(entityId); } });
            PrepareBodiesForUpdate();
            if (m_UseMultiThreading && m_ActiveEntities.size() > 1) {
                ParallelBroadPhase();
                ParallelNarrowPhase(); } else {
                BroadPhaseDetection();
                NarrowPhaseDetection(); }
            IslandDetection();
            ConstraintInitialization();
            if (m_UseMultiThreading && m_VelocityConstraints.size() > 1) {
                ParallelVelocitySolving(subStepDt);
                ParallelPositionSolving(subStepDt); } else {
                VelocitySolving();
                PositionSolving(); }
            Integration();
            StoreImpulses();
            UpdateSleeping();
            UpdateTransformsFromSolver(); }
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<float, std::milli>(endTime - startTime);
        m_Stats.updateTime = duration.count(); }
    void PhysicsPipelineSystem::PrepareBodiesForUpdate() {
        m_SolverBodies.clear();
        m_EntityToSolverIndex.clear();
        size_t solverIndex = 0;
        m_ComponentStore->ForEachComponent<PhysicsBodyComponent>([&](EntityID entityId, PhysicsBodyComponent& body) {
                bool shouldInclude = false;
                if (body.isStatic) {
                shouldInclude = true; }
                else {
                if (!body.allowSleep) {
                    shouldInclude = true; }
                else {
                    bool islandAwake = m_IslandManager->IsBodyAwake(entityId);
                    bool componentAwake = body.isAwake;
                    shouldInclude = islandAwake || componentAwake; } }
                if (!shouldInclude) {
                    return;   }
                if (!body.isStatic && m_ComponentStore->HasComponent<ColliderComponent>(entityId)) {
                    const auto& collider = m_ComponentStore->GetComponent<ColliderComponent>(entityId);
                    if (!body.massIsExplicit) {
                        float area = collider.CalculateArea();
                        float density = collider.material.density;
                        float calculatedMass = area * density;
                        if (body.mass <= 0.0f || std::abs(body.mass - calculatedMass) > 0.01f) {
                            body.SetMass(calculatedMass); } }
                    if (!body.inertiaIsExplicit) {
                        float unitInertia = collider.CalculateInertiaPerUnitMass();
                        float calculatedInertia = unitInertia * body.mass;
                        if (body.inertia <= 0.0f || std::abs(body.inertia - calculatedInertia) > 0.01f) {
                            body.SetInertia(calculatedInertia); } } }
                SolverBody solverBody;
                solverBody.entityId = entityId;
                solverBody.isStatic = body.isStatic;
                solverBody.isAwake = body.isStatic || m_IslandManager->IsBodyAwake(entityId);
                solverBody.invMass = body.inverseMass;
                solverBody.invInertia = body.inverseInertia;
                solverBody.localCenter = body.centerOfMass;
                if (m_ComponentStore->HasComponent<TransformComponent>(entityId)) {
                    const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
                    solverBody.position = transform.position;
                    solverBody.angle = transform.rotation;
                    solverBody.prevPosition = transform.previousPosition;
                    solverBody.prevAngle = transform.previousRotation; }
                solverBody.velocity = body.velocity;
                solverBody.angularVelocity = body.angularVelocity;
                solverBody.force = body.force;
                solverBody.torque = body.torque;
                solverBody.linearDamping = body.drag;            
                solverBody.angularDamping = body.angularDamping;  
                if (body.motionLocks.lockRotation) {
                    solverBody.angularVelocity = 0.0f;
                    solverBody.torque = 0.0f;
                    solverBody.invInertia = 0.0f; }
                body.ClearForces();
                m_SolverBodies.push_back(solverBody);
                m_EntityToSolverIndex[entityId] = solverIndex++; }); }
    void PhysicsPipelineSystem::BroadPhaseDetection() {
        m_BroadPhasePairs.clear();
        std::vector<uint32_t> entitiesToRemove;
        for (const auto& [entityId, proxyId] : m_ShapeProxyMap) {
            if (!m_ComponentStore->HasComponent<ColliderComponent>(entityId)) {
                entitiesToRemove.push_back(entityId); } }
        for (uint32_t entityId : entitiesToRemove) {
            uint32_t proxyId = m_ShapeProxyMap[entityId];
            m_BroadPhaseTree.DestroyProxy(proxyId);
            m_ShapeProxyMap.erase(entityId); }
        m_ComponentStore->ForEachComponent<ColliderComponent>([&](EntityID entityId, ColliderComponent& collider) {
                if (!m_ComponentStore->HasComponent<TransformComponent>(entityId))
                return;
                const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
                UpdateShapeAABB(entityId, &collider, transform.position, transform.rotation); });
        for (const auto& [entityId, proxyId] : m_ShapeProxyMap) {
            if (!m_ComponentStore->HasComponent<PhysicsBodyComponent>(entityId))
                continue;
            const auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityId);
            if (body.isStatic)
                continue;
            const auto& fatAABB = m_BroadPhaseTree.GetFatAABB(proxyId);
            BroadPhaseCallback callback;
            callback.system = this;
            callback.entityId = entityId;
            m_BroadPhaseTree.Query(fatAABB, &callback); }
        m_Stats.broadPhasePairs = m_BroadPhasePairs.size();
#ifdef _DEBUG
        if (!m_BroadPhasePairs.empty()) {
            std::cerr << "[PHYSICS] Broad phase found " << m_BroadPhasePairs.size() << " potential collision pairs\n"; }
#endif }
    void PhysicsPipelineSystem::NarrowPhaseDetection() {
        m_ContactManifolds.clear();
        m_ContactMap.clear();
        if (m_PhysicsWorldEntity != INVALID_ENTITY && m_ComponentStore) {
            auto& world = m_ComponentStore->GetComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity);
            world.contactManifolds.clear(); }
        for (const auto& [entityIdA, entityIdB] : m_BroadPhasePairs) {
            if (TestCollision(entityIdA, entityIdB)) {
                ContactManifold manifold = GenerateManifold(entityIdA, entityIdB);
                if (!manifold.points.empty()) {
#ifdef _DEBUG
                    std::cerr << "[PHYSICS] Collision detected between entities " 
                              << entityIdA << " and " << entityIdB 
                              << " with " << manifold.points.size() << " contact points\n";
#endif
                    uint64_t key = (static_cast<uint64_t>(std::min(entityIdA, entityIdB)) << 32) |
                        static_cast<uint64_t>(std::max(entityIdA, entityIdB));
                    m_ContactMap[key] = m_ContactManifolds.size();
                    m_ContactManifolds.push_back(std::move(manifold));
                    if (m_PhysicsWorldEntity != INVALID_ENTITY && m_ComponentStore) {
                        auto& world = m_ComponentStore->GetComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity);
                        world.contactManifolds.push_back(manifold);   } }
                else {
#ifdef _DEBUG
                    std::cerr << "[PHYSICS] Collision detected but manifold generation failed for entities " 
                              << entityIdA << " and " << entityIdB << "\n";
#endif } } }
        m_Stats.narrowPhaseContacts = m_ContactManifolds.size(); }
    void PhysicsPipelineSystem::IslandDetection() {
        if (m_Config.useIslandSleeping) {
            m_IslandManager->UpdateIslands(Nyon::FIXED_TIMESTEP, m_ActiveEntities);
            m_Stats.islandStats = m_IslandManager->GetStatistics(); } }
    void PhysicsPipelineSystem::ConstraintInitialization() {
        m_VelocityConstraints.clear();
        for (const auto& manifold : m_ContactManifolds) {
            VelocityConstraint vc;
            vc.normal = manifold.normal;
            vc.tangent = Math::Vector2{-manifold.normal.y, manifold.normal.x};
            vc.points.reserve(manifold.points.size());
            for (const auto& ecsPoint : manifold.points) {
                ContactPointConstraint constraintPoint;
                constraintPoint.position = ecsPoint.position;
                constraintPoint.normal = ecsPoint.normal;
                constraintPoint.separation = ecsPoint.separation;
                constraintPoint.normalImpulse = ecsPoint.normalImpulse;
                constraintPoint.tangentImpulse = ecsPoint.tangentImpulse;
                constraintPoint.normalMass = ecsPoint.normalMass;
                constraintPoint.tangentMass = ecsPoint.tangentMass;
                constraintPoint.velocityBias = ecsPoint.velocityBias;
                constraintPoint.featureId = ecsPoint.featureId;
                vc.points.push_back(constraintPoint); }
            auto itA = m_EntityToSolverIndex.find(manifold.entityIdA);
            auto itB = m_EntityToSolverIndex.find(manifold.entityIdB);
            if (itA != m_EntityToSolverIndex.end() && itB != m_EntityToSolverIndex.end()) {
                vc.indexA = itA->second;
                vc.indexB = itB->second;
                const auto& bodyA = m_SolverBodies[vc.indexA];
                const auto& bodyB = m_SolverBodies[vc.indexB];
                vc.invMassA = bodyA.invMass;
                vc.invMassB = bodyB.invMass;
                vc.invIA = bodyA.invInertia;
                vc.invIB = bodyB.invInertia;
                const auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(manifold.entityIdA);
                const auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(manifold.entityIdB);
                vc.friction = std::sqrt(colliderA.material.friction * colliderB.material.friction);
                vc.restitution = std::max(colliderA.material.restitution, colliderB.material.restitution);
                float initCosA = std::cos(bodyA.angle);
                float initSinA = std::sin(bodyA.angle);
                Math::Vector2 initWorldCentroidA = bodyA.position + Math::Vector2{
                    bodyA.localCenter.x * initCosA - bodyA.localCenter.y * initSinA,
                    bodyA.localCenter.x * initSinA + bodyA.localCenter.y * initCosA };
                float initCosB = std::cos(bodyB.angle);
                float initSinB = std::sin(bodyB.angle);
                Math::Vector2 initWorldCentroidB = bodyB.position + Math::Vector2{
                    bodyB.localCenter.x * initCosB - bodyB.localCenter.y * initSinB,
                    bodyB.localCenter.x * initSinB + bodyB.localCenter.y * initCosB };
                for (auto& point : vc.points) {
                    Math::Vector2 rA = point.position - initWorldCentroidA;
                    Math::Vector2 rB = point.position - initWorldCentroidB;
                    float rAcrossN = Math::Vector2::Cross(rA, vc.normal);
                    float rBcrossN = Math::Vector2::Cross(rB, vc.normal);
                    float kNormal = vc.invMassA + vc.invMassB
                        + vc.invIA * rAcrossN * rAcrossN
                        + vc.invIB * rBcrossN * rBcrossN;
                    point.normalMass = (kNormal > 1e-6f) ? (1.0f / kNormal) : 0.0f;
                    float rAcrossT = Math::Vector2::Cross(rA, vc.tangent);
                    float rBcrossT = Math::Vector2::Cross(rB, vc.tangent);
                    float kTangent = vc.invMassA + vc.invMassB
                        + vc.invIA * rAcrossT * rAcrossT
                        + vc.invIB * rBcrossT * rBcrossT;
                    point.tangentMass = (kTangent > 1e-6f) ? (1.0f / kTangent) : 0.0f;
                    Math::Vector2 vA = bodyA.velocity;
                    Math::Vector2 vB = bodyB.velocity;
                    float wA = bodyA.angularVelocity;
                    float wB = bodyB.angularVelocity;
                    Math::Vector2 relVel = vB + Math::Vector2::Cross(wB, rB)
                        - vA - Math::Vector2::Cross(wA, rA);
                    float vRel = Math::Vector2::Dot(relVel, vc.normal);
                    float restitutionThreshold = 0.0f;
                    if (m_PhysicsWorldEntity != INVALID_ENTITY && m_ComponentStore) {
                        restitutionThreshold = m_ComponentStore->GetComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity).restitutionThreshold; }
                    if (vRel < -restitutionThreshold) {
                        point.velocityBias = -vc.restitution * vRel; }
                    else {
                        point.velocityBias = 0.0f; }
                    uint64_t cacheKey = MakeImpulseCacheKey(manifold.entityIdA, manifold.entityIdB, point.featureId);
                    auto cacheIt = m_ImpulseCache.find(cacheKey);
                    if (cacheIt != m_ImpulseCache.end()) {
                        point.normalImpulse = cacheIt->second.normalImpulse;
                        point.tangentImpulse = cacheIt->second.tangentImpulse; } }
                m_VelocityConstraints.push_back(vc); } }
        m_Stats.activeConstraints = m_VelocityConstraints.size(); }
    void PhysicsPipelineSystem::VelocitySolving() {
        for (auto& body : m_SolverBodies) {
            if (!body.isStatic && body.isAwake && m_PhysicsWorldEntity != INVALID_ENTITY && m_ComponentStore) {
                float mass = (body.invMass > 0.0f) ? 1.0f / body.invMass : 0.0f;
                const auto& world = m_ComponentStore->GetComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity);
                body.force += world.gravity * mass; } }
        IntegrateVelocities(Nyon::FIXED_TIMESTEP);
        if (m_Config.warmStarting) {
            WarmStartConstraints(); }
        for (int i = 0; i < m_Config.velocityIterations; ++i) {
            SolveVelocityConstraints(); } }
    void PhysicsPipelineSystem::PositionSolving() {
        IntegratePositions(Nyon::FIXED_TIMESTEP);
        for (int i = 0; i < m_Config.positionIterations; ++i) {
            SolvePositionConstraints(); }
        for (size_t i = 0; i < std::min(m_SolverBodies.size(), (size_t)3); ++i) {
            const auto& body = m_SolverBodies[i]; } }
    void PhysicsPipelineSystem::Integration() {
        for (const auto& solverBody : m_SolverBodies) {
            if (m_ComponentStore->HasComponent<PhysicsBodyComponent>(solverBody.entityId)) {
                auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(solverBody.entityId);
                Math::Vector2 lockedVelocity = solverBody.velocity;
                float lockedAngularVelocity = solverBody.angularVelocity;
                if (body.motionLocks.lockTranslationX)
                    lockedVelocity.x = 0.0f;
                if (body.motionLocks.lockTranslationY)
                    lockedVelocity.y = 0.0f;
                if (body.motionLocks.lockRotation)
                    lockedAngularVelocity = 0.0f;
                body.velocity = lockedVelocity;
                body.angularVelocity = lockedAngularVelocity; } } }
    void PhysicsPipelineSystem::StoreImpulses() {
        std::unordered_map<uint64_t, bool> activeKeys;  
        for (const auto& constraint : m_VelocityConstraints) {
            uint32_t entityIdA = m_SolverBodies[constraint.indexA].entityId;
            uint32_t entityIdB = m_SolverBodies[constraint.indexB].entityId;
            for (const auto& point : constraint.points) {
                uint64_t cacheKey = MakeImpulseCacheKey(entityIdA, entityIdB, point.featureId);
                m_ImpulseCache[cacheKey] = {
                    point.normalImpulse,
                    point.tangentImpulse };
                activeKeys[cacheKey] = true; } }
        std::vector<uint64_t> keysToRemove;
        for (const auto& [key, impulse] : m_ImpulseCache) {
            if (activeKeys.find(key) == activeKeys.end()) {
                keysToRemove.push_back(key); } }
        for (uint64_t key : keysToRemove) {
            m_ImpulseCache.erase(key); } }
    void PhysicsPipelineSystem::UpdateSleeping() {
        if (!m_Config.useIslandSleeping)
            return;
        m_ComponentStore->ForEachComponent<PhysicsBodyComponent>([&](EntityID entityId, PhysicsBodyComponent& body) {
                if (body.isStatic)
                    return;
                if (!body.allowSleep) {
                    body.SetAwake(true);
                    if (m_IslandManager)
                        m_IslandManager->WakeIslandContaining(entityId);
                    return; }
                body.isAwake = m_IslandManager->IsBodyAwake(entityId); });
        m_Stats.awakeBodies = m_Stats.islandStats.awakeBodies;
        m_Stats.sleepingBodies = m_Stats.islandStats.sleepingBodies; }
    void PhysicsPipelineSystem::UpdateTransformsFromSolver() {
        for (const auto& solverBody : m_SolverBodies) {
            if (solverBody.isStatic)
                continue;   
            if (m_ComponentStore->HasComponent<TransformComponent>(solverBody.entityId)) {
                auto& transform = m_ComponentStore->GetComponent<TransformComponent>(solverBody.entityId);
                bool hasLocks = false;
                PhysicsBodyComponent::MotionLocks locks{};
                if (m_ComponentStore->HasComponent<PhysicsBodyComponent>(solverBody.entityId)) {
                    const auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(solverBody.entityId);
                    locks = body.motionLocks;
                    hasLocks = true; }
                transform.previousPosition = solverBody.prevPosition;
                transform.previousRotation = solverBody.prevAngle;
                Math::Vector2 lockedPosition = solverBody.position;
                float lockedAngle = solverBody.angle;
                if (hasLocks) {
                    if (locks.lockTranslationX)
                        lockedPosition.x = transform.position.x;  
                    if (locks.lockTranslationY)
                        lockedPosition.y = transform.position.y;  
                    if (locks.lockRotation)
                        lockedAngle = transform.rotation;   }
                transform.position = lockedPosition;
                transform.rotation = lockedAngle; } }
        for (size_t i = 0; i < std::min(m_SolverBodies.size(), (size_t)3); ++i) {
            const auto& body = m_SolverBodies[i]; } }
    bool PhysicsPipelineSystem::BroadPhaseCallback::QueryCallback(uint32_t nodeId, uint32_t userData) {
        uint32_t otherEntityId = userData;
        if (otherEntityId == entityId) {
            return true; }
        if (system->m_ComponentStore->HasComponent<ColliderComponent>(entityId) &&
                system->m_ComponentStore->HasComponent<ColliderComponent>(otherEntityId)) {
            const auto& colliderA = system->m_ComponentStore->GetComponent<ColliderComponent>(entityId);
            const auto& colliderB = system->m_ComponentStore->GetComponent<ColliderComponent>(otherEntityId);
            if (colliderA.filter.ShouldCollide(colliderB.filter)) {
                if (localPairs) {
                    localPairs->emplace_back(entityId, otherEntityId); } else {
                    system->m_BroadPhasePairs.emplace_back(entityId, otherEntityId); } } }
        return true;   }
    void PhysicsPipelineSystem::UpdateShapeAABB(uint32_t entityId, ColliderComponent* collider,
            const Math::Vector2& position, float angle) {
        Math::Vector2 min, max;
        collider->CalculateAABB(position, angle, min, max);
        Physics::AABB aabb;
        aabb.lowerBound = {min.x, min.y};
        aabb.upperBound = {max.x, max.y};
        auto it = m_ShapeProxyMap.find(entityId);
        if (it != m_ShapeProxyMap.end()) {
            Math::Vector2 displacement = {0.0f, 0.0f};
            if (m_ComponentStore->HasComponent<PhysicsBodyComponent>(entityId)) {
                const auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityId);
                displacement = body.velocity * Nyon::FIXED_TIMESTEP; }
            m_BroadPhaseTree.MoveProxy(it->second, aabb, displacement); }
        else {
            uint32_t proxyId = m_BroadPhaseTree.CreateProxy(aabb, entityId);
            m_ShapeProxyMap[entityId] = proxyId; } }
    bool PhysicsPipelineSystem::TestCollision(uint32_t entityIdA, uint32_t entityIdB) {
        if (!m_ComponentStore->HasComponent<ColliderComponent>(entityIdA) ||
                !m_ComponentStore->HasComponent<ColliderComponent>(entityIdB) ||
                !m_ComponentStore->HasComponent<TransformComponent>(entityIdA) ||
                !m_ComponentStore->HasComponent<TransformComponent>(entityIdB)) {
            return false; }
        const auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(entityIdA);
        const auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(entityIdB);
        const auto& transformA = m_ComponentStore->GetComponent<TransformComponent>(entityIdA);
        const auto& transformB = m_ComponentStore->GetComponent<TransformComponent>(entityIdB);
        Math::Vector2 minA, maxA, minB, maxB;
        colliderA.CalculateAABB(transformA.position, transformA.rotation, minA, maxA);
        colliderB.CalculateAABB(transformB.position, transformB.rotation, minB, maxB);
        return !(minA.x >= maxB.x || maxA.x <= minB.x || minA.y >= maxB.y || maxA.y <= minB.y); }
    ECS::ContactManifold PhysicsPipelineSystem::GenerateManifold(uint32_t entityIdA, uint32_t entityIdB) {
        ECS::ContactManifold manifold;
        manifold.entityIdA = entityIdA;
        manifold.entityIdB = entityIdB;
        if (!m_ComponentStore->HasComponent<ColliderComponent>(entityIdA) ||
                !m_ComponentStore->HasComponent<ColliderComponent>(entityIdB) ||
                !m_ComponentStore->HasComponent<TransformComponent>(entityIdA) ||
                !m_ComponentStore->HasComponent<TransformComponent>(entityIdB)) {
            return manifold; }
        const auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(entityIdA);
        const auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(entityIdB);
        const auto& transformA = m_ComponentStore->GetComponent<TransformComponent>(entityIdA);
        const auto& transformB = m_ComponentStore->GetComponent<TransformComponent>(entityIdB);
        uint32_t shapeIdA = 0;
        uint32_t shapeIdB = 0;
        ECS::ContactManifold generatedManifold = Physics::ManifoldGenerator::GenerateManifold(
                entityIdA, entityIdB,
                shapeIdA, shapeIdB,
                colliderA, colliderB,
                transformA, transformB
                );
        return generatedManifold; }
    void PhysicsPipelineSystem::WarmStartConstraints() {
        constexpr float WARM_START_FACTOR = 0.5f;   
        for (auto& constraint : m_VelocityConstraints) {
            const auto& bodyA = m_SolverBodies[constraint.indexA];
            const auto& bodyB = m_SolverBodies[constraint.indexB];
            float cosA = std::cos(bodyA.angle);
            float sinA = std::sin(bodyA.angle);
            Math::Vector2 worldCentroidA = bodyA.position + Math::Vector2{
                bodyA.localCenter.x * cosA - bodyA.localCenter.y * sinA,
                bodyA.localCenter.x * sinA + bodyA.localCenter.y * cosA };
            float cosB = std::cos(bodyB.angle);
            float sinB = std::sin(bodyB.angle);
            Math::Vector2 worldCentroidB = bodyB.position + Math::Vector2{
                bodyB.localCenter.x * cosB - bodyB.localCenter.y * sinB,
                bodyB.localCenter.x * sinB + bodyB.localCenter.y * cosB };
            for (auto& point : constraint.points) {
                float clampedNormalImpulse = point.normalImpulse * WARM_START_FACTOR;
                float clampedTangentImpulse = point.tangentImpulse * WARM_START_FACTOR;
                float maxImpulse = 100.0f;   
                clampedNormalImpulse = std::clamp(clampedNormalImpulse, -maxImpulse, maxImpulse);
                clampedTangentImpulse = std::clamp(clampedTangentImpulse, -maxImpulse, maxImpulse);
                Math::Vector2 P = constraint.normal * clampedNormalImpulse +
                    constraint.tangent * clampedTangentImpulse;
                if (!bodyA.isStatic) {
                    m_SolverBodies[constraint.indexA].velocity -= P * constraint.invMassA;
                    m_SolverBodies[constraint.indexA].angularVelocity -=
                        constraint.invIA * Math::Vector2::Cross(point.position - worldCentroidA, P); }
                if (!bodyB.isStatic) {
                    m_SolverBodies[constraint.indexB].velocity += P * constraint.invMassB;
                    m_SolverBodies[constraint.indexB].angularVelocity +=
                        constraint.invIB * Math::Vector2::Cross(point.position - worldCentroidB, P); } } } }
    uint64_t PhysicsPipelineSystem::MakeImpulseCacheKey(uint32_t entityIdA, uint32_t entityIdB, uint32_t featureId) const {
        uint32_t minEntity = std::min(entityIdA, entityIdB);
        uint32_t maxEntity = std::max(entityIdA, entityIdB);
        uint64_t pairKey = (static_cast<uint64_t>(minEntity) << 32) | static_cast<uint64_t>(maxEntity);
        return pairKey ^ (static_cast<uint64_t>(featureId) << 32); }
    void PhysicsPipelineSystem::SolveVelocityConstraints() {
        for (auto& constraint : m_VelocityConstraints) {
            auto& bodyA = m_SolverBodies[constraint.indexA];
            auto& bodyB = m_SolverBodies[constraint.indexB];
            float cosA = std::cos(bodyA.angle);
            float sinA = std::sin(bodyA.angle);
            Math::Vector2 liveWorldCentroidA = bodyA.position + Math::Vector2{
                bodyA.localCenter.x * cosA - bodyA.localCenter.y * sinA,
                bodyA.localCenter.x * sinA + bodyA.localCenter.y * cosA };
            float cosB = std::cos(bodyB.angle);
            float sinB = std::sin(bodyB.angle);
            Math::Vector2 liveWorldCentroidB = bodyB.position + Math::Vector2{
                bodyB.localCenter.x * cosB - bodyB.localCenter.y * sinB,
                bodyB.localCenter.x * sinB + bodyB.localCenter.y * cosB };
            for (auto& point : constraint.points) {
                Math::Vector2 rA = point.position - liveWorldCentroidA;
                Math::Vector2 rB = point.position - liveWorldCentroidB;
                Math::Vector2 dv = bodyB.velocity + Math::Vector2::Cross(bodyB.angularVelocity, rB)
                    - bodyA.velocity - Math::Vector2::Cross(bodyA.angularVelocity, rA);
                float vn = Math::Vector2::Dot(dv, constraint.normal);
                float impulse = -point.normalMass * (vn - point.velocityBias);
                float oldImpulse = point.normalImpulse;
                point.normalImpulse = std::max(oldImpulse + impulse, 0.0f);
                impulse = point.normalImpulse - oldImpulse;
                Math::Vector2 P = constraint.normal * impulse;
                if (!bodyA.isStatic) {
                    bodyA.velocity -= P * constraint.invMassA;
                    bodyA.angularVelocity -= constraint.invIA * Math::Vector2::Cross(rA, P); }
                if (!bodyB.isStatic) {
                    bodyB.velocity += P * constraint.invMassB;
                    bodyB.angularVelocity += constraint.invIB * Math::Vector2::Cross(rB, P); }
                Math::Vector2 dvTangent = bodyB.velocity + Math::Vector2::Cross(bodyB.angularVelocity, rB)
                    - bodyA.velocity - Math::Vector2::Cross(bodyA.angularVelocity, rA);
                float vt = Math::Vector2::Dot(dvTangent, constraint.tangent);
                float tangentImpulse = -point.tangentMass * vt;
                float maxFriction = constraint.friction * point.normalImpulse;
                float oldTangentImpulse = point.tangentImpulse;
                point.tangentImpulse = std::clamp(oldTangentImpulse + tangentImpulse,
                        -maxFriction, maxFriction);
                tangentImpulse = point.tangentImpulse - oldTangentImpulse;
                Math::Vector2 Pt = constraint.tangent * tangentImpulse;
                if (!bodyA.isStatic) {
                    bodyA.velocity -= Pt * constraint.invMassA;
                    bodyA.angularVelocity -= constraint.invIA * Math::Vector2::Cross(rA, Pt); }
                if (!bodyB.isStatic) {
                    bodyB.velocity += Pt * constraint.invMassB;
                    bodyB.angularVelocity += constraint.invIB * Math::Vector2::Cross(rB, Pt); } } } }
    void PhysicsPipelineSystem::SolvePositionConstraints() {
        int correctionsApplied = 0;
        for (const auto& constraint : m_VelocityConstraints) {
            auto& bodyA = m_SolverBodies[constraint.indexA];
            auto& bodyB = m_SolverBodies[constraint.indexB];
            float cosA = std::cos(bodyA.angle);
            float sinA = std::sin(bodyA.angle);
            Math::Vector2 worldCentroidA = bodyA.position + Math::Vector2{
                bodyA.localCenter.x * cosA - bodyA.localCenter.y * sinA,
                bodyA.localCenter.x * sinA + bodyA.localCenter.y * cosA };
            float cosB = std::cos(bodyB.angle);
            float sinB = std::sin(bodyB.angle);
            Math::Vector2 worldCentroidB = bodyB.position + Math::Vector2{
                bodyB.localCenter.x * cosB - bodyB.localCenter.y * sinB,
                bodyB.localCenter.x * sinB + bodyB.localCenter.y * cosB };
            for (const auto& point : constraint.points) {
                if (point.separation < -m_Config.linearSlop) {
                    Math::Vector2 rA = point.position - worldCentroidA;
                    Math::Vector2 rB = point.position - worldCentroidB;
                    float C = m_Config.baumgarte * (-point.separation - m_Config.linearSlop);
                    C = std::clamp(C, 0.0f, m_Config.maxLinearCorrection);
                    float rAcrossN = Math::Vector2::Cross(rA, constraint.normal);
                    float rBcrossN = Math::Vector2::Cross(rB, constraint.normal);
                    float kNormal = constraint.invMassA + constraint.invMassB
                        + constraint.invIA * rAcrossN * rAcrossN
                        + constraint.invIB * rBcrossN * rBcrossN;
                    float mass = (kNormal > 1e-6f) ? 1.0f / kNormal : 0.0f;
                    Math::Vector2 P = constraint.normal * (C * mass);
                    if (!bodyA.isStatic && !bodyB.isStatic) {
                        bodyA.position -= P * constraint.invMassA;
                        bodyA.angle    -= constraint.invIA * Math::Vector2::Cross(rA, P);   
                        bodyB.position += P * constraint.invMassB;
                        bodyB.angle    += constraint.invIB * Math::Vector2::Cross(rB, P);   
                        correctionsApplied++; }
                    else if (!bodyA.isStatic) {
                        bodyA.position -= P * constraint.invMassA;
                        bodyA.angle    -= constraint.invIA * Math::Vector2::Cross(rA, P);   
                        correctionsApplied++; }
                    else if (!bodyB.isStatic) {
                        bodyB.position += P * constraint.invMassB;
                        bodyB.angle    += constraint.invIB * Math::Vector2::Cross(rB, P);   
                        correctionsApplied++; } } } } }
    void PhysicsPipelineSystem::IntegrateVelocities(float dt) {
        IntegrateVelocities(dt, 0, m_SolverBodies.size()); }
    void PhysicsPipelineSystem::IntegrateVelocities(float dt, size_t start, size_t end) {
        float maxLinearSpeed = std::numeric_limits<float>::infinity();
        float maxAngularSpeed = std::numeric_limits<float>::infinity();
        if (m_PhysicsWorldEntity != INVALID_ENTITY && m_ComponentStore) {
            const auto& world = m_ComponentStore->GetComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity);
            maxLinearSpeed = world.maxLinearSpeed;
            maxAngularSpeed = world.maxAngularSpeed; }
        for (size_t i = start; i < end; ++i) {
            auto& body = m_SolverBodies[i];
            if (body.isStatic || !body.isAwake)
                continue;
            body.velocity += (body.force * body.invMass) * dt;
            body.angularVelocity += (body.torque * body.invInertia) * dt;
            if (body.linearDamping > 0.0f) {
                body.velocity *= std::pow(1.0f - body.linearDamping, dt); }
            if (body.angularDamping > 0.0f) {
                body.angularVelocity *= std::pow(1.0f - body.angularDamping, dt); }
            float speed = body.velocity.Length();
            if (speed > maxLinearSpeed && speed > 0.0f) {
                body.velocity = body.velocity * (maxLinearSpeed / speed); }
            body.angularVelocity = std::clamp(body.angularVelocity, -maxAngularSpeed, maxAngularSpeed);
            body.force = Math::Vector2{0.0f, 0.0f};
            body.torque = 0.0f; } }
    void PhysicsPipelineSystem::IntegratePositions(float dt) {
        for (auto& body : m_SolverBodies) {
            if (body.isStatic || !body.isAwake)
                continue;
            body.prevPosition = body.position;
            body.prevAngle = body.angle;
            body.position += body.velocity * dt;
            body.angle += body.angularVelocity * dt; } }
    void PhysicsPipelineSystem::ClearPersistentContacts() {
        for (auto& manifold : m_ContactManifolds) {
            manifold.persisted = false; } }
    void PhysicsPipelineSystem::ParallelBroadPhase() {
        m_BroadPhasePairs.clear();
        std::vector<uint32_t> entitiesToRemove;
        for (const auto& [entityId, proxyId] : m_ShapeProxyMap) {
            if (!m_ComponentStore->HasComponent<ColliderComponent>(entityId)) {
                entitiesToRemove.push_back(entityId); } }
        for (uint32_t entityId : entitiesToRemove) {
            uint32_t proxyId = m_ShapeProxyMap[entityId];
            m_BroadPhaseTree.DestroyProxy(proxyId);
            m_ShapeProxyMap.erase(entityId); }
        m_ComponentStore->ForEachComponent<ColliderComponent>([&](EntityID entityId, ColliderComponent& collider) {
                if (!m_ComponentStore->HasComponent<TransformComponent>(entityId))
                return;
                const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
                UpdateShapeAABB(entityId, &collider, transform.position, transform.rotation); });
        std::vector<std::future<std::vector<std::pair<uint32_t, uint32_t>>>> futures;
        for (const auto& [entityId, proxyId] : m_ShapeProxyMap) {
            if (!m_ComponentStore->HasComponent<PhysicsBodyComponent>(entityId))
                continue;
            const auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityId);
            if (body.isStatic)
                continue;
            futures.push_back(Utils::ThreadPool::Instance().Submit([this, entityId, proxyId]() -> std::vector<std::pair<uint32_t, uint32_t>> {
                std::vector<std::pair<uint32_t, uint32_t>> localPairs;
                const auto& fatAABB = m_BroadPhaseTree.GetFatAABB(proxyId);
                BroadPhaseCallback callback;
                callback.system = this;
                callback.entityId = entityId;
                callback.localPairs = &localPairs;
                m_BroadPhaseTree.Query(fatAABB, &callback);
                return localPairs; })); }
        for (auto& future : futures) {
            auto localPairs = future.get();
            m_BroadPhasePairs.insert(m_BroadPhasePairs.end(), localPairs.begin(), localPairs.end()); }
        m_Stats.broadPhasePairs = m_BroadPhasePairs.size(); }
    void PhysicsPipelineSystem::ParallelNarrowPhase() {
        m_ContactManifolds.clear();
        m_ContactMap.clear();
        if (m_PhysicsWorldEntity != INVALID_ENTITY && m_ComponentStore) {
            auto& world = m_ComponentStore->GetComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity);
            world.contactManifolds.clear(); }
        std::vector<std::future<ECS::ContactManifold>> futures;
        std::mutex manifoldsMutex;
        for (const auto& [entityIdA, entityIdB] : m_BroadPhasePairs) {
            futures.push_back(Utils::ThreadPool::Instance().Submit([this, entityIdA, entityIdB]() -> ECS::ContactManifold {
                ECS::ContactManifold manifold;
                manifold.entityIdA = entityIdA;
                manifold.entityIdB = entityIdB;
                if (!TestCollision(entityIdA, entityIdB)) {
                    return manifold;   }
                return GenerateManifold(entityIdA, entityIdB); })); }
        for (auto& future : futures) {
            ECS::ContactManifold manifold = future.get();
            if (!manifold.points.empty()) {
                std::lock_guard<std::mutex> lock(manifoldsMutex);
                uint64_t key = (static_cast<uint64_t>(std::min(manifold.entityIdA, manifold.entityIdB)) << 32) |
                    static_cast<uint64_t>(std::max(manifold.entityIdA, manifold.entityIdB));
                m_ContactMap[key] = m_ContactManifolds.size();
                m_ContactManifolds.push_back(std::move(manifold)); } }
        m_Stats.narrowPhaseContacts = m_ContactManifolds.size(); }
    void PhysicsPipelineSystem::ParallelVelocitySolving(float subStepDt) {
        std::vector<std::future<void>> futures;
        size_t batchSize = (m_SolverBodies.size() + m_NumThreads - 1) / m_NumThreads;
        for (size_t t = 0; t < m_NumThreads; ++t) {
            size_t start = t * batchSize;
            size_t end = std::min(start + batchSize, m_SolverBodies.size());
            if (start >= m_SolverBodies.size()) break;
            futures.push_back(Utils::ThreadPool::Instance().Submit([this, start, end, subStepDt]() {
                for (size_t i = start; i < end; ++i) {
                    auto& body = m_SolverBodies[i];
                    if (!body.isStatic && body.isAwake && m_PhysicsWorldEntity != INVALID_ENTITY && m_ComponentStore) {
                        float mass = (body.invMass > 0.0f) ? 1.0f / body.invMass : 0.0f;
                        const auto& world = m_ComponentStore->GetComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity);
                        body.force += world.gravity * mass; } }
                IntegrateVelocities(subStepDt, start, end); })); }
        for (auto& future : futures) {
            future.get(); }
        if (m_Config.warmStarting) {
            WarmStartConstraints(); }
        for (int i = 0; i < m_Config.velocityIterations; ++i) {
            SolveVelocityConstraints(); } }
    void PhysicsPipelineSystem::ParallelPositionSolving(float subStepDt) {
        IntegratePositions(subStepDt);
        for (int i = 0; i < m_Config.positionIterations; ++i) {
            SolvePositionConstraints(); } } }
