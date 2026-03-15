#include "nyon/ecs/systems/PhysicsPipelineSystem.h"
#include "nyon/ecs/components/JointComponent.h"
#include "nyon/physics/ManifoldGenerator.h"
#include <chrono>
#include <algorithm>
#include <iostream>
#define NYON_DEBUG_LOG(x) std::cerr << "[PHYSICS] " << x << std::endl
static int s_DebugFrameCounter = 0;
static constexpr int DEBUG_OUTPUT_INTERVAL = 25;  
#define NYON_DEBUG_LOG_EVERY(x) \
    do { \
        if (++s_DebugFrameCounter >= DEBUG_OUTPUT_INTERVAL) { \
            s_DebugFrameCounter = 0; \
            std::cerr << "[PHYSICS@" << s_DebugFrameCounter << "] " << x << std::endl; \ } \ } while(0)
namespace Nyon::ECS {
void PhysicsPipelineSystem::Initialize(EntityManager& entityManager, ComponentStore& componentStore) {
    m_ComponentStore = &componentStore;
    m_ComponentStore->ForEachComponent<PhysicsWorldComponent>([&](EntityID entityId, PhysicsWorldComponent& world) {
        m_PhysicsWorld = &world;
        NYON_DEBUG_LOG("Found PhysicsWorldComponent at entity ID: " << entityId); });
    if (!m_PhysicsWorld) {
        NYON_DEBUG_LOG("Warning: No PhysicsWorldComponent found!");
        return; }
    m_IslandManager = std::make_unique<Physics::IslandManager>(*m_ComponentStore);
    NYON_DEBUG_LOG("PhysicsPipelineSystem initialized"); }
    void PhysicsPipelineSystem::Update(float deltaTime) {
        if (!m_PhysicsWorld && m_ComponentStore) {
            m_ComponentStore->ForEachComponent<PhysicsWorldComponent>([&](EntityID entityId, PhysicsWorldComponent& world) {
                m_PhysicsWorld = &world;
                NYON_DEBUG_LOG("Found PhysicsWorldComponent at entity ID: " << entityId);
                if (!m_IslandManager) {
                    m_IslandManager = std::make_unique<Physics::IslandManager>(*m_ComponentStore);
                    NYON_DEBUG_LOG("IslandManager initialized via lazy init"); } }); }
        NYON_DEBUG_LOG("[PHYSICS] Update() called with deltaTime=" << deltaTime);
        if (!m_PhysicsWorld || !m_ComponentStore) {
            NYON_DEBUG_LOG_EVERY("[PHYSICS] Early exit - m_PhysicsWorld=" << (m_PhysicsWorld ? "valid" : "null") 
                          << ", m_ComponentStore=" << (m_ComponentStore ? "valid" : "null"));
            return; }
        if (!m_IslandManager) {
            NYON_DEBUG_LOG("Initializing IslandManager...");
            m_IslandManager = std::make_unique<Physics::IslandManager>(*m_ComponentStore); }
        auto startTime = std::chrono::high_resolution_clock::now();
        NYON_DEBUG_LOG("[PHYSICS] Processing physics step with deltaTime=" << deltaTime);
        m_ActiveEntities.clear();
        m_ComponentStore->ForEachComponent<PhysicsBodyComponent>([&](EntityID entityId, const PhysicsBodyComponent& body) {
            if (!body.isStatic) {
                m_ActiveEntities.push_back(entityId); } });
        NYON_DEBUG_LOG("[PHYSICS] Found " << m_ActiveEntities.size() << " active entities");
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
        NYON_DEBUG_LOG("[PHYSICS] Physics update completed in " << m_Stats.updateTime << " ms"); }
    void PhysicsPipelineSystem::PrepareBodiesForUpdate() {
        m_SolverBodies.clear();
        m_EntityToSolverIndex.clear();
        size_t solverIndex = 0;
        m_ComponentStore->ForEachComponent<PhysicsBodyComponent>([&](EntityID entityId, PhysicsBodyComponent& body) {
            bool shouldInclude = false;
            if (body.isStatic) {
                shouldInclude = true; }
            else {
                bool islandAwake = m_IslandManager->IsBodyAwake(entityId);
                bool componentAwake = body.isAwake;
                shouldInclude = islandAwake || componentAwake; }
            if (!shouldInclude) {
                return;   }
            if (!body.isStatic && m_ComponentStore->HasComponent<ColliderComponent>(entityId)) {
                const auto& collider = m_ComponentStore->GetComponent<ColliderComponent>(entityId);
                float area = collider.CalculateArea();
                float density = collider.material.density;
                float calculatedMass = area * density;
                if (body.mass <= 0.0f || std::abs(body.mass - calculatedMass) > 0.01f) {
                    body.SetMass(calculatedMass); }
                float unitInertia = collider.CalculateInertiaForUnitDensity();
                float calculatedInertia = unitInertia * density;
                if (body.inertia <= 0.0f || std::abs(body.inertia - calculatedInertia) > 0.01f) {
                    body.SetInertia(calculatedInertia); } }
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
            body.ClearForces();
            m_SolverBodies.push_back(solverBody);
            m_EntityToSolverIndex[entityId] = solverIndex++; });
        NYON_DEBUG_LOG_EVERY("Prepared " << m_SolverBodies.size() << " solver bodies:");
        for (size_t i = 0; i < std::min(m_SolverBodies.size(), (size_t)5); ++i) {
            const auto& body = m_SolverBodies[i];
            NYON_DEBUG_LOG_EVERY("  Body[" << i << "] entity=" << body.entityId 
                              << " pos=(" << body.position.x << "," << body.position.y 
                              << ") vel=(" << body.velocity.x << "," << body.velocity.y 
                              << ") invMass=" << body.invMass 
                              << " static=" << body.isStatic 
                              << " awake=" << body.isAwake); } }
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
        NYON_DEBUG_LOG_EVERY("BroadPhaseDetection: " << m_ShapeProxyMap.size() << " proxies in map, " 
                          << m_ActiveEntities.size() << " active entities");
        m_ComponentStore->ForEachComponent<ColliderComponent>([&](EntityID entityId, ColliderComponent& collider) {
            if (!m_ComponentStore->HasComponent<TransformComponent>(entityId))
                return;
            const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
            NYON_DEBUG_LOG_EVERY("  Entity " << entityId 
                              << " pos=(" << transform.position.x << "," << transform.position.y << ")"
                              << " colliderType=" << (int)collider.type);
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
        NYON_DEBUG_LOG_EVERY("Broad phase found " << m_BroadPhasePairs.size() << " potential pairs");
        if (!m_BroadPhasePairs.empty()) {
            for (size_t i = 0; i < std::min(m_BroadPhasePairs.size(), (size_t)5); ++i) {
                const auto& [entityA, entityB] = m_BroadPhasePairs[i];
                NYON_DEBUG_LOG_EVERY("  Pair[" << i << "] entityA=" << entityA << " entityB=" << entityB); } } }
    void PhysicsPipelineSystem::NarrowPhaseDetection() {
        m_ContactManifolds.clear();
        m_ContactMap.clear();
        if (m_PhysicsWorld) {
            m_PhysicsWorld->contactManifolds.clear(); }
        for (const auto& [entityIdA, entityIdB] : m_BroadPhasePairs) {
            if (TestCollision(entityIdA, entityIdB)) {
                ContactManifold manifold = GenerateManifold(entityIdA, entityIdB);
                if (!manifold.points.empty()) {
                    uint64_t key = (static_cast<uint64_t>(std::min(entityIdA, entityIdB)) << 32) | 
                                  static_cast<uint64_t>(std::max(entityIdA, entityIdB));
                    m_ContactMap[key] = m_ContactManifolds.size();
                    m_ContactManifolds.push_back(std::move(manifold));
                    if (m_PhysicsWorld) {
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
                            worldManifold.points.push_back(std::move(cp)); }
                        m_PhysicsWorld->contactManifolds.push_back(std::move(worldManifold)); } } } }
        m_Stats.narrowPhaseContacts = m_ContactManifolds.size();
        NYON_DEBUG_LOG_EVERY("Narrow phase generated " << m_ContactManifolds.size() << " contact manifolds");
        if (!m_ContactManifolds.empty()) {
            for (size_t i = 0; i < std::min(m_ContactManifolds.size(), (size_t)5); ++i) {
                const auto& manifold = m_ContactManifolds[i];
                NYON_DEBUG_LOG_EVERY("  Contact[" << i << "] entityA=" << manifold.entityIdA 
                                  << " entityB=" << manifold.entityIdB 
                                  << " points=" << manifold.points.size()
                                  << " normal=(" << manifold.normal.x << "," << manifold.normal.y << ")"); } } }
    void PhysicsPipelineSystem::IslandDetection() {
        if (m_Config.useIslandSleeping) {
            m_IslandManager->UpdateIslands(FIXED_TIMESTEP, m_ActiveEntities);
            m_Stats.islandStats = m_IslandManager->GetStatistics(); } }
    void PhysicsPipelineSystem::ConstraintInitialization() {
        m_VelocityConstraints.clear();
        for (const auto& manifold : m_ContactManifolds) {
            VelocityConstraint vc;
            vc.normal = manifold.normal;
            vc.tangent = Math::Vector2{-manifold.normal.y, manifold.normal.x};
            vc.points = manifold.points;
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
                for (auto& point : vc.points) {
                    Math::Vector2 rA = point.position - bodyA.position;
                    Math::Vector2 rB = point.position - bodyB.position;
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
                    if (vRel < -m_PhysicsWorld->restitutionThreshold) {
                        point.velocityBias = -vc.restitution * vRel; }
                    else {
                        point.velocityBias = 0.0f; }
                    uint64_t cacheKey = MakeImpulseCacheKey(manifold.entityIdA, manifold.entityIdB, point.featureId);
                    auto cacheIt = m_ImpulseCache.find(cacheKey);
                    if (cacheIt != m_ImpulseCache.end()) {
                        point.normalImpulse = cacheIt->second.normalImpulse;
                        point.tangentImpulse = cacheIt->second.tangentImpulse; } }
                m_VelocityConstraints.push_back(vc); } }
        m_Stats.activeConstraints = m_VelocityConstraints.size();
        NYON_DEBUG_LOG("Initialized " << m_VelocityConstraints.size() << " velocity constraints"); }
    void PhysicsPipelineSystem::VelocitySolving() {
        if (m_Config.warmStarting) {
            WarmStartConstraints(); }
        for (auto& body : m_SolverBodies) {
            if (!body.isStatic && body.isAwake && m_PhysicsWorld) {
                float mass = (body.invMass > 0.0f) ? 1.0f / body.invMass : 0.0f;
                body.force += m_PhysicsWorld->gravity * mass; } }
        IntegrateVelocities(FIXED_TIMESTEP);
        for (int i = 0; i < m_Config.velocityIterations; ++i) {
            SolveVelocityConstraints(); } }
    void PhysicsPipelineSystem::PositionSolving() {
        NYON_DEBUG_LOG_EVERY("Position solving: starting with " << m_VelocityConstraints.size() << " constraints");
        IntegratePositions(FIXED_TIMESTEP);
        for (int i = 0; i < m_Config.positionIterations; ++i) {
            NYON_DEBUG_LOG_EVERY("  Position iteration " << i);
            SolvePositionConstraints(); }
        NYON_DEBUG_LOG_EVERY("Position solving completed. Sample body positions:");
        for (size_t i = 0; i < std::min(m_SolverBodies.size(), (size_t)3); ++i) {
            const auto& body = m_SolverBodies[i];
            NYON_DEBUG_LOG_EVERY("  Body[" << i << "] pos=(" << body.position.x << "," << body.position.y 
                              << ") angle=" << body.angle); } }
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
            if (!body.isStatic) {
                body.isAwake = m_IslandManager->IsBodyAwake(entityId); } });
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
        NYON_DEBUG_LOG_EVERY("Updated " << m_SolverBodies.size() << " bodies:");
        for (size_t i = 0; i < std::min(m_SolverBodies.size(), (size_t)3); ++i) {
            const auto& body = m_SolverBodies[i];
            NYON_DEBUG_LOG_EVERY("  Body[" << i << "] entity=" << body.entityId 
                              << " pos=(" << body.position.x << "," << body.position.y 
                              << ") vel=(" << body.velocity.x << "," << body.velocity.y 
                              << ") static=" << body.isStatic); } }
    bool PhysicsPipelineSystem::BroadPhaseCallback::QueryCallback(uint32_t nodeId, uint32_t userData) {
        uint32_t otherEntityId = userData;
        static int s_BroadPhaseDebugCounter = 0;
        const int BROAD_PHASE_DEBUG_INTERVAL = 50;  
        if (++s_BroadPhaseDebugCounter >= BROAD_PHASE_DEBUG_INTERVAL) {
            s_BroadPhaseDebugCounter = 0;
            std::cerr << "[PHYSICS@0]   QueryCallback: querying entity=" << entityId 
                      << " found node=" << nodeId << " otherEntity=" << otherEntityId << std::endl; }
        if (otherEntityId <= entityId) {
            if (++s_BroadPhaseDebugCounter >= BROAD_PHASE_DEBUG_INTERVAL) {
                s_BroadPhaseDebugCounter = 0;
                std::cerr << "[PHYSICS@0]     Skipping: otherEntityId <= entityId" << std::endl; }
            return true; }
        if (system->m_ComponentStore->HasComponent<ColliderComponent>(entityId) &&
            system->m_ComponentStore->HasComponent<ColliderComponent>(otherEntityId)) {
            const auto& colliderA = system->m_ComponentStore->GetComponent<ColliderComponent>(entityId);
            const auto& colliderB = system->m_ComponentStore->GetComponent<ColliderComponent>(otherEntityId);
            if (colliderA.filter.ShouldCollide(colliderB.filter)) {
                system->m_BroadPhasePairs.emplace_back(entityId, otherEntityId);
                if (++s_BroadPhaseDebugCounter >= BROAD_PHASE_DEBUG_INTERVAL) {
                    s_BroadPhaseDebugCounter = 0;
                    std::cerr << "[PHYSICS@0]     Added pair: (" << entityId << "," << otherEntityId << ")" << std::endl; } }
            else {
                if (++s_BroadPhaseDebugCounter >= BROAD_PHASE_DEBUG_INTERVAL) {
                    s_BroadPhaseDebugCounter = 0;
                    std::cerr << "[PHYSICS@0]     Filtered out by collision filter" << std::endl; } } }
        else {
            if (++s_BroadPhaseDebugCounter >= BROAD_PHASE_DEBUG_INTERVAL) {
                s_BroadPhaseDebugCounter = 0;
                std::cerr << "[PHYSICS@0]     One or both entities missing ColliderComponent" << std::endl; } }
        return true;   }
    void PhysicsPipelineSystem::UpdateShapeAABB(uint32_t entityId, ColliderComponent* collider, 
                                              const Math::Vector2& position, float angle) {
        Math::Vector2 min, max;
        collider->CalculateAABB(position, angle, min, max);
        Physics::AABB aabb;
        aabb.lowerBound = {min.x, min.y};
        aabb.upperBound = {max.x, max.y};
        NYON_DEBUG_LOG_EVERY("  AABB entity=" << entityId 
                          << " center=(" << position.x << "," << position.y << ")"
                          << " bounds=[(" << aabb.lowerBound.x << "," << aabb.lowerBound.y << ")-"
                          << "(" << aabb.upperBound.x << "," << aabb.upperBound.y << ")]");
        auto it = m_ShapeProxyMap.find(entityId);
        if (it != m_ShapeProxyMap.end()) {
            Math::Vector2 displacement = {0.0f, 0.0f};
            if (m_ComponentStore->HasComponent<PhysicsBodyComponent>(entityId)) {
                const auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityId);
                displacement = body.velocity * FIXED_TIMESTEP; }
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
    PhysicsPipelineSystem::ContactManifold PhysicsPipelineSystem::GenerateManifold(uint32_t entityIdA, uint32_t entityIdB) {
        ContactManifold manifold;
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
        manifold.points.reserve(generatedManifold.points.size());
        manifold.normal = generatedManifold.normal;
        for (size_t i = 0; i < generatedManifold.points.size(); ++i) {
            const auto& point = generatedManifold.points[i];
            ContactPoint contactPoint;
            contactPoint.position = point.position;
            contactPoint.normal = point.normal;
            contactPoint.separation = point.separation;
            contactPoint.normalImpulse = 0.0f;
            contactPoint.tangentImpulse = 0.0f;
            contactPoint.featureId = point.featureId != 0 ? point.featureId : (i + 1);
            manifold.points.push_back(contactPoint); }
        return manifold; }
    void PhysicsPipelineSystem::WarmStartConstraints() {
        for (auto& constraint : m_VelocityConstraints) {
            const auto& bodyA = m_SolverBodies[constraint.indexA];
            const auto& bodyB = m_SolverBodies[constraint.indexB];
            for (auto& point : constraint.points) {
                Math::Vector2 P = point.normal * point.normalImpulse + 
                                constraint.tangent * point.tangentImpulse;
                if (!bodyA.isStatic) {
                    m_SolverBodies[constraint.indexA].velocity -= P * constraint.invMassA;
                    m_SolverBodies[constraint.indexA].angularVelocity -= 
                        constraint.invIA * Math::Vector2::Cross(point.position - bodyA.position, P); }
                if (!bodyB.isStatic) {
                    m_SolverBodies[constraint.indexB].velocity += P * constraint.invMassB;
                    m_SolverBodies[constraint.indexB].angularVelocity += 
                        constraint.invIB * Math::Vector2::Cross(point.position - bodyB.position, P); } } } }
    uint64_t PhysicsPipelineSystem::MakeImpulseCacheKey(uint32_t entityIdA, uint32_t entityIdB, uint32_t featureId) const {
        uint32_t minEntity = std::min(entityIdA, entityIdB);
        uint32_t maxEntity = std::max(entityIdA, entityIdB);
        uint64_t pairKey = (static_cast<uint64_t>(minEntity) << 32) | static_cast<uint64_t>(maxEntity);
        return pairKey ^ (static_cast<uint64_t>(featureId) << 32); }
    void PhysicsPipelineSystem::SolveVelocityConstraints() {
        for (auto& constraint : m_VelocityConstraints) {
            auto& bodyA = m_SolverBodies[constraint.indexA];
            auto& bodyB = m_SolverBodies[constraint.indexB];
            for (auto& point : constraint.points) {
                Math::Vector2 rA = point.position - bodyA.position;
                Math::Vector2 rB = point.position - bodyB.position;
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
            for (const auto& point : constraint.points) {
                if (point.separation < -m_Config.linearSlop) {
                    Math::Vector2 rA = point.position - bodyA.position;
                    Math::Vector2 rB = point.position - bodyB.position;
                    float C = m_Config.baumgarte * (-point.separation - m_Config.linearSlop);
                    C = std::min(C, m_Config.maxLinearCorrection);
                    float rAcrossN = Math::Vector2::Cross(rA, constraint.normal);
                    float rBcrossN = Math::Vector2::Cross(rB, constraint.normal);
                    float kNormal = constraint.invMassA + constraint.invMassB
                                  + constraint.invIA * rAcrossN * rAcrossN
                                  + constraint.invIB * rBcrossN * rBcrossN;
                    float mass = (kNormal > 1e-6f) ? 1.0f / kNormal : 0.0f;
                    Math::Vector2 P = constraint.normal * (C * mass);
                    NYON_DEBUG_LOG_EVERY("  Contact correction: separation=" << point.separation 
                                      << " normal=(" << constraint.normal.x << "," << constraint.normal.y << ")"
                                      << " correction=(" << P.x << "," << P.y << ")");
                    if (!bodyA.isStatic && !bodyB.isStatic) {
                        bodyA.position -= P * constraint.invMassA;
                        bodyB.position += P * constraint.invMassB;
                        correctionsApplied++; }
                    else if (!bodyA.isStatic) {
                        bodyA.position -= P * constraint.invMassA;
                        correctionsApplied++; }
                    else if (!bodyB.isStatic) {
                        bodyB.position += P * constraint.invMassB;
                        correctionsApplied++; } } } }
        NYON_DEBUG_LOG_EVERY("Position constraints solved with " << correctionsApplied << " corrections applied"); }
    void PhysicsPipelineSystem::IntegrateVelocities(float dt) {
        for (auto& body : m_SolverBodies) {
            if (body.isStatic || !body.isAwake)
                continue;
            body.velocity += (body.force * body.invMass) * dt;
            body.angularVelocity += (body.torque * body.invInertia) * dt;
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
            manifold.persisted = false; } } }