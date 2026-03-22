#include "nyon/ecs/systems/ParticlePipelineSystem.h"
#include "nyon/ecs/components/ParticleComponent.h"
#include "nyon/ecs/components/ParticleEmitterComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
#include <algorithm>
#include <cmath>
namespace Nyon::ECS {
    ParticlePipelineSystem::ParticlePipelineSystem() {
        Utils::ThreadPool::Initialize();
        m_NumThreads = Utils::ThreadPool::Instance().GetThreadCount(); }
    void ParticlePipelineSystem::Initialize(EntityManager& entityManager, ComponentStore& componentStore) {
        m_ComponentStore = &componentStore;
        componentStore.ForEachComponent<PhysicsWorldComponent>([&](EntityID entityId, PhysicsWorldComponent& world) {
            m_PhysicsWorldEntity = entityId;
            m_Gravity = world.gravity;    }); }
    void ParticlePipelineSystem::Update(float deltaTime) {
        if (!m_ComponentStore) return;
        if (m_PhysicsWorldEntity != INVALID_ENTITY && 
            m_ComponentStore->HasComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity)) {
            const auto& world = m_ComponentStore->GetComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity);
            m_Gravity = world.gravity;
            m_EnableSleep = world.enableSleep;
            m_MaxLinearSpeed = world.maxLinearSpeed; }
        ProcessEmitters(deltaTime);
        if (m_ActiveParticles.empty())
            return;
        const size_t particleCount = m_ActiveParticles.size();
        std::vector<std::future<void>> physicsFutures;
        size_t batchSize = (particleCount + m_NumThreads - 1) / m_NumThreads;
        for (size_t t = 0; t < m_NumThreads; ++t) {
            size_t start = t * batchSize;
            size_t end = std::min(start + batchSize, particleCount);
            if (start >= particleCount) break;
            physicsFutures.push_back(
                Utils::ThreadPool::Instance().Submit([this, start, end, deltaTime]() {
                    UpdateParticlePhysicsParallel(start, end, deltaTime); })
            ); }
        for (auto& future : physicsFutures) {
            future.get(); }
        if (m_EnableCollisions && particleCount > 1) {
            BuildSpatialHash(m_CellSize);
            if (m_UseSpatialHash && particleCount > 100) {
                DetectParticleCollisionsParallel(); }
            else {
                DetectCollisionsBruteForce(); } }
        DetectParticleBodyCollisions();
        ProcessParticleLifecycle(deltaTime);
        CleanupDeadParticles(); }
    void ParticlePipelineSystem::UpdateParticlePhysicsParallel(size_t startIndex, size_t endIndex, float dt) {
        for (size_t i = startIndex; i < endIndex; ++i) {
            EntityID entityId = m_ActiveParticles[i];
            if (!m_ComponentStore->HasComponent<PhysicsBodyComponent>(entityId) ||
                !m_ComponentStore->HasComponent<TransformComponent>(entityId) ||
                !m_ComponentStore->HasComponent<ParticleComponent>(entityId))
                continue;
            auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityId);
            auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
            auto& particle = m_ComponentStore->GetComponent<ParticleComponent>(entityId);
            Math::Vector2 previousPosition = transform.position;
            float previousAlpha = particle.alpha;
            float previousSizeScale = particle.sizeScale;
            bool shouldSleep = false;
            if (m_EnableSleep) {
                float speedSq = body.velocity.LengthSquared();
                shouldSleep = (speedSq < 0.01f);   }
            if (!shouldSleep) {
                float gravityScale = m_GravityScale;
                if (particle.emitterEntityId != INVALID_ENTITY &&
                    m_ComponentStore->HasComponent<ParticleEmitterComponent>(particle.emitterEntityId)) {
                    const auto& emitter = m_ComponentStore->GetComponent<ParticleEmitterComponent>(particle.emitterEntityId);
                    gravityScale = emitter.gravityScale; }
                if (body.inverseMass > 0.0f) {
                    body.velocity += m_Gravity * gravityScale * dt; }
                if (body.drag > 0.0f) {
                    float damping = std::exp(-body.drag * dt);
                    body.velocity *= damping; }
                if (m_MaxLinearSpeed > 0.0f) {
                    float speedSq = body.velocity.LengthSquared();
                    if (speedSq > m_MaxLinearSpeed * m_MaxLinearSpeed) {
                        float speed = std::sqrt(speedSq);
                        body.velocity *= (m_MaxLinearSpeed / speed); } }
                transform.position += body.velocity * dt; }
            transform.previousPosition = previousPosition;
            if (particle.lifetime > 0.0f) {
                particle.age += dt;
                float t = particle.age / particle.lifetime;
                t = std::clamp(t, 0.0f, 1.0f);
                particle.alpha = particle.alphaStart + (particle.alphaEnd - particle.alphaStart) * t;
                particle.sizeScale = particle.sizeScale;  
                if (particle.age >= particle.lifetime) {
                    particle.alive = false; } }
            particle.prevAlpha = previousAlpha;
            particle.prevSizeScale = previousSizeScale; } }
    void ParticlePipelineSystem::ProcessCollisionPair(size_t i, size_t j) {
        EntityID entityIdA = m_ActiveParticles[i];
        EntityID entityIdB = m_ActiveParticles[j];
        if (!m_ComponentStore->HasComponent<TransformComponent>(entityIdA) ||
            !m_ComponentStore->HasComponent<TransformComponent>(entityIdB) ||
            !m_ComponentStore->HasComponent<PhysicsBodyComponent>(entityIdA) ||
            !m_ComponentStore->HasComponent<PhysicsBodyComponent>(entityIdB) ||
            !m_ComponentStore->HasComponent<ColliderComponent>(entityIdA) ||
            !m_ComponentStore->HasComponent<ColliderComponent>(entityIdB))
            return;
        auto& transformA = m_ComponentStore->GetComponent<TransformComponent>(entityIdA);
        auto& transformB = m_ComponentStore->GetComponent<TransformComponent>(entityIdB);
        auto& bodyA = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityIdA);
        auto& bodyB = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityIdB);
        auto& colliderA = m_ComponentStore->GetComponent<ColliderComponent>(entityIdA);
        auto& colliderB = m_ComponentStore->GetComponent<ColliderComponent>(entityIdB);
        float dx = transformB.position.x - transformA.position.x;
        float dy = transformB.position.y - transformA.position.y;
        float distSq = dx * dx + dy * dy;
        float minDist = colliderA.GetCircle().radius + colliderB.GetCircle().radius;
        float minDistSq = minDist * minDist;
        if (distSq < minDistSq && distSq > 0.001f) {
            float dist = std::sqrt(distSq);
            float nx = dx / dist;
            float ny = dy / dist;
            float overlap = minDist - dist;
            float invMass1 = bodyA.inverseMass;
            float invMass2 = bodyB.inverseMass;
            float totalInvMass = invMass1 + invMass2;
            float correctionX = nx * overlap * (invMass1 / totalInvMass);
            float correctionY = ny * overlap * (invMass1 / totalInvMass);
            transformA.position -= Math::Vector2(correctionX, correctionY);
            transformB.position += Math::Vector2(correctionX, correctionY);
            float relativeVx = bodyB.velocity.x - bodyA.velocity.x;
            float relativeVy = bodyB.velocity.y - bodyA.velocity.y;
            float velAlongNormal = relativeVx * nx + relativeVy * ny;
            if (velAlongNormal < 0) {
                float restitution = m_Restitution;   
                float colliderRestitution = (colliderA.material.restitution + colliderB.material.restitution) * 0.5f;
                if (colliderRestitution > 0.0f) {
                    restitution = colliderRestitution; }
                float impulse = -(1.0f + restitution) * velAlongNormal / totalInvMass;
                bodyA.velocity -= Math::Vector2(nx * impulse * invMass1, ny * impulse * invMass1);
                bodyB.velocity += Math::Vector2(nx * impulse * invMass2, ny * impulse * invMass2); } } }
    void ParticlePipelineSystem::DetectCollisionsBruteForce() {
        const size_t particleCount = m_ActiveParticles.size();
        for (size_t i = 0; i < particleCount; ++i) {
            for (size_t j = i + 1; j < particleCount; ++j) {
                ProcessCollisionPair(i, j); } } }
    void ParticlePipelineSystem::BuildSpatialHash(float cellSize) {
        m_SpatialHash.clear();
        m_CellSize = cellSize;
        const size_t particleCount = m_ActiveParticles.size();
        std::vector<std::vector<std::pair<int, int>>> threadResults(m_NumThreads);
        std::vector<std::future<void>> futures;
        size_t batchSize = (particleCount + m_NumThreads - 1) / m_NumThreads;
        for (size_t t = 0; t < m_NumThreads; ++t) {
            size_t start = t * batchSize;
            size_t end = std::min(start + batchSize, particleCount);
            if (start >= particleCount) break;
            futures.push_back(
                Utils::ThreadPool::Instance().Submit([this, t, start, end, cellSize, &threadResults]() {
                    threadResults[t] = ComputeCellIndices(start, end, cellSize); })
            ); }
        for (auto& future : futures) {
            future.get(); }
        for (const auto& results : threadResults) {
            for (const auto& [cellKey, particleIdx] : results) {
                EntityID entityId = m_ActiveParticles[particleIdx];
                m_SpatialHash[cellKey].particleEntities.push_back(entityId); } } }
    std::vector<std::pair<int, int>> ParticlePipelineSystem::ComputeCellIndices(
        size_t startIndex, size_t endIndex, float cellSize) {
        std::vector<std::pair<int, int>> results;
        results.reserve(endIndex - startIndex);
        for (size_t i = startIndex; i < endIndex; ++i) {
            EntityID entityId = m_ActiveParticles[i];
            if (!m_ComponentStore->HasComponent<TransformComponent>(entityId))
                continue;
            const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
            int cellX = static_cast<int>(transform.position.x / cellSize);
            int cellY = static_cast<int>(transform.position.y / cellSize);
            int cellKey = (cellX << 16) | (cellY & 0xFFFF);
            results.emplace_back(cellKey, static_cast<int>(i)); }
        return results; }
    void ParticlePipelineSystem::ProcessEmitters(float deltaTime) {
        if (!m_ComponentStore) return;
        m_ComponentStore->ForEachComponent<ParticleEmitterComponent>([&](EntityID entityId, ParticleEmitterComponent& emitter) {
            if (!emitter.active)
                return;
            if (emitter.currentCount >= emitter.maxParticles && emitter.burstCount == 0) {
                if (!emitter.loop) {
                    emitter.active = false;
                    return; }
                if (emitter.spawnRate <= 0.0f)
                    return; }
            if (emitter.spawnRate > 0.0f) {
                emitter.spawnTimer += deltaTime;
                while (emitter.spawnTimer >= 1.0f / emitter.spawnRate) {
                    emitter.spawnTimer -= 1.0f / emitter.spawnRate;
                    if (emitter.currentCount < emitter.maxParticles) {
                        Math::Vector2 spawnPos = SampleSpawnPosition(emitter);
                        SpawnParticleFromEmitter(entityId, spawnPos); } } }
            if (emitter.burstCount > 0 && emitter.currentCount < emitter.maxParticles) {
                uint32_t toSpawn = std::min(emitter.burstCount, emitter.maxParticles - emitter.currentCount);
                for (uint32_t i = 0; i < toSpawn; ++i) {
                    Math::Vector2 spawnPos = SampleSpawnPosition(emitter);
                    SpawnParticleFromEmitter(entityId, spawnPos); }
                emitter.burstCount = 0;    } }); }
    void ParticlePipelineSystem::DetectParticleCollisionsParallel() {
        const size_t particleCount = m_ActiveParticles.size();
        std::vector<std::pair<EntityID, EntityID>> collisionPairs;
        collisionPairs.reserve(particleCount * 4);
        for (const auto& [key, cell] : m_SpatialHash) {
            if (cell.particleEntities.empty()) continue;
            const int keyX = key >> 16;
            const int keyY = key & 0xFFFF;
            for (int dy = 0; dy <= 1; ++dy) {
                for (int dx = (dy == 0) ? 1 : -1; dx <= 1; ++dx) {
                    int neighborKey = ((keyX + dx) << 16) | (keyY + dy);
                    auto neighborIt = m_SpatialHash.find(neighborKey);
                    if (neighborIt == m_SpatialHash.end()) continue;
                    const auto& neighborCell = neighborIt->second;
                    for (EntityID idA : cell.particleEntities) {
                        for (EntityID idB : neighborCell.particleEntities) {
                            if (idA < idB) {
                                collisionPairs.emplace_back(idA, idB); } } } } } }
        std::vector<std::future<void>> collisionFutures;
        const size_t pairCount = collisionPairs.size();
        if (pairCount == 0) return;
        size_t pairBatchSize = (pairCount + m_NumThreads - 1) / m_NumThreads;
        for (size_t t = 0; t < m_NumThreads; ++t) {
            size_t pairStart = t * pairBatchSize;
            size_t pairEnd = std::min(pairStart + pairBatchSize, pairCount);
            if (pairStart >= pairCount) break;
            collisionFutures.push_back(
                Utils::ThreadPool::Instance().Submit([this, pairStart, pairEnd, &collisionPairs]() {
                    for (size_t p = pairStart; p < pairEnd; ++p) {
                        ProcessCollisionPair(collisionPairs[p].first, collisionPairs[p].second); } })
            ); }
        for (auto& future : collisionFutures) {
            future.get(); } }
    void ParticlePipelineSystem::ProcessParticleLifecycle(float deltaTime) {
        for (EntityID entityId : m_ActiveParticles) {
            if (!m_ComponentStore->HasComponent<ParticleComponent>(entityId))
                continue;
            auto& particle = m_ComponentStore->GetComponent<ParticleComponent>(entityId);
            if (!particle.alive || (particle.lifetime > 0.0f && particle.age >= particle.lifetime)) {
                if (particle.emitterEntityId != INVALID_ENTITY &&
                    m_ComponentStore->HasComponent<ParticleEmitterComponent>(particle.emitterEntityId)) {
                    auto& emitter = m_ComponentStore->GetComponent<ParticleEmitterComponent>(particle.emitterEntityId);
                    if (emitter.onDeath) {
                        emitter.onDeath(entityId); } } } } }
    void ParticlePipelineSystem::CleanupDeadParticles() {
        auto it = m_ActiveParticles.begin();
        while (it != m_ActiveParticles.end()) {
            EntityID entityId = *it;
            if (m_ComponentStore->HasComponent<ParticleComponent>(entityId)) {
                auto& particle = m_ComponentStore->GetComponent<ParticleComponent>(entityId);
                if (!particle.alive || (particle.lifetime > 0.0f && particle.age >= particle.lifetime)) {
                    if (particle.emitterEntityId != INVALID_ENTITY &&
                        m_ComponentStore->HasComponent<ParticleEmitterComponent>(particle.emitterEntityId)) {
                        auto& emitter = m_ComponentStore->GetComponent<ParticleEmitterComponent>(particle.emitterEntityId);
                        if (emitter.currentCount > 0) {
                            emitter.currentCount--; } }
                    it = m_ActiveParticles.erase(it);
                    continue; } }
            ++it; }
        m_ComponentStore->ForEachComponent<ParticleEmitterComponent>([&](EntityID entityId, ParticleEmitterComponent& emitter) {
            uint32_t activeCount = 0;
            for (EntityID particleId : m_ActiveParticles) {
                if (m_ComponentStore->HasComponent<ParticleComponent>(particleId)) {
                    auto& particle = m_ComponentStore->GetComponent<ParticleComponent>(particleId);
                    if (particle.emitterEntityId == entityId && particle.alive) {
                        activeCount++; } } }
            emitter.currentCount = activeCount; }); }
    Math::Vector2 ParticlePipelineSystem::SampleSpawnPosition(const ParticleEmitterComponent& emitter) const {
        std::uniform_real_distribution<float> uniformDist(0.0f, 1.0f);
        std::uniform_real_distribution<float> angleDist(0.0f, 2.0f * 3.14159265359f);
        switch (emitter.emissionShape) {
            case ParticleEmitterComponent::EmissionShape::Point: {
                return {0.0f, 0.0f}; }
            case ParticleEmitterComponent::EmissionShape::Circle: {
                float angle = angleDist(m_Rng);
                float radius = std::sqrt(uniformDist(m_Rng)) * emitter.emissionRadius;
                return {radius * std::cos(angle), radius * std::sin(angle)}; }
            case ParticleEmitterComponent::EmissionShape::Rectangle: {
                return {
                    (uniformDist(m_Rng) - 0.5f) * emitter.emissionSize.x,
                    (uniformDist(m_Rng) - 0.5f) * emitter.emissionSize.y }; }
            case ParticleEmitterComponent::EmissionShape::Annulus: {
                float angle = angleDist(m_Rng);
                float minR = emitter.emissionInnerRadius;
                float maxR = emitter.emissionRadius;
                float radius = std::sqrt(uniformDist(m_Rng) * (maxR * maxR - minR * minR) + minR * minR);
                return {radius * std::cos(angle), radius * std::sin(angle)}; }
            default:
                return {0.0f, 0.0f}; } }
    void ParticlePipelineSystem::SpawnParticleFromEmitter(EntityID emitterEntityId, const Math::Vector2& spawnPosition) {
        if (!m_ComponentStore || !m_ComponentStore->HasComponent<ParticleEmitterComponent>(emitterEntityId))
            return;
        const auto& emitter = m_ComponentStore->GetComponent<ParticleEmitterComponent>(emitterEntityId);
        const auto& params = emitter.spawnParams;
        std::uniform_real_distribution<float> speedDist(params.minSpeed, params.maxSpeed);
        std::uniform_real_distribution<float> angleDist(
            params.minAngleDeg * 3.14159265359f / 180.0f,
            params.maxAngleDeg * 3.14159265359f / 180.0f
        );
        std::uniform_real_distribution<float> radiusDist(params.minRadius, params.maxRadius);
        std::uniform_real_distribution<float> massDist(params.minMass, params.maxMass);
        std::uniform_real_distribution<float> lifetimeDist(params.minLifetime, params.maxLifetime);
        std::uniform_real_distribution<float> colorDist(0.0f, 1.0f);
        float speed = speedDist(m_Rng);
        float angle = angleDist(m_Rng);
        float radius = radiusDist(m_Rng);
        float mass = massDist(m_Rng);
        float lifetime = lifetimeDist(m_Rng);
        Math::Vector2 velocity{
            speed * std::cos(angle),
            speed * std::sin(angle) };
        Math::Vector3 colorStart{
            colorDist(m_Rng) * (params.colorStartMax.x - params.colorStartMin.x) + params.colorStartMin.x,
            colorDist(m_Rng) * (params.colorStartMax.y - params.colorStartMin.y) + params.colorStartMin.y,
            colorDist(m_Rng) * (params.colorStartMax.z - params.colorStartMin.z) + params.colorStartMin.z };
        Math::Vector3 colorEnd{
            colorDist(m_Rng) * (params.colorEndMax.x - params.colorEndMin.x) + params.colorEndMin.x,
            colorDist(m_Rng) * (params.colorEndMax.y - params.colorEndMin.y) + params.colorEndMin.y,
            colorDist(m_Rng) * (params.colorEndMax.z - params.colorEndMin.z) + params.colorEndMin.z };
        if (emitter.onSpawn) { } }
    void ParticlePipelineSystem::DetectParticleBodyCollisions() {
        if (!m_ComponentStore || m_ActiveParticles.empty())
            return;
        EntityID worldEntity = INVALID_ENTITY;
        PhysicsWorldComponent* physicsWorld = nullptr;
        m_ComponentStore->ForEachComponent<PhysicsWorldComponent>([&](EntityID entityId, PhysicsWorldComponent& world) {
            worldEntity = entityId;
            physicsWorld = &world; });
        if (!physicsWorld)
            return;
        for (EntityID particleId : m_ActiveParticles) {
            if (!m_ComponentStore->HasComponent<ParticleComponent>(particleId) ||
                !m_ComponentStore->HasComponent<TransformComponent>(particleId) ||
                !m_ComponentStore->HasComponent<ColliderComponent>(particleId))
                continue;
            const auto& particle = m_ComponentStore->GetComponent<ParticleComponent>(particleId);
            const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(particleId);
            const auto& collider = m_ComponentStore->GetComponent<ColliderComponent>(particleId);
            if (particle.emitterEntityId != INVALID_ENTITY &&
                m_ComponentStore->HasComponent<ParticleEmitterComponent>(particle.emitterEntityId)) {
                const auto& emitter = m_ComponentStore->GetComponent<ParticleEmitterComponent>(particle.emitterEntityId);
                if (!emitter.collidesWithBodies)
                    continue;
                uint16_t particleCategory = emitter.collisionCategory;
                uint16_t particleMask = emitter.collisionMask;
                m_ComponentStore->ForEachComponent<PhysicsBodyComponent>([&](EntityID bodyId, PhysicsBodyComponent& body) {
                    if (bodyId == particleId) return;  
                    if (!m_ComponentStore->HasComponent<TransformComponent>(bodyId) ||
                        !m_ComponentStore->HasComponent<ColliderComponent>(bodyId))
                        return;
                    const auto& bodyTransform = m_ComponentStore->GetComponent<TransformComponent>(bodyId);
                    const auto& bodyCollider = m_ComponentStore->GetComponent<ColliderComponent>(bodyId);
                    uint16_t bodyCategory = bodyCollider.filter.categoryBits;
                    uint16_t bodyMask = bodyCollider.filter.maskBits;
                    bool collide = (particleCategory & bodyMask) != 0 && (bodyCategory & particleMask) != 0;
                    if (!collide)
                        return;
                    float dx = bodyTransform.position.x - transform.position.x;
                    float dy = bodyTransform.position.y - transform.position.y;
                    float distSq = dx * dx + dy * dy;
                    float minDist = collider.GetCircle().radius + bodyCollider.GetCircle().radius;
                    if (distSq < minDist * minDist && distSq > 0.001f) {
                        if (physicsWorld->callbacks.beginContact) {
                            physicsWorld->callbacks.beginContact(particleId, bodyId); } } }); } } } }
