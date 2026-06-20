#pragma once
#include "nyon/ecs/System.h"
#include "nyon/ecs/components/ParticleComponent.h"
#include "nyon/ecs/components/ParticleEmitterComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/utils/ThreadPool.h"
#include <vector>
#include <future>
#include <unordered_map>
#include <random>
namespace Nyon::ECS {
    class ParticlePipelineSystem : public System {
    public:
        ParticlePipelineSystem();
        ~ParticlePipelineSystem() override = default;
        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override;
        void Update(float deltaTime) override;
        void SpawnParticleFromEmitter(EntityID emitterEntityId, const Math::Vector2& spawnPosition);
        void SetGravityScale(float scale) { m_GravityScale = scale; }
        void SetDamping(float damping) { m_Damping = damping; }
        void SetRestitution(float restitution) { m_Restitution = restitution; }
        void EnableCollisions(bool enable) { m_EnableCollisions = enable; }
        void EnableSpatialHash(bool enable) { m_UseSpatialHash = enable; }
        const std::vector<EntityID>& GetActiveParticles() const { return m_ActiveParticles; }
    private:
        void ProcessEmitters(float deltaTime);
        void UpdateParticlePhysicsParallel(size_t startIndex, size_t endIndex, float dt);
        void BuildSpatialHash(float cellSize);
        std::vector<std::pair<int, int>> ComputeCellIndices(size_t startIndex, size_t endIndex, float cellSize);
        void DetectParticleCollisionsParallel();
        void DetectCollisionsBruteForce();
        void ProcessCollisionPair(size_t i, size_t j);
        void DetectParticleBodyCollisions();
        void ProcessParticleLifecycle(float deltaTime);
        void CleanupDeadParticles();
        Math::Vector2 SampleSpawnPosition(const ParticleEmitterComponent& emitter) const;
        ComponentStore* m_ComponentStore = nullptr;
        EntityID m_PhysicsWorldEntity = INVALID_ENTITY;
        Math::Vector2 m_Gravity = {0.0f, -980.0f};
        bool m_EnableSleep = true;
        float m_MaxLinearSpeed = 1000.0f;   
        float m_GravityScale = 1.0f;
        float m_Damping = 0.99f;
        float m_Restitution = 0.8f;   
        bool m_EnableCollisions = true;
        bool m_UseSpatialHash = true;
        size_t m_NumThreads = 1;
        struct SpatialCell {
            std::vector<EntityID> particleEntities;    };
        std::unordered_map<int, SpatialCell> m_SpatialHash;
        float m_CellSize = 50.0f;
        std::vector<EntityID> m_ActiveParticles;
        mutable std::mt19937 m_Rng{std::random_device{}()}; }; }
