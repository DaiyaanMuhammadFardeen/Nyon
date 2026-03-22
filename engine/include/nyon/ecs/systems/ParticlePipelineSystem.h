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

namespace Nyon::ECS
{
    /**
     * @brief High-performance particle system with parallel updates and collision detection.
     * 
     * Particles are now full ECS citizens with:
     * - Entity representation in the ECS world
     * - PhysicsBodyComponent for physics simulation (gravity, velocity, mass)
     * - ColliderComponent for collision detection and filtering
     * - ParticleComponent for particle-specific properties (lifetime, color, callbacks)
     * 
     * Implements the complete 6-phase architecture from Section 9.3:
     * Phase 1: Tick emitters (main thread, fast) - spawn new particles
     * Phase 2: Parallel particle physics update (ThreadPool) - gravity, drag, integration
     * Phase 3: Parallel particle-particle broadphase (spatial hash) - collision detection
     * Phase 4: Particle-body broadphase (optional, future implementation)
     * Phase 5: Lifecycle management (main thread) - call death callbacks
     * Phase 6: Post-update cleanup (main thread) - remove dead particles
     */
    class ParticlePipelineSystem : public System
    {
    public:
        ParticlePipelineSystem();
        ~ParticlePipelineSystem() override = default;

        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override;
        void Update(float deltaTime) override;
        
        // Particle creation via emitters (ECS-based)
        void SpawnParticleFromEmitter(EntityID emitterEntityId, const Math::Vector2& spawnPosition);
        
        // Set simulation parameters
        void SetGravityScale(float scale) { m_GravityScale = scale; }
        void SetDamping(float damping) { m_Damping = damping; }
        void SetRestitution(float restitution) { m_Restitution = restitution; }
        
        // Enable/disable features
        void EnableCollisions(bool enable) { m_EnableCollisions = enable; }
        void EnableSpatialHash(bool enable) { m_UseSpatialHash = enable; }
        
        // Get active particles
        const std::vector<EntityID>& GetActiveParticles() const { return m_ActiveParticles; }

    private:
        // Phase 1: Tick emitters (main thread)
        void ProcessEmitters(float deltaTime);
        
        // Phase 2: Parallel particle physics update
        void UpdateParticlePhysicsParallel(size_t startIndex, size_t endIndex, float dt);
        
        // Phase 3: Parallel spatial hash construction and collision detection
        void BuildSpatialHash(float cellSize);
        std::vector<std::pair<int, int>> ComputeCellIndices(size_t startIndex, size_t endIndex, float cellSize);
        void DetectParticleCollisionsParallel();
        void DetectCollisionsBruteForce();
        void ProcessCollisionPair(size_t i, size_t j);
        
        // Phase 4: Particle-body collisions (TODO - future implementation)
        void DetectParticleBodyCollisions();
        
        // Phase 5: Lifecycle management (main thread)
        void ProcessParticleLifecycle(float deltaTime);
        
        // Phase 6: Post-update cleanup (main thread)
        void CleanupDeadParticles();
        
        // Helper methods
        Math::Vector2 SampleSpawnPosition(const ParticleEmitterComponent& emitter) const;
        
        // Component references
        ComponentStore* m_ComponentStore = nullptr;
        EntityID m_PhysicsWorldEntity = INVALID_ENTITY;
        Math::Vector2 m_Gravity = {0.0f, -980.0f};
        
        // Physics world settings (cached from PhysicsWorldComponent)
        bool m_EnableSleep = true;
        float m_MaxLinearSpeed = 1000.0f;  // Maximum linear speed (from PhysicsWorldComponent)
        
        // Simulation parameters - NO HARDCODED VALUES
        float m_GravityScale = 1.0f;
        float m_Damping = 0.99f;
        float m_Restitution = 0.8f;  // Configurable restitution
        bool m_EnableCollisions = true;
        bool m_UseSpatialHash = true;
        
        // Threading
        size_t m_NumThreads = 1;
        
        // Spatial hash data structures
        struct SpatialCell {
            std::vector<EntityID> particleEntities;  // Store entity IDs, not indices
        };
        std::unordered_map<int, SpatialCell> m_SpatialHash;
        float m_CellSize = 50.0f;
        
        // Active particle entities (ECS-based)
        std::vector<EntityID> m_ActiveParticles;
        
        // RNG for sampling
        mutable std::mt19937 m_Rng{std::random_device{}()};
    };
}
