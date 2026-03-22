#pragma once

#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include "nyon/ecs/EntityManager.h"
#include <functional>
#include <random>

namespace Nyon::ECS
{
    /**
     * @brief Parameters for spawning new particles
     * 
     * Defines ranges for particle properties that are sampled uniformly at spawn time.
     */
    struct ParticleSpawnParams
    {
        // Ranges — sampled uniformly at spawn time
        float minSpeed = 50.0f;
        float maxSpeed = 200.0f;
        
        float minAngleDeg = 0.0f;
        float maxAngleDeg = 360.0f;   // emission cone
        
        float minRadius = 4.0f;
        float maxRadius = 16.0f;      // ColliderComponent circle radius
        
        float minMass = 1.0f;
        float maxMass = 10.0f;
        
        float minLifetime = 1.0f;
        float maxLifetime = 3.0f;     // < 0 = use component default (eternal)
        
        float minDrag = 0.0f;
        float maxDrag = 0.05f;
        
        float minRestitution = 0.3f;
        float maxRestitution = 0.8f;
        
        float minFriction = 0.1f;
        float maxFriction = 0.5f;
        
        // Colour
        Math::Vector3 colorStartMin{1.0f, 1.0f, 1.0f};
        Math::Vector3 colorStartMax{1.0f, 1.0f, 1.0f};
        Math::Vector3 colorEndMin{0.0f, 0.0f, 0.0f};
        Math::Vector3 colorEndMax{0.0f, 0.0f, 0.0f};
        
        float alphaStart = 1.0f;
        float alphaEnd = 0.0f;
        
        // Shape override (empty = use radius as circle)
        // Set to a PolygonShape to emit non-circular particles
        bool useCircle = true;
        
        void Reset()
        {
            minSpeed = 50.0f;
            maxSpeed = 200.0f;
            minAngleDeg = 0.0f;
            maxAngleDeg = 360.0f;
            minRadius = 4.0f;
            maxRadius = 16.0f;
            minMass = 1.0f;
            maxMass = 10.0f;
            minLifetime = 1.0f;
            maxLifetime = 3.0f;
            minDrag = 0.0f;
            maxDrag = 0.05f;
            minRestitution = 0.3f;
            maxRestitution = 0.8f;
            minFriction = 0.1f;
            maxFriction = 0.5f;
            colorStartMin = {1.0f, 1.0f, 1.0f};
            colorStartMax = {1.0f, 1.0f, 1.0f};
            colorEndMin = {0.0f, 0.0f, 0.0f};
            colorEndMax = {0.0f, 0.0f, 0.0f};
            alphaStart = 1.0f;
            alphaEnd = 0.0f;
            useCircle = true;
        }
    };

    /**
     * @brief Particle emitter component - defines spawn behaviour
     * 
     * Lives on an emitter entity and controls:
     * - Spawn rate and burst behavior
     * - Emission shape (point, circle, rectangle, annulus)
     * - Initial particle property ranges
     * - Physics interaction settings
     * - Developer callbacks for custom behavior
     */
    struct ParticleEmitterComponent
    {
        // === Spawn rate ===
        float spawnRate = 10.0f;      // particles/second; 0 = burst only
        uint32_t burstCount = 0;      // number of particles to spawn in burst
        float spawnTimer = 0.0f;      // accumulator for spawn timing
        uint32_t maxParticles = 1000; // max alive at once from this emitter
        uint32_t currentCount = 0;    // tracked by ParticlePipelineSystem
        bool loop = true;             // whether to continue spawning after reaching max
        bool active = true;           // whether emitter is currently active
        
        // === Spawn area ===
        enum class EmissionShape 
        { 
            Point,      // spawn at exact position
            Circle,     // spawn within circle of emissionRadius
            Rectangle,  // spawn within rectangle of emissionSize
            Annulus     // spawn within ring (emissionInnerRadius to emissionRadius)
        } emissionShape = EmissionShape::Point;
        
        float emissionRadius = 0.0f;         // for Circle/Annulus
        float emissionInnerRadius = 0.0f;    // for Annulus
        Math::Vector2 emissionSize{0.0f, 0.0f};  // for Rectangle
        
        // === Initial condition ranges ===
        ParticleSpawnParams spawnParams;
        
        // === Physics settings ===
        bool affectedByPhysicsWorld = true;  // uses PhysicsWorldComponent.gravity
        float gravityScale = 1.0f;           // per-emitter gravity multiplier
        bool collidesWithBodies = false;     // enter main physics pipeline
        bool collidesWithParticles = true;   // use dedicated particle broadphase
        
        // === Collision layer (inherits ColliderComponent::Filter system) ===
        uint16_t collisionCategory = 0x0002; // default particle layer
        uint16_t collisionMask = 0xFFFF;     // collides with everything by default
        
        // === Developer hooks (all called on the game thread, not worker threads) ===
        
        /// Called when a particle entity is spawned. Set any custom component data here.
        std::function<void(EntityID particleEntity)> onSpawn;
        
        /// Called each physics step per alive particle. Runs after standard physics update.
        /// Use this for custom forces, colour animation, size animation, etc.
        /// WARNING: Must be thread-safe if ParticlePipelineSystem uses parallel update.
        std::function<void(EntityID particleEntity, float deltaTime)> onUpdate;
        
        /// Called when particle lifetime expires. Use to spawn secondary effects.
        std::function<void(EntityID particleEntity)> onDeath;
        
        /// Called on particle-body collision (only if collidesWithBodies = true).
        std::function<void(EntityID particleEntity, EntityID bodyEntity)> onCollision;
        
        // === RNG seed (per-emitter for reproducibility) ===
        uint64_t seed = 0;  // 0 = random seed from std::random_device
        
        void Reset()
        {
            spawnRate = 10.0f;
            burstCount = 0;
            spawnTimer = 0.0f;
            maxParticles = 1000;
            currentCount = 0;
            loop = true;
            active = true;
            
            emissionShape = EmissionShape::Point;
            emissionRadius = 0.0f;
            emissionInnerRadius = 0.0f;
            emissionSize = {0.0f, 0.0f};
            
            spawnParams.Reset();
            
            affectedByPhysicsWorld = true;
            gravityScale = 1.0f;
            collidesWithBodies = false;
            collidesWithParticles = true;
            
            collisionCategory = 0x0002;
            collisionMask = 0xFFFF;
            
            onSpawn = nullptr;
            onUpdate = nullptr;
            onDeath = nullptr;
            onCollision = nullptr;
            
            seed = 0;
        }
    };
}
