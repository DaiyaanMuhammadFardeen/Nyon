#pragma once

#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"

namespace Nyon::ECS
{
    /**
     * @brief Particle component for particle system simulation
     * 
     * Particles are now full ECS citizens with:
     * - Entity representation in the ECS world
     * - PhysicsBodyComponent for physics simulation (velocity, mass, forces)
     * - TransformComponent for position/rotation/scale
     * - ColliderComponent for collision detection and filtering
     * - ParticleComponent for particle-specific properties (lifecycle, visuals)
     * 
     * All kinematic properties live in PhysicsBodyComponent and TransformComponent.
     * This component adds particle-specific lifecycle and visual properties only.
     */
    struct ParticleComponent
    {
        // === Lifecycle ===
        float lifetime = -1.0f;       // seconds remaining; < 0 = eternal
        float age = 0.0f;             // seconds since spawn
        bool alive = true;            // whether particle is still alive
        
        // === Visual ===
        float alpha = 1.0f;           // independent opacity
        float alphaStart = 1.0f;      // opacity at spawn
        float alphaEnd = 0.0f;        // opacity at death
        Math::Vector3 colorStart{1.0f, 1.0f, 1.0f};  // RGB at spawn
        Math::Vector3 colorEnd{1.0f, 1.0f, 1.0f};    // RGB at death
        float sizeScale = 1.0f;       // multiplier on ColliderComponent radius
        
        // === Emitter reference ===
        EntityID emitterEntityId = INVALID_ENTITY;
        
        // === Developer payload ===
        uint64_t userData = 0;        // opaque per-particle data for callbacks
        
        // === Interpolation (set by ParticlePipelineSystem) ===
        float prevAlpha = 1.0f;
        float prevSizeScale = 1.0f;
        Math::Vector3 prevColorStart{1.0f, 1.0f, 1.0f};
        Math::Vector3 prevColorEnd{1.0f, 1.0f, 1.0f};
        
        void Reset()
        {
            lifetime = -1.0f;
            age = 0.0f;
            alive = true;
            alpha = 1.0f;
            alphaStart = 1.0f;
            alphaEnd = 0.0f;
            colorStart = {1.0f, 1.0f, 1.0f};
            colorEnd = {1.0f, 1.0f, 1.0f};
            sizeScale = 1.0f;
            emitterEntityId = INVALID_ENTITY;
            userData = 0;
            prevAlpha = 1.0f;
            prevSizeScale = 1.0f;
            prevColorStart = {1.0f, 1.0f, 1.0f};
            prevColorEnd = {1.0f, 1.0f, 1.0f};
        }
    };
}
