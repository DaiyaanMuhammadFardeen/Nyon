#pragma once
#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include "nyon/ecs/EntityManager.h"
#include <functional>
#include <random>
namespace Nyon::ECS {
    struct ParticleSpawnParams {
        float minSpeed = 50.0f;
        float maxSpeed = 200.0f;
        float minAngleDeg = 0.0f;
        float maxAngleDeg = 360.0f;    
        float minRadius = 4.0f;
        float maxRadius = 16.0f;       
        float minMass = 1.0f;
        float maxMass = 10.0f;
        float minLifetime = 1.0f;
        float maxLifetime = 3.0f;      
        float minDrag = 0.0f;
        float maxDrag = 0.05f;
        float minRestitution = 0.3f;
        float maxRestitution = 0.8f;
        float minFriction = 0.1f;
        float maxFriction = 0.5f;
        Math::Vector3 colorStartMin{1.0f, 1.0f, 1.0f};
        Math::Vector3 colorStartMax{1.0f, 1.0f, 1.0f};
        Math::Vector3 colorEndMin{0.0f, 0.0f, 0.0f};
        Math::Vector3 colorEndMax{0.0f, 0.0f, 0.0f};
        float alphaStart = 1.0f;
        float alphaEnd = 0.0f;
        bool useCircle = true;
        void Reset() {
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
            useCircle = true; } };
    struct ParticleEmitterComponent {
        float spawnRate = 10.0f;       
        uint32_t burstCount = 0;       
        float spawnTimer = 0.0f;       
        uint32_t maxParticles = 1000;  
        uint32_t currentCount = 0;     
        bool loop = true;              
        bool active = true;            
        enum class EmissionShape { 
            Point,       
            Circle,      
            Rectangle,   
            Annulus       } emissionShape = EmissionShape::Point;
        float emissionRadius = 0.0f;          
        float emissionInnerRadius = 0.0f;     
        Math::Vector2 emissionSize{0.0f, 0.0f};   
        ParticleSpawnParams spawnParams;
        bool affectedByPhysicsWorld = true;   
        float gravityScale = 1.0f;            
        bool collidesWithBodies = false;      
        bool collidesWithParticles = true;    
        uint16_t collisionCategory = 0x0002;  
        uint16_t collisionMask = 0xFFFF;      
        std::function<void(EntityID particleEntity)> onSpawn;
        std::function<void(EntityID particleEntity, float deltaTime)> onUpdate;
        std::function<void(EntityID particleEntity)> onDeath;
        std::function<void(EntityID particleEntity, EntityID bodyEntity)> onCollision;
        uint64_t seed = 0;   
        void Reset() {
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
            seed = 0; } }; }
