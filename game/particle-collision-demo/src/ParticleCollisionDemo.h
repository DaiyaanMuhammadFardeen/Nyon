#pragma once

#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/systems/ParticleRenderSystem.h"
#include "nyon/ecs/components/ParticleComponent.h"
#include <vector>
#include <random>

namespace Nyon {

class ParticleCollisionDemo : public ECSApplication {
public:
    ParticleCollisionDemo();
    ~ParticleCollisionDemo() override = default;

    void OnECSStart() override;
    void OnECSUpdate(float deltaTime) override;

private:
    void InitializeParticles();
    void UpdateParticles(float deltaTime);
    void HandleCollisions();

    std::vector<Particle> m_Particles;

    // Simulation parameters
    static constexpr int NUM_PARTICLES = 1'000;
    static constexpr float GRAVITY = -980.0f;
    static constexpr float DAMPING = 0.99f;
    static constexpr float BOUNCE_DAMPING = 0.8f;
    static constexpr float WORLD_WIDTH = 1280.0f;
    static constexpr float WORLD_HEIGHT = 720.0f;

    std::mt19937 m_Rng;
};

} // namespace Nyon