#pragma once

#include "nyon/ecs/System.h"
#include "nyon/graphics/ParticleRenderer.h"
#include "nyon/Particle.h"
#include <vector>

namespace Nyon {

namespace ECS {

class ParticleRenderSystem : public System {
public:
    ParticleRenderSystem();
    ~ParticleRenderSystem() override = default;

    void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override;
    void Update(float deltaTime) override;
    void Render(float alpha);  // Called during render phase

    // Set the particles to render (called by the demo)
    void SetParticles(const std::vector<Nyon::Particle>& particles);

private:
    std::unique_ptr<Graphics::ParticleRenderer> m_ParticleRenderer;
    std::vector<Nyon::Particle> m_Particles;
};

} // namespace ECS
} // namespace Nyon