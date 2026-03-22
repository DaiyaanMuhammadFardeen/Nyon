#pragma once

#include "nyon/ecs/System.h"
#include "nyon/graphics/ParticleRenderer.h"
#include "nyon/ecs/components/ParticleComponent.h"
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

    // ECS-based - particles are entities with ParticleComponent
    // No need for SetParticles/GetParticles anymore

private:
    std::unique_ptr<Graphics::ParticleRenderer> m_ParticleRenderer;
    ComponentStore* m_ComponentStore = nullptr;  // Reference to component store
};

} // namespace ECS
} // namespace Nyon