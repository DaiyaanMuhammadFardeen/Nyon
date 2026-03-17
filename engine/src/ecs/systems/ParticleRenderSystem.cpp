#include "nyon/ecs/systems/ParticleRenderSystem.h"
#include <glm/gtc/matrix_transform.hpp>

namespace Nyon::ECS {

ParticleRenderSystem::ParticleRenderSystem() 
    : m_ViewProjection(1.0f) {
    m_ParticleRenderer = std::make_unique<Graphics::ParticleRenderer>();
}

void ParticleRenderSystem::Initialize(EntityManager& entityManager, ComponentStore& componentStore) {
    m_ParticleRenderer->Init();
}

void ParticleRenderSystem::Update(float deltaTime) {
    // Update is called during fixed update - just prepare data
    // Rendering happens in Render() during interpolate/render phase
}

void ParticleRenderSystem::Render(float alpha) {
    if (m_Particles.empty()) return;

    m_ParticleRenderer->BeginFrame();

    // Submit all particles for rendering
    for (const auto& p : m_Particles) {
        m_ParticleRenderer->SubmitCircle(p.x, p.y, p.radius, p.r, p.g, p.b);
    }

    // Use the view-projection matrix set by ECSApplication
    m_ParticleRenderer->Flush(m_ViewProjection);
}

void ParticleRenderSystem::SetParticles(const std::vector<Nyon::Particle>& particles) {
    m_Particles = particles;
}

} // namespace Nyon::ECS