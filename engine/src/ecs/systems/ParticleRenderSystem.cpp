#include "nyon/ecs/systems/ParticleRenderSystem.h"
#include <glm/gtc/matrix_transform.hpp>

namespace Nyon::ECS {

ParticleRenderSystem::ParticleRenderSystem() {
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

    // Create view-projection matrix (assuming 1280x720 window)
    glm::mat4 vp = glm::ortho(0.0f, 1280.0f, 0.0f, 720.0f, -1.0f, 1.0f);
    m_ParticleRenderer->Flush(vp);
}

void ParticleRenderSystem::SetParticles(const std::vector<Nyon::Particle>& particles) {
    m_Particles = particles;
}

} // namespace Nyon::ECS