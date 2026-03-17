#include "nyon/ecs/systems/ParticleRenderSystem.h"
#include "nyon/graphics/Renderer2D.h"
#include "nyon/core/Application.h"
#include <GLFW/glfw3.h>

namespace Nyon::ECS {

ParticleRenderSystem::ParticleRenderSystem() 
    : m_ParticleRenderer(std::make_unique<Graphics::ParticleRenderer>()) {
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

    // Use the active camera's view-projection matrix from Renderer2D
    const auto& camera = Graphics::Renderer2D::GetActiveCamera();
    
    // Get screen dimensions from window
    GLFWwindow* window = nullptr;
    try { window = Nyon::Application::Get().GetWindow(); } catch (...) {}
    int width = 1280, height = 720;
    if (window) glfwGetFramebufferSize(window, &width, &height);
    
    glm::mat4 vp = camera.GetViewProjectionMatrix(static_cast<float>(width), static_cast<float>(height));
    
    // Flush with the VP matrix from the active camera
    m_ParticleRenderer->Flush(vp);
}

void ParticleRenderSystem::SetParticles(const std::vector<Nyon::Particle>& particles) {
    m_Particles = particles;
}

} // namespace Nyon::ECS