#include "nyon/ecs/systems/ParticleRenderSystem.h"
#include "nyon/graphics/Renderer2D.h"
#include "nyon/core/Application.h"
#include "nyon/ecs/components/ParticleComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include <GLFW/glfw3.h>
namespace Nyon::ECS {
ParticleRenderSystem::ParticleRenderSystem() 
    : m_ParticleRenderer(std::make_unique<Graphics::ParticleRenderer>()) { }
void ParticleRenderSystem::Initialize(EntityManager& entityManager, ComponentStore& componentStore) {
    m_ParticleRenderer->Init();
    m_ComponentStore = &componentStore; }
void ParticleRenderSystem::Update(float deltaTime) { }
void ParticleRenderSystem::Render(float alpha) {
    if (!m_ComponentStore) return;
    m_ParticleRenderer->BeginFrame();
    m_ComponentStore->ForEachComponent<ParticleComponent>([&](EntityID entityId, const ParticleComponent& particle) {
        if (!particle.alive) return;
        if (!m_ComponentStore->HasComponent<TransformComponent>(entityId)) return;
        const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
        if (!m_ComponentStore->HasComponent<ColliderComponent>(entityId)) return;
        const auto& collider = m_ComponentStore->GetComponent<ColliderComponent>(entityId);
        float ix = transform.previousPosition.x + (transform.position.x - transform.previousPosition.x) * alpha;
        float iy = transform.previousPosition.y + (transform.position.y - transform.previousPosition.y) * alpha;
        float ia = particle.prevAlpha + (particle.alpha - particle.prevAlpha) * alpha;
        float radius = collider.GetCircle().radius * particle.sizeScale;
        float r = particle.colorStart.x + (particle.colorEnd.x - particle.colorStart.x) * ia;
        float g = particle.colorStart.y + (particle.colorEnd.y - particle.colorStart.y) * ia;
        float b = particle.colorStart.z + (particle.colorEnd.z - particle.colorStart.z) * ia;
        m_ParticleRenderer->SubmitCircle(ix, iy, radius, r, g, b); });
    const auto& camera = Graphics::Renderer2D::GetActiveCamera();
    GLFWwindow* window = nullptr;
    try { window = Nyon::Application::Get().GetWindow(); } catch (...) {}
    int width = 1280, height = 720;
    if (window) glfwGetFramebufferSize(window, &width, &height);
    glm::mat4 vp = camera.GetViewProjectionMatrix(static_cast<float>(width), static_cast<float>(height));
    m_ParticleRenderer->Flush(vp); } }  