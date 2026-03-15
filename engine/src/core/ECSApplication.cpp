#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/systems/InputSystem.h"
#include "nyon/ecs/systems/RenderSystem.h"
#include "nyon/ecs/systems/DebugRenderSystem.h"
#include "nyon/ecs/systems/PhysicsPipelineSystem.h"
#include "nyon/utils/InputManager.h"
#include <iostream>
#ifdef _DEBUG
#define NYON_DEBUG_LOG(x) std::cerr << x << std::endl
#else
#define NYON_DEBUG_LOG(x)
#endif
namespace Nyon {
    ECSApplication::ECSApplication(const char* title, int width, int height)
        : Application(title, width, height)
        , m_ComponentStore(m_EntityManager)
        , m_SystemManager(m_EntityManager, m_ComponentStore)
        , m_ECSInitialized(false) {
        NYON_DEBUG_LOG("[DEBUG] ECSApplication constructor called"); }
    ECSApplication::~ECSApplication() {
        NYON_DEBUG_LOG("[DEBUG] ECSApplication destructor called"); }
    void ECSApplication::OnStart() {
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnStart() called");
        Utils::InputManager::Init(GetWindow());
        OnECSStart();
        m_SystemManager.AddSystem(std::make_unique<ECS::InputSystem>());
        m_SystemManager.AddSystem(std::make_unique<ECS::PhysicsPipelineSystem>());
        m_SystemManager.AddSystem(std::make_unique<ECS::DebugRenderSystem>());
        m_RenderSystem = std::make_unique<ECS::RenderSystem>();
        m_RenderSystem->Initialize(m_EntityManager, m_ComponentStore);
        m_ECSInitialized = true;
        m_DebugRenderSystem = m_SystemManager.GetSystem<ECS::DebugRenderSystem>();
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnStart() completed"); }
    void ECSApplication::OnFixedUpdate(float deltaTime) {
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnFixedUpdate() called with delta time: " << deltaTime);
        if (m_ECSInitialized) {
            NYON_DEBUG_LOG("[DEBUG] Calling SystemManager.Update() - should update PhysicsPipelineSystem");
            m_SystemManager.Update(deltaTime);
            OnECSFixedUpdate(deltaTime);
            OnECSUpdate(deltaTime); }
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnFixedUpdate() completed"); }
    void ECSApplication::OnInterpolateAndRender(float alpha) {
        if (m_ECSInitialized && m_RenderSystem) {
            m_RenderSystem->SetInterpolationAlpha(alpha);
            m_RenderSystem->Update(0.0f);   }
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnInterpolateAndRender() called with alpha: " << alpha);
        if (m_DebugRenderSystem) {
            m_DebugRenderSystem->SetInterpolationAlpha(alpha);
            Graphics::Renderer2D::BeginScene();
            m_DebugRenderSystem->RenderDebugInfo();
            Graphics::Renderer2D::EndScene(); } }
    ECS::DebugRenderSystem* ECSApplication::GetDebugRenderSystem() {
        return m_DebugRenderSystem; } }