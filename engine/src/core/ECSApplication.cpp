#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/systems/InputSystem.h"
#include "nyon/ecs/systems/RenderSystem.h"
#include "nyon/ecs/systems/PhysicsPipelineSystem.h"
#include "nyon/ecs/systems/DebugRenderSystem.h"
#include "nyon/ecs/systems/ParticleRenderSystem.h"
#include "nyon/ecs/systems/CameraSystem.h"
#include "nyon/utils/InputManager.h"
#include <glm/gtc/matrix_transform.hpp>
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
        m_SystemManager.AddSystem(std::make_unique<ECS::CameraSystem>());   
        m_SystemManager.AddSystem(std::make_unique<ECS::PhysicsPipelineSystem>());
        m_RenderSystem = std::make_unique<ECS::RenderSystem>();
        m_RenderSystem->Initialize(m_EntityManager, m_ComponentStore);
        m_DebugRenderSystem = std::make_unique<ECS::DebugRenderSystem>();
        m_DebugRenderSystem->Initialize(m_EntityManager, m_ComponentStore);
        m_DebugRenderSystem->SetFlags(true, false, false, false, false);   
        m_ECSInitialized = true;
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnStart() completed"); }
    void ECSApplication::OnFixedUpdate(float deltaTime) {
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnFixedUpdate() called with delta time: " << deltaTime);
        if (m_ECSInitialized) {
            static bool f1PrevState = false;
            bool f1CurrState = Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_F1);
            if (f1CurrState && !f1PrevState) {
                m_DebugOverlayEnabled = !m_DebugOverlayEnabled;
                std::cerr << "[DEBUG] Debug overlay " << (m_DebugOverlayEnabled ? "enabled" : "disabled") << "\n"; }
            f1PrevState = f1CurrState;
            if (m_DebugOverlayEnabled && m_DebugRenderSystem) {
                m_DebugRenderSystem->Update(deltaTime); }
            NYON_DEBUG_LOG("[DEBUG] Calling SystemManager.Update() - should update PhysicsPipelineSystem");
            m_SystemManager.Update(deltaTime);
            OnECSFixedUpdate(deltaTime);
            OnECSUpdate(deltaTime); }
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnFixedUpdate() completed"); }
    void ECSApplication::OnInterpolateAndRender(float alpha) {
        if (m_ECSInitialized && m_RenderSystem) {
            m_RenderSystem->SetInterpolationAlpha(alpha);
            m_RenderSystem->Update(0.0f);  
            if (m_DebugOverlayEnabled && m_DebugRenderSystem) {
                m_DebugRenderSystem->SetInterpolationAlpha(alpha);
                m_DebugRenderSystem->RenderDebugInfo(); }
            auto* particleSystem = m_SystemManager.GetSystem<ECS::ParticleRenderSystem>();
            if (particleSystem) {
                particleSystem->Render(alpha); } }
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnInterpolateAndRender() called with alpha: " << alpha); } }