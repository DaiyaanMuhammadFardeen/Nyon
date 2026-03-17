#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/systems/InputSystem.h"
#include "nyon/ecs/systems/RenderSystem.h"
#include "nyon/ecs/systems/PhysicsPipelineSystem.h"
#include "nyon/ecs/systems/DebugRenderSystem.h"
#include "nyon/ecs/systems/ParticleRenderSystem.h"
#include "nyon/utils/InputManager.h"
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

// Debug logging macro - only output in debug builds
#ifdef _DEBUG
#define NYON_DEBUG_LOG(x) std::cerr << x << std::endl
#else
#define NYON_DEBUG_LOG(x)
#endif

namespace Nyon
{
    ECSApplication::ECSApplication(const char* title, int width, int height)
        : Application(title, width, height)
        , m_ComponentStore(m_EntityManager)
        , m_SystemManager(m_EntityManager, m_ComponentStore)
        , m_ECSInitialized(false)
    {
        NYON_DEBUG_LOG("[DEBUG] ECSApplication constructor called");
    }
    
    ECSApplication::~ECSApplication()
    {
        NYON_DEBUG_LOG("[DEBUG] ECSApplication destructor called");
    }
    
    void ECSApplication::OnStart()
    {
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnStart() called");
        
        // Initialize input manager with the window
        Utils::InputManager::Init(GetWindow());
        
        // Call game-specific ECS initialization FIRST
        // This allows games to create PhysicsWorldComponent and other required components
        // before ECS systems are initialized
        OnECSStart();
        
        // NOW initialize ECS systems in proper order (after game has created required components)
        m_SystemManager.AddSystem(std::make_unique<ECS::InputSystem>());
        m_SystemManager.AddSystem(std::make_unique<ECS::PhysicsPipelineSystem>());
        // RenderSystem is NOT added to SystemManager - it's called separately during interpolation
        // Debug renderer has been completely disabled per user request

        
        // Initialize RenderSystem separately - only called during OnInterpolateAndRender
        m_RenderSystem = std::make_unique<ECS::RenderSystem>();
        m_RenderSystem->Initialize(m_EntityManager, m_ComponentStore);
        
        // Initialize DebugRenderSystem for physics debug overlay
        m_DebugRenderSystem = std::make_unique<ECS::DebugRenderSystem>();
        m_DebugRenderSystem->Initialize(m_EntityManager, m_ComponentStore);
        m_DebugRenderSystem->SetFlags(true, false, false, false, false);  // Only draw shapes by default
        
        m_ECSInitialized = true;
        
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnStart() completed");

    }
    
    void ECSApplication::OnFixedUpdate(float deltaTime)
    {
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnFixedUpdate() called with delta time: " << deltaTime);
        
        if (m_ECSInitialized)
        {
            // Check for F1 key to toggle debug overlay
            static bool f1PrevState = false;
            bool f1CurrState = Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_F1);
            if (f1CurrState && !f1PrevState) {
                m_DebugOverlayEnabled = !m_DebugOverlayEnabled;
                std::cerr << "[DEBUG] Debug overlay " << (m_DebugOverlayEnabled ? "enabled" : "disabled") << "\n";
            }
            f1PrevState = f1CurrState;
            
            // Update debug render system if enabled
            if (m_DebugOverlayEnabled && m_DebugRenderSystem) {
                m_DebugRenderSystem->Update(deltaTime);
            }
            
            // Update only non-render ECS systems (physics, input, etc.)
            NYON_DEBUG_LOG("[DEBUG] Calling SystemManager.Update() - should update PhysicsPipelineSystem");
            m_SystemManager.Update(deltaTime);
            
            // Call game-specific fixed-step physics logic
            OnECSFixedUpdate(deltaTime);
            
            // Call game-specific ECS update (user logic after physics)
            OnECSUpdate(deltaTime);
        }
        
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnFixedUpdate() completed");
    }
    
    void ECSApplication::OnInterpolateAndRender(float alpha)
    {
        if (m_ECSInitialized && m_RenderSystem)
        {
            // Pass interpolation alpha to RenderSystem for smooth rendering
            m_RenderSystem->SetInterpolationAlpha(alpha);
            // Update render system with interpolation
            m_RenderSystem->Update(0.0f); // Delta time not used in rendering
            
            // Render debug overlay if enabled
            if (m_DebugOverlayEnabled && m_DebugRenderSystem) {
                m_DebugRenderSystem->SetInterpolationAlpha(alpha);
                m_DebugRenderSystem->RenderDebugInfo();
            }

            // Render particles if particle render system exists
            auto* particleSystem = m_SystemManager.GetSystem<ECS::ParticleRenderSystem>();
            if (particleSystem) {
                // Compute dynamic view-projection matrix based on actual window size
                GLFWwindow* window = GetWindow();
                int width = 1280, height = 720;
                if (window) {
                    glfwGetFramebufferSize(window, &width, &height);
                }
                
                // Create orthographic projection matching Renderer2D's convention
                glm::mat4 vp = glm::ortho(0.0f, static_cast<float>(width), 
                                         0.0f, static_cast<float>(height), 
                                         -1.0f, 1.0f);
                
                particleSystem->SetViewProjection(vp);
                particleSystem->Render(alpha);
            }
        }
        
        // Rendering is handled by the RenderSystem
        // This method exists for compatibility but delegates to ECS systems
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnInterpolateAndRender() called with alpha: " << alpha);
    }
}