#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/systems/InputSystem.h"
#include "nyon/ecs/systems/RenderSystem.h"
#include "nyon/ecs/systems/DebugRenderSystem.h"
#include "nyon/ecs/systems/PhysicsPipelineSystem.h"
#include "nyon/utils/InputManager.h"
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
        m_SystemManager.AddSystem(std::make_unique<ECS::DebugRenderSystem>());
        
        // Initialize RenderSystem separately - only called during OnInterpolateAndRender
        m_RenderSystem = std::make_unique<ECS::RenderSystem>();
        m_RenderSystem->Initialize(m_EntityManager, m_ComponentStore);
        
        m_ECSInitialized = true;
        
        // Cache DebugRenderSystem pointer to avoid dynamic_cast every frame
        m_DebugRenderSystem = m_SystemManager.GetSystem<ECS::DebugRenderSystem>();
        
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnStart() completed");
    }
    
    void ECSApplication::OnFixedUpdate(float deltaTime)
    {
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnFixedUpdate() called with delta time: " << deltaTime);
        
        if (m_ECSInitialized)
        {
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
        }
        
        // Rendering is handled by the RenderSystem
        // This method exists for compatibility but delegates to ECS systems
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnInterpolateAndRender() called with alpha: " << alpha);
        
        // Render debug information if DebugRenderSystem exists (use cached pointer)
        if (m_DebugRenderSystem)
        {
            // Pass interpolation alpha to DebugRenderSystem for smooth rendering
            m_DebugRenderSystem->SetInterpolationAlpha(alpha);
            
            // Wrap debug rendering in its own BeginScene/EndScene to ensure
            // proper setup of vertex buffers, shader uniforms, and GL state.
            // This prevents stale matrices from corrupting debug output when
            // future changes add post-processing, multiple render targets, etc.
            Graphics::Renderer2D::BeginScene();
            m_DebugRenderSystem->RenderDebugInfo();
            Graphics::Renderer2D::EndScene();
        }
    }
    
    ECS::DebugRenderSystem* ECSApplication::GetDebugRenderSystem()
    {
        // Return cached pointer - no dynamic_cast needed
        return m_DebugRenderSystem;
    }
}