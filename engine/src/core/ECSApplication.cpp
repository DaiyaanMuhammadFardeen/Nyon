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
        
        // Initialize ECS systems in proper order
m_SystemManager.AddSystem(std::make_unique<ECS::InputSystem>());
m_SystemManager.AddSystem(std::make_unique<ECS::PhysicsPipelineSystem>());
m_SystemManager.AddSystem(std::make_unique<ECS::RenderSystem>());
m_SystemManager.AddSystem(std::make_unique<ECS::DebugRenderSystem>());
        
        m_ECSInitialized = true;
        
        // Cache DebugRenderSystem pointer to avoid dynamic_cast every frame
        m_DebugRenderSystem = m_SystemManager.GetSystem<ECS::DebugRenderSystem>();
        
        // Call game-specific ECS initialization
        OnECSStart();
        
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnStart() completed");
    }
    
    void ECSApplication::OnFixedUpdate(float deltaTime)
    {
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnFixedUpdate() called with delta time: " << deltaTime);
        
        if (m_ECSInitialized)
        {
            // Update only non-render ECS systems (physics, input, etc.)
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
        if (m_ECSInitialized)
        {
            // Pass interpolation alpha to RenderSystem for smooth rendering
            ECS::RenderSystem* renderSystem = m_SystemManager.GetSystem<ECS::RenderSystem>();
            if (renderSystem)
            {
                renderSystem->SetInterpolationAlpha(alpha);
                // Update render system with interpolation
                renderSystem->Update(0.0f); // Delta time not used in rendering
            }
        }
        
        // Rendering is handled by the RenderSystem
        // This method exists for compatibility but delegates to ECS systems
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnInterpolateAndRender() called with alpha: " << alpha);
        
        // Render debug information if DebugRenderSystem exists (use cached pointer)
        if (m_DebugRenderSystem)
        {
            m_DebugRenderSystem->RenderDebugInfo();
        }
    }
    
    ECS::DebugRenderSystem* ECSApplication::GetDebugRenderSystem()
    {
        // Return cached pointer - no dynamic_cast needed
        return m_DebugRenderSystem;
    }
}