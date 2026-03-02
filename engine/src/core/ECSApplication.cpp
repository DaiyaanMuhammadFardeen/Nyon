#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/systems/InputSystem.h"
// Removed legacy system includes
#include "nyon/ecs/systems/RenderSystem.h"
#include "nyon/ecs/systems/DebugRenderSystem.h"
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
        m_SystemManager.AddSystem(std::make_unique<ECS::RenderSystem>());
        
        m_ECSInitialized = true;
        
        // Call game-specific ECS initialization
        OnECSStart();
        
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnStart() completed");
    }
    
    void ECSApplication::OnFixedUpdate(float deltaTime)
    {
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnFixedUpdate() called with delta time: " << deltaTime);
        
        if (m_ECSInitialized)
        {
            // Update all ECS systems
            m_SystemManager.Update(deltaTime);
            
            // Call game-specific ECS update
            OnECSUpdate(deltaTime);
        }
        
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnFixedUpdate() completed");
    }
    
    void ECSApplication::OnInterpolateAndRender(float alpha)
    {
        // Pass interpolation alpha to RenderSystem for smooth rendering
        ECS::RenderSystem* renderSystem = m_SystemManager.GetSystem<ECS::RenderSystem>();
        if (renderSystem)
        {
            renderSystem->SetInterpolationAlpha(alpha);
        }
        
        // Update render system with interpolation
        if (m_ECSInitialized)
        {
            // Only update render system during interpolation phase
            renderSystem->Update(0.0f); // Delta time not used in rendering
        }
        
        // Rendering is handled by the RenderSystem
        // This method exists for compatibility but delegates to ECS systems
        NYON_DEBUG_LOG("[DEBUG] ECSApplication::OnInterpolateAndRender() called with alpha: " << alpha);
        
        // Render debug information if DebugRenderSystem exists
        ECS::DebugRenderSystem* debugSystem = GetDebugRenderSystem();
        if (debugSystem)
        {
            debugSystem->RenderDebugInfo();
        }
    }
    
    ECS::DebugRenderSystem* ECSApplication::GetDebugRenderSystem()
    {
        return m_SystemManager.GetSystem<ECS::DebugRenderSystem>();
    }
}