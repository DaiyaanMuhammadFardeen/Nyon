#pragma once

#include "nyon/core/Application.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/SystemManager.h"

// Forward declarations
namespace Nyon::ECS {
    class DebugRenderSystem;
    class RenderSystem;
}
#include <memory>

namespace Nyon
{
    /**
     * @brief ECS-enabled application base class.
     * 
     * Extends the base Application class with full ECS support.
     * Handles ECS initialization, system updates, and entity management.
     */
    class ECSApplication : public Application
    {
    public:
        ECSApplication(const char* title, int width, int height);
        virtual ~ECSApplication();
        
        // ECS accessors for derived classes and testing
        ECS::EntityManager& GetEntityManager() { return m_EntityManager; }
        ECS::ComponentStore& GetComponentStore() { return m_ComponentStore; }
        ECS::SystemManager& GetSystemManager() { return m_SystemManager; }
        
        // Get DebugRenderSystem if it exists
        ECS::DebugRenderSystem* GetDebugRenderSystem();
        
    protected:
        
        // Methods that can be overridden by games
        virtual void OnECSStart() {}  // Called after ECS initialization
        virtual void OnECSUpdate(float deltaTime) {}  // Called after ECS systems update (user logic hook)
        virtual void OnECSFixedUpdate(float deltaTime) {}  // Called during fixed-step update (physics logic hook)
        
        // Flag to enable/disable debug renderer (can be set by derived classes)
        bool m_EnableDebugRenderer = true;
        
        // Override base Application methods
        void OnStart() override final;
        void OnFixedUpdate(float deltaTime) override final;
        void OnInterpolateAndRender(float alpha) override final;
        
    private:
        ECS::EntityManager m_EntityManager;
        ECS::ComponentStore m_ComponentStore;
        ECS::SystemManager m_SystemManager;
        
        bool m_ECSInitialized;
        ECS::DebugRenderSystem* m_DebugRenderSystem = nullptr;  // Cached pointer to avoid dynamic_cast every frame
        std::unique_ptr<ECS::RenderSystem> m_RenderSystem;  // Separate render system - only called during interpolation
    };
}