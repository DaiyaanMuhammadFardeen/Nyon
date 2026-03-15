#pragma once

#include "nyon/core/Application.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/SystemManager.h"

// Forward declarations
namespace Nyon::ECS {
    class RenderSystem;
    class DebugRenderSystem;
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
        
    protected:
        
        // Methods that can be overridden by games
        virtual void OnECSStart() {}  // Called after ECS initialization
        virtual void OnECSUpdate(float deltaTime) {}  // Called after ECS systems update (user logic hook)
        virtual void OnECSFixedUpdate(float deltaTime) {}  // Called during fixed-step update (physics logic hook)
        
        // Override base Application methods
        void OnStart() override final;
        void OnFixedUpdate(float deltaTime) override final;
        void OnInterpolateAndRender(float alpha) override final;
        
    private:
        ECS::EntityManager m_EntityManager;
        ECS::ComponentStore m_ComponentStore;
        ECS::SystemManager m_SystemManager;
        
        bool m_ECSInitialized;
        std::unique_ptr<ECS::RenderSystem> m_RenderSystem;  // Separate render system - only called during interpolation
        std::unique_ptr<ECS::DebugRenderSystem> m_DebugRenderSystem;  // Debug overlay renderer
        bool m_DebugOverlayEnabled = false;  // F1 toggle flag
    };
}