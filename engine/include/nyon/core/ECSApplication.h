#pragma once
#include "nyon/core/Application.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/SystemManager.h"
namespace Nyon::ECS {
    class RenderSystem;
    class DebugRenderSystem;
    class ParticleRenderSystem; }
namespace Nyon {
    class ECSApplication : public Application {
    public:
        ECSApplication(const char* title, int width, int height);
        virtual ~ECSApplication();
        ECS::EntityManager& GetEntityManager() { return m_EntityManager; }
        ECS::ComponentStore& GetComponentStore() { return m_ComponentStore; }
        ECS::SystemManager& GetSystemManager() { return m_SystemManager; }
    protected:
        virtual void OnECSStart() {}   
        virtual void OnECSUpdate(float deltaTime) {}   
        virtual void OnECSFixedUpdate(float deltaTime) {}   
        void OnStart() override final;
        void OnFixedUpdate(float deltaTime) override final;
        void OnInterpolateAndRender(float alpha) override final;
    private:
        ECS::EntityManager m_EntityManager;
        ECS::ComponentStore m_ComponentStore;
        ECS::SystemManager m_SystemManager;
        bool m_ECSInitialized;
        std::unique_ptr<ECS::RenderSystem> m_RenderSystem;   
        std::unique_ptr<ECS::DebugRenderSystem> m_DebugRenderSystem;   
        bool m_DebugOverlayEnabled = false;    }; }