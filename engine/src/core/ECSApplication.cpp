#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/systems/InputSystem.h"
#include "nyon/ecs/systems/PhysicsSystem.h"
#include "nyon/ecs/systems/CollisionSystem.h"
#include "nyon/ecs/systems/RenderSystem.h"
#include "nyon/utils/InputManager.h"
#include <iostream>

namespace Nyon
{
    ECSApplication::ECSApplication(const char* title, int width, int height)
        : Application(title, width, height)
        , m_ComponentStore(m_EntityManager)
        , m_SystemManager(m_EntityManager, m_ComponentStore)
        , m_ECSInitialized(false)
    {
        std::cerr << "[DEBUG] ECSApplication constructor called" << std::endl;
    }
    
    ECSApplication::~ECSApplication()
    {
        std::cerr << "[DEBUG] ECSApplication destructor called" << std::endl;
    }
    
    void ECSApplication::OnStart()
    {
        std::cerr << "[DEBUG] ECSApplication::OnStart() called" << std::endl;
        
        // Initialize input manager with the window
        Utils::InputManager::Init(GetWindow());
        
        // Initialize ECS systems in proper order
        m_SystemManager.AddSystem(std::make_unique<ECS::InputSystem>());
        m_SystemManager.AddSystem(std::make_unique<ECS::PhysicsSystem>());
        m_SystemManager.AddSystem(std::make_unique<ECS::CollisionSystem>());
        m_SystemManager.AddSystem(std::make_unique<ECS::RenderSystem>());
        
        m_ECSInitialized = true;
        
        // Call game-specific ECS initialization
        OnECSStart();
        
        std::cerr << "[DEBUG] ECSApplication::OnStart() completed" << std::endl;
    }
    
    void ECSApplication::OnFixedUpdate(float deltaTime)
    {
        std::cerr << "[DEBUG] ECSApplication::OnFixedUpdate() called with delta time: " << deltaTime << std::endl;
        
        if (m_ECSInitialized)
        {
            // Update all ECS systems
            m_SystemManager.Update(deltaTime);
            
            // Call game-specific ECS update
            OnECSUpdate(deltaTime);
        }
        
        std::cerr << "[DEBUG] ECSApplication::OnFixedUpdate() completed" << std::endl;
    }
    
    void ECSApplication::OnInterpolateAndRender(float alpha)
    {
        // Rendering is handled by the RenderSystem
        // This method exists for compatibility but delegates to ECS systems
        std::cerr << "[DEBUG] ECSApplication::OnInterpolateAndRender() called with alpha: " << alpha << std::endl;
    }
}