#include "nyon/ecs/SystemManager.h"

namespace Nyon::ECS
{
    SystemManager::SystemManager(EntityManager& entityManager, ComponentStore& componentStore)
        : m_EntityManager(&entityManager)
        , m_ComponentStore(&componentStore)
    {
    }
    
    SystemManager::~SystemManager()
    {
        Shutdown();
    }
    
    void SystemManager::Update(float deltaTime)
    {
        for (auto& system : m_Systems)
        {
            system->Update(deltaTime);
        }
    }
    
    void SystemManager::Shutdown()
    {
        for (auto& system : m_Systems)
        {
            system->Shutdown();
        }
        m_Systems.clear();
    }
}