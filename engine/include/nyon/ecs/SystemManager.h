#pragma once

#include "nyon/ecs/System.h"
#include <vector>
#include <memory>

namespace Nyon::ECS
{
    /**
     * @brief System manager that orchestrates all ECS systems.
     * 
     * Maintains system execution order and manages system lifecycle.
     */
    class SystemManager
    {
    public:
        SystemManager(EntityManager& entityManager, ComponentStore& componentStore);
        ~SystemManager();
        
        /**
         * @brief Add a system to be managed.
         * @tparam T System type
         * @param system Unique pointer to the system
         */
        template<typename T>
        void AddSystem(std::unique_ptr<T> system)
        {
            system->Initialize(*m_EntityManager, *m_ComponentStore);
            m_Systems.push_back(std::move(system));
        }
        
        /**
         * @brief Update all systems in order.
         * @param deltaTime Time elapsed since last frame
         */
        void Update(float deltaTime);
        
        /**
         * @brief Shutdown all systems.
         */
        void Shutdown();
        
    private:
        EntityManager* m_EntityManager;
        ComponentStore* m_ComponentStore;
        std::vector<std::unique_ptr<System>> m_Systems;
    };
}