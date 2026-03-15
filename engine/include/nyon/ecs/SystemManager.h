#pragma once
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/System.h"
#include <vector>
#include <memory>
#include <unordered_map>
#include <typeindex>
namespace Nyon::ECS {
    /**
     * @brief System manager that orchestrates all ECS systems.
     * 
     * Maintains system execution order and manages system lifecycle.
     */
    class SystemManager {
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
            m_SystemLookup[typeid(T)] = m_Systems.back().get();
        }
        /**
         * @brief Update all systems in order.
         * @param deltaTime Time elapsed since last frame
         */
        void Update(float deltaTime);
        /**
         * @brief Get a system by type.
         * @tparam T System type
         * @return Pointer to the system, or nullptr if not found
         */
        template<typename T>
        T* GetSystem()
        {
            auto it = m_SystemLookup.find(typeid(T));
            return it != m_SystemLookup.end() ? static_cast<T*>(it->second) : nullptr;
        }
        /**
         * @brief Shutdown all systems.
         */
        void Shutdown();
    private:
        EntityManager* m_EntityManager;
        ComponentStore* m_ComponentStore;
        std::vector<std::unique_ptr<System>> m_Systems;
        std::unordered_map<std::type_index, System*> m_SystemLookup;  
    };
}