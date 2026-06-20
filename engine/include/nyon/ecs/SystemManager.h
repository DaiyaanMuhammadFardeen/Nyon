#pragma once
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/System.h"
#include <vector>
#include <memory>
#include <unordered_map>
#include <typeindex>
namespace Nyon::ECS {
    class SystemManager {
    public:
        SystemManager(EntityManager& entityManager, ComponentStore& componentStore);
        ~SystemManager();
        template<typename T>
        void AddSystem(std::unique_ptr<T> system) {
            system->Initialize(*m_EntityManager, *m_ComponentStore);
            m_Systems.push_back(std::move(system));
            m_SystemLookup[typeid(T)] = m_Systems.back().get(); }
        void Update(float deltaTime);
        template<typename T>
        T* GetSystem() {
            auto it = m_SystemLookup.find(typeid(T));
            return it != m_SystemLookup.end() ? static_cast<T*>(it->second) : nullptr; }
        void Shutdown();
    private:
        EntityManager* m_EntityManager;
        ComponentStore* m_ComponentStore;
        std::vector<std::unique_ptr<System>> m_Systems;
        std::unordered_map<std::type_index, System*> m_SystemLookup;   }; }