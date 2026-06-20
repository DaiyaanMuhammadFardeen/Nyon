#pragma once
#include "EntityManager.h"
#include "ComponentStore.h"
namespace Nyon::ECS {
    class System {
    public:
        virtual ~System() = default;
        virtual void Initialize(EntityManager& entityManager, ComponentStore& componentStore) {
            m_EntityManager = &entityManager;
            m_ComponentStore = &componentStore; }
        virtual void Update(float deltaTime) = 0;
        virtual void Shutdown() {}
    protected:
        EntityManager* m_EntityManager = nullptr;
        ComponentStore* m_ComponentStore = nullptr; }; }