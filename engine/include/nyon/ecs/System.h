#pragma once

#include "EntityManager.h"
#include "ComponentStore.h"

namespace Nyon::ECS
{
    /**
     * @brief Base class for all ECS systems.
     * 
     * Systems process entities with specific component combinations.
     * Each system defines what components it needs and implements update logic.
     */
    class System
    {
    public:
        virtual ~System() = default;
        
        /**
         * @brief Initialize the system (called once at startup).
         * @param entityManager Reference to the entity manager
         * @param componentStore Reference to the component store
         */
        virtual void Initialize(EntityManager& entityManager, ComponentStore& componentStore)
        {
            m_EntityManager = &entityManager;
            m_ComponentStore = &componentStore;
        }
        
        /**
         * @brief Update the system for a single frame.
         * @param deltaTime Time elapsed since last frame
         */
        virtual void Update(float deltaTime) = 0;
        
        /**
         * @brief Called when the system is being shut down.
         */
        virtual void Shutdown() {}
        
    protected:
        EntityManager* m_EntityManager = nullptr;
        ComponentStore* m_ComponentStore = nullptr;
    };
}