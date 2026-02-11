#pragma once

#include "EntityManager.h"
#include <unordered_map>
#include <vector>
#include <typeindex>
#include <memory>
#include <cassert>
#include <algorithm>  // For std::find and std::remove

namespace Nyon::ECS
{
    /**
     * @brief Storage system for ECS components using Structure of Arrays pattern.
     * 
     * Stores components in contiguous arrays for cache-friendly access.
     * Each component type gets its own storage container.
     */
    class ComponentStore
    {
    private:
        // Base class for component storage containers
        struct IComponentContainer
        {
            virtual ~IComponentContainer() = default;
            virtual void RemoveComponent(EntityID entity) = 0;
            virtual bool HasComponent(EntityID entity) const = 0;
        };
        
        // Template container for specific component types
        template<typename T>
        struct ComponentContainer : public IComponentContainer
        {
            std::unordered_map<EntityID, T> components;
            std::vector<EntityID> entityList;
            
            void RemoveComponent(EntityID entity) override
            {
                auto it = components.find(entity);
                if (it != components.end()) {
                    components.erase(it);
                    // Remove from entity list using algorithm functions
                    auto it2 = std::find(entityList.begin(), entityList.end(), entity);
                    if (it2 != entityList.end()) {
                        entityList.erase(it2);
                    }
                }
            }
            
            bool HasComponent(EntityID entity) const override
            {
                return components.find(entity) != components.end();
            }
        };
        
    public:
        ComponentStore(EntityManager& entityManager);
        
        /**
         * @brief Add a component to an entity.
         * @tparam T Component type
         * @param entity Entity to add component to
         * @param component Component data to add
         */
        template<typename T>
        void AddComponent(EntityID entity, T&& component)
        {
            assert(m_EntityManager.IsEntityValid(entity));
            
            auto& container = GetOrCreateContainer<T>();
            container.components[entity] = std::forward<T>(component);
            
            // Add to entity list if not already present
            if (std::find(container.entityList.begin(), container.entityList.end(), entity) == container.entityList.end()) {
                container.entityList.push_back(entity);
            }
        }
        
        /**
         * @brief Remove a component from an entity.
         * @tparam T Component type
         * @param entity Entity to remove component from
         */
        template<typename T>
        void RemoveComponent(EntityID entity)
        {
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt != m_Containers.end()) {
                static_cast<ComponentContainer<T>*>(containerIt->second.get())->RemoveComponent(entity);
            }
        }
        
        /**
         * @brief Get a component reference for an entity.
         * @tparam T Component type
         * @param entity Entity to get component from
         * @return Reference to the component
         */
        template<typename T>
        T& GetComponent(EntityID entity)
        {
            assert(HasComponent<T>(entity));
            auto& container = GetContainer<T>();
            return container.components.at(entity);
        }
        
        /**
         * @brief Get a component const reference for an entity.
         * @tparam T Component type
         * @param entity Entity to get component from
         * @return Const reference to the component
         */
        template<typename T>
        const T& GetComponent(EntityID entity) const
        {
            assert(HasComponent<T>(entity));
            const auto& container = GetContainer<T>();
            return container.components.at(entity);
        }
        
        /**
         * @brief Check if an entity has a specific component.
         * @tparam T Component type
         * @param entity Entity to check
         * @return True if entity has the component
         */
        template<typename T>
        bool HasComponent(EntityID entity) const
        {
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt != m_Containers.end()) {
                return containerIt->second->HasComponent(entity);
            }
            return false;
        }
        
        /**
         * @brief Get all entities that have a specific component type.
         * @tparam T Component type
         * @return Vector of entity IDs with this component
         */
        template<typename T>
        const std::vector<EntityID>& GetEntitiesWithComponent() const
        {
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt != m_Containers.end()) {
                return static_cast<const ComponentContainer<T>*>(containerIt->second.get())->entityList;
            }
            static std::vector<EntityID> empty;
            return empty;
        }
        
        /**
         * @brief Remove all components for a specific entity.
         * @param entity Entity to remove all components from
         */
        void RemoveAllComponents(EntityID entity);
        
    private:
        EntityManager& m_EntityManager;
        std::unordered_map<std::type_index, std::unique_ptr<IComponentContainer>> m_Containers;
        
        template<typename T>
        ComponentContainer<T>& GetOrCreateContainer()
        {
            auto typeIdx = std::type_index(typeid(T));
            auto it = m_Containers.find(typeIdx);
            if (it == m_Containers.end()) {
                auto container = std::make_unique<ComponentContainer<T>>();
                it = m_Containers.emplace(typeIdx, std::move(container)).first;
            }
            return *static_cast<ComponentContainer<T>*>(it->second.get());
        }
        
        template<typename T>
        ComponentContainer<T>& GetContainer()
        {
            auto typeIdx = std::type_index(typeid(T));
            auto it = m_Containers.find(typeIdx);
            assert(it != m_Containers.end());
            return *static_cast<ComponentContainer<T>*>(it->second.get());
        }
        
        template<typename T>
        const ComponentContainer<T>& GetContainer() const
        {
            auto typeIdx = std::type_index(typeid(T));
            auto it = m_Containers.find(typeIdx);
            assert(it != m_Containers.end());
            return *static_cast<const ComponentContainer<T>*>(it->second.get());
        }
    };
}