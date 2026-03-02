#pragma once

#include "EntityManager.h"
#include <vector>
#include <typeindex>
#include <memory>
#include <cassert>
#include <algorithm>  // For std::find and std::remove
#include <unordered_map>

namespace Nyon::ECS
{
    /**
     * @brief Storage system for ECS components using true Structure of Arrays pattern.
     * 
     * Stores components in contiguous arrays for cache-friendly access.
     * Each component type gets its own storage container with dense indexing.
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
            virtual EntityID GetEntityAtIndex(size_t index) const = 0;
            virtual size_t GetComponentCount() const = 0;
        };
        
        // Template container for specific component types using SoA pattern
        template<typename T>
        struct ComponentContainer : public IComponentContainer
        {
            std::vector<T> components;           // Dense array of components
            std::vector<EntityID> entityIds;     // Parallel array of entity IDs
            std::vector<bool> activeFlags;       // Active flag for each component
            
            void RemoveComponent(EntityID entity) override
            {
                // Find entity in our dense arrays
                for (size_t i = 0; i < entityIds.size(); ++i)
                {
                    if (entityIds[i] == entity && activeFlags[i])
                    {
                        activeFlags[i] = false;
                        // Component data remains but is marked inactive
                        return;
                    }
                }
            }
            
            bool HasComponent(EntityID entity) const override
            {
                for (size_t i = 0; i < entityIds.size(); ++i)
                {
                    if (entityIds[i] == entity && activeFlags[i])
                    {
                        return true;
                    }
                }
                return false;
            }
            
            EntityID GetEntityAtIndex(size_t index) const override
            {
                if (index < entityIds.size() && activeFlags[index])
                {
                    return entityIds[index];
                }
                return INVALID_ENTITY;
            }
            
            size_t GetComponentCount() const override
            {
                size_t count = 0;
                for (bool active : activeFlags)
                {
                    if (active) count++;
                }
                return count;
            }
            
            // Add component to dense arrays
            void AddComponent(EntityID entity, T&& component)
            {
                // Check if entity already has this component
                for (size_t i = 0; i < entityIds.size(); ++i)
                {
                    if (entityIds[i] == entity && activeFlags[i])
                    {
                        // Update existing component
                        components[i] = std::forward<T>(component);
                        return;
                    }
                }
                
                // Add new component
                components.push_back(std::forward<T>(component));
                entityIds.push_back(entity);
                activeFlags.push_back(true);
            }
            
            // Get component reference by index (for iteration)
            T& GetComponentByIndex(size_t index)
            {
                assert(index < components.size() && activeFlags[index]);
                return components[index];
            }
            
            const T& GetComponentByIndex(size_t index) const
            {
                assert(index < components.size() && activeFlags[index]);
                return components[index];
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
            container.AddComponent(entity, std::forward<T>(component));
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
            
            // Find the component for this entity
            for (size_t i = 0; i < container.entityIds.size(); ++i)
            {
                if (container.entityIds[i] == entity && container.activeFlags[i])
                {
                    return container.components[i];
                }
            }
            
            // This should never happen due to the assert above
            assert(false);
            static T dummy{};
            return dummy;
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
            
            // Find the component for this entity
            for (size_t i = 0; i < container.entityIds.size(); ++i)
            {
                if (container.entityIds[i] == entity && container.activeFlags[i])
                {
                    return container.components[i];
                }
            }
            
            // This should never happen due to the assert above
            assert(false);
            static T dummy{};
            return dummy;
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
        std::vector<EntityID> GetEntitiesWithComponent() const
        {
            std::vector<EntityID> entities;
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt != m_Containers.end()) {
                const auto& container = *static_cast<const ComponentContainer<T>*>(containerIt->second.get());
                for (size_t i = 0; i < container.entityIds.size(); ++i)
                {
                    if (container.activeFlags[i])
                    {
                        entities.push_back(container.entityIds[i]);
                    }
                }
            }
            return entities;
        }
        
        /**
         * @brief Iterate over all components of a specific type (cache-friendly).
         * @tparam T Component type
         * @tparam Func Function type that takes (EntityID, T&) or (const EntityID, const T&)
         * @param func Function to call for each component
         */
        template<typename T, typename Func>
        void ForEachComponent(Func&& func)
        {
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt != m_Containers.end()) {
                auto& container = *static_cast<ComponentContainer<T>*>(containerIt->second.get());
                for (size_t i = 0; i < container.components.size(); ++i)
                {
                    if (container.activeFlags[i])
                    {
                        func(container.entityIds[i], container.components[i]);
                    }
                }
            }
        }
        
        /**
         * @brief Iterate over all components of a specific type (const version).
         * @tparam T Component type
         * @tparam Func Function type that takes (EntityID, const T&)
         * @param func Function to call for each component
         */
        template<typename T, typename Func>
        void ForEachComponent(Func&& func) const
        {
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt != m_Containers.end()) {
                const auto& container = *static_cast<const ComponentContainer<T>*>(containerIt->second.get());
                for (size_t i = 0; i < container.components.size(); ++i)
                {
                    if (container.activeFlags[i])
                    {
                        func(container.entityIds[i], container.components[i]);
                    }
                }
            }
        }
        
        /**
         * @brief Get component count for a specific type (fast O(1)).
         * @tparam T Component type
         * @return Number of active components of this type
         */
        template<typename T>
        size_t GetComponentCount() const
        {
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt != m_Containers.end()) {
                return containerIt->second->GetComponentCount();
            }
            return 0;
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