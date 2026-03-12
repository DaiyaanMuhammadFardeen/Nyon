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
            std::unordered_map<EntityID, size_t> indexMap; // O(1) lookup map
            
            void RemoveComponent(EntityID entity) override
            {
                auto it = indexMap.find(entity);
                if (it == indexMap.end()) return;
                
                size_t idx = it->second;
                size_t last = components.size() - 1;
                
                if (idx != last)
                {
                    // Swap-and-pop: move last element into the removed slot
                    components[idx]  = std::move(components[last]);
                    entityIds[idx]   = entityIds[last];
                    activeFlags[idx] = activeFlags[last];
                    indexMap[entityIds[idx]] = idx;
                }
                
                // Remove the last element
                components.pop_back();
                entityIds.pop_back();
                activeFlags.pop_back();
                indexMap.erase(it);
            }
            
            bool HasComponent(EntityID entity) const override
            {
                auto it = indexMap.find(entity);
                return it != indexMap.end() && activeFlags[it->second];
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
                return indexMap.size();
            }
            
            // Add component to dense arrays
            void AddComponent(EntityID entity, T&& component)
            {
                // Check if entity already has this component
                auto it = indexMap.find(entity);
                if (it != indexMap.end() && activeFlags[it->second])
                {
                    // Update existing component
                    components[it->second] = std::forward<T>(component);
                    return;
                }
                
                // Add new component
                indexMap[entity] = components.size();
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
            
            // Get all entities with this component (O(1) - returns cached vector)
            const std::vector<EntityID>& GetEntities() const
            {
                return entityIds;
            }
            
            // Get active flags for direct access
            const std::vector<bool>& GetActiveFlags() const
            {
                return activeFlags;
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
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt == m_Containers.end()) {
                std::terminate(); // Component type not registered
            }
            
            auto& container = *static_cast<ComponentContainer<T>*>(containerIt->second.get());
            auto mapIt = container.indexMap.find(entity);
            if (mapIt == container.indexMap.end() || !container.activeFlags[mapIt->second]) {
                std::terminate(); // Entity does not have this component
            }
            
            return container.components[mapIt->second];
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
            auto containerIt = m_Containers.find(typeid(T));
            if (containerIt == m_Containers.end()) {
                std::terminate(); // Component type not registered
            }
            
            const auto& container = *static_cast<const ComponentContainer<T>*>(containerIt->second.get());
            auto mapIt = container.indexMap.find(entity);
            if (mapIt == container.indexMap.end() || !container.activeFlags[mapIt->second]) {
                std::terminate(); // Entity does not have this component
            }
            
            return container.components[mapIt->second];
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
                const auto& container = *static_cast<const ComponentContainer<T>*>(containerIt->second.get());
                return container.GetEntities();
            }
            static std::vector<EntityID> empty;
            return empty;
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
                // Iterate using indexMap for O(1) access
                for (const auto& [entityId, index] : container.indexMap)
                {
                    if (container.activeFlags[index])
                    {
                        func(entityId, container.components[index]);
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
                // Iterate using indexMap for O(1) access
                for (const auto& [entityId, index] : container.indexMap)
                {
                    if (container.activeFlags[index])
                    {
                        func(entityId, container.components[index]);
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