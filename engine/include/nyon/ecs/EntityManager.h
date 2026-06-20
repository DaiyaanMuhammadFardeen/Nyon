#pragma once

#include <cstdint>
#include <vector>
#include <bitset>
#include <unordered_set>

namespace Nyon::ECS
{
    using EntityID = uint32_t;
    static constexpr EntityID INVALID_ENTITY = static_cast<EntityID>(-1);
    
    class ComponentStore;
    
    /**
     * @brief Manages entity creation, destruction, and lifecycle.
     * 
     * Handles entity ID generation and tracks which entities are active.
     * Uses a simple incrementing ID system with recycling of destroyed entities.
     */
    class EntityManager
    {
    public:
        EntityManager();
        
        /**
         * @brief Create a new entity and return its ID.
         * @return Unique EntityID for the new entity
         */
        EntityID CreateEntity();
        
        /**
         * @brief Destroy an entity and mark its ID as available for reuse.
         * @param entity Entity to destroy
         */
        void DestroyEntity(EntityID entity);
        
        /**
         * @brief Destroy an entity and remove all its components.
         * @param entity Entity to destroy
         * @param componentStore Component store to remove components from
         */
        void DestroyEntity(EntityID entity, ComponentStore& componentStore);
        
        /**
         * @brief Check if an entity is currently active/alive.
         * @param entity Entity to check
         * @return True if entity exists and is active
         */
        bool IsEntityValid(EntityID entity) const;
        
        /**
         * @brief Get the total number of active entities.
         * @return Count of active entities
         */
        size_t GetActiveEntityCount() const;
        
        /**
         * @brief Get all currently active entity IDs.
         * @return Unordered set of active entity IDs
         */
        const std::unordered_set<EntityID>& GetActiveEntities() const;
        
    private:
        EntityID m_NextID;
        std::vector<bool> m_EntityStates;  // true = active, false = destroyed
        std::unordered_set<EntityID> m_ActiveEntities;
        std::vector<EntityID> m_FreeIDs;   // recycled IDs for reuse
    };
}