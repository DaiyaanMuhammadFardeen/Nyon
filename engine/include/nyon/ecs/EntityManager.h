#pragma once
#include <cstdint>
#include <vector>
#include <bitset>
#include <unordered_set>
namespace Nyon::ECS {
    using EntityID = uint32_t;
    static constexpr EntityID INVALID_ENTITY = static_cast<EntityID>(-1);
    class EntityManager {
    public:
        EntityManager();
        EntityID CreateEntity();
        void DestroyEntity(EntityID entity);
        bool IsEntityValid(EntityID entity) const;
        size_t GetActiveEntityCount() const;
        const std::unordered_set<EntityID>& GetActiveEntities() const;
    private:
        EntityID m_NextID;
        std::vector<bool> m_EntityStates;   
        std::unordered_set<EntityID> m_ActiveEntities;
        std::vector<EntityID> m_FreeIDs;     }; }