#include "nyon/ecs/EntityManager.h"
namespace Nyon::ECS {
    EntityManager::EntityManager()
        : m_NextID(0) { }
    EntityID EntityManager::CreateEntity() {
        EntityID id;
        if (!m_FreeIDs.empty()) {
            id = m_FreeIDs.back();
            m_FreeIDs.pop_back();
            m_EntityStates[id] = true; }
        else {
            id = m_NextID++;
            m_EntityStates.push_back(true); }
        m_ActiveEntities.insert(id);
        return id; }
    void EntityManager::DestroyEntity(EntityID entity) {
        if (entity >= m_EntityStates.size() || !m_EntityStates[entity]) {
            return;   }
        m_EntityStates[entity] = false;
        m_ActiveEntities.erase(entity);
        m_FreeIDs.push_back(entity); }
    bool EntityManager::IsEntityValid(EntityID entity) const {
        return entity < m_EntityStates.size() && m_EntityStates[entity]; }
    size_t EntityManager::GetActiveEntityCount() const {
        return m_ActiveEntities.size(); }
    const std::unordered_set<EntityID>& EntityManager::GetActiveEntities() const {
        return m_ActiveEntities; } }