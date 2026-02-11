#include "nyon/ecs/ComponentStore.h"

namespace Nyon::ECS
{
    ComponentStore::ComponentStore(EntityManager& entityManager)
        : m_EntityManager(entityManager)
    {
    }
    
    void ComponentStore::RemoveAllComponents(EntityID entity)
    {
        for (auto& pair : m_Containers)
        {
            pair.second->RemoveComponent(entity);
        }
    }
}