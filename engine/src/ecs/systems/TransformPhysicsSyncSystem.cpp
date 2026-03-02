#include "nyon/ecs/systems/TransformPhysicsSyncSystem.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"

namespace Nyon::ECS
{
    void TransformPhysicsSyncSystem::Initialize(EntityManager& entityManager, ComponentStore& componentStore)
    {
        // Store reference to component store for later use
        m_ComponentStore = &componentStore;
        
        // Get all entities with physics components that should be synchronized
        const auto& physicsEntities = componentStore.GetEntitiesWithComponent<PhysicsBodyComponent>();
        
        m_SyncEntities.reserve(physicsEntities.size());
        
        for (auto entityId : physicsEntities)
        {
            PhysicsBodyComponent* physicsBody = &componentStore.GetComponent<PhysicsBodyComponent>(entityId);
            ColliderComponent* collider = &componentStore.GetComponent<ColliderComponent>(entityId);
            TransformComponent* transform = &componentStore.GetComponent<TransformComponent>(entityId);
            
            // Only sync entities that have both physics and transform components
            if (physicsBody && transform)
            {
                SyncEntity syncEntity;
                syncEntity.entityId = entityId;
                syncEntity.physicsBody = physicsBody;
                syncEntity.collider = collider;
                syncEntity.transform = transform;
                syncEntity.lastPosition = transform->position;
                syncEntity.lastRotation = transform->rotation;
                syncEntity.forceTransformUpdate = false;
                
                m_SyncEntities.push_back(syncEntity);
            }
        }
    }
    
    void TransformPhysicsSyncSystem::Update(float deltaTime)
    {
        for (auto& entity : m_SyncEntities)
        {
            switch (m_SyncMode)
            {
                case SyncMode::PhysicsToTransform:
                    SyncPhysicsToTransform(entity);
                    break;
                    
                case SyncMode::TransformToPhysics:
                    SyncTransformToPhysics(entity);
                    break;
                    
                case SyncMode::Bidirectional:
                    if (HasExternalTransformChange(entity))
                    {
                        SyncTransformToPhysics(entity);
                    }
                    else
                    {
                        SyncPhysicsToTransform(entity);
                    }
                    break;
            }
            
            // Update last known values
            entity.lastPosition = entity.transform->position;
            entity.lastRotation = entity.transform->rotation;
        }
    }
    
    void TransformPhysicsSyncSystem::SyncPhysicsToTransform(SyncEntity& entity)
    {
        // Update transform position from physics body
        entity.transform->position = entity.transform->position + 
            (entity.physicsBody->velocity * m_InterpolationFactor);
            
        // Update transform rotation from physics body
        entity.transform->rotation = entity.transform->rotation + 
            (entity.physicsBody->angularVelocity * m_InterpolationFactor);
            
        // Update collider proxy in broad phase if it exists
        if (entity.collider)
        {
            UpdateColliderProxy(entity);
        }
    }
    
    void TransformPhysicsSyncSystem::SyncTransformToPhysics(SyncEntity& entity)
    {
        // Update physics body position from transform
        entity.physicsBody->velocity = (entity.transform->position - entity.lastPosition) / 
            (m_InterpolationFactor > 0.0f ? m_InterpolationFactor : 1.0f);
            
        // Update physics body rotation from transform
        float rotationDelta = entity.transform->rotation - entity.lastRotation;
        entity.physicsBody->angularVelocity = rotationDelta / 
            (m_InterpolationFactor > 0.0f ? m_InterpolationFactor : 1.0f);
            
        // Wake up sleeping bodies when transformed externally
        if (entity.physicsBody->isAwake == false)
        {
            entity.physicsBody->SetAwake(true);
        }
        
        // Update collider proxy in broad phase if it exists
        if (entity.collider)
        {
            UpdateColliderProxy(entity);
        }
    }
    
    bool TransformPhysicsSyncSystem::HasExternalTransformChange(const SyncEntity& entity) const
    {
        // Check if transform was changed externally (not by physics system)
        Math::Vector2 positionDelta = entity.transform->position - entity.lastPosition;
        float rotationDelta = std::abs(entity.transform->rotation - entity.lastRotation);
        
        return (positionDelta.Length() > POSITION_CHANGE_THRESHOLD) || 
               (rotationDelta > ROTATION_CHANGE_THRESHOLD) ||
               entity.forceTransformUpdate;
    }
    
    void TransformPhysicsSyncSystem::UpdateColliderProxy(SyncEntity& entity)
    {
        if (!entity.collider || entity.collider->proxyId == -1)
            return;
            
        // This would update the broad phase proxy with new transform
        // Implementation depends on your DynamicTree integration
        // entity.collider->proxyId would be used to update the proxy
        entity.forceTransformUpdate = false;
    }
}
