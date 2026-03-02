#pragma once

#include "nyon/ecs/System.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
#include <vector>

namespace Nyon::ECS
{
    /**
     * @brief Transform-Physics synchronization system.
     * 
     * Handles bidirectional synchronization between TransformComponent and Physics components:
     * - Physics → Transform: Updates visual transforms from physics simulation
     * - Transform → Physics: Updates physics from manual transform changes (teleportation, etc.)
     * 
     * This system ensures that visual representation matches physics state while
     * allowing for external transform manipulation when needed.
     */
    class TransformPhysicsSyncSystem : public System
    {
    public:
        void Update(float deltaTime) override;
        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override;
        
        // Synchronization modes
        enum class SyncMode
        {
            PhysicsToTransform,  // Physics drives transform (normal simulation)
            TransformToPhysics,  // Transform drives physics (teleportation)
            Bidirectional        // Both directions (careful with conflicts)
        };
        
        void SetSyncMode(SyncMode mode) { m_SyncMode = mode; }
        SyncMode GetSyncMode() const { return m_SyncMode; }
        
    private:
        // Entity tracking structure
        struct SyncEntity
        {
            uint32_t entityId;
            PhysicsBodyComponent* physicsBody;
            ColliderComponent* collider;
            TransformComponent* transform;
            Math::Vector2 lastPosition;    // For detecting external changes
            float lastRotation;            // For detecting external changes
            bool forceTransformUpdate;     // Force update regardless of change detection
        };
        
        // Internal methods
        void SyncPhysicsToTransform(SyncEntity& entity);
        void SyncTransformToPhysics(SyncEntity& entity);
        bool HasExternalTransformChange(const SyncEntity& entity) const;
        void UpdateColliderProxy(SyncEntity& entity);
        
        // Component references
        ComponentStore* m_ComponentStore = nullptr;
        std::vector<SyncEntity> m_SyncEntities;
        
        // System configuration
        SyncMode m_SyncMode = SyncMode::PhysicsToTransform;
        bool m_EnableInterpolation = true;     // Enable transform interpolation
        float m_InterpolationFactor = 1.0f;    // Interpolation blend factor
        float m_DeltaTime = 0.016f;            // Last frame's deltaTime for velocity calculations
        
        // Change detection thresholds
        static constexpr float POSITION_CHANGE_THRESHOLD = 0.1f;
        static constexpr float ROTATION_CHANGE_THRESHOLD = 0.01f; // radians
    };
}
