#pragma once

#include "nyon/ecs/System.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/physics/DynamicTree.h"
#include <vector>
#include <unordered_map>

namespace Nyon::ECS
{
    /**
     * @brief Collision pipeline system integrating broad and narrow phase detection.
     * 
     * Manages the complete collision detection pipeline from broad phase queries
     * through narrow phase manifold generation to contact processing.
     */
    class CollisionPipelineSystem : public System
    {
    public:
        void Update(float deltaTime) override;
        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override;
        
    private:
        // Contact structures
        struct ContactPair
        {
            uint32_t entityIdA;
            uint32_t entityIdB;
            uint32_t shapeIdA;
            uint32_t shapeIdB;
            bool operator==(const ContactPair& other) const
            {
                return (entityIdA == other.entityIdA && entityIdB == other.entityIdB &&
                        shapeIdA == other.shapeIdA && shapeIdB == other.shapeIdB);
            }
        };
        
        struct ContactPairHash
        {
            size_t operator()(const ContactPair& pair) const
            {
                return ((std::hash<uint32_t>{}(pair.entityIdA) * 31 + 
                        std::hash<uint32_t>{}(pair.entityIdB)) * 31 +
                        std::hash<uint32_t>{}(pair.shapeIdA)) * 31 +
                        std::hash<uint32_t>{}(pair.shapeIdB);
            }
        };
        
        // Broad phase callback
        struct BroadPhaseCallback : public Physics::ITreeQueryCallback
        {
            CollisionPipelineSystem* system;
            uint32_t entityId;
            uint32_t shapeId;
            
            bool QueryCallback(uint32_t nodeId, uint32_t userData) override;
        };
        
        // Internal methods
        void UpdateBroadPhase();
        void ProcessNarrowPhase();
        void UpdateContacts();
        void DestroyInactiveContacts();
        
        // Shape-AABB management
        void UpdateShapeAABB(uint32_t entityId, uint32_t shapeId, 
                           ColliderComponent* collider, 
                           const Math::Vector2& position);
        
        // Component references
        ComponentStore* m_ComponentStore = nullptr;
        PhysicsWorldComponent* m_PhysicsWorld;
        std::vector<std::tuple<uint32_t, PhysicsBodyComponent*, ColliderComponent*>> m_Colliders;
        
        // Broad phase structure
        Physics::DynamicTree m_BroadPhaseTree;
        std::unordered_map<uint32_t, uint32_t> m_ShapeProxyMap; // shapeId -> proxyId
        
        // Contact tracking
        std::unordered_map<ContactPair, uint32_t, ContactPairHash> m_ContactMap;
        std::vector<ContactPair> m_ActivePairs;
        
        // Timing
        float m_Accumulator = 0.0f;
    };
}
