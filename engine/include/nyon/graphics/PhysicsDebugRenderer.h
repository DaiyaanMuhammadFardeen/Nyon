#pragma once
#include "nyon/graphics/Renderer2D.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include <vector>
namespace Nyon::Graphics {
    /**
     * @brief Debug rendering flags for physics visualization
     */
    enum class DebugRenderFlag {
        None                = 0,
        Shapes              = (1 << 0),
        AABBs               = (1 << 1),
        Contacts            = (1 << 2),
        Manifolds           = (1 << 3),
        Joints              = (1 << 4),
        CentersOfMass       = (1 << 5),
        VelocityVectors     = (1 << 6),
        CollisionNormals    = (1 << 7),
        Sensors             = (1 << 8),
        ChainLinks          = (1 << 9),
        All                 = 0xFFFF
    };
    inline DebugRenderFlag operator|(DebugRenderFlag a, DebugRenderFlag b)
    {
        return static_cast<DebugRenderFlag>(static_cast<int>(a) | static_cast<int>(b));
    }
    inline bool HasFlag(DebugRenderFlag flags, DebugRenderFlag test)
    {
        return (static_cast<int>(flags) & static_cast<int>(test)) != 0;
    }
    /**
     * @brief Physics debug renderer with full manifold and contact visualization
     */
    class PhysicsDebugRenderer {
    public:
        PhysicsDebugRenderer();
        ~PhysicsDebugRenderer();
        void SetActiveFlags(DebugRenderFlag flags);
        DebugRenderFlag GetActiveFlags() const { return m_ActiveFlags; }
        void DrawCollider(const ECS::ColliderComponent& collider,
                         const ECS::TransformComponent& transform,
                         const Math::Vector3& color = {1.0f, 1.0f, 1.0f});
        void DrawCircleShape(const ECS::ColliderComponent::CircleShape& circle,
                            const ECS::TransformComponent& transform,
                            const Math::Vector3& color);
        void DrawPolygonShape(const ECS::ColliderComponent::PolygonShape& polygon,
                             const ECS::TransformComponent& transform,
                             const Math::Vector3& color);
        void DrawCapsuleShape(const ECS::ColliderComponent::CapsuleShape& capsule,
                             const ECS::TransformComponent& transform,
                             const Math::Vector3& color);
        void DrawSegmentShape(const ECS::ColliderComponent::SegmentShape& segment,
                             const ECS::TransformComponent& transform,
                             const Math::Vector3& color);
        void DrawChainShape(const ECS::ColliderComponent::ChainShape& chain,
                           const ECS::TransformComponent& transform,
                           const Math::Vector3& color);
        void DrawManifold(const ECS::ContactManifold& manifold);
        void DrawContactPoint(const Math::Vector2& position,
                            const Math::Vector2& normal,
                            float separation,
                            bool isTouching);
        void DrawAABB(const Math::Vector2& min, const Math::Vector2& max,
                     const Math::Vector3& color = {1.0f, 0.0f, 0.0f});
        void DrawTransform(const Math::Vector2& position, float rotation,
                          float scale = 1.0f);
        void DrawCenterOfMass(const Math::Vector2& position,
                             const Math::Vector3& color = {1.0f, 1.0f, 0.0f});
        void DrawVelocityVector(const Math::Vector2& position,
                               const Math::Vector2& velocity,
                               const Math::Vector3& color = {0.0f, 1.0f, 0.0f},
                               float scale = 1.0f);
        static Math::Vector3 ColorFromHSV(float h, float s, float v);
        static Math::Vector3 LerpColor(const Math::Vector3& a,
                                      const Math::Vector3& b,
                                      float t);
    private:
        DebugRenderFlag m_ActiveFlags = DebugRenderFlag::None;
    };
}
