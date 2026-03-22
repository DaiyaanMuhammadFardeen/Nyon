#pragma once
#include "nyon/ecs/System.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include "nyon/graphics/PhysicsDebugRenderer.h"
#include <vector>
namespace Nyon::ECS {
    class DebugRenderSystem : public System {
    public:
        DebugRenderSystem();
        void Update(float deltaTime) override;
        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override;
        void RenderDebugInfo();
        void SetInterpolationAlpha(float alpha) { m_Alpha = alpha; }
        float GetInterpolationAlpha() const { return m_Alpha; }
        void SetFlags(bool drawShapes, bool drawJoints, bool drawAABBs, 
                     bool drawContacts, bool drawCOM);
    private:
        void DrawShapes();
        void DrawJoints();
        void DrawAABBs();
        void DrawContacts();
        void DrawCenterOfMass();
        void DrawIslands();
        void DrawCircleShape(const Math::Vector2& position,
                           const ColliderComponent::CircleShape& circle,
                           const Math::Vector3& color);
        void DrawPolygonShape(const Math::Vector2& position, float angle,
                            const ColliderComponent::PolygonShape& polygon,
                            const Math::Vector3& color);
        void DrawCapsuleShape(const Math::Vector2& position, float angle,
                            const ColliderComponent::CapsuleShape& capsule,
                            const Math::Vector3& color);
        ComponentStore* m_ComponentStore = nullptr;
        EntityID m_PhysicsWorldEntity = INVALID_ENTITY;
        Graphics::PhysicsDebugRenderer m_DebugRenderer;
        bool m_DrawShapes = true;
        bool m_DrawJoints = true;
        bool m_DrawAABBs = false;
        bool m_DrawContacts = false;
        bool m_DrawCOM = false;
        bool m_DrawIslands = false;
        float m_Alpha = 1.0f; }; }
