#pragma once
#include "nyon/ecs/System.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include "nyon/graphics/Renderer2D.h"
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
        class NyonDebugRenderer {
        public:
            void DrawLine(const Math::Vector2& p1, const Math::Vector2& p2, 
                         const Math::Vector3& color);
            void DrawSolidCircle(const Math::Vector2& center, float radius, 
                               const Math::Vector3& color);
            void DrawSolidPolygon(const std::vector<Math::Vector2>& vertices, 
                                const Math::Vector3& color);
            void DrawPoint(const Math::Vector2& point, float size, 
                          const Math::Vector3& color);
            void QueueLine(const Math::Vector2& p1, const Math::Vector2& p2, 
                          const Math::Vector3& color);
            void QueueCircle(const Math::Vector2& center, float radius, 
                           const Math::Vector3& color);
            void QueuePolygon(const std::vector<Math::Vector2>& vertices, 
                            const Math::Vector3& color);
            void QueuePoint(const Math::Vector2& point, float size, 
                          const Math::Vector3& color);
        private:
            static constexpr int CIRCLE_SEGMENTS = 16;
            void DrawCircleOutline(const Math::Vector2& center, float radius, 
                                 const Math::Vector3& color); };
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
        void QueueLine(const Math::Vector2& p1, const Math::Vector2& p2, const Math::Vector3& color);
        void QueueCircle(const Math::Vector2& center, float radius, const Math::Vector3& color);
        void QueuePolygon(const std::vector<Math::Vector2>& vertices, const Math::Vector3& color);
        void QueuePoint(const Math::Vector2& point, float size, const Math::Vector3& color);
        ComponentStore* m_ComponentStore = nullptr;
        PhysicsWorldComponent* m_PhysicsWorld;
        NyonDebugRenderer m_Renderer;
        struct DebugDrawCommand {
            enum Type { LINE, CIRCLE, POLYGON, POINT } type;
            Math::Vector2 p1, p2;   
            Math::Vector2 center;   
            float radius;           
            std::vector<Math::Vector2> vertices;   
            Math::Vector3 color;
            float size;              };
        std::vector<DebugDrawCommand> m_DebugCommands;
        bool m_DrawShapes = true;
        bool m_DrawJoints = true;
        bool m_DrawAABBs = false;
        bool m_DrawContacts = false;
        bool m_DrawCOM = false;
        bool m_DrawIslands = false;
        float m_Alpha = 1.0f;
        bool m_ShouldRender = false; }; }
