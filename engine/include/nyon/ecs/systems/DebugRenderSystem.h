#pragma once

#include "nyon/ecs/System.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include "nyon/graphics/Renderer2D.h"
#include <vector>

namespace Nyon::ECS
{
    /**
     * @brief Debug render system for physics visualization.
     * 
     * Renders physics debug information including shapes, joints, AABBs,
     * contact points, and other diagnostic information.
     */
    class DebugRenderSystem : public System
    {
    public:
        DebugRenderSystem();
        void Update(float deltaTime) override;
        void Initialize(EntityManager& entityManager, ComponentStore& componentStore) override;
        
        // Debug drawing configuration
        void SetFlags(bool drawShapes, bool drawJoints, bool drawAABBs, 
                     bool drawContacts, bool drawCOM);
        
    private:
        // Concrete renderer implementation using Nyon's Renderer2D
        class NyonDebugRenderer
        {
        public:
            void DrawLine(const Math::Vector2& p1, const Math::Vector2& p2, 
                         const Math::Vector3& color);
            void DrawSolidCircle(const Math::Vector2& center, float radius, 
                               const Math::Vector3& color);
            void DrawSolidPolygon(const std::vector<Math::Vector2>& vertices, 
                                const Math::Vector3& color);
            void DrawPoint(const Math::Vector2& point, float size, 
                          const Math::Vector3& color);
            
        private:
            static constexpr int CIRCLE_SEGMENTS = 16;
            void DrawCircleOutline(const Math::Vector2& center, float radius, 
                                 const Math::Vector3& color);
        };
        
        // Internal drawing methods
        void DrawShapes();
        void DrawJoints();
        void DrawAABBs();
        void DrawContacts();
        void DrawCenterOfMass();
        void DrawIslands();
        
        // Shape drawing helpers
        void DrawCircleShape(const Math::Vector2& position,
                           const ColliderComponent::CircleShape& circle,
                           const Math::Vector3& color);
        void DrawPolygonShape(const Math::Vector2& position, float angle,
                            const ColliderComponent::PolygonShape& polygon,
                            const Math::Vector3& color);
        void DrawCapsuleShape(const Math::Vector2& position, float angle,
                            const ColliderComponent::CapsuleShape& capsule,
                            const Math::Vector3& color);
        
        // Component references
        PhysicsWorldComponent* m_PhysicsWorld;
        std::vector<std::tuple<uint32_t, PhysicsBodyComponent*, ColliderComponent*>> m_Entities;
        
        // Debug renderer implementation
        NyonDebugRenderer m_Renderer;
        
        // Drawing flags
        bool m_DrawShapes = true;
        bool m_DrawJoints = true;
        bool m_DrawAABBs = false;
        bool m_DrawContacts = false;
        bool m_DrawCOM = false;
        bool m_DrawIslands = false;
    };
}
