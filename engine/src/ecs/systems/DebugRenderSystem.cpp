#include "nyon/ecs/systems/DebugRenderSystem.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/components/TransformComponent.h"
#include <cfloat>

// Debug logging macro - only output in debug builds
#ifdef _DEBUG
#define NYON_DEBUG_LOG(x) std::cerr << x << std::endl
#else
#define NYON_DEBUG_LOG(x)
#endif

namespace Nyon::ECS
{
    // Add debug output to constructor
    DebugRenderSystem::DebugRenderSystem()
    {
        NYON_DEBUG_LOG("[DEBUG] DebugRenderSystem constructor called");
    }
    
    // Implement the missing NyonDebugRenderer methods using actual Renderer2D API
    
    void DebugRenderSystem::NyonDebugRenderer::DrawLine(const Math::Vector2& p1, const Math::Vector2& p2, 
                                                       const Math::Vector3& color)
    {
        // Direct drawing (for immediate mode)
        Graphics::Renderer2D::DrawLine(p1, p2, color);
    }
    
    void DebugRenderSystem::NyonDebugRenderer::DrawSolidCircle(const Math::Vector2& center, float radius, 
                                                              const Math::Vector3& color)
    {
        // Use proper circle drawing with 16 segments
        Graphics::Renderer2D::DrawSolidCircle(center, radius, color, 16);
    }
    
    void DebugRenderSystem::NyonDebugRenderer::DrawPoint(const Math::Vector2& point, float size, 
                                                        const Math::Vector3& color)
    {
        // Draw point as a small quad
        Math::Vector2 quadSize(size, size);
        Math::Vector2 origin(size * 0.5f, size * 0.5f);
        Graphics::Renderer2D::DrawQuad(point, quadSize, origin, color);
    }
    
    void DebugRenderSystem::NyonDebugRenderer::DrawCircleOutline(const Math::Vector2& center, float radius, 
                                                                const Math::Vector3& color)
    {
        // Draw circle outline as a quad outline for now
        // This is a simplification - proper circle would need multiple line segments
        Math::Vector2 size(radius * 2.0f, radius * 2.0f);
        Math::Vector2 origin(radius, radius);
        Graphics::Renderer2D::DrawQuad(center, size, origin, color);
    }
    
    void DebugRenderSystem::Initialize(EntityManager& entityManager, ComponentStore& componentStore)
    {
        // Store reference to component store for later use
        // Physics world entity may not exist yet at this point (created in OnECSStart),
        // so we do NOT cache m_PhysicsWorld here - it is resolved lazily in Update().
        m_ComponentStore = &componentStore;
    }
    
    void DebugRenderSystem::Update(float deltaTime)
    {
        // Lazily resolve the physics world pointer each update in case it was
        // created after this system was initialized (e.g. in OnECSStart).
        if (!m_PhysicsWorld && m_ComponentStore)
        {
            const auto& worldEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsWorldComponent>();
            if (!worldEntities.empty())
            {
                m_PhysicsWorld = &m_ComponentStore->GetComponent<PhysicsWorldComponent>(worldEntities[0]);
            }
        }

        if (!m_PhysicsWorld)
            return;
            
        // Clear previous debug commands
        m_DebugCommands.clear();
        
        // Collect debug draw commands (don't render yet)
        if (m_DrawShapes) DrawShapes();
        if (m_DrawJoints) DrawJoints();
        if (m_DrawAABBs) DrawAABBs();
        if (m_DrawContacts) DrawContacts();
        if (m_DrawCOM) DrawCenterOfMass();
        if (m_DrawIslands) DrawIslands();
        
        // Signal that we have debug info ready to render
        m_ShouldRender = !m_DebugCommands.empty();
    }
    
    void DebugRenderSystem::RenderDebugInfo()
    {
        // Only render if we have debug commands and rendering is enabled
        if (!m_ShouldRender || m_DebugCommands.empty())
            return;
            
        // DON'T call BeginScene() - it would clear the buffers that RenderSystem already drew!
        // Just execute the debug draw commands directly
        for (const auto& cmd : m_DebugCommands)
        {
            switch (cmd.type)
            {
                case DebugDrawCommand::LINE:
                    Graphics::Renderer2D::DrawLine(cmd.p1, cmd.p2, cmd.color);
                    break;
                case DebugDrawCommand::CIRCLE:
                    Graphics::Renderer2D::DrawCircle(cmd.center, cmd.radius, cmd.color);
                    break;
                case DebugDrawCommand::POLYGON:
                    if (!cmd.vertices.empty())
                    {
                        Graphics::Renderer2D::DrawPolygon(cmd.vertices, cmd.color);
                    }
                    break;
                case DebugDrawCommand::POINT:
                    Graphics::Renderer2D::DrawQuad(cmd.p1, 
                                                  {cmd.size, cmd.size},
                                                  {cmd.size * 0.5f, cmd.size * 0.5f}, 
                                                  cmd.color);
                    break;
            }
        }
        
        // DON'T Flush here - let EndScene from RenderSystem flush everything together!
        // Flushing separately causes flickering and double-buffering issues
        // Graphics::Renderer2D::Flush();  // REMOVED - causes flickering!
        
        // Reset for next frame
        m_ShouldRender = false;
    }
    
    void DebugRenderSystem::SetFlags(bool drawShapes, bool drawJoints, bool drawAABBs, 
                                   bool drawContacts, bool drawCOM)
    {
        m_DrawShapes = drawShapes;
        m_DrawJoints = drawJoints;
        m_DrawAABBs = drawAABBs;
        m_DrawContacts = drawContacts;
        m_DrawCOM = drawCOM;
    }
    
    void DebugRenderSystem::DrawShapes()
    {
        // Query fresh entities each frame to avoid stale pointers
        const auto& bodyEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsBodyComponent>();
        
        for (auto entityId : bodyEntities)
        {
            const auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityId);
            const ColliderComponent* collider = nullptr;
            
            if (m_ComponentStore->HasComponent<ColliderComponent>(entityId))
            {
                collider = &m_ComponentStore->GetComponent<ColliderComponent>(entityId);
            }
            
            // Get transform data with interpolation for smooth rendering
            Math::Vector2 position = {0.0f, 0.0f};
            float angle = 0.0f;
            if (m_ComponentStore->HasComponent<TransformComponent>(entityId))
            {
                const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
                // Use interpolated position and rotation based on render alpha
                position = transform.GetInterpolatedPosition(m_Alpha);
                angle = transform.GetInterpolatedRotation(m_Alpha);
            }
            
            // Color based on body state
            Math::Vector3 color = {0.0f, 1.0f, 0.0f}; // Green default
            if (body.isStatic) color = {0.5f, 0.5f, 0.5f}; // Gray for static
            if (!body.isAwake) color = {0.2f, 0.2f, 0.8f}; // Blue for sleeping
            if (collider && collider->isSensor) color = {1.0f, 1.0f, 0.0f}; // Yellow for sensors
            
            if (collider)
            {
                switch (collider->GetType())
                {
                    case ColliderComponent::ShapeType::Circle:
                        DrawCircleShape(position, collider->GetCircle(), color);
                        break;
                        
                    case ColliderComponent::ShapeType::Polygon:
                        DrawPolygonShape(position, angle, collider->GetPolygon(), color);
                        break;
                        
                    case ColliderComponent::ShapeType::Capsule:
                        DrawCapsuleShape(position, angle, collider->GetCapsule(), color);
                        break;
                        
                    default:
                        break;
                }
            }
        }
    }
    
    void DebugRenderSystem::DrawJoints()
    {
        // Joints not implemented yet - joint solver is not available
        // Set m_DrawJoints = false to disable until joint implementation is complete
    }
    
    void DebugRenderSystem::DrawAABBs()
    {
        // Query fresh entities each frame to avoid stale pointers
        const auto& bodyEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsBodyComponent>();
        
        for (auto entityId : bodyEntities)
        {
            if (!m_ComponentStore->HasComponent<ColliderComponent>(entityId))
                continue;
                
            const auto& collider = m_ComponentStore->GetComponent<ColliderComponent>(entityId);
            
            // Use interpolated transform position and rotation for smooth rendering
            Math::Vector2 position = {0.0f, 0.0f};
            float angle = 0.0f;
            if (m_ComponentStore->HasComponent<TransformComponent>(entityId))
            {
                const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
                position = transform.GetInterpolatedPosition(m_Alpha);
                angle = transform.GetInterpolatedRotation(m_Alpha);
            }
            
            Math::Vector2 min, max;
            collider.CalculateAABB(position, angle, min, max);
            
            // Draw AABB outline using queued commands
            Math::Vector3 color = {1.0f, 0.0f, 0.0f}; // Red for AABBs
            QueueLine({min.x, min.y}, {max.x, min.y}, color);
            QueueLine({max.x, min.y}, {max.x, max.y}, color);
            QueueLine({max.x, max.y}, {min.x, max.y}, color);
            QueueLine({min.x, max.y}, {min.x, min.y}, color);
        }
    }
    
    void DebugRenderSystem::DrawContacts()
    {
        if (!m_PhysicsWorld) return;
        
        // Draw contact points and normals from physics world
        for (const auto& manifold : m_PhysicsWorld->contactManifolds)
        {
            if (!manifold.touching) continue;
            
            for (const auto& cp : manifold.points)
            {
                // Draw contact point as small red circle
                QueueCircle(cp.position, 3.0f, {1.f, 0.f, 0.f});
                
                // Draw contact normal as green line
                Math::Vector2 normalEnd = cp.position + manifold.normal * 10.0f;
                QueueLine(cp.position, normalEnd, {0.f, 1.f, 0.f});
            }
        }
    }
    
    void DebugRenderSystem::DrawCenterOfMass()
    {
        // Query fresh entities each frame to avoid stale pointers
        const auto& bodyEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsBodyComponent>();
        
        for (auto entityId : bodyEntities)
        {
            const auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityId);
            
            Math::Vector2 position = {0.0f, 0.0f};
            if (m_ComponentStore->HasComponent<TransformComponent>(entityId))
            {
                const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
                position = transform.GetInterpolatedPosition(m_Alpha);
            }
            
            Math::Vector2 com = position + body.centerOfMass;
            
            // Draw center of mass as a small point using queued command
            Math::Vector3 color = {1.0f, 0.0f, 1.0f}; // Magenta
            QueuePoint(com, 3.0f, color);
        }
    }
    
    void DebugRenderSystem::DrawIslands()
    {
        // Islands not implemented yet - sleep/island system needs implementation
        // Set m_DrawIslands = false to disable
    }
    
    void DebugRenderSystem::QueueLine(const Math::Vector2& p1, const Math::Vector2& p2, const Math::Vector3& color)
    {
        DebugDrawCommand cmd;
        cmd.type = DebugDrawCommand::LINE;
        cmd.p1 = p1;
        cmd.p2 = p2;
        cmd.color = color;
        m_DebugCommands.push_back(cmd);
    }
    
    void DebugRenderSystem::QueueCircle(const Math::Vector2& center, float radius, const Math::Vector3& color)
    {
        DebugDrawCommand cmd;
        cmd.type = DebugDrawCommand::CIRCLE;
        cmd.center = center;
        cmd.radius = radius;
        cmd.color = color;
        m_DebugCommands.push_back(cmd);
    }
    
    void DebugRenderSystem::QueuePolygon(const std::vector<Math::Vector2>& vertices, const Math::Vector3& color)
    {
        DebugDrawCommand cmd;
        cmd.type = DebugDrawCommand::POLYGON;
        cmd.vertices = vertices;
        cmd.color = color;
        m_DebugCommands.push_back(cmd);
    }
    
    void DebugRenderSystem::QueuePoint(const Math::Vector2& point, float size, const Math::Vector3& color)
    {
        DebugDrawCommand cmd;
        cmd.type = DebugDrawCommand::POINT;
        cmd.p1 = point;
        cmd.size = size;
        cmd.color = color;
        m_DebugCommands.push_back(cmd);
    }
    
    void DebugRenderSystem::DrawCircleShape(const Math::Vector2& position, 
                                          const ColliderComponent::CircleShape& circle,
                                          const Math::Vector3& color)
    {
        Math::Vector2 worldCenter = position + circle.center;
        QueueCircle(worldCenter, circle.radius, color);
    }
    
    void DebugRenderSystem::DrawPolygonShape(const Math::Vector2& position, float angle,
                                           const ColliderComponent::PolygonShape& polygon,
                                           const Math::Vector3& color)
    {
        if (polygon.vertices.empty()) return;
        
        std::vector<Math::Vector2> worldVertices;
        worldVertices.reserve(polygon.vertices.size());
        
        // Transform vertices to world space
        float cosAngle = cos(angle);
        float sinAngle = sin(angle);
        
        for (const auto& vertex : polygon.vertices)
        {
            // Rotate and translate
            Math::Vector2 rotated{
                vertex.x * cosAngle - vertex.y * sinAngle,
                vertex.x * sinAngle + vertex.y * cosAngle
            };
            worldVertices.push_back(position + rotated);
        }
        
        QueuePolygon(worldVertices, color);
    }
    
    void DebugRenderSystem::DrawCapsuleShape(const Math::Vector2& position, float angle,
                                           const ColliderComponent::CapsuleShape& capsule,
                                           const Math::Vector3& color)
    {
        // Draw capsule as two circles connected by rectangles
        // Simplified implementation - would normally draw proper capsule shape
        
        Math::Vector2 center1 = position + capsule.center1;
        Math::Vector2 center2 = position + capsule.center2;
        
        QueueCircle(center1, capsule.radius, color);
        QueueCircle(center2, capsule.radius, color);
                
        // Draw connecting line
        QueueLine(center1, center2, color);
    }
}
