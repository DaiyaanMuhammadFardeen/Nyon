#include "nyon/ecs/systems/DebugRenderSystem.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/components/TransformComponent.h"
#include <cfloat>
#include <iostream>

namespace Nyon::ECS
{
    // Add debug output to constructor
    DebugRenderSystem::DebugRenderSystem()
    {
        std::cout << "[DEBUG] DebugRenderSystem constructor called\n";
    }
    
    // Implement the missing NyonDebugRenderer methods using actual Renderer2D API
    
    void DebugRenderSystem::NyonDebugRenderer::DrawLine(const Math::Vector2& p1, const Math::Vector2& p2, 
                                                       const Math::Vector3& color)
    {
        // Direct drawing (for immediate mode)
        Graphics::Renderer2D::DrawLine(p1, p2, color);
    }
    
    void DebugRenderSystem::NyonDebugRenderer::QueueLine(const Math::Vector2& p1, const Math::Vector2& p2, 
                                                        const Math::Vector3& color)
    {
        // This would queue the command, but we need access to the parent system
        // For now, we'll implement this in the parent class methods
    }
    
    void DebugRenderSystem::NyonDebugRenderer::DrawSolidCircle(const Math::Vector2& center, float radius, 
                                                              const Math::Vector3& color)
    {
        // Approximate circle with a quad (rectangle) since Renderer2D doesn't have circle drawing
        Math::Vector2 size(radius * 2.0f, radius * 2.0f);
        Math::Vector2 origin(radius, radius); // Center origin
        Graphics::Renderer2D::DrawQuad(center, size, origin, color);
    }
    
    void DebugRenderSystem::NyonDebugRenderer::QueueCircle(const Math::Vector2& center, float radius, 
                                                          const Math::Vector3& color)
    {
        // Implementation moved to parent class
    }
    
    void DebugRenderSystem::NyonDebugRenderer::DrawSolidPolygon(const std::vector<Math::Vector2>& vertices, 
                                                               const Math::Vector3& color)
    {
        if (vertices.size() < 3) return;
        
        // For now, approximate with bounding box quad since Renderer2D doesn't have polygon drawing
        Math::Vector2 min(FLT_MAX, FLT_MAX);
        Math::Vector2 max(-FLT_MAX, -FLT_MAX);
        
        // Find bounding box
        for (const auto& vertex : vertices) {
            min.x = std::min(min.x, vertex.x);
            min.y = std::min(min.y, vertex.y);
            max.x = std::max(max.x, vertex.x);
            max.y = std::max(max.y, vertex.y);
        }
        
        Math::Vector2 center = {(min.x + max.x) * 0.5f, (min.y + max.y) * 0.5f};
        Math::Vector2 size = {max.x - min.x, max.y - min.y};
        Math::Vector2 origin = {size.x * 0.5f, size.y * 0.5f};
        
        Graphics::Renderer2D::DrawQuad(center, size, origin, color);
    }
    
    void DebugRenderSystem::NyonDebugRenderer::QueuePolygon(const std::vector<Math::Vector2>& vertices, 
                                                           const Math::Vector3& color)
    {
        // Implementation moved to parent class
    }
    
    void DebugRenderSystem::NyonDebugRenderer::DrawPoint(const Math::Vector2& point, float size, 
                                                        const Math::Vector3& color)
    {
        // Draw point as a small quad
        Math::Vector2 quadSize(size, size);
        Math::Vector2 origin(size * 0.5f, size * 0.5f);
        Graphics::Renderer2D::DrawQuad(point, quadSize, origin, color);
    }
    
    void DebugRenderSystem::NyonDebugRenderer::QueuePoint(const Math::Vector2& point, float size, 
                                                         const Math::Vector3& color)
    {
        // Implementation moved to parent class
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
        m_ComponentStore = &componentStore;
        
        // Get physics world component
        const auto& worldEntities = componentStore.GetEntitiesWithComponent<PhysicsWorldComponent>();
        if (!worldEntities.empty())
        {
            m_PhysicsWorld = &componentStore.GetComponent<PhysicsWorldComponent>(worldEntities[0]);
        }
        
        // No pointer caching - query fresh each Update() call
        // This prevents dangling pointers when new entities are added
        
        // Renderer is now directly connected to Nyon's Renderer2D
        // No initialization needed as it's handled by the concrete implementation
    }
    
    void DebugRenderSystem::Update(float deltaTime)
    {
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
            
        // Begin rendering debug information
        Graphics::Renderer2D::BeginScene();
            
        // Execute all queued debug draw commands
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
        
        // End rendering debug information
        Graphics::Renderer2D::EndScene();
        
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
            
            // Get transform data from TransformComponent if available so debug shapes match the scene
            Math::Vector2 position = {0.0f, 0.0f};
            float angle = 0.0f;
            if (m_ComponentStore->HasComponent<TransformComponent>(entityId))
            {
                const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
                position = transform.position;
                angle = transform.rotation;
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
        // Would iterate through JointComponents and draw joint visualization
        // Placeholder implementation
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
            
            // Use transform position and rotation if available so AABBs line up with rotated shapes
            Math::Vector2 position = {0.0f, 0.0f};
            float angle = 0.0f;
            if (m_ComponentStore->HasComponent<TransformComponent>(entityId))
            {
                const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
                position = transform.position;
                angle = transform.rotation;
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
        // Would draw contact points and normals
        // Placeholder implementation
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
                position = transform.position;
            }
            
            Math::Vector2 com = position + body.centerOfMass;
            
            // Draw center of mass as a small point using queued command
            Math::Vector3 color = {1.0f, 0.0f, 1.0f}; // Magenta
            QueuePoint(com, 3.0f, color);
        }
    }
    
    void DebugRenderSystem::DrawIslands()
    {
        // Would visualize island grouping for performance debugging
        // Placeholder implementation
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
