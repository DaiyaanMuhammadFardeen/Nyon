#include "nyon/ecs/systems/DebugRenderSystem.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
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
        // Use Renderer2D to draw the line directly (color is already Vector3)
        Graphics::Renderer2D::DrawLine(p1, p2, color);
    }
    
    void DebugRenderSystem::NyonDebugRenderer::DrawSolidCircle(const Math::Vector2& center, float radius, 
                                                              const Math::Vector3& color)
    {
        // Approximate circle with a quad (rectangle) since Renderer2D doesn't have circle drawing
        Math::Vector2 size(radius * 2.0f, radius * 2.0f);
        Math::Vector2 origin(radius, radius); // Center origin
        Graphics::Renderer2D::DrawQuad(center, size, origin, color);
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
        // Store reference to component store
        m_ComponentStore = &componentStore;
        
        // Get physics world component
        const auto& worldEntities = componentStore.GetEntitiesWithComponent<PhysicsWorldComponent>();
        if (!worldEntities.empty())
        {
            m_PhysicsWorld = &componentStore.GetComponent<PhysicsWorldComponent>(worldEntities[0]);
        }
        
        // Get all entities with physics components for debugging
        const auto& bodyEntities = componentStore.GetEntitiesWithComponent<PhysicsBodyComponent>();
        for (auto entityId : bodyEntities)
        {
            PhysicsBodyComponent* body = &componentStore.GetComponent<PhysicsBodyComponent>(entityId);
            ColliderComponent* collider = &componentStore.GetComponent<ColliderComponent>(entityId);
            
            if (body)
            {
                m_Entities.emplace_back(entityId, body, collider);
            }
        }
        
        // Renderer is now directly connected to Nyon's Renderer2D
        // No initialization needed as it's handled by the concrete implementation
    }
    
    void DebugRenderSystem::Update(float deltaTime)
    {
        if (!m_PhysicsWorld)
            return;
            
        // Begin rendering debug information
        Graphics::Renderer2D::BeginScene();
            
        // Draw based on enabled flags
        if (m_DrawShapes) DrawShapes();
        if (m_DrawJoints) DrawJoints();
        if (m_DrawAABBs) DrawAABBs();
        if (m_DrawContacts) DrawContacts();
        if (m_DrawCOM) DrawCenterOfMass();
        if (m_DrawIslands) DrawIslands();
        
        // End rendering debug information
        Graphics::Renderer2D::EndScene();
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
        for (const auto& [entityId, body, collider] : m_Entities)
        {
            // Get transform data (would come from TransformComponent)
            Math::Vector2 position = {0.0f, 0.0f};
            float angle = 0.0f;
            
            // Color based on body state
            Math::Vector3 color = {0.0f, 1.0f, 0.0f}; // Green default
            if (body->isStatic) color = {0.5f, 0.5f, 0.5f}; // Gray for static
            if (!body->isAwake) color = {0.2f, 0.2f, 0.8f}; // Blue for sleeping
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
        for (const auto& [entityId, body, collider] : m_Entities)
        {
            if (!collider) continue;
            
            Math::Vector2 position = {0.0f, 0.0f};
            Math::Vector2 min, max;
            collider->CalculateAABB(position, min, max);
            
            // Draw AABB outline
            Math::Vector3 color = {1.0f, 0.0f, 0.0f}; // Red for AABBs
            m_Renderer.DrawLine({min.x, min.y}, {max.x, min.y}, color);
            m_Renderer.DrawLine({max.x, min.y}, {max.x, max.y}, color);
            m_Renderer.DrawLine({max.x, max.y}, {min.x, max.y}, color);
            m_Renderer.DrawLine({min.x, max.y}, {min.x, min.y}, color);
        }
    }
    
    void DebugRenderSystem::DrawContacts()
    {
        // Would draw contact points and normals
        // Placeholder implementation
    }
    
    void DebugRenderSystem::DrawCenterOfMass()
    {
        for (const auto& [entityId, body, collider] : m_Entities)
        {
            Math::Vector2 position = {0.0f, 0.0f}; // From TransformComponent
            Math::Vector2 com = position + body->centerOfMass;
            
            // Draw center of mass as a small point
            Math::Vector3 color = {1.0f, 0.0f, 1.0f}; // Magenta
            m_Renderer.DrawPoint(com, 3.0f, color);
        }
    }
    
    void DebugRenderSystem::DrawIslands()
    {
        // Would visualize island grouping for performance debugging
        // Placeholder implementation
    }
    
    void DebugRenderSystem::DrawCircleShape(const Math::Vector2& position, 
                                          const ColliderComponent::CircleShape& circle,
                                          const Math::Vector3& color)
    {
        Math::Vector2 worldCenter = position + circle.center;
        m_Renderer.DrawSolidCircle(worldCenter, circle.radius, color);
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
        
        m_Renderer.DrawSolidPolygon(worldVertices, color);
    }
    
    void DebugRenderSystem::DrawCapsuleShape(const Math::Vector2& position, float angle,
                                           const ColliderComponent::CapsuleShape& capsule,
                                           const Math::Vector3& color)
    {
        // Draw capsule as two circles connected by rectangles
        // Simplified implementation - would normally draw proper capsule shape
        
        Math::Vector2 center1 = position + capsule.center1;
        Math::Vector2 center2 = position + capsule.center2;
        
        m_Renderer.DrawSolidCircle(center1, capsule.radius, color);
        m_Renderer.DrawSolidCircle(center2, capsule.radius, color);
                
        // Draw connecting line
        m_Renderer.DrawLine(center1, center2, color);
    }
}
