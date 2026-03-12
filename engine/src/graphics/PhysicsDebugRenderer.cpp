#include "nyon/graphics/PhysicsDebugRenderer.h"
#include <cmath>

namespace Nyon::Graphics
{
    PhysicsDebugRenderer::PhysicsDebugRenderer() 
        : m_ActiveFlags(DebugRenderFlag::None)
    {
    }
    
    PhysicsDebugRenderer::~PhysicsDebugRenderer()
    {
    }
    
    void PhysicsDebugRenderer::SetActiveFlags(DebugRenderFlag flags)
    {
        m_ActiveFlags = flags;
    }
    
    void PhysicsDebugRenderer::DrawCollider(const ECS::ColliderComponent& collider,
                                           const ECS::TransformComponent& transform,
                                           const Math::Vector3& color)
    {
        if (!HasFlag(m_ActiveFlags, DebugRenderFlag::Shapes))
            return;
        
        switch (collider.GetType())
        {
            case ECS::ColliderComponent::ShapeType::Circle:
                DrawCircleShape(collider.GetCircle(), transform, color);
                break;
                
            case ECS::ColliderComponent::ShapeType::Polygon:
                DrawPolygonShape(collider.GetPolygon(), transform, color);
                break;
                
            case ECS::ColliderComponent::ShapeType::Capsule:
                DrawCapsuleShape(collider.GetCapsule(), transform, color);
                break;
                
            case ECS::ColliderComponent::ShapeType::Segment:
                DrawSegmentShape(collider.GetSegment(), transform, color);
                break;
                
            // case ECS::ColliderComponent::ShapeType::Chain:
            //     DrawChainShape(collider.GetChain(), transform, color);
            //     break;
                
            default:
                break;
        }
        
        // Draw AABB if enabled
        if (HasFlag(m_ActiveFlags, DebugRenderFlag::AABBs))
        {
            Math::Vector2 aabbMin, aabbMax;
            collider.CalculateAABB(transform.position, transform.rotation, aabbMin, aabbMax);
            DrawAABB(aabbMin, aabbMax, Math::Vector3{1.0f, 0.5f, 0.0f});
        }
        
        // Draw center of mass if enabled
        if (HasFlag(m_ActiveFlags, DebugRenderFlag::CentersOfMass))
        {
            DrawCenterOfMass(transform.position, Math::Vector3{1.0f, 1.0f, 0.0f});
        }
    }
    
    void PhysicsDebugRenderer::DrawCircleShape(const ECS::ColliderComponent::CircleShape& circle,
                                              const ECS::TransformComponent& transform,
                                              const Math::Vector3& color)
    {
        Math::Vector2 worldCenter = transform.position + circle.center;
        
        // Apply rotation to the circle (though circles are rotationally symmetric)
        Renderer2D::DrawSolidCircle(worldCenter, circle.radius, color, 32);
        Renderer2D::DrawCircle(worldCenter, circle.radius, color * 0.5f, 32);
        
        // Draw orientation marker to show rotation
        if (std::abs(transform.rotation) > 0.001f)
        {
            Math::Vector2 orientMarker = worldCenter + Math::Vector2{
                std::cos(transform.rotation),
                std::sin(transform.rotation)
            } * circle.radius;
            
            Renderer2D::DrawLine(worldCenter, orientMarker, color * 1.5f, 2.0f);
        }
    }
    
    void PhysicsDebugRenderer::DrawPolygonShape(const ECS::ColliderComponent::PolygonShape& polygon,
                                               const ECS::TransformComponent& transform,
                                               const Math::Vector3& color)
    {
        if (polygon.vertices.empty()) return;
        
        // Transform vertices to world space
        std::vector<Math::Vector2> worldVertices;
        worldVertices.reserve(polygon.vertices.size());
        
        float cosRot = std::cos(transform.rotation);
        float sinRot = std::sin(transform.rotation);
        
        for (const auto& vertex : polygon.vertices)
        {
            Math::Vector2 rotated = {
                vertex.x * cosRot - vertex.y * sinRot,
                vertex.x * sinRot + vertex.y * cosRot
            };
            worldVertices.push_back(transform.position + rotated);
        }
        
        // Draw filled polygon
        Renderer2D::DrawSolidPolygon(worldVertices, color * 0.7f);
        
        // Draw outline
        Renderer2D::DrawPolygon(worldVertices, color);
        
        // Draw normals if collision normals flag is set
        if (HasFlag(m_ActiveFlags, DebugRenderFlag::CollisionNormals))
        {
            for (size_t i = 0; i < polygon.normals.size() && i < worldVertices.size(); ++i)
            {
                size_t next = (i + 1) % worldVertices.size();
                Math::Vector2 edgeCenter = (worldVertices[i] + worldVertices[next]) * 0.5f;
                
                // Rotate normal by transform rotation
                Math::Vector2 worldNormal = {
                    polygon.normals[i].x * cosRot - polygon.normals[i].y * sinRot,
                    polygon.normals[i].x * sinRot + polygon.normals[i].y * cosRot
                };
                
                Math::Vector2 normalEnd = edgeCenter + worldNormal * 20.0f;
                Renderer2D::DrawLine(edgeCenter, normalEnd, Math::Vector3{0.0f, 1.0f, 0.0f}, 2.0f);
            }
        }
        
        // Draw centroid
        if (HasFlag(m_ActiveFlags, DebugRenderFlag::CentersOfMass))
        {
            Math::Vector2 worldCentroid = transform.position + Math::Vector2{
                polygon.centroid.x * cosRot - polygon.centroid.y * sinRot,
                polygon.centroid.x * sinRot + polygon.centroid.y * cosRot
            };
            DrawCenterOfMass(worldCentroid, Math::Vector3{0.0f, 1.0f, 1.0f});
        }
    }
    
    void PhysicsDebugRenderer::DrawCapsuleShape(const ECS::ColliderComponent::CapsuleShape& capsule,
                                               const ECS::TransformComponent& transform,
                                               const Math::Vector3& color)
    {
        // Transform capsule centers to world space
        Math::Vector2 center1 = transform.position + capsule.center1;
        Math::Vector2 center2 = transform.position + capsule.center2;
        
        // Apply rotation
        float cosRot = std::cos(transform.rotation);
        float sinRot = std::sin(transform.rotation);
        
        Math::Vector2 rotatedCenter1 = {
            capsule.center1.x * cosRot - capsule.center1.y * sinRot,
            capsule.center1.x * sinRot + capsule.center1.y * cosRot
        };
        
        Math::Vector2 rotatedCenter2 = {
            capsule.center2.x * cosRot - capsule.center2.y * sinRot,
            capsule.center2.x * sinRot + capsule.center2.y * cosRot
        };
        
        center1 = transform.position + rotatedCenter1;
        center2 = transform.position + rotatedCenter2;
        
        // Draw solid capsule
        Renderer2D::DrawSolidCapsule(center1, center2, capsule.radius, color, 16);
        
        // Draw outline
        Renderer2D::DrawCapsule(center1, center2, capsule.radius, color * 0.5f, 16);
    }
    
    void PhysicsDebugRenderer::DrawSegmentShape(const ECS::ColliderComponent::SegmentShape& segment,
                                               const ECS::TransformComponent& transform,
                                               const Math::Vector3& color)
    {
        // Transform segment endpoints to world space
        Math::Vector2 rotatedP1 = {
            segment.point1.x * std::cos(transform.rotation) - segment.point1.y * std::sin(transform.rotation),
            segment.point1.x * std::sin(transform.rotation) + segment.point1.y * std::cos(transform.rotation)
        };
        
        Math::Vector2 rotatedP2 = {
            segment.point2.x * std::cos(transform.rotation) - segment.point2.y * std::sin(transform.rotation),
            segment.point2.x * std::sin(transform.rotation) + segment.point2.y * std::cos(transform.rotation)
        };
        
        Math::Vector2 worldP1 = transform.position + rotatedP1;
        Math::Vector2 worldP2 = transform.position + rotatedP2;
        
        if (segment.radius > 0.0f)
        {
            // Draw as thick segment with rounded ends
            Renderer2D::DrawSegment(worldP1, worldP2, segment.radius * 2.0f, color);
        }
        else
        {
            // Draw as thin line
            Renderer2D::DrawLine(worldP1, worldP2, color, 2.0f);
        }
    }
    
    void PhysicsDebugRenderer::DrawChainShape(const ECS::ColliderComponent::ChainShape& chain,
                                             const ECS::TransformComponent& transform,
                                             const Math::Vector3& color)
    {
        if (chain.vertices.empty()) return;
        
        // Transform vertices to world space
        std::vector<Math::Vector2> worldVertices;
        worldVertices.reserve(chain.vertices.size());
        
        float cosRot = std::cos(transform.rotation);
        float sinRot = std::sin(transform.rotation);
        
        for (const auto& vertex : chain.vertices)
        {
            Math::Vector2 rotated = {
                vertex.x * cosRot - vertex.y * sinRot,
                vertex.x * sinRot + vertex.y * cosRot
            };
            worldVertices.push_back(transform.position + rotated);
        }
        
        // Draw chain links
        if (HasFlag(m_ActiveFlags, DebugRenderFlag::ChainLinks))
        {
            for (size_t i = 0; i < worldVertices.size(); ++i)
            {
                Renderer2D::DrawSolidCircle(worldVertices[i], 3.0f, Math::Vector3{1.0f, 0.0f, 1.0f}, 8);
            }
        }
        
        // Draw the chain
        Renderer2D::DrawChain(worldVertices, color, chain.radius > 0.0f ? chain.radius * 2.0f : 1.0f, chain.isLoop);
    }
    
    void PhysicsDebugRenderer::DrawManifold(const ECS::ContactManifold& manifold)
    {
        if (!HasFlag(m_ActiveFlags, DebugRenderFlag::Manifolds) || !HasFlag(m_ActiveFlags, DebugRenderFlag::Contacts))
            return;
        
        for (const auto& contact : manifold.points)
        {
            DrawContactPoint(contact.position, contact.normal, contact.separation, manifold.touching);
        }
    }
    
    void PhysicsDebugRenderer::DrawContactPoint(const Math::Vector2& position,
                                               const Math::Vector2& normal,
                                               float separation,
                                               bool isTouching)
    {
        if (!HasFlag(m_ActiveFlags, DebugRenderFlag::Contacts))
            return;
        
        // Color based on penetration depth
        Math::Vector3 color;
        float penetration = -separation;
        
        if (penetration < 0.5f)
            color = Math::Vector3{0.0f, 1.0f, 0.0f};  // Light contact
        else if (penetration < 2.0f)
            color = Math::Vector3{1.0f, 1.0f, 0.0f};  // Moderate penetration
        else
            color = Math::Vector3{1.0f, 0.0f, 0.0f};  // Deep penetration
        
        // Draw contact point
        Renderer2D::DrawSolidCircle(position, 4.0f, color, 16);
        Renderer2D::DrawCircle(position, 4.0f, color * 0.5f, 16);
        
        // Draw normal vector
        Math::Vector2 normalEnd = position + normal * 30.0f;
        Renderer2D::DrawLine(position, normalEnd, Math::Vector3{1.0f, 1.0f, 1.0f}, 2.0f);
        
        // Draw arrowhead
        float arrowSize = 6.0f;
        Math::Vector2 arrowBase = normalEnd - normal * arrowSize;
        Math::Vector2 perp = {-normal.y, normal.x};
        
        Math::Vector2 arrowLeft = arrowBase + perp * (arrowSize * 0.5f);
        Math::Vector2 arrowRight = arrowBase - perp * (arrowSize * 0.5f);
        
        Renderer2D::DrawLine(normalEnd, arrowLeft, Math::Vector3{1.0f, 1.0f, 1.0f}, 2.0f);
        Renderer2D::DrawLine(normalEnd, arrowRight, Math::Vector3{1.0f, 1.0f, 1.0f}, 2.0f);
    }
    
    void PhysicsDebugRenderer::DrawAABB(const Math::Vector2& min, const Math::Vector2& max,
                                       const Math::Vector3& color)
    {
        Renderer2D::DrawAABB(min, max, color);
    }
    
    void PhysicsDebugRenderer::DrawTransform(const Math::Vector2& position, float rotation,
                                            float scale)
    {
        Renderer2D::DrawTransform(position, rotation, scale);
    }
    
    void PhysicsDebugRenderer::DrawCenterOfMass(const Math::Vector2& position,
                                               const Math::Vector3& color)
    {
        const float size = 6.0f;
        
        // Draw cross
        Renderer2D::DrawLine(
            {position.x - size, position.y},
            {position.x + size, position.y},
            color, 2.0f
        );
        
        Renderer2D::DrawLine(
            {position.x, position.y - size},
            {position.x, position.y + size},
            color, 2.0f
        );
    }
    
    void PhysicsDebugRenderer::DrawVelocityVector(const Math::Vector2& position,
                                                 const Math::Vector2& velocity,
                                                 const Math::Vector3& color,
                                                 float scale)
    {
        if (velocity.LengthSquared() < 0.0001f) return;
        
        Math::Vector2 endPos = position + velocity * scale;
        Renderer2D::DrawLine(position, endPos, color, 2.0f);
        
        // Draw arrowhead
        float arrowSize = 8.0f;
        Math::Vector2 dir = velocity.Normalize();
        Math::Vector2 arrowBase = endPos - dir * arrowSize;
        
        Math::Vector2 perp = {-dir.y, dir.x};
        Math::Vector2 arrowLeft = arrowBase + perp * (arrowSize * 0.5f);
        Math::Vector2 arrowRight = arrowBase - perp * (arrowSize * 0.5f);
        
        Renderer2D::DrawLine(endPos, arrowLeft, color, 2.0f);
        Renderer2D::DrawLine(endPos, arrowRight, color, 2.0f);
    }
    
    Math::Vector3 PhysicsDebugRenderer::ColorFromHSV(float h, float s, float v)
    {
        float r, g, b;
        
        int i = static_cast<int>(std::floor(h * 6.0f));
        float f = h * 6.0f - static_cast<float>(i);
        float p = v * (1.0f - s);
        float q = v * (1.0f - f * s);
        float t = v * (1.0f - (1.0f - f) * s);
        
        switch (i % 6)
        {
            case 0: r = v; g = t; b = p; break;
            case 1: r = q; g = v; b = p; break;
            case 2: r = p; g = v; b = t; break;
            case 3: r = p; g = q; b = v; break;
            case 4: r = t; g = p; b = v; break;
            case 5: r = v; g = p; b = q; break;
            default: r = g = b = v; break;
        }
        
        return Math::Vector3{r, g, b};
    }
    
    Math::Vector3 PhysicsDebugRenderer::LerpColor(const Math::Vector3& a,
                                                 const Math::Vector3& b,
                                                 float t)
    {
        t = std::max(0.0f, std::min(1.0f, t));
        return Math::Vector3{
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t,
            a.z + (b.z - a.z) * t
        };
    }
}
