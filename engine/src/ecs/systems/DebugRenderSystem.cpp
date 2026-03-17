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
    
    void DebugRenderSystem::Initialize(EntityManager& entityManager, ComponentStore& componentStore)
    {
        // Store reference to component store for later use
        // Physics world entity may not exist yet at this point (created in OnECSStart),
        // so we do NOT cache m_PhysicsWorld here - it is resolved lazily in Update().
        m_ComponentStore = &componentStore;
    }
    
    void DebugRenderSystem::Update(float deltaTime)
    {
        // Lazily find the physics world entity each update in case it is created
        // after this system is initialized (e.g. in OnECSStart).
        if (m_PhysicsWorldEntity == INVALID_ENTITY && m_ComponentStore)
        {
            const auto& worldEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsWorldComponent>();
            if (!worldEntities.empty())
            {
                m_PhysicsWorldEntity = worldEntities[0];
            }
        }

        // If we don't have a world entity yet, nothing to draw
        if (m_PhysicsWorldEntity == INVALID_ENTITY)
            return;
        
        // Set up debug renderer flags based on our configuration
        Graphics::DebugRenderFlag flags = Graphics::DebugRenderFlag::None;
        if (m_DrawShapes) flags = flags | Graphics::DebugRenderFlag::Shapes;
        if (m_DrawAABBs) flags = flags | Graphics::DebugRenderFlag::AABBs;
        if (m_DrawContacts) flags = flags | Graphics::DebugRenderFlag::Contacts | Graphics::DebugRenderFlag::Manifolds;
        if (m_DrawCOM) flags = flags | Graphics::DebugRenderFlag::CentersOfMass;
        
        m_DebugRenderer.SetActiveFlags(flags);
        
        // Render debug information directly
        if (m_DrawShapes) DrawShapes();
        if (m_DrawJoints) DrawJoints();
        if (m_DrawAABBs) DrawAABBs();
        if (m_DrawContacts) DrawContacts();
        if (m_DrawCOM) DrawCenterOfMass();
        if (m_DrawIslands) DrawIslands();
    }
    
    void DebugRenderSystem::RenderDebugInfo()
    {
        // No-op - all rendering now happens directly in Update() via PhysicsDebugRenderer
        // This method is kept for API compatibility but does nothing
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
                
                // Create a temporary transform for the debug renderer
                ECS::TransformComponent renderTransform;
                renderTransform.position = position;
                renderTransform.rotation = angle;
                
                // Color based on body state
                Math::Vector3 color = {0.0f, 1.0f, 0.0f}; // Green default
                if (body.isStatic) color = {0.5f, 0.5f, 0.5f}; // Gray for static
                if (!body.isAwake) color = {0.2f, 0.2f, 0.8f}; // Blue for sleeping
                if (collider && collider->isSensor) color = {1.0f, 1.0f, 0.0f}; // Yellow for sensors
                
                if (collider)
                {
                    m_DebugRenderer.DrawCollider(*collider, renderTransform, color);
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
        // AABBs are now drawn as part of DrawShapes when the flag is enabled
        // This method is kept for compatibility but does nothing
        // The PhysicsDebugRenderer handles AABB drawing in DrawCollider
    }
    
    void DebugRenderSystem::DrawContacts()
    {
        if (!m_ComponentStore || m_PhysicsWorldEntity == INVALID_ENTITY)
            return;

        if (!m_ComponentStore->HasComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity))
            return;

        const auto& world = m_ComponentStore->GetComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity);

        // Draw contact points and normals from physics world
        for (const auto& manifold : world.contactManifolds)
        {
            if (!manifold.touching) continue;
            
            m_DebugRenderer.DrawManifold(manifold);
        }
    }
    
    void DebugRenderSystem::DrawCenterOfMass()
    {
        // Centers of mass are now drawn as part of DrawShapes when the flag is enabled
        // This method is kept for compatibility but does nothing
        // The PhysicsDebugRenderer handles CoM drawing in DrawCollider
    }
    
    void DebugRenderSystem::DrawIslands()
    {
        // Islands not implemented yet - sleep/island system needs implementation
        // Set m_DrawIslands = false to disable
    }
    
    void DebugRenderSystem::DrawCircleShape(const Math::Vector2& position, 
                                          const ColliderComponent::CircleShape& circle,
                                          const Math::Vector3& color)
    {
        // Create temporary transform
        ECS::TransformComponent transform;
        transform.position = position;
        
        m_DebugRenderer.DrawCircleShape(circle, transform, color);
    }
    
    void DebugRenderSystem::DrawPolygonShape(const Math::Vector2& position, float angle,
                                           const ColliderComponent::PolygonShape& polygon,
                                           const Math::Vector3& color)
    {
        // Create temporary transform
        ECS::TransformComponent transform;
        transform.position = position;
        transform.rotation = angle;
        
        m_DebugRenderer.DrawPolygonShape(polygon, transform, color);
    }
    
    void DebugRenderSystem::DrawCapsuleShape(const Math::Vector2& position, float angle,
                                           const ColliderComponent::CapsuleShape& capsule,
                                           const Math::Vector3& color)
    {
        // Create temporary transform
        ECS::TransformComponent transform;
        transform.position = position;
        transform.rotation = angle;
        
        m_DebugRenderer.DrawCapsuleShape(capsule, transform, color);
    }
}
