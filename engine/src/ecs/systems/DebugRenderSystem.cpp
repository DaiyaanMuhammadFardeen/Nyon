#include "nyon/ecs/systems/DebugRenderSystem.h"
#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/components/TransformComponent.h"
#include <cfloat>
#ifdef _DEBUG
#define NYON_DEBUG_LOG(x) std::cerr << x << std::endl
#else
#define NYON_DEBUG_LOG(x)
#endif
namespace Nyon::ECS {
    DebugRenderSystem::DebugRenderSystem() {
        NYON_DEBUG_LOG("[DEBUG] DebugRenderSystem constructor called"); }
    void DebugRenderSystem::Initialize(EntityManager& entityManager, ComponentStore& componentStore) {
        m_ComponentStore = &componentStore; }
    void DebugRenderSystem::Update(float deltaTime) {
        if (m_PhysicsWorldEntity == INVALID_ENTITY && m_ComponentStore) {
            const auto& worldEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsWorldComponent>();
            if (!worldEntities.empty()) {
                m_PhysicsWorldEntity = worldEntities[0]; } }
        if (m_PhysicsWorldEntity == INVALID_ENTITY)
            return;
        Graphics::DebugRenderFlag flags = Graphics::DebugRenderFlag::None;
        if (m_DrawShapes) flags = flags | Graphics::DebugRenderFlag::Shapes;
        if (m_DrawAABBs) flags = flags | Graphics::DebugRenderFlag::AABBs;
        if (m_DrawContacts) flags = flags | Graphics::DebugRenderFlag::Contacts | Graphics::DebugRenderFlag::Manifolds;
        if (m_DrawCOM) flags = flags | Graphics::DebugRenderFlag::CentersOfMass;
        m_DebugRenderer.SetActiveFlags(flags);
        if (m_DrawShapes) DrawShapes();
        if (m_DrawJoints) DrawJoints();
        if (m_DrawAABBs) DrawAABBs();
        if (m_DrawContacts) DrawContacts();
        if (m_DrawCOM) DrawCenterOfMass();
        if (m_DrawIslands) DrawIslands(); }
    void DebugRenderSystem::RenderDebugInfo() { }
    void DebugRenderSystem::SetFlags(bool drawShapes, bool drawJoints, bool drawAABBs, 
                                   bool drawContacts, bool drawCOM) {
        m_DrawShapes = drawShapes;
        m_DrawJoints = drawJoints;
        m_DrawAABBs = drawAABBs;
        m_DrawContacts = drawContacts;
        m_DrawCOM = drawCOM; }
    void DebugRenderSystem::DrawShapes() {
        const auto& bodyEntities = m_ComponentStore->GetEntitiesWithComponent<PhysicsBodyComponent>();
        for (auto entityId : bodyEntities) {
            const auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityId);
            const ColliderComponent* collider = nullptr;
            if (m_ComponentStore->HasComponent<ColliderComponent>(entityId)) {
                collider = &m_ComponentStore->GetComponent<ColliderComponent>(entityId); }
            Math::Vector2 position = {0.0f, 0.0f};
            float angle = 0.0f;
            if (m_ComponentStore->HasComponent<TransformComponent>(entityId)) {
                const auto& transform = m_ComponentStore->GetComponent<TransformComponent>(entityId);
                position = transform.GetInterpolatedPosition(m_Alpha);
                angle = transform.GetInterpolatedRotation(m_Alpha);
                ECS::TransformComponent renderTransform;
                renderTransform.position = position;
                renderTransform.rotation = angle;
                Math::Vector3 color = {0.0f, 1.0f, 0.0f};  
                if (body.isStatic) color = {0.5f, 0.5f, 0.5f};  
                if (!body.isAwake) color = {0.2f, 0.2f, 0.8f};  
                if (collider && collider->isSensor) color = {1.0f, 1.0f, 0.0f};  
                if (collider) {
                    m_DebugRenderer.DrawCollider(*collider, renderTransform, color); } } } }
    void DebugRenderSystem::DrawJoints() { }
    void DebugRenderSystem::DrawAABBs() { }
    void DebugRenderSystem::DrawContacts() {
        if (!m_ComponentStore || m_PhysicsWorldEntity == INVALID_ENTITY)
            return;
        if (!m_ComponentStore->HasComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity))
            return;
        const auto& world = m_ComponentStore->GetComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity);
        for (const auto& manifold : world.contactManifolds) {
            if (!manifold.touching) continue;
            m_DebugRenderer.DrawManifold(manifold); } }
    void DebugRenderSystem::DrawCenterOfMass() { }
    void DebugRenderSystem::DrawIslands() { }
    void DebugRenderSystem::DrawCircleShape(const Math::Vector2& position, 
                                          const ColliderComponent::CircleShape& circle,
                                          const Math::Vector3& color) {
        ECS::TransformComponent transform;
        transform.position = position;
        m_DebugRenderer.DrawCircleShape(circle, transform, color); }
    void DebugRenderSystem::DrawPolygonShape(const Math::Vector2& position, float angle,
                                           const ColliderComponent::PolygonShape& polygon,
                                           const Math::Vector3& color) {
        ECS::TransformComponent transform;
        transform.position = position;
        transform.rotation = angle;
        m_DebugRenderer.DrawPolygonShape(polygon, transform, color); }
    void DebugRenderSystem::DrawCapsuleShape(const Math::Vector2& position, float angle,
                                           const ColliderComponent::CapsuleShape& capsule,
                                           const Math::Vector3& color) {
        ECS::TransformComponent transform;
        transform.position = position;
        transform.rotation = angle;
        m_DebugRenderer.DrawCapsuleShape(capsule, transform, color); } }
