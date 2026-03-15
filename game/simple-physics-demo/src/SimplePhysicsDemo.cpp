#include "SimplePhysicsDemo.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/systems/DebugRenderSystem.h"
#include <iostream>
#include <iomanip>
using namespace Nyon;
SimplePhysicsDemo::SimplePhysicsDemo()
    : ECSApplication("Nyon – Simple Physics Demo", 1280, 720) { }
void SimplePhysicsDemo::OnECSStart() {
    CreateWorld();
    CreatePlatform();
    CreateFallingBox(); }
void SimplePhysicsDemo::OnECSFixedUpdate(float deltaTime) {
    if (!m_DebugConfigured) {
        if (auto* dbg = GetDebugRenderSystem()) {
            dbg->SetFlags(true, false, false, true, false); }
        m_DebugConfigured = true; }
    m_SimTime += deltaTime;
    if (m_SimTime >= m_NextLogTime) {
        m_NextLogTime += LOG_INTERVAL_S;
        auto& cs = GetComponentStore();
        if (cs.HasComponent<ECS::TransformComponent>(m_BoxEntity)) {
            const auto& t  = cs.GetComponent<ECS::TransformComponent>(m_BoxEntity);
            const auto& rb = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BoxEntity);
            std::cerr << std::fixed << std::setprecision(2)
                      << "[DEMO t=" << m_SimTime << "s]"
                      << "  box pos=(" << t.position.x << ", " << t.position.y << ")"
                      << "  vel=("     << rb.velocity.x << ", " << rb.velocity.y << ")"
                      << "  awake="    << rb.isAwake
                      << "\n"; } }
    if (!m_CollisionDetected) {
        auto& cs            = GetComponentStore();
        const auto& worlds  = cs.GetEntitiesWithComponent<ECS::PhysicsWorldComponent>();
        if (!worlds.empty()) {
            const auto& world = cs.GetComponent<ECS::PhysicsWorldComponent>(worlds[0]);
            for (const auto& manifold : world.contactManifolds) {
                if (!manifold.touching)
                    continue;
                bool involvesPlatform =
                    (manifold.entityIdA == m_PlatformEntity ||
                     manifold.entityIdB == m_PlatformEntity);
                bool involvesBox =
                    (manifold.entityIdA == m_BoxEntity ||
                     manifold.entityIdB == m_BoxEntity);
                if (involvesPlatform && involvesBox) {
                    m_CollisionDetected  = true;
                    m_FirstCollisionTime = m_SimTime;
                    std::cerr << "[DEMO t=" << m_SimTime << "s]"
                              << "  COLLISION DETECTED  "
                              << "  contact points=" << manifold.points.size()
                              << "  normal=("
                              << manifold.normal.x << ", "
                              << manifold.normal.y << ")\n";
                    break; } } } }
    if (m_CollisionDetected) {
        auto& cs = GetComponentStore();
        if (cs.HasComponent<ECS::PhysicsBodyComponent>(m_BoxEntity)) {
            const auto& rb = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BoxEntity);
            if (!rb.isAwake) {
                const auto& t = cs.GetComponent<ECS::TransformComponent>(m_BoxEntity);
                std::cerr << "[DEMO t=" << m_SimTime << "s]"
                          << "  BOX AT REST"
                          << "  final pos=(" << t.position.x << ", " << t.position.y << ")\n"; } } }
    if (m_SimTime >= DEMO_DURATION_S) {
        std::cerr << "[DEMO] Simulation complete after "
                  << m_SimTime << " s.  Closing.\n";
        Close(); } }
void SimplePhysicsDemo::CreateWorld() {
    auto& entities  = GetEntityManager();
    auto& cs        = GetComponentStore();
    ECS::EntityID worldEntity = entities.CreateEntity();
    ECS::PhysicsWorldComponent world;
    world.gravity = { 0.0f, -980.0f };
    world.timeStep              = 1.0f / 60.0f;
    world.velocityIterations    = 8;
    world.positionIterations    = 3;
    world.linearSlop            = 0.5f;
    world.maxLinearCorrection   = 5.0f;    
    world.enableSleep           = true;
    world.enableWarmStarting    = true;    
    world.enableContinuous      = false;   
    cs.AddComponent(worldEntity, std::move(world)); }
void SimplePhysicsDemo::CreatePlatform() {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    m_PlatformEntity = entities.CreateEntity();
    ECS::TransformComponent t;
    t.position         = { 640.0f, 70.0f };
    t.previousPosition = t.position;    
    t.rotation         = 0.0f;
    t.previousRotation = 0.0f;
    ECS::PhysicsBodyComponent body;
    body.isStatic = true;
    body.UpdateMassProperties();
    ECS::ColliderComponent::PolygonShape platformShape({ { -400.0f, -25.0f }, {  400.0f, -25.0f }, {  400.0f,  25.0f }, { -400.0f,  25.0f } });
    ECS::ColliderComponent platformCollider(platformShape);
    platformCollider.material.friction    = 0.6f;
    platformCollider.material.restitution = 0.1f;
    platformCollider.material.density     = 0.0f;   
    ECS::RenderComponent platformRender({ 800.0f, 50.0f }, { 0.35f, 0.35f, 0.35f });
    platformRender.origin = { 400.0f, 25.0f };
    cs.AddComponent(m_PlatformEntity, std::move(t));
    cs.AddComponent(m_PlatformEntity, std::move(body));
    cs.AddComponent(m_PlatformEntity, std::move(platformCollider));
    cs.AddComponent(m_PlatformEntity, std::move(platformRender)); }
void SimplePhysicsDemo::CreateFallingBox() {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    m_BoxEntity = entities.CreateEntity();
    ECS::TransformComponent t;
    t.position         = { 640.0f, 500.0f };
    t.previousPosition = t.position;
    t.rotation         = 0.0f;
    t.previousRotation = 0.0f;
    ECS::ColliderComponent::PolygonShape boxShape({ { -25.0f, -25.0f }, {  25.0f, -25.0f }, {  25.0f,  25.0f }, { -25.0f,  25.0f } });
    ECS::ColliderComponent boxCollider(boxShape);
    boxCollider.material.friction    = 0.4f;
    boxCollider.material.restitution = 0.2f;
    boxCollider.material.density     = 0.0008f;  
    ECS::PhysicsBodyComponent body;
    body.mass = 2.0f; {
        float area    = boxCollider.CalculateArea();            
        float density = (area > 0.0f) ? body.mass / area : 0.0f;
        body.inertia  = boxCollider.CalculateInertiaForUnitDensity() * density; }
    body.UpdateMassProperties();
    body.isAwake    = true;
    body.allowSleep = true;
    ECS::RenderComponent boxRender({ 50.0f, 50.0f }, { 0.25f, 0.45f, 1.0f });
    boxRender.origin = { 25.0f, 25.0f };
    cs.AddComponent(m_BoxEntity, std::move(t));
    cs.AddComponent(m_BoxEntity, std::move(body));
    cs.AddComponent(m_BoxEntity, std::move(boxCollider));
    cs.AddComponent(m_BoxEntity, std::move(boxRender));
    std::cerr << "[DEMO] Box created at y=500. "
              << "Platform top surface at y=" << (70.0f + 25.0f) << ". "
              << "Box will land at box-centre y=" << (70.0f + 25.0f + 25.0f)
              << " (~0.88 s at 980 px/s²).\n"; }