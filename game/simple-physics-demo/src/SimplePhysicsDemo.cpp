#include "SimplePhysicsDemo.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/components/BehaviorComponent.h"
#include "nyon/utils/InputManager.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
using namespace Nyon;
SimplePhysicsDemo::SimplePhysicsDemo()
    : ECSApplication("Nyon – Simple Physics Demo", 1280, 720) { }
void SimplePhysicsDemo::OnECSStart() {
    CreateWorld();
    CreatePlatform();
    CreateSlopedPlatform();
    CreatePlayerQuad(); }
void SimplePhysicsDemo::OnECSFixedUpdate(float deltaTime) {
    m_SimTime += deltaTime;
    if (m_SimTime >= m_NextLogTime) {
        m_NextLogTime += LOG_INTERVAL_S;
        auto& cs = GetComponentStore();
        if (cs.HasComponent<ECS::TransformComponent>(m_PlayerEntity)) {
            const auto& t  = cs.GetComponent<ECS::TransformComponent>(m_PlayerEntity);
            const auto& rb = cs.GetComponent<ECS::PhysicsBodyComponent>(m_PlayerEntity);
            std::cerr << std::fixed << std::setprecision(2)
                      << "[DEMO t=" << m_SimTime << "s]"
                      << "  player pos=(" << t.position.x << ", " << t.position.y << ")"
                      << "  vel=("     << rb.velocity.x << ", " << rb.velocity.y << ")"
                      << "  awake="    << rb.isAwake
                      << "\n"; } }
    HandlePlayerInput(deltaTime);
    const float spawnInterval = 1.0f;
    while (m_SimTime >= m_NextAutoSpawnTime) {
        int width, height;
        glfwGetWindowSize(GetWindow(), &width, &height);
        std::uniform_real_distribution<float> xDist(100.0f, float(width - 100));
        float spawnX = xDist(m_Rng);  
        float spawnY = float(height) * 0.8f;
        std::cerr << "[DEMO] Auto-spawning circle at (" << spawnX << ", " << spawnY << ")\n";
        CreateSpawnedCircle(spawnX, spawnY);
        m_NextAutoSpawnTime += spawnInterval; }
    if (Nyon::Utils::InputManager::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT)) {
        std::cerr << "[DEMO] Mouse left button pressed!" << std::endl;
        SpawnQuadAtMousePosition(); }
    DespawnOutOfBoundsObjects(); }
void SimplePhysicsDemo::CreateWorld() {
    auto& entities  = GetEntityManager();
    auto& cs        = GetComponentStore();
    ECS::EntityID worldEntity = entities.CreateEntity();
    ECS::PhysicsWorldComponent world;
    world.gravity = { 0.0f, -980.0f };
    world.timeStep              = 1.0f / 60.0f;
    world.velocityIterations    = 8;
    world.positionIterations    = 3;
    world.linearSlop            = 0.05f;
    world.maxLinearCorrection   = 2.0f;    
    world.enableSleep           = true;
    world.enableWarmStarting    = true;    
    world.enableContinuous      = false;   
    cs.AddComponent(worldEntity, std::move(world)); }
void SimplePhysicsDemo::CreatePlatform() {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    m_PlatformEntity = entities.CreateEntity();
    ECS::TransformComponent t;
    t.position         = { 640.0f, 100.0f };
    t.previousPosition = t.position;    
    t.rotation         = 0.0f;
    t.previousRotation = 0.0f;
    ECS::PhysicsBodyComponent body;
    body.isStatic = true;
    body.UpdateMassProperties();
    ECS::ColliderComponent::PolygonShape platformShape({ { -400.0f, -25.0f }, {  400.0f, -25.0f }, {  400.0f,  25.0f }, { -400.0f,  25.0f } });
    ECS::ColliderComponent platformCollider(platformShape);
    platformCollider.material.friction    = 0.6f;
    platformCollider.material.restitution = 0.8f;
    platformCollider.material.density     = 0.0f;   
    ECS::RenderComponent platformRender({ 800.0f, 50.0f }, { 0.35f, 0.35f, 0.35f });
    platformRender.origin = { 400.0f, 25.0f };
    cs.AddComponent(m_PlatformEntity, std::move(t));
    cs.AddComponent(m_PlatformEntity, std::move(body));
    cs.AddComponent(m_PlatformEntity, std::move(platformCollider));
    cs.AddComponent(m_PlatformEntity, std::move(platformRender)); }
void SimplePhysicsDemo::CreateSlopedPlatform() {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    ECS::EntityID slope1Entity = entities.CreateEntity();
    ECS::TransformComponent t1;
    t1.position         = { 150.0f, 80.0f };   
    t1.previousPosition = t1.position;
    t1.rotation         = -0.436332f;
    t1.previousRotation = t1.rotation;
    ECS::PhysicsBodyComponent body1;
    body1.isStatic = true;
    body1.UpdateMassProperties();
    ECS::ColliderComponent::PolygonShape shape1({ { -250.0f, -25.0f }, {  250.0f, -25.0f }, {  250.0f,  25.0f }, { -250.0f,  25.0f } });
    ECS::ColliderComponent collider1(shape1);
    collider1.material.friction    = 0.5f;
    collider1.material.restitution = 0.8f;
    collider1.material.density     = 0.0f;
    ECS::RenderComponent render1({ 500.0f, 50.0f }, { 0.4f, 0.4f, 0.8f });
    render1.origin = { 250.0f, 25.0f };
    cs.AddComponent(slope1Entity, std::move(t1));
    cs.AddComponent(slope1Entity, std::move(body1));
    cs.AddComponent(slope1Entity, std::move(collider1));
    cs.AddComponent(slope1Entity, std::move(render1));
    ECS::EntityID slope2Entity = entities.CreateEntity();
    ECS::TransformComponent t2;
    t2.position         = { 1130.0f, 80.0f };   
    t2.previousPosition = t2.position;
    t2.rotation         = 0.436332f;
    t2.previousRotation = t2.rotation;
    ECS::PhysicsBodyComponent body2;
    body2.isStatic = true;
    body2.UpdateMassProperties();
    ECS::ColliderComponent::PolygonShape shape2({ { -250.0f, -25.0f }, {  250.0f, -25.0f }, {  250.0f,  25.0f }, { -250.0f,  25.0f } });
    ECS::ColliderComponent collider2(shape2);
    collider2.material.friction    = 0.7f;   
    collider2.material.restitution = 0.8f;
    collider2.material.density     = 0.0f;
    ECS::RenderComponent render2({ 500.0f, 50.0f }, { 0.8f, 0.4f, 0.4f });
    render2.origin = { 250.0f, 25.0f };
    cs.AddComponent(slope2Entity, std::move(t2));
    cs.AddComponent(slope2Entity, std::move(body2));
    cs.AddComponent(slope2Entity, std::move(collider2));
    cs.AddComponent(slope2Entity, std::move(render2)); }
void SimplePhysicsDemo::CreatePlayerQuad() {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    m_PlayerEntity = entities.CreateEntity();
    ECS::TransformComponent t;
    t.position         = { 640.0f, 360.0f };
    t.previousPosition = t.position;
    t.rotation         = 0.0f;
    t.previousRotation = 0.0f;
    ECS::ColliderComponent::CircleShape playerShape;
    playerShape.center = { 0.0f, 0.0f };
    playerShape.radius = 25.0f;
    ECS::ColliderComponent playerCollider(playerShape);
    playerCollider.material.friction    = 0.4f;
    playerCollider.material.restitution = 0.8f;   
    playerCollider.material.density     = 0.08f;
    ECS::PhysicsBodyComponent body; {
        float area    = playerCollider.CalculateArea();            
        float density = (area > 0.0f) ? body.mass / area : 0.0f;
        body.SetMass(5.0f);
        body.SetInertia(playerCollider.CalculateInertiaPerUnitMass() * body.mass); }
    body.angularDamping = 0.1f;    
    body.drag           = 0.01f;   
    body.UpdateMassProperties();
    body.isAwake    = true;
    body.allowSleep = false;   
    body.motionLocks.lockRotation = false;
    ECS::RenderComponent playerRender({ 50.0f, 50.0f }, { 1.0f, 0.5f, 0.0f });   
    playerRender.origin      = { 25.0f, 25.0f };
    playerRender.shapeType   = ECS::RenderComponent::ShapeType::Circle;
    ECS::BehaviorComponent behavior;
    cs.AddComponent(m_PlayerEntity, std::move(t));
    cs.AddComponent(m_PlayerEntity, std::move(body));
    cs.AddComponent(m_PlayerEntity, std::move(playerCollider));
    cs.AddComponent(m_PlayerEntity, std::move(playerRender));
    cs.AddComponent(m_PlayerEntity, std::move(behavior));
    std::cerr << "[DEMO] Player circle created at (640, 360). "
              << "Use WASD to move and jump.\n"; }
void SimplePhysicsDemo::CreateSpawnedQuad(float x, float y) {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    ECS::EntityID spawnedEntity = entities.CreateEntity();
    m_SpawnedQuads.push_back(spawnedEntity);
    ECS::TransformComponent t;
    t.position         = { x, y };
    t.previousPosition = t.position;
    t.rotation         = 0.0f;
    t.previousRotation = 0.0f;
    ECS::ColliderComponent::PolygonShape spawnShape({ { -25.0f, -25.0f }, {  25.0f, -25.0f }, {  25.0f,  25.0f }, { -25.0f,  25.0f } });
    ECS::ColliderComponent spawnCollider(spawnShape);
    spawnCollider.material.friction    = 0.4f;
    spawnCollider.material.restitution = 0.8f;
    spawnCollider.material.density     = 0.0008f;
    ECS::PhysicsBodyComponent body; {
        float area    = spawnCollider.CalculateArea();
        body.SetMass(2.0f);
        body.SetInertia(spawnCollider.CalculateInertiaPerUnitMass() * body.mass); }
    body.angularDamping = 0.05f;   
    body.drag           = 0.01f;   
    body.UpdateMassProperties();
    body.isAwake    = true;
    body.allowSleep = true;
    body.motionLocks.lockRotation = false;
    ECS::RenderComponent spawnRender({ 50.0f, 50.0f }, { 0.5f, 1.0f, 0.5f });   
    spawnRender.origin = { 25.0f, 25.0f };
    cs.AddComponent(spawnedEntity, std::move(t));
    cs.AddComponent(spawnedEntity, std::move(body));
    cs.AddComponent(spawnedEntity, std::move(spawnCollider));
    cs.AddComponent(spawnedEntity, std::move(spawnRender)); }
void SimplePhysicsDemo::HandlePlayerInput(float deltaTime) {
    auto& cs = GetComponentStore();
    if (!cs.HasComponent<ECS::PhysicsBodyComponent>(m_PlayerEntity))
        return;
    auto& body = cs.GetComponent<ECS::PhysicsBodyComponent>(m_PlayerEntity);
    float horizontal = 0.0f;
    if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_A)) {
        horizontal -= 1.0f; }
    if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_D)) {
        horizontal += 1.0f; }
    if (horizontal != 0.0f) {
        body.velocity.x = horizontal * PLAYER_MOVE_SPEED; }
    else {
        body.velocity.x *= 0.9f; }
    if (Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_W) && IsPlayerGrounded()) {
        body.velocity.y += PLAYER_JUMP_FORCE / body.mass;
        body.SetAwake(true); } }
void SimplePhysicsDemo::CreateSpawnedCircle(float x, float y) {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    ECS::EntityID spawnedEntity = entities.CreateEntity();
    m_SpawnedQuads.push_back(spawnedEntity);
    std::uniform_real_distribution<float> radiusDist(10.0f, 20.0f);
    float radius = radiusDist(m_Rng);
    constexpr float platformTop = 70.0f + 25.0f;  
    float spawnY = std::max(y, platformTop + radius + 1.0f);
    ECS::TransformComponent t;
    t.position         = { x, spawnY };
    t.previousPosition = t.position;
    t.rotation         = 0.0f;
    t.previousRotation = 0.0f;
    ECS::ColliderComponent::CircleShape circleShape;
    circleShape.center = { 0.0f, 0.0f };
    circleShape.radius = radius;
    ECS::ColliderComponent circleCollider(circleShape);
    circleCollider.material.friction    = 0.4f;
    circleCollider.material.restitution = 0.8f;
    circleCollider.material.density     = 0.0008f;
    ECS::PhysicsBodyComponent body; {
        float area = circleCollider.CalculateArea();
        body.SetMass(1.0f);
        body.SetInertia(circleCollider.CalculateInertiaPerUnitMass() * body.mass); }
    body.angularDamping = 0.05f;   
    body.drag           = 0.01f;   
    body.UpdateMassProperties();
    body.isAwake = true;
    body.allowSleep = true;
    body.motionLocks.lockRotation = false;
    std::uniform_real_distribution<float> colorDist(0.0f, 1.0f);
    float r = colorDist(m_Rng);
    float g = colorDist(m_Rng);
    float b = colorDist(m_Rng);
    ECS::RenderComponent render({ radius * 2.0f, radius * 2.0f }, { r, g, b });
    render.origin = { radius, radius };
    render.shapeType = ECS::RenderComponent::ShapeType::Circle;
    cs.AddComponent(spawnedEntity, std::move(t));
    cs.AddComponent(spawnedEntity, std::move(body));
    cs.AddComponent(spawnedEntity, std::move(circleCollider));
    cs.AddComponent(spawnedEntity, std::move(render)); }
bool SimplePhysicsDemo::IsPlayerGrounded() {
    auto& cs = GetComponentStore();
    auto worldEntities = cs.GetEntitiesWithComponent<ECS::PhysicsWorldComponent>();
    if (worldEntities.empty()) {
        std::cerr << "[DEMO] IsPlayerGrounded: No physics world found\n";
        return false; }
    const auto& world = cs.GetComponent<ECS::PhysicsWorldComponent>(worldEntities[0]);
    const Math::Vector2 up{0.0f, 1.0f};
    std::cerr << "[DEMO] IsPlayerGrounded: Checking " << world.contactManifolds.size() << " manifolds\n";
    for (const auto& manifold : world.contactManifolds) {
        if (manifold.points.empty())
            continue;
        bool isPlayerA = (manifold.entityIdA == m_PlayerEntity);
        bool isPlayerB = (manifold.entityIdB == m_PlayerEntity);
        if (!isPlayerA && !isPlayerB)
            continue;
        Math::Vector2 contactNormal = isPlayerA ? -manifold.normal : manifold.normal;
        float dotUp = Math::Vector2::Dot(contactNormal, up);
        if (dotUp > 0.7f) {
            for (const auto& pt : manifold.points) {
                if (pt.separation < 1.0f) {
                    return true; } } } }
    return false; }
void SimplePhysicsDemo::SpawnQuadAtMousePosition() {
    double mouseX, mouseY;
    Nyon::Utils::InputManager::GetMousePosition(mouseX, mouseY);
    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);
    float worldX = static_cast<float>(mouseX);
    float worldY = static_cast<float>(height - mouseY);   
    std::cerr << "[DEMO] Spawning circle at mouse position (" 
              << worldX << ", " << worldY << ")\n";
    CreateSpawnedCircle(worldX, worldY); }
void SimplePhysicsDemo::DespawnOutOfBoundsObjects() {
    auto& cs = GetComponentStore();
    std::vector<ECS::EntityID> entitiesToDespawn;
    for (const auto& entityId : m_SpawnedQuads) {
        if (!cs.HasComponent<ECS::TransformComponent>(entityId))
            continue;
        const auto& transform = cs.GetComponent<ECS::TransformComponent>(entityId);
        if (transform.position.y < DESPAWN_Y_THRESHOLD) {
            entitiesToDespawn.push_back(entityId);
            std::cerr << "[DEMO] Despawning entity at y=" << transform.position.y 
                      << " (below threshold " << DESPAWN_Y_THRESHOLD << ")\n"; } }
    for (const auto& entityId : entitiesToDespawn) {
        GetEntityManager().DestroyEntity(entityId);
        m_SpawnedQuads.erase(
            std::remove(m_SpawnedQuads.begin(), m_SpawnedQuads.end(), entityId),
            m_SpawnedQuads.end()
        ); }
    if (!entitiesToDespawn.empty()) {
        std::cerr << "[DEMO] Despawned " << entitiesToDespawn.size() << " out-of-bounds entities\n"; } }