#include "FlappyDemo.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/components/CameraComponent.h"
#include "nyon/utils/InputManager.h"
#include <iostream>
#include <algorithm>
using namespace Nyon;
FlappyDemo::FlappyDemo() : ECSApplication("Flappy Bird", 1280, 720) { }
void FlappyDemo::OnECSStart() {
    std::cerr << "\n=== FLAPPY BIRD ===\n";
    std::cerr << "Press SPACE to flap, R to restart after game over.\n\n";
    CreateWorld();
    CreateCamera();
    CreateBird(); }
void FlappyDemo::CreateWorld() {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    m_WorldEntity = entities.CreateEntity();
    ECS::PhysicsWorldComponent world;
    world.gravity                = { 0.0f, -1200.0f };
    world.timeStep               = 1.0f / 60.0f;
    world.velocityIterations     = 8;
    world.positionIterations     = 3;
    world.linearSlop             = 0.3f;
    world.maxLinearCorrection    = 4.0f;
    world.enableSleep            = false;
    world.enableWarmStarting     = true;
    world.enableContinuous       = false;
    world.baumgarteBeta          = 0.2f;
    world.contactHertz           = 30.0f;
    world.contactDampingRatio    = 1.0f;
    world.contactPushSpeed       = 30.0f;
    cs.AddComponent(m_WorldEntity, std::move(world)); }
void FlappyDemo::CreateCamera() {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    m_CameraEntity = entities.CreateEntity();
    ECS::CameraComponent cam(1280.0f, 720.0f);
    cam.isActive = true;
    cam.camera.zoom = 1.0f;
    cam.camera.position = { 0.0f, 0.0f };
    cs.AddComponent(m_CameraEntity, std::move(cam)); }
void FlappyDemo::CreateBird() {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    m_BirdEntity = entities.CreateEntity();
    ECS::TransformComponent t;
    t.position = { BIRD_X, 500.0f };
    t.previousPosition = t.position;
    t.rotation = 0.0f;
    t.previousRotation = 0.0f;
    ECS::PhysicsBodyComponent body;
    body.SetMass(1.0f);
    body.UpdateMassProperties();
    body.isAwake = true;
    body.allowSleep = false;
    body.drag = 0.0f;
    body.angularDamping = 10.0f;
    body.motionLocks.lockRotation = false;
    body.motionLocks.lockTranslationX = true;   
    ECS::ColliderComponent::CircleShape circle;
    circle.radius = BIRD_RADIUS;
    ECS::ColliderComponent collider(circle);
    collider.material.friction = 0.0f;
    collider.material.restitution = 0.0f;
    collider.material.density = 0.1f;
    ECS::RenderComponent render({ BIRD_RADIUS * 2, BIRD_RADIUS * 2 }, { 1.0f, 0.8f, 0.0f });
    render.origin = { BIRD_RADIUS, BIRD_RADIUS };
    render.shapeType = ECS::RenderComponent::ShapeType::Circle;
    cs.AddComponent(m_BirdEntity, std::move(t));
    cs.AddComponent(m_BirdEntity, std::move(body));
    cs.AddComponent(m_BirdEntity, std::move(collider));
    cs.AddComponent(m_BirdEntity, std::move(render)); }
ECS::EntityID FlappyDemo::CreatePipeSegment(float x, float centerY, float height) {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    ECS::EntityID entity = entities.CreateEntity();
    float halfW = PIPE_WIDTH / 2.0f;
    float halfH = height / 2.0f;
    ECS::TransformComponent t;
    t.position = { x, centerY };
    t.previousPosition = t.position;
    t.rotation = 0.0f;
    t.previousRotation = 0.0f;
    ECS::PhysicsBodyComponent body;
    body.isStatic = true;
    body.UpdateMassProperties();
    ECS::ColliderComponent::PolygonShape shape({ { -halfW, -halfH }, {  halfW, -halfH }, {  halfW,  halfH }, { -halfW,  halfH } });
    ECS::ColliderComponent collider(shape);
    collider.material.friction = 0.0f;
    collider.material.restitution = 0.0f;
    collider.material.density = 0.0f;
    ECS::RenderComponent render({ PIPE_WIDTH, height }, { 0.2f, 0.8f, 0.2f });
    render.origin = { halfW, halfH };
    cs.AddComponent(entity, std::move(t));
    cs.AddComponent(entity, std::move(body));
    cs.AddComponent(entity, std::move(collider));
    cs.AddComponent(entity, std::move(render));
    return entity; }
void FlappyDemo::SpawnPipePair() {
    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);
    float screenH = static_cast<float>(height);
    std::uniform_real_distribution<float> gapDist(
        PIPE_MIN_HEIGHT + PIPE_GAP / 2.0f,
        screenH - PIPE_MIN_HEIGHT - PIPE_GAP / 2.0f);
    float gapCenterY = gapDist(m_Rng);
    float pipeX = static_cast<float>(width) + PIPE_WIDTH;
    float gapTop = gapCenterY + PIPE_GAP / 2.0f;
    float topHeight = screenH - gapTop;
    float topCenterY = gapTop + topHeight / 2.0f;
    float gapBottom = gapCenterY - PIPE_GAP / 2.0f;
    float botHeight = gapBottom;
    float botCenterY = gapBottom / 2.0f;
    if (topHeight > 0.5f) {
        ECS::EntityID topPipe = CreatePipeSegment(pipeX, topCenterY, topHeight);
        m_Pipes.push_back(topPipe); }
    if (botHeight > 0.5f) {
        ECS::EntityID botPipe = CreatePipeSegment(pipeX, botCenterY, botHeight);
        m_Pipes.push_back(botPipe); } }
void FlappyDemo::DestroyOffscreenPipes() {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    auto it = m_Pipes.begin();
    while (it != m_Pipes.end()) {
        if (cs.HasComponent<ECS::TransformComponent>(*it)) {
            const auto& t = cs.GetComponent<ECS::TransformComponent>(*it);
            if (t.position.x < -PIPE_WIDTH) {
                entities.DestroyEntity(*it, cs);
                it = m_Pipes.erase(it);
                continue; } }
        ++it; } }
void FlappyDemo::HandleInput() {
    if (Utils::InputManager::IsKeyPressed(GLFW_KEY_SPACE)) {
        switch (m_State) {
            case GameState::MENU:
                m_State = GameState::PLAYING;
            case GameState::PLAYING: {
                auto& cs = GetComponentStore();
                if (cs.HasComponent<ECS::PhysicsBodyComponent>(m_BirdEntity)) {
                    auto& body = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BirdEntity);
                    body.velocity.y = 0.0f;   
                    body.ApplyLinearImpulse({ 0.0f, FLAP_IMPULSE }); }
                break; }
            case GameState::GAME_OVER:
                break; } }
    if (Utils::InputManager::IsKeyPressed(GLFW_KEY_R) && m_State == GameState::GAME_OVER) {
        ResetGame(); } }
void FlappyDemo::CheckCollisions() {
    auto& cs = GetComponentStore();
    auto worldEntities = cs.GetEntitiesWithComponent<ECS::PhysicsWorldComponent>();
    if (worldEntities.empty()) return;
    const auto& world = cs.GetComponent<ECS::PhysicsWorldComponent>(worldEntities[0]);
    for (const auto& manifold : world.contactManifolds) {
        if (manifold.points.empty()) continue;
        bool isBirdA = (manifold.entityIdA == m_BirdEntity);
        bool isBirdB = (manifold.entityIdB == m_BirdEntity);
        if (!isBirdA && !isBirdB) continue;
        m_State = GameState::GAME_OVER;
        std::cerr << "\n*** GAME OVER! Score: " << m_Score << " ***\n";
        std::cerr << "Press R to restart.\n";
        return; } }
void FlappyDemo::UpdateScore(float birdX) {
    auto& cs = GetComponentStore();
    for (auto pipeId : m_Pipes) {
        if (cs.HasComponent<ECS::TransformComponent>(pipeId)) {
            const auto& t = cs.GetComponent<ECS::TransformComponent>(pipeId);
            if (t.position.x + PIPE_WIDTH / 2.0f < birdX && t.position.x > m_LastScoredX) {
                if (t.position.y < 360.0f) {
                    m_Score++;
                    m_LastScoredX = t.position.x;
                    std::cerr << "[FLAPPY] Score: " << m_Score << "\n"; } } } } }
void FlappyDemo::ResetGame() {
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();
    for (auto pipeId : m_Pipes) {
        entities.DestroyEntity(pipeId, cs); }
    m_Pipes.clear();
    if (cs.HasComponent<ECS::TransformComponent>(m_BirdEntity)) {
        auto& t = cs.GetComponent<ECS::TransformComponent>(m_BirdEntity);
        t.position = { BIRD_X, 500.0f };
        t.previousPosition = t.position;
        t.rotation = 0.0f;
        t.previousRotation = 0.0f; }
    if (cs.HasComponent<ECS::PhysicsBodyComponent>(m_BirdEntity)) {
        auto& body = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BirdEntity);
        body.velocity = { 0.0f, 0.0f };
        body.angularVelocity = 0.0f; }
    m_State = GameState::MENU;
    m_Score = 0;
    m_PipeSpawnTimer = 0.0f;
    m_LastScoredX = -1000.0f;
    std::cerr << "\n=== GAME RESET ===\n";
    std::cerr << "Press SPACE to start.\n\n"; }
void FlappyDemo::OnECSFixedUpdate(float deltaTime) {
    auto& cs = GetComponentStore();
    HandleInput();
    if (m_State == GameState::MENU) {
        if (cs.HasComponent<ECS::PhysicsBodyComponent>(m_BirdEntity)) {
            auto& body = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BirdEntity);
            body.velocity = { 0.0f, 0.0f };
            body.angularVelocity = 0.0f; }
        if (cs.HasComponent<ECS::TransformComponent>(m_BirdEntity)) {
            auto& t = cs.GetComponent<ECS::TransformComponent>(m_BirdEntity);
            t.position.y = 500.0f + std::sin(static_cast<float>(glfwGetTime()) * 2.0f) * 15.0f;
            t.previousPosition = t.position;
            t.rotation = 0.0f;
            t.previousRotation = 0.0f; }
        return; }
    if (m_State == GameState::GAME_OVER)
        return;
    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);
    for (auto pipeId : m_Pipes) {
        if (cs.HasComponent<ECS::TransformComponent>(pipeId)) {
            auto& t = cs.GetComponent<ECS::TransformComponent>(pipeId);
            t.position.x -= SCROLL_SPEED * deltaTime;
            t.previousPosition.x = t.position.x; } }
    m_PipeSpawnTimer += deltaTime;
    if (m_PipeSpawnTimer >= PIPE_SPAWN_INTERVAL) {
        m_PipeSpawnTimer -= PIPE_SPAWN_INTERVAL;
        SpawnPipePair(); }
    DestroyOffscreenPipes();
    if (cs.HasComponent<ECS::PhysicsBodyComponent>(m_BirdEntity) &&
        cs.HasComponent<ECS::TransformComponent>(m_BirdEntity)) {
        const auto& body = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BirdEntity);
        auto& t = cs.GetComponent<ECS::TransformComponent>(m_BirdEntity);
        float targetAngle = body.velocity.y * 0.003f;
        t.rotation = std::clamp(targetAngle, -0.5f, 1.5f); }
    CheckCollisions();
    if (cs.HasComponent<ECS::TransformComponent>(m_BirdEntity)) {
        const auto& t = cs.GetComponent<ECS::TransformComponent>(m_BirdEntity);
        if (t.position.y < -BIRD_RADIUS * 2) {
            m_State = GameState::GAME_OVER;
            std::cerr << "\n*** GAME OVER! Score: " << m_Score << " ***\n";
            std::cerr << "Press R to restart.\n";
            return; } }
    if (cs.HasComponent<ECS::TransformComponent>(m_BirdEntity)) {
        const auto& t = cs.GetComponent<ECS::TransformComponent>(m_BirdEntity);
        UpdateScore(t.position.x); } }
