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

// ============================================================================
//  Constructor
// ============================================================================
FlappyDemo::FlappyDemo() : ECSApplication("Flappy Bird", 1280, 720)
{
}

// ============================================================================
//  OnECSStart  –  runs once before the first frame
// ============================================================================
void FlappyDemo::OnECSStart()
{
    std::cerr << "\n=== FLAPPY BIRD ===\n";
    std::cerr << "Press SPACE to flap, R to restart after game over.\n\n";

    CreateWorld();
    CreateCamera();
    CreateBird();
}

// ============================================================================
//  CreateWorld  –  physics configuration
// ============================================================================
void FlappyDemo::CreateWorld()
{
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

    cs.AddComponent(m_WorldEntity, std::move(world));
}

// ============================================================================
//  CreateCamera  –  fixed camera
// ============================================================================
void FlappyDemo::CreateCamera()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    m_CameraEntity = entities.CreateEntity();

    ECS::CameraComponent cam(1280.0f, 720.0f);
    cam.isActive = true;
    cam.camera.zoom = 1.0f;
    cam.camera.position = { 0.0f, 0.0f };

    cs.AddComponent(m_CameraEntity, std::move(cam));
}

// ============================================================================
//  CreateBird  –  dynamic circle affected by gravity
// ============================================================================
void FlappyDemo::CreateBird()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    m_BirdEntity = entities.CreateEntity();

    // Transform
    ECS::TransformComponent t;
    t.position = { BIRD_X, 500.0f };
    t.previousPosition = t.position;
    t.rotation = 0.0f;
    t.previousRotation = 0.0f;

    // Physics body
    ECS::PhysicsBodyComponent body;
    body.SetMass(1.0f);
    body.UpdateMassProperties();
    body.isAwake = true;
    body.allowSleep = false;
    body.drag = 0.0f;
    body.angularDamping = 10.0f;
    body.motionLocks.lockRotation = false;
    body.motionLocks.lockTranslationX = true;  // Keep bird at fixed X

    // Circle collider
    ECS::ColliderComponent::CircleShape circle;
    circle.radius = BIRD_RADIUS;
    ECS::ColliderComponent collider(circle);
    collider.material.friction = 0.0f;
    collider.material.restitution = 0.0f;
    collider.material.density = 0.1f;

    // Render
    ECS::RenderComponent render({ BIRD_RADIUS * 2, BIRD_RADIUS * 2 }, { 1.0f, 0.8f, 0.0f });
    render.origin = { BIRD_RADIUS, BIRD_RADIUS };
    render.shapeType = ECS::RenderComponent::ShapeType::Circle;

    cs.AddComponent(m_BirdEntity, std::move(t));
    cs.AddComponent(m_BirdEntity, std::move(body));
    cs.AddComponent(m_BirdEntity, std::move(collider));
    cs.AddComponent(m_BirdEntity, std::move(render));
}

// ============================================================================
//  CreatePipeSegment  –  a single static rectangle (top or bottom)
// ============================================================================
ECS::EntityID FlappyDemo::CreatePipeSegment(float x, float centerY, float height)
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    ECS::EntityID entity = entities.CreateEntity();

    float halfW = PIPE_WIDTH / 2.0f;
    float halfH = height / 2.0f;

    // Transform
    ECS::TransformComponent t;
    t.position = { x, centerY };
    t.previousPosition = t.position;
    t.rotation = 0.0f;
    t.previousRotation = 0.0f;

    // Static body
    ECS::PhysicsBodyComponent body;
    body.isStatic = true;
    body.UpdateMassProperties();

    // Polygon collider (rectangle)
    ECS::ColliderComponent::PolygonShape shape({
        { -halfW, -halfH },
        {  halfW, -halfH },
        {  halfW,  halfH },
        { -halfW,  halfH }
    });

    ECS::ColliderComponent collider(shape);
    collider.material.friction = 0.0f;
    collider.material.restitution = 0.0f;
    collider.material.density = 0.0f;

    // Render
    ECS::RenderComponent render({ PIPE_WIDTH, height }, { 0.2f, 0.8f, 0.2f });
    render.origin = { halfW, halfH };

    cs.AddComponent(entity, std::move(t));
    cs.AddComponent(entity, std::move(body));
    cs.AddComponent(entity, std::move(collider));
    cs.AddComponent(entity, std::move(render));

    return entity;
}

// ============================================================================
//  SpawnPipePair  –  create top + bottom pipe with a random gap
// ============================================================================
void FlappyDemo::SpawnPipePair()
{
    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);
    float screenH = static_cast<float>(height);

    // Random gap vertical position
    std::uniform_real_distribution<float> gapDist(
        PIPE_MIN_HEIGHT + PIPE_GAP / 2.0f,
        screenH - PIPE_MIN_HEIGHT - PIPE_GAP / 2.0f);
    float gapCenterY = gapDist(m_Rng);

    float pipeX = static_cast<float>(width) + PIPE_WIDTH;

    // Top pipe: extends from screen top down to gap top
    float gapTop = gapCenterY + PIPE_GAP / 2.0f;
    float topHeight = screenH - gapTop;
    float topCenterY = gapTop + topHeight / 2.0f;

    // Bottom pipe: extends from screen bottom up to gap bottom
    float gapBottom = gapCenterY - PIPE_GAP / 2.0f;
    float botHeight = gapBottom;
    float botCenterY = gapBottom / 2.0f;

    if (topHeight > 0.5f)
    {
        ECS::EntityID topPipe = CreatePipeSegment(pipeX, topCenterY, topHeight);
        m_Pipes.push_back(topPipe);
    }

    if (botHeight > 0.5f)
    {
        ECS::EntityID botPipe = CreatePipeSegment(pipeX, botCenterY, botHeight);
        m_Pipes.push_back(botPipe);
    }
}

// ============================================================================
//  DestroyOffscreenPipes  –  remove pipes that have scrolled past the left edge
// ============================================================================
void FlappyDemo::DestroyOffscreenPipes()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    auto it = m_Pipes.begin();
    while (it != m_Pipes.end())
    {
        if (cs.HasComponent<ECS::TransformComponent>(*it))
        {
            const auto& t = cs.GetComponent<ECS::TransformComponent>(*it);
            if (t.position.x < -PIPE_WIDTH)
            {
                entities.DestroyEntity(*it, cs);
                it = m_Pipes.erase(it);
                continue;
            }
        }
        ++it;
    }
}

// ============================================================================
//  HandleInput
// ============================================================================
void FlappyDemo::HandleInput()
{
    if (Utils::InputManager::IsKeyPressed(GLFW_KEY_SPACE))
    {
        switch (m_State)
        {
            case GameState::MENU:
                m_State = GameState::PLAYING;
                // fall through to flap
            case GameState::PLAYING:
            {
                auto& cs = GetComponentStore();
                if (cs.HasComponent<ECS::PhysicsBodyComponent>(m_BirdEntity))
                {
                    auto& body = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BirdEntity);
                    body.velocity.y = 0.0f;  // reset vertical velocity for consistent flap
                    body.ApplyLinearImpulse({ 0.0f, FLAP_IMPULSE });
                }
                break;
            }
            case GameState::GAME_OVER:
                break;
        }
    }

    if (Utils::InputManager::IsKeyPressed(GLFW_KEY_R) && m_State == GameState::GAME_OVER)
    {
        ResetGame();
    }
}

// ============================================================================
//  CheckCollisions  –  detect bird hitting pipes via contact manifolds
// ============================================================================
void FlappyDemo::CheckCollisions()
{
    auto& cs = GetComponentStore();

    auto worldEntities = cs.GetEntitiesWithComponent<ECS::PhysicsWorldComponent>();
    if (worldEntities.empty()) return;

    const auto& world = cs.GetComponent<ECS::PhysicsWorldComponent>(worldEntities[0]);

    for (const auto& manifold : world.contactManifolds)
    {
        if (manifold.points.empty()) continue;

        bool isBirdA = (manifold.entityIdA == m_BirdEntity);
        bool isBirdB = (manifold.entityIdB == m_BirdEntity);

        if (!isBirdA && !isBirdB) continue;

        // Bird hit a pipe → game over
        m_State = GameState::GAME_OVER;
        std::cerr << "\n*** GAME OVER! Score: " << m_Score << " ***\n";
        std::cerr << "Press R to restart.\n";
        return;
    }
}

// ============================================================================
//  UpdateScore  –  increment score when bird passes a pipe's X position
// ============================================================================
void FlappyDemo::UpdateScore(float birdX)
{
    auto& cs = GetComponentStore();

    for (auto pipeId : m_Pipes)
    {
        if (cs.HasComponent<ECS::TransformComponent>(pipeId))
        {
            const auto& t = cs.GetComponent<ECS::TransformComponent>(pipeId);
            // Only score once per pipe: when pipe's right edge passes bird
            if (t.position.x + PIPE_WIDTH / 2.0f < birdX && t.position.x > m_LastScoredX)
            {
                // Score only for the bottom pipe of each pair (to avoid double-count)
                // Bottom pipes have lower Y center (closer to ground)
                if (t.position.y < 360.0f)
                {
                    m_Score++;
                    m_LastScoredX = t.position.x;
                    std::cerr << "[FLAPPY] Score: " << m_Score << "\n";
                }
            }
        }
    }
}

// ============================================================================
//  ResetGame  –  full reset to MENU state
// ============================================================================
void FlappyDemo::ResetGame()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    // Destroy all pipes
    for (auto pipeId : m_Pipes)
    {
        entities.DestroyEntity(pipeId, cs);
    }
    m_Pipes.clear();

    // Reset bird
    if (cs.HasComponent<ECS::TransformComponent>(m_BirdEntity))
    {
        auto& t = cs.GetComponent<ECS::TransformComponent>(m_BirdEntity);
        t.position = { BIRD_X, 500.0f };
        t.previousPosition = t.position;
        t.rotation = 0.0f;
        t.previousRotation = 0.0f;
    }
    if (cs.HasComponent<ECS::PhysicsBodyComponent>(m_BirdEntity))
    {
        auto& body = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BirdEntity);
        body.velocity = { 0.0f, 0.0f };
        body.angularVelocity = 0.0f;
    }

    // Reset state
    m_State = GameState::MENU;
    m_Score = 0;
    m_PipeSpawnTimer = 0.0f;
    m_LastScoredX = -1000.0f;

    std::cerr << "\n=== GAME RESET ===\n";
    std::cerr << "Press SPACE to start.\n\n";
}

// ============================================================================
//  OnECSFixedUpdate  –  runs every 1/60 s tick
// ============================================================================
void FlappyDemo::OnECSFixedUpdate(float deltaTime)
{
    auto& cs = GetComponentStore();

    HandleInput();

    if (m_State == GameState::MENU)
    {
        // Keep bird alive and floating in MENU
        if (cs.HasComponent<ECS::PhysicsBodyComponent>(m_BirdEntity))
        {
            auto& body = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BirdEntity);
            body.velocity = { 0.0f, 0.0f };
            body.angularVelocity = 0.0f;
        }
        if (cs.HasComponent<ECS::TransformComponent>(m_BirdEntity))
        {
            auto& t = cs.GetComponent<ECS::TransformComponent>(m_BirdEntity);
            t.position.y = 500.0f + std::sin(static_cast<float>(glfwGetTime()) * 2.0f) * 15.0f;
            t.previousPosition = t.position;
            t.rotation = 0.0f;
            t.previousRotation = 0.0f;
        }
        return;
    }

    if (m_State == GameState::GAME_OVER)
        return;

    // --- PLAYING state ---

    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);

    // 1. Scroll pipes leftward
    for (auto pipeId : m_Pipes)
    {
        if (cs.HasComponent<ECS::TransformComponent>(pipeId))
        {
            auto& t = cs.GetComponent<ECS::TransformComponent>(pipeId);
            t.position.x -= SCROLL_SPEED * deltaTime;
            t.previousPosition.x = t.position.x;
        }
    }

    // 2. Spawn pipes at interval
    m_PipeSpawnTimer += deltaTime;
    if (m_PipeSpawnTimer >= PIPE_SPAWN_INTERVAL)
    {
        m_PipeSpawnTimer -= PIPE_SPAWN_INTERVAL;
        SpawnPipePair();
    }

    // 3. Destroy off-screen pipes
    DestroyOffscreenPipes();

    // 4. Bird visual tilt based on velocity
    if (cs.HasComponent<ECS::PhysicsBodyComponent>(m_BirdEntity) &&
        cs.HasComponent<ECS::TransformComponent>(m_BirdEntity))
    {
        const auto& body = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BirdEntity);
        auto& t = cs.GetComponent<ECS::TransformComponent>(m_BirdEntity);
        float targetAngle = body.velocity.y * 0.003f;
        t.rotation = std::clamp(targetAngle, -0.5f, 1.5f);
    }

    // 5. Check pipe collisions
    CheckCollisions();

    // 6. Check if bird fell below screen or went above
    if (cs.HasComponent<ECS::TransformComponent>(m_BirdEntity))
    {
        const auto& t = cs.GetComponent<ECS::TransformComponent>(m_BirdEntity);

        // Fell below screen
        if (t.position.y < -BIRD_RADIUS * 2)
        {
            m_State = GameState::GAME_OVER;
            std::cerr << "\n*** GAME OVER! Score: " << m_Score << " ***\n";
            std::cerr << "Press R to restart.\n";
            return;
        }

        // Clamp bird X to fixed position (prevent drift)
        // lockTranslationX is set in CreateBird
    }

    // 7. Update score
    if (cs.HasComponent<ECS::TransformComponent>(m_BirdEntity))
    {
        const auto& t = cs.GetComponent<ECS::TransformComponent>(m_BirdEntity);
        UpdateScore(t.position.x);
    }
}
