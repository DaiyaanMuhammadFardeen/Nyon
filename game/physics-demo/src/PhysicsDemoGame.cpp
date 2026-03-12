#include "PhysicsDemoGame.h"

#include "nyon/ecs/EntityManager.h"
#include "nyon/ecs/ComponentStore.h"
#include "nyon/ecs/SystemManager.h"

#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"

#include "nyon/ecs/systems/PhysicsIntegrationSystem.h"
#include "nyon/ecs/systems/DebugRenderSystem.h"

#include "nyon/utils/InputManager.h"

#include <GLFW/glfw3.h>

using namespace Nyon;

PhysicsDemoGame::PhysicsDemoGame()
    : ECSApplication("Nyon Physics Demo – Full Showcase", 1280, 720)
{
}

void PhysicsDemoGame::OnECSStart()
{
    CreatePhysicsWorld();
    CreateBounds();
    CreateStackTest();
    CreateRampFrictionTest();
    CreateBounceTest();
}

void PhysicsDemoGame::OnECSFixedUpdate(float deltaTime)
{
    // Apply timescale (slow motion / fast forward)
    float scaledDelta = deltaTime * m_TimeScale;

    HandleGlobalInput(scaledDelta);

    if (m_Paused)
        return;

    // Game-specific per-tick logic could go here (e.g., scripted forces),
    // but this demo mainly relies on the physics pipeline + input-driven spawning.
}

void PhysicsDemoGame::OnECSUpdate(float /*deltaTime*/)
{
    // Optional: could display HUD / statistics here by querying PhysicsWorldComponent profile/counters.
}

void PhysicsDemoGame::CreatePhysicsWorld()
{
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();
    auto& systems    = GetSystemManager();

    // Create world entity with global physics settings
    ECS::EntityID worldEntity = entities.CreateEntity();

    ECS::PhysicsWorldComponent world;
    world.gravity = { 0.0f, 980.0f };          // Pixels / s^2, y-positive down
    world.timeStep = 1.0f / 60.0f;
    world.velocityIterations = 8;
    world.positionIterations = 3;
    world.subStepCount = 4;
    world.enableSleep = true;
    world.enableWarmStarting = true;
    world.enableContinuous = true;

    // Enable rich debug visualization: shapes, contacts, etc.
    world.SetDebugDraw(
        /*shapes*/   true,
        /*joints*/   false,
        /*aabbs*/    false,
        /*contacts*/ true,
        /*islands*/  false
    );

    components.AddComponent(worldEntity, std::move(world));

    // Register physics integration system (simplified physics pipeline)
    systems.AddSystem(std::make_unique<ECS::PhysicsIntegrationSystem>());
}

void PhysicsDemoGame::CreateBounds()
{
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    // Static border walls around the play area to keep bodies on screen.
    const float width  = 1280.0f;
    const float height = 720.0f;
    const float thickness = 40.0f;

    using Nyon::Math::Vector2;

    auto createWall = [&](const Vector2& center, const Vector2& halfExtents)
    {
        ECS::EntityID wall = entities.CreateEntity();

        ECS::TransformComponent t;
        t.position = center;

        ECS::PhysicsBodyComponent body;
        body.isStatic = true;
        body.UpdateMassProperties();

        ECS::ColliderComponent::PolygonShape shape({
            {-halfExtents.x, -halfExtents.y},
            { halfExtents.x, -halfExtents.y},
            { halfExtents.x,  halfExtents.y},
            {-halfExtents.x,  halfExtents.y}
        });

        ECS::ColliderComponent collider(shape);
        collider.material.friction    = 0.8f;
        collider.material.restitution = 0.1f;

        components.AddComponent(wall, std::move(t));
        components.AddComponent(wall, std::move(body));
        components.AddComponent(wall, std::move(collider));
    };

    // Floor
    createWall({width * 0.5f, height + thickness * 0.5f}, {width * 0.5f, thickness * 0.5f});
    // Ceiling
    createWall({width * 0.5f, -thickness * 0.5f},        {width * 0.5f, thickness * 0.5f});
    // Left wall
    createWall({-thickness * 0.5f, height * 0.5f},       {thickness * 0.5f, height * 0.5f});
    // Right wall
    createWall({width + thickness * 0.5f, height * 0.5f},{thickness * 0.5f, height * 0.5f});
}

void PhysicsDemoGame::CreateStackTest()
{
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    using Nyon::Math::Vector2;

    const int columns = 6;
    const int rows    = 8;
    const float boxSize = 35.0f;
    const float startX  = 350.0f;
    const float startY  = 650.0f;

    for (int x = 0; x < columns; ++x)
    {
        for (int y = 0; y < rows; ++y)
        {
            ECS::EntityID box = entities.CreateEntity();

            ECS::TransformComponent t;
            t.position = {
                startX + x * (boxSize * 2.2f),
                startY - y * (boxSize * 2.2f)
            };

            ECS::PhysicsBodyComponent body;
            body.mass = 1.0f;
            body.UpdateMassProperties();

            ECS::ColliderComponent::PolygonShape shape({
                {-boxSize, -boxSize},
                { boxSize, -boxSize},
                { boxSize,  boxSize},
                {-boxSize,  boxSize}
            });

            ECS::ColliderComponent collider(shape);
            collider.material.friction    = 0.4f;
            collider.material.restitution = 0.0f;

            components.AddComponent(box, std::move(t));
            components.AddComponent(box, std::move(body));
            components.AddComponent(box, std::move(collider));
        }
    }
}

void PhysicsDemoGame::CreateRampFrictionTest()
{
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    using Nyon::Math::Vector2;

    // Create an inclined static ramp
    {
        ECS::EntityID ramp = entities.CreateEntity();

        ECS::TransformComponent t;
        t.position = {950.0f, 550.0f};
        t.rotation = -0.35f; // Slight incline

        ECS::PhysicsBodyComponent body;
        body.isStatic = true;
        body.UpdateMassProperties();

        float halfWidth  = 250.0f;
        float halfHeight = 15.0f;

        ECS::ColliderComponent::PolygonShape shape({
            {-halfWidth, -halfHeight},
            { halfWidth, -halfHeight},
            { halfWidth,  halfHeight},
            {-halfWidth,  halfHeight}
        });

        ECS::ColliderComponent collider(shape);
        collider.material.friction    = 0.8f; // High friction ramp surface
        collider.material.restitution = 0.0f;

        components.AddComponent(ramp, std::move(t));
        components.AddComponent(ramp, std::move(body));
        components.AddComponent(ramp, std::move(collider));
    }

    // Drop boxes with varying friction on the ramp
    const int boxCount = 5;
    const float baseX  = 850.0f;
    const float baseY  = 350.0f;
    const float spacing = 60.0f;

    for (int i = 0; i < boxCount; ++i)
    {
        float friction = 0.1f + 0.2f * static_cast<float>(i); // 0.1, 0.3, 0.5, ...
        float restitution = 0.0f;

        ECS::EntityID box = entities.CreateEntity();

        ECS::TransformComponent t;
        t.position = {baseX + i * spacing, baseY};

        ECS::PhysicsBodyComponent body;
        body.mass = 1.0f;
        body.UpdateMassProperties();

        float half = 20.0f;
        ECS::ColliderComponent::PolygonShape shape({
            {-half, -half},
            { half, -half},
            { half,  half},
            {-half,  half}
        });

        ECS::ColliderComponent collider(shape);
        collider.material.friction    = friction;
        collider.material.restitution = restitution;

        components.AddComponent(box, std::move(t));
        components.AddComponent(box, std::move(body));
        components.AddComponent(box, std::move(collider));
    }
}

void PhysicsDemoGame::CreateBounceTest()
{
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    using Nyon::Math::Vector2;

    const int ballCount = 6;
    const float baseX   = 200.0f;
    const float baseY   = 100.0f;
    const float spacing = 80.0f;
    const float radius  = 20.0f;

    for (int i = 0; i < ballCount; ++i)
    {
        float restitution = 0.1f * static_cast<float>(i); // 0.0, 0.1, ..., 0.5
        float friction    = 0.2f;

        ECS::EntityID ball = entities.CreateEntity();

        ECS::TransformComponent t;
        t.position = {baseX + i * spacing, baseY};

        ECS::PhysicsBodyComponent body;
        body.mass = 1.0f;
        body.UpdateMassProperties();
        body.isBullet = (i == ballCount - 1); // Last ball as high-speed bullet candidate

        ECS::ColliderComponent collider(radius);
        collider.material.friction    = friction;
        collider.material.restitution = restitution;

        components.AddComponent(ball, std::move(t));
        components.AddComponent(ball, std::move(body));
        components.AddComponent(ball, std::move(collider));
    }
}

void PhysicsDemoGame::HandleGlobalInput(float /*deltaTime*/)
{
    using Nyon::Utils::InputManager;

    // Pause / resume
    if (InputManager::IsKeyPressed(GLFW_KEY_P))
    {
        m_Paused = !m_Paused;
    }

    // Slow motion toggle
    if (InputManager::IsKeyPressed(GLFW_KEY_1))
    {
        m_TimeScale = 0.25f;
    }
    if (InputManager::IsKeyPressed(GLFW_KEY_2))
    {
        m_TimeScale = 1.0f;
    }
    if (InputManager::IsKeyPressed(GLFW_KEY_3))
    {
        m_TimeScale = 2.0f;
    }

    // Spawn demo shapes at mouse position
    if (InputManager::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
    {
        double mx = 0.0, my = 0.0;
        InputManager::GetMousePosition(mx, my);
        SpawnBoxAt(static_cast<float>(mx), static_cast<float>(my), 25.0f, 0.2f, 0.5f);
    }
    if (InputManager::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
    {
        double mx = 0.0, my = 0.0;
        InputManager::GetMousePosition(mx, my);
        SpawnBallAt(static_cast<float>(mx), static_cast<float>(my), 18.0f, 0.6f, 0.2f);
    }
}

void PhysicsDemoGame::SpawnBoxAt(float x, float y, float size, float restitution, float friction)
{
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    ECS::EntityID box = entities.CreateEntity();

    ECS::TransformComponent t;
    t.position = {x, y};

    ECS::PhysicsBodyComponent body;
    body.mass = 1.0f;
    body.UpdateMassProperties();

    float half = size;
    ECS::ColliderComponent::PolygonShape shape({
        {-half, -half},
        { half, -half},
        { half,  half},
        {-half,  half}
    });

    ECS::ColliderComponent collider(shape);
    collider.material.friction    = friction;
    collider.material.restitution = restitution;

    components.AddComponent(box, std::move(t));
    components.AddComponent(box, std::move(body));
    components.AddComponent(box, std::move(collider));
}

void PhysicsDemoGame::SpawnBallAt(float x, float y, float radius, float restitution, float friction)
{
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    ECS::EntityID ball = entities.CreateEntity();

    ECS::TransformComponent t;
    t.position = {x, y};

    ECS::PhysicsBodyComponent body;
    body.mass = 1.0f;
    body.UpdateMassProperties();

    ECS::ColliderComponent collider(radius);
    collider.material.friction    = friction;
    collider.material.restitution = restitution;

    components.AddComponent(ball, std::move(t));
    components.AddComponent(ball, std::move(body));
    components.AddComponent(ball, std::move(collider));
}

