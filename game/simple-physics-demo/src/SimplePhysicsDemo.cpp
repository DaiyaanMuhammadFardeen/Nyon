#include "SimplePhysicsDemo.h"

#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/components/BehaviorComponent.h"
#include "nyon/ecs/systems/DebugRenderSystem.h"
#include "nyon/utils/InputManager.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <random>

using namespace Nyon;

// ============================================================================
//  Constructor
// ============================================================================
SimplePhysicsDemo::SimplePhysicsDemo()
    : ECSApplication("Nyon – Simple Physics Demo", 1280, 720)
{
    // Disable debug renderer for this demo
    m_EnableDebugRenderer = false;
}

// ============================================================================
//  OnECSStart  –  world + entity setup
//  NOTE: systems (including the debug renderer) have NOT been registered yet
//  when this function runs.  Any system-level configuration must be deferred
//  to the first OnECSFixedUpdate tick.
// ============================================================================
void SimplePhysicsDemo::OnECSStart()
{
    CreateWorld();
    CreatePlatform();
    CreatePlayerQuad();
}

// ============================================================================
//  OnECSFixedUpdate  –  runs at exactly 1/60 s per tick
// ============================================================================
void SimplePhysicsDemo::OnECSFixedUpdate(float deltaTime)
{
    // ── advance simulation clock ──────────────────────────────────────────────
    m_SimTime += deltaTime;

    // ── periodic player position logging ────────────────────────────────────────
    if (m_SimTime >= m_NextLogTime)
    {
        m_NextLogTime += LOG_INTERVAL_S;

        auto& cs = GetComponentStore();
        if (cs.HasComponent<ECS::TransformComponent>(m_PlayerEntity))
        {
            const auto& t  = cs.GetComponent<ECS::TransformComponent>(m_PlayerEntity);
            const auto& rb = cs.GetComponent<ECS::PhysicsBodyComponent>(m_PlayerEntity);
            std::cerr << std::fixed << std::setprecision(2)
                      << "[DEMO t=" << m_SimTime << "s]"
                      << "  player pos=(" << t.position.x << ", " << t.position.y << ")"
                      << "  vel=("     << rb.velocity.x << ", " << rb.velocity.y << ")"
                      << "  awake="    << rb.isAwake
                      << "\n";
        }
    }

    // Handle player input
    HandlePlayerInput(deltaTime);

    // Auto-spawn random circles at intervals
    while (m_SimTime >= m_NextAutoSpawnTime)
    {
        int width, height;
        glfwGetWindowSize(GetWindow(), &width, &height);

        std::uniform_real_distribution<float> xDist(100.0f, float(width - 100));
        float spawnX = xDist(m_Rng);
        float spawnY = float(height) * 0.8f;

        std::cerr << "[DEMO] Auto-spawning circle at (" << spawnX << ", " << spawnY << ")\n";
        CreateSpawnedCircle(spawnX, spawnY);

        // Schedule next spawn
        m_NextAutoSpawnTime += std::uniform_real_distribution<float>(m_SpawnIntervalMin, m_SpawnIntervalMax)(m_Rng);
    }

    // Check for mouse click to spawn circles
    if (Nyon::Utils::InputManager::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
    {
        std::cerr << "[DEMO] Mouse left button pressed!" << std::endl;
        SpawnQuadAtMousePosition();
    }

    // ── auto-close after DEMO_DURATION_S ─────────────────────────────────────
    if (m_SimTime >= DEMO_DURATION_S)
    {
        std::cerr << "[DEMO] Simulation complete after "
                  << m_SimTime << " s.  Closing.\n";
        Close();
    }
}

// ============================================================================
//  CreateWorld
// ============================================================================
void SimplePhysicsDemo::CreateWorld()
{
    auto& entities  = GetEntityManager();
    auto& cs        = GetComponentStore();

    ECS::EntityID worldEntity = entities.CreateEntity();

    ECS::PhysicsWorldComponent world;

    // ── gravity ──────────────────────────────────────────────────────────────
    // The renderer uses glm::ortho(0, W, 0, H) — Y-up convention.
    // "Down" is therefore negative-Y.  Standard ~1g in pixel units.
    world.gravity = { 0.0f, -980.0f };

    // ── solver settings ──────────────────────────────────────────────────────
    world.timeStep              = 1.0f / 60.0f;
    world.velocityIterations    = 8;
    world.positionIterations    = 3;

    // linearSlop: the minimum penetration depth that triggers position
    // correction.  The engine's default (0.005) is Box2D's metre-scale value
    // and is far too small for pixel-scale physics — position correction fires
    // perpetually and injects energy.  0.5 pixels is appropriate here.
    // NOTE: PhysicsPipelineSystem currently reads m_Config.linearSlop (0.005)
    // rather than this field.  Fix that pipeline bug for this to take effect,
    // but set it here so the world component is already correct.
    world.linearSlop            = 0.5f;
    world.maxLinearCorrection   = 5.0f;   // pixels per step (was 0.2 — too tight)

    world.enableSleep           = true;
    world.enableWarmStarting    = true;   // no-op until StoreImpulses() is implemented
    world.enableContinuous      = false;  // CCD not wired up yet

    cs.AddComponent(worldEntity, std::move(world));
}

// ============================================================================
//  CreatePlatform  –  wide static floor near the bottom of the screen
// ============================================================================
void SimplePhysicsDemo::CreatePlatform()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    m_PlatformEntity = entities.CreateEntity();

    // ── transform ────────────────────────────────────────────────────────────
    // Center of the platform in world (Y-up) coordinates.
    // Y = 100 places it near the bottom of a 720-pixel-high window.
    ECS::TransformComponent t;
    t.position         = { 640.0f, 100.0f };
    t.previousPosition = t.position;   // ensure interpolation starts correctly
    t.rotation         = 0.0f;
    t.previousRotation = 0.0f;

    // ── physics body (static) ────────────────────────────────────────────────
    ECS::PhysicsBodyComponent body;
    body.isStatic = true;
    // UpdateMassProperties() with isStatic=true forces:
    //   inverseMass = 0, inverseInertia = 0  (body never moves)
    body.UpdateMassProperties();

    // ── collider ─────────────────────────────────────────────────────────────
    // Vertices are centered on the origin (transform.position is the centroid).
    // Width = 800, height = 50, so half-extents are ±400, ±25.
    //
    // Winding check (Y-up, shoelace sum):
    //   (-400,-25)→(400,-25)→(400,25)→(-400,25)  → sum = +80,000 (CCW) ✓
    // After the outward-normal fix (Bug 1: {edge.y,-edge.x}):
    //   top face normal  = {0, +1}  (upward)  ✓
    //   bottom face normal = {0, -1} (downward) ✓
    ECS::ColliderComponent::PolygonShape platformShape({
        { -400.0f, -25.0f },
        {  400.0f, -25.0f },
        {  400.0f,  25.0f },
        { -400.0f,  25.0f }
    });

    ECS::ColliderComponent platformCollider(platformShape);
    platformCollider.material.friction    = 0.6f;
    platformCollider.material.restitution = 0.1f;
    platformCollider.material.density     = 0.0f;  // irrelevant for static body

    // ── render component ─────────────────────────────────────────────────────
    // CRITICAL (Bug 10 first report): origin must equal {halfWidth, halfHeight}
    // so that DrawQuad pivots around transform.position — matching the physics
    // polygon which is also centred on transform.position.
    //
    // With origin = {0,0} the quad's top-left corner would be placed at
    // transform.position, giving a 400 px horizontal and 25 px vertical offset
    // from the physics body. That is why shapes appear to be in the wrong place.
    //
    // DrawQuad corners with origin = {400, 25}:
    //   corner 0: position + (-400, -25)  ← matches polygon vertex ✓
    //   corner 2: position + ( 400,  25)  ← matches polygon vertex ✓
    ECS::RenderComponent platformRender({ 800.0f, 50.0f }, { 0.35f, 0.35f, 0.35f });
    platformRender.origin = { 400.0f, 25.0f };

    cs.AddComponent(m_PlatformEntity, std::move(t));
    cs.AddComponent(m_PlatformEntity, std::move(body));
    cs.AddComponent(m_PlatformEntity, std::move(platformCollider));
    cs.AddComponent(m_PlatformEntity, std::move(platformRender));
}

// ============================================================================
//  CreatePlayerQuad  –  controllable player quad with WASD movement and jump
// ============================================================================
void SimplePhysicsDemo::CreatePlayerQuad()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    m_PlayerEntity = entities.CreateEntity();

    // ── transform ────────────────────────────────────────────────────────────
    // Start at center of screen
    ECS::TransformComponent t;
    t.position         = { 640.0f, 360.0f };
    t.previousPosition = t.position;
    t.rotation         = 0.0f;
    t.previousRotation = 0.0f;

    // ── collider (set up FIRST so we can derive inertia from it) ─────────────
    // 50×50 square, centered on the origin.
    ECS::ColliderComponent::PolygonShape playerShape({
        { -25.0f, -25.0f },
        {  25.0f, -25.0f },
        {  25.0f,  25.0f },
        { -25.0f,  25.0f }
    });

    ECS::ColliderComponent playerCollider(playerShape);
    playerCollider.material.friction    = 0.4f;
    playerCollider.material.restitution = 0.1f;  // Low restitution for better control
    playerCollider.material.density     = 0.0008f;

    // ── physics body ─────────────────────────────────────────────────────────
    ECS::PhysicsBodyComponent body;
    body.mass = 2.0f;

    // Calculate inertia from collider shape
    {
        float area    = playerCollider.CalculateArea();           // 2500.0 px²
        float density = (area > 0.0f) ? body.mass / area : 0.0f;
        body.inertia  = playerCollider.CalculateInertiaForUnitDensity() * density;
    }

    body.UpdateMassProperties();
    body.isAwake    = true;
    body.allowSleep = false;  // Player should never sleep

    // Lock rotation for player to prevent uncontrollable spinning
    body.motionLocks.lockRotation = true;

    // ── render component ─────────────────────────────────────────────────────
    ECS::RenderComponent playerRender({ 50.0f, 50.0f }, { 1.0f, 0.5f, 0.0f });  // Orange color
    playerRender.origin = { 25.0f, 25.0f };

    // ── behavior component for input handling ────────────────────────────────
    ECS::BehaviorComponent behavior;
    
    cs.AddComponent(m_PlayerEntity, std::move(t));
    cs.AddComponent(m_PlayerEntity, std::move(body));
    cs.AddComponent(m_PlayerEntity, std::move(playerCollider));
    cs.AddComponent(m_PlayerEntity, std::move(playerRender));
    cs.AddComponent(m_PlayerEntity, std::move(behavior));

    std::cerr << "[DEMO] Player quad created at (640, 360). "
              << "Use WASD to move and jump.\n";
}

// ============================================================================
//  CreateSpawnedQuad  –  non-controllable quad spawned at mouse position
// ============================================================================
void SimplePhysicsDemo::CreateSpawnedQuad(float x, float y)
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    ECS::EntityID spawnedEntity = entities.CreateEntity();
    m_SpawnedQuads.push_back(spawnedEntity);

    // ── transform ────────────────────────────────────────────────────────────
    ECS::TransformComponent t;
    t.position         = { x, y };
    t.previousPosition = t.position;
    t.rotation         = 0.0f;
    t.previousRotation = 0.0f;

    // ── collider ─────────────────────────────────────────────────────────────
    ECS::ColliderComponent::PolygonShape spawnShape({
        { -25.0f, -25.0f },
        {  25.0f, -25.0f },
        {  25.0f,  25.0f },
        { -25.0f,  25.0f }
    });

    ECS::ColliderComponent spawnCollider(spawnShape);
    spawnCollider.material.friction    = 0.4f;
    spawnCollider.material.restitution = 0.2f;
    spawnCollider.material.density     = 0.0008f;

    // ── physics body ─────────────────────────────────────────────────────────
    ECS::PhysicsBodyComponent body;
    body.mass = 2.0f;

    {
        float area    = spawnCollider.CalculateArea();
        float density = (area > 0.0f) ? body.mass / area : 0.0f;
        body.inertia  = spawnCollider.CalculateInertiaForUnitDensity() * density;
    }

    body.UpdateMassProperties();
    body.isAwake    = true;
    body.allowSleep = true;

    // ── render component ─────────────────────────────────────────────────────
    ECS::RenderComponent spawnRender({ 50.0f, 50.0f }, { 0.5f, 1.0f, 0.5f });  // Green color
    spawnRender.origin = { 25.0f, 25.0f };

    cs.AddComponent(spawnedEntity, std::move(t));
    cs.AddComponent(spawnedEntity, std::move(body));
    cs.AddComponent(spawnedEntity, std::move(spawnCollider));
    cs.AddComponent(spawnedEntity, std::move(spawnRender));
}

// ============================================================================
//  HandlePlayerInput  –  process WASD input and apply forces/velocities
// ============================================================================
void SimplePhysicsDemo::HandlePlayerInput(float deltaTime)
{
    auto& cs = GetComponentStore();
    
    if (!cs.HasComponent<ECS::PhysicsBodyComponent>(m_PlayerEntity))
        return;
    
    auto& body = cs.GetComponent<ECS::PhysicsBodyComponent>(m_PlayerEntity);
    
    // Horizontal movement (A/D)
    float horizontal = 0.0f;
    if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_A))
    {
        horizontal -= 1.0f;
    }
    if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_D))
    {
        horizontal += 1.0f;
    }

    // Apply horizontal velocity directly for responsive controls
    if (horizontal != 0.0f)
    {
        body.velocity.x = horizontal * PLAYER_MOVE_SPEED;
    }
    else
    {
        // Dampen horizontal velocity when no input
        body.velocity.x *= 0.9f;
    }

    // Jump (W) - only when grounded (using contact manifold ground detection)
    if (Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_W) && IsPlayerGrounded())
    {
        body.velocity.y = PLAYER_JUMP_FORCE;
        body.SetAwake(true);
        std::cerr << "[DEMO] Jump triggered (vel=" << body.velocity.y << ")\n";
    }

    // NOTE: PhysicsPipelineSystem already applies world gravity.
    // Do not apply additional gravity here to avoid doubling gravity and causing tunneling.
}

// ============================================================================
//  CreateSpawnedCircle  –  hit-test insensitive random circles
// ============================================================================
void SimplePhysicsDemo::CreateSpawnedCircle(float x, float y)
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    ECS::EntityID spawnedEntity = entities.CreateEntity();
    m_SpawnedQuads.push_back(spawnedEntity);

    // Random radius between 10 and 30 pixels
    std::uniform_real_distribution<float> radiusDist(10.0f, 30.0f);
    float radius = radiusDist(m_Rng);

    // Ensure the circle spawns on top of the platform (above y = 95)
    constexpr float platformTop = 70.0f + 25.0f; // platform y + half height
    float spawnY = std::max(y, platformTop + radius + 1.0f);

    // Transform
    ECS::TransformComponent t;
    t.position         = { x, spawnY };
    t.previousPosition = t.position;
    t.rotation         = 0.0f;
    t.previousRotation = 0.0f;

    // Collider/shape
    ECS::ColliderComponent::CircleShape circleShape;
    circleShape.center = { 0.0f, 0.0f };
    circleShape.radius = radius;

    ECS::ColliderComponent circleCollider(circleShape);
    circleCollider.material.friction    = 0.4f;
    circleCollider.material.restitution = 0.2f;
    circleCollider.material.density     = 0.0008f;

    // Physics body
    ECS::PhysicsBodyComponent body;
    body.mass = 2.0f;
    body.UpdateMassProperties();
    body.isAwake = true;
    body.allowSleep = true;

    // Render
    ECS::RenderComponent render({ radius * 2.0f, radius * 2.0f }, { 0.5f, 1.0f, 0.5f });
    render.origin = { radius, radius };

    cs.AddComponent(spawnedEntity, std::move(t));
    cs.AddComponent(spawnedEntity, std::move(body));
    cs.AddComponent(spawnedEntity, std::move(circleCollider));
    cs.AddComponent(spawnedEntity, std::move(render));
}

// ============================================================================
//  Player grounded test (based on contact manifolds)
// ============================================================================
bool SimplePhysicsDemo::IsPlayerGrounded()
{
    auto& cs = GetComponentStore();

    // Find the physics world entity. There is only one in this demo.
    auto worldEntities = cs.GetEntitiesWithComponent<ECS::PhysicsWorldComponent>();
    if (worldEntities.empty())
    {
        std::cerr << "[DEMO] IsPlayerGrounded: No physics world found\n";
        return false;
    }

    const auto& world = cs.GetComponent<ECS::PhysicsWorldComponent>(worldEntities[0]);
    const Math::Vector2 up{0.0f, 1.0f};

    std::cerr << "[DEMO] IsPlayerGrounded: Checking " << world.contactManifolds.size() << " manifolds\n";

    for (const auto& manifold : world.contactManifolds)
    {
        if (manifold.points.empty())
            continue;

        bool isPlayerA = (manifold.entityIdA == m_PlayerEntity);
        bool isPlayerB = (manifold.entityIdB == m_PlayerEntity);
        if (!isPlayerA && !isPlayerB)
            continue;

        Math::Vector2 contactNormal = isPlayerA ? manifold.normal : -manifold.normal;

        // Consider it “ground” if the contact normal has a large upward component
        float dotUp = Math::Vector2::Dot(contactNormal, up);
        std::cerr << "[DEMO] IsPlayerGrounded: Manifold normal=(" << contactNormal.x << ", " << contactNormal.y << ") dotUp=" << dotUp << "\n";
        if (dotUp > 0.7f)
        {
            // Also ensure the contact points are not far apart (i.e., actually touching)
            for (const auto& pt : manifold.points)
            {
                std::cerr << "[DEMO] IsPlayerGrounded: Point separation=" << pt.separation << "\n";
                if (pt.separation < 1.0f)
                {
                    std::cerr << "[DEMO] IsPlayerGrounded: Grounded!\n";
                    return true;
                }
            }
        }
    }

    std::cerr << "[DEMO] IsPlayerGrounded: Not grounded\n";
    return false;
}


// ============================================================================
//  SpawnQuadAtMousePosition  –  convert mouse coordinates to world space and spawn
// ============================================================================
void SimplePhysicsDemo::SpawnQuadAtMousePosition()
{
    double mouseX, mouseY;
    Nyon::Utils::InputManager::GetMousePosition(mouseX, mouseY);
    
    // Mouse coordinates are in screen space (origin at top-left)
    // Our rendering uses Y-up convention, so we need to flip Y
    // The renderer uses glm::ortho(0, width, 0, height)
    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);
    
    float worldX = static_cast<float>(mouseX);
    float worldY = static_cast<float>(height - mouseY);  // Flip Y for Y-up convention
    
    std::cerr << "[DEMO] Spawning circle at mouse position (" 
              << worldX << ", " << worldY << ")\n";
    
    CreateSpawnedCircle(worldX, worldY);
}