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

// ============================================================================
//  Constructor
// ============================================================================
SimplePhysicsDemo::SimplePhysicsDemo()
    : ECSApplication("Nyon – Simple Physics Demo", 1280, 720)
{
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
    CreateSlopedPlatform();
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

    // 1. Define your spawn interval (0.1 seconds = 100ms)
    const float spawnInterval = 1.0f;

    while (m_SimTime >= m_NextAutoSpawnTime)
    {
        int width, height;
        glfwGetWindowSize(GetWindow(), &width, &height);

        // 2. Define the random X distribution range
        std::uniform_real_distribution<float> xDist(100.0f, float(width - 100));

        float spawnX = xDist(m_Rng); // Random X position
        float spawnY = float(height) * 0.8f;

        std::cerr << "[DEMO] Auto-spawning circle at (" << spawnX << ", " << spawnY << ")\n";
        CreateSpawnedCircle(spawnX, spawnY);

        // 3. Increment by exactly 100ms
        m_NextAutoSpawnTime += spawnInterval;
    }

    // Check for mouse click to spawn circles
    if (Nyon::Utils::InputManager::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
    {
        std::cerr << "[DEMO] Mouse left button pressed!" << std::endl;
        SpawnQuadAtMousePosition();
    }

    // Despawn objects that fall below the screen
    DespawnOutOfBoundsObjects();
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
    world.linearSlop            = 0.05f;
    world.maxLinearCorrection   = 2.0f;   // pixels per step (was 0.2 — too tight)

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
    platformCollider.material.restitution = 0.8f;
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
//  CreateSlopedPlatform  –  static platform with a slope
// ============================================================================
void SimplePhysicsDemo::CreateSlopedPlatform()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    // Slope 1: positioned below and to the left of main platform, rotated -25 degrees
    ECS::EntityID slope1Entity = entities.CreateEntity();

    ECS::TransformComponent t1;
    t1.position         = { 150.0f, 80.0f };  // Below main platform (y=100), left side
    t1.previousPosition = t1.position;
    // Rotate by -25 degrees (about -0.436 radians)
    t1.rotation         = -0.436332f;
    t1.previousRotation = t1.rotation;

    ECS::PhysicsBodyComponent body1;
    body1.isStatic = true;
    body1.UpdateMassProperties();

    ECS::ColliderComponent::PolygonShape shape1({
        { -250.0f, -25.0f },
        {  250.0f, -25.0f },
        {  250.0f,  25.0f },
        { -250.0f,  25.0f }
    });

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
    
    // Slope 2: mirrored, positioned below and to the right of main platform, rotated +25 degrees
    ECS::EntityID slope2Entity = entities.CreateEntity();

    ECS::TransformComponent t2;
    t2.position         = { 1130.0f, 80.0f };  // Below main platform (y=100), right side
    t2.previousPosition = t2.position;
    // Rotate by +25 degrees (about 0.436 radians)
    t2.rotation         = 0.436332f;
    t2.previousRotation = t2.rotation;

    ECS::PhysicsBodyComponent body2;
    body2.isStatic = true;
    body2.UpdateMassProperties();

    ECS::ColliderComponent::PolygonShape shape2({
        { -250.0f, -25.0f },
        {  250.0f, -25.0f },
        {  250.0f,  25.0f },
        { -250.0f,  25.0f }
    });

    ECS::ColliderComponent collider2(shape2);
    collider2.material.friction    = 0.7f;  // Higher friction for contrast
    collider2.material.restitution = 0.8f;
    collider2.material.density     = 0.0f;

    ECS::RenderComponent render2({ 500.0f, 50.0f }, { 0.8f, 0.4f, 0.4f });
    render2.origin = { 250.0f, 25.0f };

    cs.AddComponent(slope2Entity, std::move(t2));
    cs.AddComponent(slope2Entity, std::move(body2));
    cs.AddComponent(slope2Entity, std::move(collider2));
    cs.AddComponent(slope2Entity, std::move(render2));
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
    // Circle with radius 25px (equivalent to the previous 50x50 square's half-extent)
    ECS::ColliderComponent::CircleShape playerShape;
    playerShape.center = { 0.0f, 0.0f };
    playerShape.radius = 25.0f;

    ECS::ColliderComponent playerCollider(playerShape);
    playerCollider.material.friction    = 0.4f;
    playerCollider.material.restitution = 0.8f;  // Low restitution for better control
    playerCollider.material.density     = 0.08f;

    // ── physics body ─────────────────────────────────────────────────────────
    ECS::PhysicsBodyComponent body;

    // Calculate inertia from collider shape
    {
        float area    = playerCollider.CalculateArea();           // π * r²
        float density = (area > 0.0f) ? body.mass / area : 0.0f;
        body.SetMass(5.0f);
        body.SetInertia(playerCollider.CalculateInertiaPerUnitMass() * body.mass);
    }

    // Set damping for better control
    body.angularDamping = 0.1f;   // higher damping for player to prevent spinning
    body.drag           = 0.01f;  // small linear air resistance

    body.UpdateMassProperties();
    body.isAwake    = true;
    body.allowSleep = false;  // Player should never sleep

    // Lock rotation for player to prevent unwanted spinning
    body.motionLocks.lockRotation = false;

    // ── render component ─────────────────────────────────────────────────────
    // Render as a circle: width/height = diameter, origin = radius
    ECS::RenderComponent playerRender({ 50.0f, 50.0f }, { 1.0f, 0.5f, 0.0f });  // Orange color
    playerRender.origin      = { 25.0f, 25.0f };
    playerRender.shapeType   = ECS::RenderComponent::ShapeType::Circle;

    // ── behavior component for input handling ────────────────────────────────
    ECS::BehaviorComponent behavior;
    
    cs.AddComponent(m_PlayerEntity, std::move(t));
    cs.AddComponent(m_PlayerEntity, std::move(body));
    cs.AddComponent(m_PlayerEntity, std::move(playerCollider));
    cs.AddComponent(m_PlayerEntity, std::move(playerRender));
    cs.AddComponent(m_PlayerEntity, std::move(behavior));

    std::cerr << "[DEMO] Player circle created at (640, 360). "
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
    spawnCollider.material.restitution = 0.8f;
    spawnCollider.material.density     = 0.0008f;

    // ── physics body ─────────────────────────────────────────────────────────
    ECS::PhysicsBodyComponent body;

    {
        float area    = spawnCollider.CalculateArea();
        body.SetMass(2.0f);
        body.SetInertia(spawnCollider.CalculateInertiaPerUnitMass() * body.mass);
    }

    // Set damping to prevent erratic spinning
    body.angularDamping = 0.05f;  // gentle continuous spin decay
    body.drag           = 0.01f;  // small linear air resistance

    body.UpdateMassProperties();
    body.isAwake    = true;
    body.allowSleep = true;

    // Lock rotation to prevent spinning
    body.motionLocks.lockRotation = false;

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
        // Apply impulse by changing velocity directly, taking into account the mass
        body.velocity.y += PLAYER_JUMP_FORCE / body.mass;
        body.SetAwake(true);
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
    std::uniform_real_distribution<float> radiusDist(10.0f, 20.0f);
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
    circleCollider.material.restitution = 0.8f;
    circleCollider.material.density     = 0.0008f;

    // Physics body
    ECS::PhysicsBodyComponent body;
    
    // Calculate inertia from collider shape
    {
        float area = circleCollider.CalculateArea();
        body.SetMass(1.0f);
        body.SetInertia(circleCollider.CalculateInertiaPerUnitMass() * body.mass);
    }
    
    // Set damping to prevent erratic spinning
    body.angularDamping = 0.05f;  // gentle continuous spin decay
    body.drag           = 0.01f;  // small linear air resistance
    
    body.UpdateMassProperties();
    body.isAwake = true;
    body.allowSleep = true;

    // Lock rotation to prevent spinning
    body.motionLocks.lockRotation = false;

    // Generate random color for this circle
    std::uniform_real_distribution<float> colorDist(0.0f, 1.0f);
    float r = colorDist(m_Rng);
    float g = colorDist(m_Rng);
    float b = colorDist(m_Rng);

    // Render
    ECS::RenderComponent render({ radius * 2.0f, radius * 2.0f }, { r, g, b });
    render.origin = { radius, radius };
    render.shapeType = ECS::RenderComponent::ShapeType::Circle;

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
    
        // manifold.normal points FROM A TOWARD B (i.e., from player into platform when player is A).
        // To get the "surface normal pointing up toward player" direction:
        //   - If player is A: negate (flip A→B downward to B→A upward)
        //   - If player is B: use as-is (A→B points upward from platform toward player)
        Math::Vector2 contactNormal = isPlayerA ? -manifold.normal : manifold.normal;
    
        // Consider it "ground" if the contact normal has a large upward component
        float dotUp = Math::Vector2::Dot(contactNormal, up);
        if (dotUp > 0.7f)
        {
            // Also ensure the contact points are not far apart (i.e., actually touching)
            for (const auto& pt : manifold.points)
            {
                if (pt.separation < 1.0f)
                {
                    return true;
                }
            }
        }
    }

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

// ============================================================================
//  DespawnOutOfBoundsObjects  –  remove objects that fall below the screen
// ============================================================================
void SimplePhysicsDemo::DespawnOutOfBoundsObjects()
{
    auto& cs = GetComponentStore();
    
    // Collect entities to despawn
    std::vector<ECS::EntityID> entitiesToDespawn;
    
    for (const auto& entityId : m_SpawnedQuads)
    {
        if (!cs.HasComponent<ECS::TransformComponent>(entityId))
            continue;
            
        const auto& transform = cs.GetComponent<ECS::TransformComponent>(entityId);
        
        // Check if entity has fallen below the despawn threshold
        if (transform.position.y < DESPAWN_Y_THRESHOLD)
        {
            entitiesToDespawn.push_back(entityId);
            std::cerr << "[DEMO] Despawning entity at y=" << transform.position.y 
                      << " (below threshold " << DESPAWN_Y_THRESHOLD << ")\n";
        }
    }
    
    // Remove entities from component store and tracking list
    for (const auto& entityId : entitiesToDespawn)
    {
        // Destroy the entity (this removes all its components)
        GetEntityManager().DestroyEntity(entityId);
        
        // Remove from tracking list
        m_SpawnedQuads.erase(
            std::remove(m_SpawnedQuads.begin(), m_SpawnedQuads.end(), entityId),
            m_SpawnedQuads.end()
        );
    }
    
    if (!entitiesToDespawn.empty())
    {
        std::cerr << "[DEMO] Despawned " << entitiesToDespawn.size() << " out-of-bounds entities\n";
    }
}