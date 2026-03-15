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
    CreateFallingBox();
}

// ============================================================================
//  OnECSFixedUpdate  –  runs at exactly 1/60 s per tick
// ============================================================================
void SimplePhysicsDemo::OnECSFixedUpdate(float deltaTime)
{
    // ── one-time debug renderer configuration ────────────────────────────────
    // Systems are registered after OnECSStart returns, so GetDebugRenderSystem()
    // is safe to call here (from the very first tick onwards).
    if (!m_DebugConfigured)
    {
        if (auto* dbg = GetDebugRenderSystem())
        {
            // shapes=true  joints=false  aabbs=false  contacts=true  com=false
            dbg->SetFlags(true, false, false, true, false);
        }
        m_DebugConfigured = true;
    }

    // ── advance simulation clock ──────────────────────────────────────────────
    m_SimTime += deltaTime;

    // ── periodic box position logging ────────────────────────────────────────
    if (m_SimTime >= m_NextLogTime)
    {
        m_NextLogTime += LOG_INTERVAL_S;

        auto& cs = GetComponentStore();
        if (cs.HasComponent<ECS::TransformComponent>(m_BoxEntity))
        {
            const auto& t  = cs.GetComponent<ECS::TransformComponent>(m_BoxEntity);
            const auto& rb = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BoxEntity);
            std::cerr << std::fixed << std::setprecision(2)
                      << "[DEMO t=" << m_SimTime << "s]"
                      << "  box pos=(" << t.position.x << ", " << t.position.y << ")"
                      << "  vel=("     << rb.velocity.x << ", " << rb.velocity.y << ")"
                      << "  awake="    << rb.isAwake
                      << "\n";
        }
    }

    // ── collision detection via PhysicsWorldComponent contact manifolds ───────
    // Requires Bug 2 fix (NarrowPhaseDetection must populate
    // m_PhysicsWorld->contactManifolds).  After that fix this will correctly
    // trigger the first time the box and platform contact manifold is generated.
    if (!m_CollisionDetected)
    {
        auto& cs            = GetComponentStore();
        const auto& worlds  = cs.GetEntitiesWithComponent<ECS::PhysicsWorldComponent>();

        if (!worlds.empty())
        {
            const auto& world = cs.GetComponent<ECS::PhysicsWorldComponent>(worlds[0]);

            for (const auto& manifold : world.contactManifolds)
            {
                if (!manifold.touching)
                    continue;

                // Confirm it involves our two entities (order may vary).
                bool involvesPlatform =
                    (manifold.entityIdA == m_PlatformEntity ||
                     manifold.entityIdB == m_PlatformEntity);
                bool involvesBox =
                    (manifold.entityIdA == m_BoxEntity ||
                     manifold.entityIdB == m_BoxEntity);

                if (involvesPlatform && involvesBox)
                {
                    m_CollisionDetected  = true;
                    m_FirstCollisionTime = m_SimTime;

                    std::cerr << "[DEMO t=" << m_SimTime << "s]"
                              << "  COLLISION DETECTED  "
                              << "  contact points=" << manifold.points.size()
                              << "  normal=("
                              << manifold.normal.x << ", "
                              << manifold.normal.y << ")\n";
                    break;
                }
            }
        }
    }

    // ── report resting state ─────────────────────────────────────────────────
    // Once the box is known to be sleeping (fully settled on the platform)
    // print a final summary.
    if (m_CollisionDetected)
    {
        auto& cs = GetComponentStore();
        if (cs.HasComponent<ECS::PhysicsBodyComponent>(m_BoxEntity))
        {
            const auto& rb = cs.GetComponent<ECS::PhysicsBodyComponent>(m_BoxEntity);
            if (!rb.isAwake)
            {
                const auto& t = cs.GetComponent<ECS::TransformComponent>(m_BoxEntity);
                std::cerr << "[DEMO t=" << m_SimTime << "s]"
                          << "  BOX AT REST"
                          << "  final pos=(" << t.position.x << ", " << t.position.y << ")\n";
            }
        }
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
    // Y = 70 places it near the bottom of a 720-pixel-high window.
    ECS::TransformComponent t;
    t.position         = { 640.0f, 70.0f };
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
//  CreateFallingBox  –  small dynamic box that falls onto the platform
// ============================================================================
void SimplePhysicsDemo::CreateFallingBox()
{
    auto& entities = GetEntityManager();
    auto& cs       = GetComponentStore();

    m_BoxEntity = entities.CreateEntity();

    // ── transform ────────────────────────────────────────────────────────────
    // Start at Y = 500 (near the top of the window in Y-up space).
    // The platform top surface is at Y = 70 + 25 = 95.
    // The box bottom edge starts at Y = 500 - 25 = 475, so there is a
    // 380-pixel gap.  At 980 px/s² the box takes ~0.88 s to reach the platform.
    ECS::TransformComponent t;
    t.position         = { 640.0f, 500.0f };
    t.previousPosition = t.position;
    t.rotation         = 0.0f;
    t.previousRotation = 0.0f;

    // ── collider (set up FIRST so we can derive inertia from it) ─────────────
    // 50×50 square, centered on the origin.
    // Winding: same CCW pattern as platform → correct outward normals after fix.
    ECS::ColliderComponent::PolygonShape boxShape({
        { -25.0f, -25.0f },
        {  25.0f, -25.0f },
        {  25.0f,  25.0f },
        { -25.0f,  25.0f }
    });

    ECS::ColliderComponent boxCollider(boxShape);
    boxCollider.material.friction    = 0.4f;
    boxCollider.material.restitution = 0.2f;
    boxCollider.material.density     = 0.0008f; // mass / area = 2 / 2500

    // ── physics body ─────────────────────────────────────────────────────────
    ECS::PhysicsBodyComponent body;
    body.mass = 2.0f;

    // CRITICAL (Bug B1): the engine never computes inertia from the collider
    // shape — it defaults to 1.0, giving every body identical (wrong) rotational
    // resistance regardless of size or density.
    //
    // We compute it manually here using the collider's own methods so that the
    // result is correct for any shape the collider holds:
    //
    //   density = mass / area
    //   inertia = CalculateInertiaForUnitDensity() * density
    //
    // For a 50×50 square:
    //   area    = 2500 px²
    //   density = 2.0 / 2500 = 0.0008
    //   CalculateInertiaForUnitDensity() ≈ 1,041,667   (shoelace integral)
    //   inertia ≈ 833.33 px⁴·kg         (= mass/12 * (w² + h²))
    //   inverseInertia ≈ 0.0012
    {
        float area    = boxCollider.CalculateArea();           // 2500.0 px²
        float density = (area > 0.0f) ? body.mass / area : 0.0f;
        body.inertia  = boxCollider.CalculateInertiaForUnitDensity() * density;
    }

    // UpdateMassProperties reads body.mass and body.inertia and sets:
    //   inverseMass    = 1 / mass    = 0.5
    //   inverseInertia = 1 / inertia ≈ 0.0012
    body.UpdateMassProperties();

    body.isAwake    = true;
    body.allowSleep = true;

    // ── render component ─────────────────────────────────────────────────────
    // Same pivot correction as the platform: origin = half-size centres the
    // quad on transform.position, matching the physics polygon.
    ECS::RenderComponent boxRender({ 50.0f, 50.0f }, { 0.25f, 0.45f, 1.0f });
    boxRender.origin = { 25.0f, 25.0f };

    cs.AddComponent(m_BoxEntity, std::move(t));
    cs.AddComponent(m_BoxEntity, std::move(body));
    cs.AddComponent(m_BoxEntity, std::move(boxCollider));
    cs.AddComponent(m_BoxEntity, std::move(boxRender));

    std::cerr << "[DEMO] Box created at y=500. "
              << "Platform top surface at y=" << (70.0f + 25.0f) << ". "
              << "Box will land at box-centre y=" << (70.0f + 25.0f + 25.0f)
              << " (~0.88 s at 980 px/s²).\n";
}