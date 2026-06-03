# Nyon Engine — Comprehensive Engineering Report
**Target Audience:** Agentic AI Implementation Engineer  
**Codebase:** `daiyaanmuhammadfardeen-nyon` (post-GPU-renderer / post-multithreaded-physics iteration)  
**Date:** 2026-03-17  
**Severity Scale:** 🔴 Critical (crash/data-race/wrong-output) · 🟠 High (wrong behaviour/major leak) · 🟡 Medium (architectural debt/performance) · 🔵 Low (cosmetic/cleanup)

---

## Table of Contents

1. [File System & Structure Issues](#1-file-system--structure-issues)
2. [Dead Code & Backup Artifacts](#2-dead-code--backup-artifacts)
3. [Particle System — Architecture Overhaul Required](#3-particle-system--architecture-overhaul-required)
4. [Physics Pipeline — Bugs & Thread Safety](#4-physics-pipeline--bugs--thread-safety)
5. [Renderer2D — Logic Bugs & Residual Backup Code](#5-renderer2d--logic-bugs--residual-backup-code)
6. [ECSApplication — Lifecycle & Ordering Bugs](#6-ecsapplication--lifecycle--ordering-bugs)
7. [ComponentStore & EntityManager](#7-componentstore--entitymanager)
8. [ThreadPool](#8-threadpool)
9. [Debug Systems](#9-debug-systems)
10. [Input System](#10-input-system)
11. [GPU Parallelisation Opportunities (CPU Fallbacks)](#11-gpu-parallelisation-opportunities-cpu-fallbacks)
12. [Implementation Priority Order](#12-implementation-priority-order)

---

### 2.5 🟡 `PhysicsBodyComponent::ShouldCollide()` — hardcoded stub

**File:** `engine/include/nyon/ecs/components/PhysicsBodyComponent.h`

```cpp
bool ShouldCollide() const { return true; }
```

Collision layer/mask filtering is never applied. The method is not called anywhere in the codebase.

**Action:** Either implement collision filtering (add `uint32_t collisionCategory = 0x0001; uint32_t collisionMask = 0xFFFF;` to both `PhysicsBodyComponent` and `ColliderComponent`, and wire it into `PhysicsPipelineSystem::TestCollision`) or remove the stub entirely.

---

### 2.6 🟡 `DebugRenderSystem::DrawJoints()` and `DrawIslands()` — empty stubs

**File:** `engine/src/ecs/systems/DebugRenderSystem.cpp`

```cpp
void DebugRenderSystem::DrawJoints() { }
void DebugRenderSystem::DrawIslands() { }
```

Both are called conditionally (`if (m_DrawJoints)`, `if (m_DrawIslands)`) but do nothing. `m_DrawJoints` defaults to `true` in the header, meaning joint drawing is "enabled" but silent.

**Action:** Either implement them or set their corresponding flags to `false` by default and leave a `// TODO` comment. Do not leave silently-enabled-but-empty rendering paths.

---

### 2.7 🟡 `NyonDebugRenderer::DrawSolidPolygon` — declared, not implemented

**File:** `engine/include/nyon/ecs/systems/DebugRenderSystem.h`

`DrawSolidPolygon` is declared in the nested `NyonDebugRenderer` class but has no definition in `DebugRenderSystem.cpp`. This will cause a linker error the moment it is called.

**Action:** Implement it by forwarding to `Graphics::Renderer2D::DrawSolidPolygon()`.

---

### 2.8 🟡 `ManifoldGenerator::SegmentCollision` — always returns empty manifold

**File:** `engine/src/physics/ManifoldGenerator.cpp`

```cpp
ECS::ContactManifold ManifoldGenerator::SegmentCollision(...) {
    manifold.touching = false;
    ...
    return manifold;
}
```

Segment vs. any shape is silently unresolved. Any entity with a `SegmentShape` collider will never generate contacts.

**Action:** Implement segment-circle, segment-polygon, and segment-capsule collision, or throw a `std::logic_error("SegmentCollision not implemented")` to surface the gap clearly during development.

---

### 2.9 🔵 `#include <algorithm>` in `ComponentStore.h` — unnecessary public header pollution

**File:** `engine/include/nyon/ecs/ComponentStore.h`

```cpp
#include <algorithm>   // comment says "for std::remove"
```

`std::remove` is only used inside `EntityManager.cpp`, not in `ComponentStore.h`. This forces all consumers of `ComponentStore.h` to transitively compile `<algorithm>`.

**Action:** Remove from the header. Add to `EntityManager.cpp`.

---

## 3. Particle System — Architecture Overhaul Required

This is the most significant area of work. The current particle system is an isolated demo utility, not an engine feature. It has no physics integration, no developer API, and critical hardcoded values.

---

### 3.1 🔴 Hardcoded 1280×720 viewport in `ParticleRenderSystem::Render`

**File:** `engine/src/ecs/systems/ParticleRenderSystem.cpp`

```cpp
glm::mat4 vp = glm::ortho(0.0f, 1280.0f, 0.0f, 720.0f, -1.0f, 1.0f);
m_ParticleRenderer->Flush(vp);
```

This is hardcoded to a specific resolution. If the window is any other size, particles will be rendered at completely wrong screen positions. The `Renderer2D` system uses a proper VP uniform — particles bypass this entirely.

**Action:** `ParticleRenderSystem` must receive the engine's shared VP matrix. Options:
- Pass it via a `SetViewProjection(const glm::mat4&)` call from `ECSApplication::OnInterpolateAndRender`.
- Read it from a `CameraComponent` entity (preferred for extensibility).
- At minimum, query `Application::Get()` for the window width/height and construct ortho from that dynamically.

---

### 3.2 🔴 `ParticleRenderSystem::Render` ignores the `alpha` interpolation parameter

**File:** `engine/src/ecs/systems/ParticleRenderSystem.cpp`

```cpp
void ParticleRenderSystem::Render(float alpha) {
    ...
    for (const auto& p : m_Particles) {
        m_ParticleRenderer->SubmitCircle(p.x, p.y, ...);  // raw current position, no interpolation
    }
}
```

The `alpha` parameter exists precisely to lerp between `previousPosition` and `position` for smooth sub-frame rendering. All other render systems (including `RenderSystem` and `DebugRenderSystem`) correctly call `transform.GetInterpolatedPosition(alpha)`. The particle system does not, causing visible jitter at any framerate that differs from the physics rate.

**Action:** The `Particle` struct must gain `prevX, prevY` fields. `ParticleRenderSystem::Render` must interpolate: `float rx = p.prevX + (p.x - p.prevX) * alpha`.

---

### 3.3 🔴 `SetParticles` copies the full particle vector every frame

**File:** `engine/src/ecs/systems/ParticleRenderSystem.cpp`

```cpp
void ParticleRenderSystem::SetParticles(const std::vector<Nyon::Particle>& particles) {
    m_Particles = particles;  // full deep copy every frame
}
```

For 1000 particles (the demo), this is 1000 × ~40 bytes = ~40 KB copied every frame. For the stated `ParticleRenderer::MAX_PARTICLES = 4'000'000`, this would be ~160 MB/frame — catastrophic.

**Action:** The particle system must own its data. See §3.5 for the full redesign. At minimum, change to a `std::span` or pointer handoff pattern. The system should update its own particles, not receive a copy from the game layer.

---

### 3.4 🟠 Particle physics lives in the game demo, not in the engine

**File:** `game/particle-collision-demo/src/ParticleCollisionDemo.cpp`

`UpdateParticles()` and `HandleCollisions()` are custom-written in the demo. The engine's `PhysicsPipelineSystem` is completely unaware of particles. This means:

- Particles do not collide with physics bodies (walls, shapes, etc.)
- Particles ignore the `PhysicsWorldComponent` gravity setting
- There is no spatial broadphase acceleration in the engine for particles; the demo re-implements a grid from scratch every frame (3D vector-of-vectors allocated on the heap each call)

**Action:** Design a `ParticleWorldComponent` and a `ParticlePipelineSystem` that runs particle physics in parallel using the `ThreadPool`. Particles should query the engine's `DynamicTree` for nearby physics bodies to enable particle-body interaction. See §3.5 for the full architecture.

---

### 3.5 🟠 `Particle` struct has no developer customizability

**File:** `engine/include/nyon/Particle.h`

```cpp
struct Particle {
    float x, y, vx, vy, radius, r, g, b, mass;
};
```

This is a plain data struct with no support for: lifetime, opacity/alpha, custom per-particle properties, emitter association, or physics material. There is no API for developers to hook particle behaviour.

**Action — Full Particle System Architecture:**

**A. Redesign `Particle` as a SoA-friendly struct:**

```cpp
// engine/include/nyon/ecs/components/ParticleComponent.h
struct Particle {
    // Kinematics (updated by ParticlePipelineSystem)
    float x, y;
    float prevX, prevY;       // for interpolation
    float vx, vy;
    float radius;
    float mass;
    float invMass;            // precomputed
    // Visual
    float r, g, b, a;        // alpha support
    // Lifecycle
    float lifetime;           // seconds remaining; negative = eternal
    float age;                // seconds since spawn
    // Material
    float restitution;
    float friction;
    float drag;
    // Emitter reference
    uint32_t emitterEntityId; // which emitter owns this particle
    // Developer payload
    uint64_t userData;        // opaque per-particle data
};
```

**B. Create `ParticleEmitterComponent`:**

```cpp
struct ParticleEmitterComponent {
    // Spawn parameters
    float spawnRate = 10.0f;           // particles/second (0 = burst only)
    uint32_t burstCount = 0;
    float spawnTimer = 0.0f;
    uint32_t maxParticles = 1000;
    bool loop = true;
    bool active = true;

    // Initial conditions (ranges, sampled at spawn time)
    float minSpeed = 50.0f, maxSpeed = 200.0f;
    float minAngle = 0.0f, maxAngle = 360.0f; // degrees
    float minRadius = 4.0f, maxRadius = 16.0f;
    float minLifetime = 1.0f, maxLifetime = 3.0f;
    float minMass = 1.0f, maxMass = 10.0f;

    // Spawn-time colour range
    Math::Vector3 colorStart = {1,1,1};
    Math::Vector3 colorEnd   = {1,1,1};

    // Developer hooks (all optional; called per-particle)
    std::function<void(Particle&)>        onSpawn;
    std::function<void(Particle&, float)> onUpdate;   // called after physics step
    std::function<void(Particle&)>        onDeath;

    // Physics interaction
    bool affectedByGravity = true;
    bool collidesWithBodies = false;   // uses DynamicTree broadphase
    bool collidesWithParticles = true;
};
```

**C. Create `ParticlePipelineSystem`** (replaces game-level `HandleCollisions`):

Responsibilities:
1. Spawn new particles from `ParticleEmitterComponent` entities.
2. Apply gravity from `PhysicsWorldComponent`.
3. Integrate velocities (parallel via `ThreadPool`, each thread owns a contiguous range of particles).
4. Spatial hashing / grid broadphase for particle-particle collisions (port from demo but make it a member, not a heap-allocated local).
5. Query `DynamicTree` for particle-body collisions (optional, guarded by `collidesWithBodies`).
6. Apply drag and damping.
7. Tick down lifetimes and kill dead particles (swap-remove into a free list).
8. Invoke developer `onUpdate` callbacks.

**D. `ParticleRenderSystem`** becomes a pure GPU-submission layer:

- Reads from the `ParticlePipelineSystem`'s output particle buffer (by pointer/span, no copy).
- Applies interpolation using `prevX/prevY` and the `alpha` parameter.
- Submits to `ParticleRenderer::Flush`.
- Uses the engine's shared VP matrix.

---

### 3.6 🟠 `ParticleCollisionDemo` spatial hash is reallocated from heap every frame

**File:** `game/particle-collision-demo/src/ParticleCollisionDemo.cpp`

```cpp
std::vector<std::vector<std::vector<size_t>>> grid(GRID_WIDTH, std::vector<...>(GRID_HEIGHT));
```

This allocates a 3D vector hierarchy (26 × 15 = 390 `std::vector` objects) on the heap every call to `HandleCollisions()`. With 1000 particles at 60 Hz this is ~23,400 heap allocations per second.

**Action:** Once `ParticlePipelineSystem` is introduced (§3.5-C), it should hold a reusable spatial hash as a member: a flat `std::vector<uint32_t>` with a separate `std::vector<uint32_t> cellStart` index, rebuilt each step but never reallocated unless particle count grows.

---

### 3.7 🟡 Variable shadowing in `HandleCollisions` — confusing but not a crash

**File:** `game/particle-collision-demo/src/ParticleCollisionDemo.cpp`

```cpp
for (int nx = cx + dx; ...) {          // outer: int nx (cell neighbour)
    for (size_t i ...) {
        float nx = dx_pos / dist;       // inner: float nx (collision normal)
        float ny = dy_pos / dist;
```

The outer loop variable `int nx` (cell index) is shadowed by `float nx` (normal component). They are in separate scopes and the compiler will not error, but this is a maintenance hazard and static analysers will flag it. The normal vector components should be renamed `normalX / normalY`.

---

### 3.8 🟡 Particle colours all default to white in demo

**File:** `game/particle-collision-demo/src/ParticleCollisionDemo.cpp`

```cpp
p.r = 1.0f; p.g = 1.0f; p.b = 1.0f;
```

The colour distribution is not used despite a `std::uniform_real_distribution<float> color(0.0f, 1.0f)` being declared but never applied to `r/g/b`. This is dead code in the initialiser.

**Action:** Apply the colour distribution: `p.r = color(m_Rng); p.g = color(m_Rng); p.b = color(m_Rng);`

---

## 4. Physics Pipeline — Bugs & Thread Safety

### 4.1 🔴 `ParallelVelocitySolving` uses `FIXED_TIMESTEP` instead of `subStepDt`

**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`

```cpp
// In the parallel lambda:
IntegrateVelocities(FIXED_TIMESTEP, start, end);  // BUG: ignores subStepDt
```

The outer loop computes `float subStepDt = deltaTime / numSubSteps;` and calls `IntegratePositions(subStepDt)` for sequential mode, but the parallel path always passes `FIXED_TIMESTEP`. When `numSubSteps == 2` (which happens when any body exceeds 400 units/s), the parallel path integrates at twice the intended rate, producing incorrect velocities.

**Action:** Capture `subStepDt` in the lambda:

```cpp
futures.push_back(Utils::ThreadPool::Instance().Submit([this, start, end, subStepDt]() {
    ...
    IntegrateVelocities(subStepDt, start, end);
}));
```

---

### 4.2 🔴 `m_ComponentStore->GetComponent<PhysicsWorldComponent>` called from multiple threads without synchronisation

**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`

Inside the `ParallelVelocitySolving` thread lambda:

```cpp
const auto& world = m_ComponentStore->GetComponent<PhysicsWorldComponent>(m_PhysicsWorldEntity);
body.force += world.gravity * mass;
```

`m_ComponentStore->GetComponent` performs an `unordered_map` lookup and pointer dereference. Multiple worker threads call this simultaneously, and the underlying `ComponentContainer::indexMap` is a non-atomic `std::unordered_map`. Even if the map is not modified, concurrent reads of `std::unordered_map` are only safe if **no thread is writing**. Since gravity is constant per frame, cache the world component pointer before spawning threads.

**Action:** Before the parallel section, read `world.gravity` into a local `Math::Vector2 cachedGravity` and capture it in the lambda by value.

---

### 4.3 🔴 `DynamicTree::Query` called from multiple threads — thread safety unverified

**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp` → `ParallelBroadPhase`

```cpp
m_BroadPhaseTree.Query(fatAABB, &callback);  // called from N threads simultaneously
```

`m_BroadPhaseTree` is a single shared `Physics::DynamicTree` instance. `Query` traverses the tree structure. Unless `DynamicTree` explicitly documents its `Query` method as safe for concurrent readers, and no writer can modify the tree during traversal, this is a data race. The tree is also written to in the same function via `UpdateShapeAABB` which calls `CreateProxy`/`MoveProxy` — but those happen before the parallel section. Audit `DynamicTree::Query` to confirm it is read-only. If it is, add a `// Thread-safe (read-only traversal)` comment. If it is not, add a `std::shared_mutex` and use shared locks in `Query` / exclusive locks in mutations.

---

### 4.4 🔴 `PrepareBodiesForUpdate` modifies component data during `ForEachComponent` iteration

**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`

```cpp
m_ComponentStore->ForEachComponent<PhysicsBodyComponent>([&](EntityID entityId, PhysicsBodyComponent& body) {
    ...
    body.SetMass(calculatedMass);    // Writes back into the container being iterated
    body.SetInertia(calculatedInertia);
    body.ClearForces();
```

`SetMass` and `SetInertia` modify the `body` reference, which is directly the component stored in the container (passed by non-const reference). `ForEachComponent` iterates over `container.indexMap` (an `unordered_map`). Modifying the component data itself is safe (it doesn't change the map), but any future path that calls `RemoveComponent` or `AddComponent` inside the callback would invalidate the iterator. Document this constraint clearly. Current code is safe but fragile.

**Action:** Add a `static_assert` or comment at the callback site warning that no add/remove operations are permitted inside `ForEachComponent` callbacks. Consider a dedicated `RecalculateMassProperties` function called once at entity creation.

---

### 4.5 🟠 Duplicate `FIXED_TIMESTEP` constant — risk of divergence

**Files:**  
- `engine/include/nyon/core/Application.h`: `static constexpr double FIXED_TIMESTEP = 1.0 / 60.0;`  
- `engine/include/nyon/ecs/systems/PhysicsPipelineSystem.h`: `static constexpr float FIXED_TIMESTEP = 1.0f / 60.0f;`

Two different types (`double` vs `float`), two different scopes, same semantic meaning. If one is ever changed, the other will silently diverge.

**Action:** Create `engine/include/nyon/EngineConstants.h`:

```cpp
namespace Nyon {
    inline constexpr double FIXED_TIMESTEP_D = 1.0 / 60.0;
    inline constexpr float  FIXED_TIMESTEP   = 1.0f / 60.0f;
    inline constexpr float  MAX_FRAME_TIME   = 0.25f;
}
```

Remove the local definitions from both files and include the central header.

---

### 4.6 🟠 Duplicate `ContactPoint` / `ContactManifold` struct definitions

Three separate definitions exist for what is logically the same type:

| Location | Type | Fields |
|----------|------|--------|
| `PhysicsWorldComponent.h` | `ECS::ContactPoint`, `ECS::ContactManifold` | Public-facing, includes callback info |
| `PhysicsPipelineSystem.h` | Private `ContactPoint`, `ContactManifold` | Has `velocityBias`, lacks `localNormal/localPoint` |
| `ManifoldGenerator.h/cpp` | Uses `ECS::ContactManifold` | — |

`PhysicsPipelineSystem::NarrowPhaseDetection()` manually copies fields from the internal `ContactManifold` to `ECS::ContactManifold` (lines 3299–3311). This is error-prone — any new field added to one must be manually synced to the other.

**Action:** Consolidate into a single canonical type in `engine/include/nyon/physics/ContactTypes.h`. The internal solver-only field `velocityBias` can be a separate `SolverContactPoint` that embeds `ContactPoint`.

---

### 4.7 🟠 `UpdateGroundedState` is never called

**File:** `engine/include/nyon/ecs/components/PhysicsBodyComponent.h`

`UpdateGroundedState(bool currentlyGrounded)` and the `isGrounded` / `groundedFrames` fields exist but `PhysicsPipelineSystem` never calls this method. The `isGrounded` flag is always its default value (`false`), making it useless for any game logic that relies on it.

**Action:** Call `body.UpdateGroundedState(hasGroundContact)` at the end of each physics step in `PhysicsPipelineSystem`, where `hasGroundContact` is derived from whether the body has a contact normal with a significant upward component.

---

### 4.8 🟡 Substep adaptive logic only goes to 2 substeps

**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`

```cpp
int numSubSteps = 1;
if (std::sqrt(maxSpeedSquared) > SUBSTEP_SPEED_THRESHOLD) {
    numSubSteps = 2;
}
```

The `PhysicsWorldComponent` already has a `subStepCount` field, but the pipeline ignores it entirely and uses this ad hoc threshold instead. For very fast-moving bullet bodies (`isBullet = true`), 2 substeps is often insufficient for CCD.

**Action:** Use `physicsWorld.subStepCount` as the base, and clamp max substeps to a configurable value (e.g. 8). Remove the `SUBSTEP_SPEED_THRESHOLD` magic number.

---

### 4.9 🟡 `PhysicsPipelineSystem::m_NumThreads` may be 0 after initialization

**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`

```cpp
m_NumThreads = Utils::ThreadPool::Instance().GetThreadCount();
```

`ThreadPool` does not expose a `GetThreadCount()` in its header as seen — if the method doesn't exist or returns 0, the parallel batch-size calculation `(m_SolverBodies.size() + m_NumThreads - 1) / m_NumThreads` performs a division by zero.

**Action:** Add `GetThreadCount()` to `ThreadPool.h` returning `m_Workers.size()`. Add a `NYON_ASSERT(m_NumThreads > 0)` guard in the parallel paths.

---

### 4.10 🟡 Manifold key collision: entity ID pair could alias

**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`

```cpp
uint64_t key = (static_cast<uint64_t>(std::min(entityIdA, entityIdB)) << 32) |
               static_cast<uint64_t>(std::max(entityIdA, entityIdB));
```

This is fine as long as entity IDs are 32-bit. Since `EntityID` is likely `uint32_t`, this is safe. However, `MakeImpulseCacheKey` adds a `featureId` parameter:

```cpp
uint64_t MakeImpulseCacheKey(uint32_t entityIdA, uint32_t entityIdB, uint32_t featureId) const;
```

But the above contact map key does NOT include `featureId`. Two contacts between the same entity pair (e.g. a polygon-polygon collision with multiple contact points) will overwrite each other in `m_ContactMap`. Only the last manifold survives.

**Action:** Include the shape IDs in the contact map key, or switch from a flat `ContactManifold` per pair to a `std::vector<ContactManifold>` per pair.

---

## 5. Renderer2D — Logic Bugs & Residual Backup Code

### 5.1 🟠 `DrawSolidPolygon` has reversed winding order

**File:** `engine/src/graphics/Renderer2D.cpp`

```cpp
s_Instance->PushPolyFillVert(v0.x, v0.y, cr, cg, cb);
s_Instance->PushPolyFillVert(v2.x, v2.y, cr, cg, cb);   // ← v2 BEFORE v1
s_Instance->PushPolyFillVert(v1.x, v1.y, cr, cg, cb);
```

A triangle fan from `v0` should be `(v0, v1, v2)` in CCW order (OpenGL default front-face). This emits `(v0, v2, v1)` — CW winding. If back-face culling (`glEnable(GL_CULL_FACE)`) is ever enabled, all filled polygons will be invisible. Even without culling, the winding is semantically wrong and will produce incorrect normals if lighting is added.

**Action:** Swap to `PushPolyFillVert(v0), PushPolyFillVert(v1), PushPolyFillVert(v2)`.

---

### 5.2 🟠 `RenderSystem::Update` debug counter prints stale value

**File:** `engine/include/nyon/ecs/systems/RenderSystem.h`

```cpp
static int s_RenderDebugCounter = 0;
if (++s_RenderDebugCounter >= 10) {
    s_RenderDebugCounter = 0;  // reset BEFORE printing
    std::cerr << "[RENDER@" << s_RenderDebugCounter << "] ...";  // always prints 0
}
```

The counter is reset to 0 before being used in the format string, so it always prints `[RENDER@0]`. This makes the log spam rather than the intended periodic report.

**Action:** Print before resetting, or use a separate variable for the display value.

---

### 5.3 🟡 No shared camera / VP matrix — each renderer hardcodes its own

Four places in the codebase construct or imply a VP matrix independently:
- `Renderer2D::EndScene` (applies `u_VP` via OpenGL uniform)
- `ParticleRenderSystem::Render` (hardcoded ortho, §3.1)
- `PhysicsDebugRenderer` (delegates to `Renderer2D`)
- `DebugRenderSystem` (delegates to `Renderer2D`)

There is no `CameraComponent` or `CameraSystem` in the ECS. All rendering assumes a fixed orthographic projection. As a result, adding panning, zooming, or multiple cameras requires changes in multiple places.

**Action:** Introduce `engine/include/nyon/ecs/components/CameraComponent.h`:

```cpp
struct CameraComponent {
    Math::Vector2 position = {0, 0};
    float zoom = 1.0f;
    float nearPlane = -1.0f;
    float farPlane  =  1.0f;
    bool isActive = true;
    glm::mat4 GetViewProjection(float viewportWidth, float viewportHeight) const;
};
```

Introduce a `CameraSystem` that runs before all render systems and writes the active VP matrix to a thread-local/frame-local structure consumed by `Renderer2D::BeginScene` and `ParticleRenderer::Flush`.

---

### 5.4 🟡 `RenderSystem` calls `Renderer2D::Shutdown()` in its own `Shutdown()`

**File:** `engine/include/nyon/ecs/systems/RenderSystem.h`

```cpp
void Shutdown() override {
    Graphics::Renderer2D::Shutdown();
}
```

`Application::~Application()` also calls `Graphics::Renderer2D::Shutdown()`. If `RenderSystem::Shutdown()` is called by `SystemManager::Shutdown()` (which it is, via destructor), and then `Application` destructor also calls it, `Renderer2D::Shutdown()` runs twice. If `Shutdown()` is not idempotent (destroys OpenGL objects a second time), this is undefined behaviour.

**Action:** Make `Renderer2D::Shutdown()` idempotent (guard with `if (!s_Instance) return;`) and remove it from `RenderSystem::Shutdown()`. Renderer2D lifecycle should be owned by `Application` only.

---

### 5.5 🔵 `DrawSolidCircle` and `DrawCircle` silently discard their `segments` parameter

**File:** `engine/src/graphics/Renderer2D.cpp`

```cpp
void Renderer2D::DrawSolidCircle(const Math::Vector2& center, float radius,
                                 const Math::Vector3& color, int ) {  // ← unnamed param
```

The `segments` parameter is accepted in the signature for API compatibility with the old CPU renderer but silently ignored (GPU SDF circle has fixed 16-segment mesh). The public `Renderer2D.h` header presumably declares this parameter. Callers who pass `segments=32` believe they're getting a smoother circle; they are not.

**Action:** Either remove the `segments` parameter from the public API (breaking change), or document clearly in the header: `/** segments is ignored in GPU mode — SDF produces perfect circles regardless **/`.

---

## 6. ECSApplication — Lifecycle & Ordering Bugs

### 6.1 🟠 `OnECSStart()` called before engine systems are added — ordering hazard

**File:** `engine/src/core/ECSApplication.cpp`

```cpp
void ECSApplication::OnStart() {
    Utils::InputManager::Init(GetWindow());
    OnECSStart();                                              // ← user code runs FIRST
    m_SystemManager.AddSystem(std::make_unique<ECS::InputSystem>());
    m_SystemManager.AddSystem(std::make_unique<ECS::PhysicsPipelineSystem>());
    ...
}
```

`OnECSStart()` in `ParticleCollisionDemo` adds `ParticleRenderSystem` to the `SystemManager`. This means the user's `ParticleRenderSystem` is added at index 0 in the system list, but `InputSystem` and `PhysicsPipelineSystem` are added after. `SystemManager::Update()` iterates in insertion order, so particle rendering updates before physics — which should have no practical effect today but establishes a fragile ordering contract.

More critically, if a user's `OnECSStart()` creates entities and adds components, those entities exist but `PhysicsPipelineSystem` is not yet initialized (its `Initialize()` is called inside `AddSystem`). If the physics system's `Initialize` scans for existing entities, it would find them. But if it doesn't (it queries lazily in `Update`), then on the first frame those entities may have an uninitialized physics state.

**Action:** Move engine system registration BEFORE `OnECSStart()`. This guarantees engine systems are always at the front of the update order and are initialized before user code adds entities.

---

### 6.2 🟠 `OnECSUpdate` is called inside `OnFixedUpdate` — semantically wrong

**File:** `engine/src/core/ECSApplication.cpp`

```cpp
void ECSApplication::OnFixedUpdate(float deltaTime) {
    ...
    m_SystemManager.Update(deltaTime);
    OnECSFixedUpdate(deltaTime);
    OnECSUpdate(deltaTime);     // ← called at fixed 60Hz, not per-frame
}
```

The method is named `OnECSUpdate` which implies a per-frame variable-rate update (like `Update` in Unity). But because it is called from `OnFixedUpdate`, it runs at the fixed physics rate (60 Hz by default). Any user code that tries to use this for visual or input-responsive logic will experience one-step-per-physics-tick behaviour. The `ParticleCollisionDemo` relies on this to push particles to the `ParticleRenderSystem`, which is why it works — but it is conceptually wrong.

**Action:** Call `OnECSUpdate` from a separate `OnUpdate(float deltaTime)` override chain called once per rendered frame, not per physics step. Create a distinct `Application::OnUpdate` → `ECSApplication::OnUpdate` → `OnECSUpdate` chain.

---

### 6.3 🟡 `m_RenderSystem` and `m_DebugRenderSystem` bypass `SystemManager`

**File:** `engine/include/nyon/core/ECSApplication.h`

```cpp
std::unique_ptr<ECS::RenderSystem> m_RenderSystem;
std::unique_ptr<ECS::DebugRenderSystem> m_DebugRenderSystem;
```

These are managed directly by `ECSApplication` rather than registered in `SystemManager`. Consequences:

- They do not participate in `SystemManager::Shutdown()` (though they are destroyed by `ECSApplication`'s destructor via RAII, so leaks don't occur).
- There is no way for a developer to retrieve them via `GetSystem<RenderSystem>()`.
- Their `Initialize()` is called manually rather than through the standard `AddSystem` path.

**Action:** Register both in `SystemManager` via `AddSystem` but mark them as non-removable engine systems. Alternatively, accept this design but document it explicitly.

---

### 6.4 🟡 `GetSystem<ParticleRenderSystem>()` linear scan every render frame

**File:** `engine/src/core/ECSApplication.cpp`

```cpp
auto* particleSystem = m_SystemManager.GetSystem<ECS::ParticleRenderSystem>();
if (particleSystem) {
    particleSystem->Render(alpha);
}
```

`GetSystem<T>()` performs a linear type scan over the system list every render frame (typically 60–144 times/second). Cache the pointer after `OnECSStart()` completes.

**Action:** Add `ECS::ParticleRenderSystem* m_ParticleRenderSystem = nullptr;` as a member, set it once after `OnStart()`, and use the cached pointer in `OnInterpolateAndRender`.

---

## 7. ComponentStore & EntityManager

### 7.1 🟠 `ComponentStore::GetComponent` calls `std::terminate()` on missing component — no error context

**File:** `engine/include/nyon/ecs/ComponentStore.h`

```cpp
if (containerIt == m_Containers.end()) {
    std::terminate();   // ← zero diagnostic info
}
```

`std::terminate()` produces no stack trace, no message, and no indication of which entity or component type was missing. Every `GetComponent` call is a potential silent crash.

**Action:** Replace with:

```cpp
NYON_ASSERT_MSG(containerIt != m_Containers.end(),
    "ComponentStore::GetComponent — component type not registered: " + std::string(typeid(T).name()));
```

Where `NYON_ASSERT_MSG` logs to `std::cerr` then calls `std::abort()` in debug builds, and throws in release.

---

### 7.2 🟡 `EntityManager::DestroyEntity` is O(n) in entity count

**File:** `engine/src/ecs/EntityManager.cpp`

```cpp
m_ActiveEntities.erase(
    std::remove(m_ActiveEntities.begin(), m_ActiveEntities.end(), entity),
    m_ActiveEntities.end()
);
```

Linear scan over all active entities. For a simulation with 10,000 entities, destroying one entity scans all 10,000. Batch destruction (destroying all dead particles at frame end) is O(n²).

**Action:** Replace `m_ActiveEntities` with an `std::unordered_set<EntityID>` for O(1) removal, or use a generational index scheme (entity = index + generation, the standard approach for ECS).

---

### 7.3 🟡 `ForEachComponent` iterates over `indexMap` (hash map) — non-deterministic order

**File:** `engine/include/nyon/ecs/ComponentStore.h`

```cpp
for (const auto& [entityId, index] : container.indexMap) {
```

`std::unordered_map` iteration order is non-deterministic across runs (implementation-defined). Physics and rendering systems that iterate all components of a type will produce slightly different results between runs if any floating-point operations are order-sensitive. This makes reproduction of physics bugs extremely difficult.

**Action:** For performance-critical iteration (physics update, rendering), iterate `container.entityIds` and `container.components` in parallel by index (both are `std::vector` and are in a stable, consistent order). Use `indexMap` only for random-access lookups.

---

## 8. ThreadPool

### 8.1 🟠 `WaitAll()` can deadlock if called from a worker thread

**File:** `engine/src/utils/ThreadPool.cpp`

```cpp
void ThreadPool::WaitAll() {
    std::unique_lock<std::mutex> lock(m_QueueMutex);
    m_AllDoneCondition.wait(lock, [this] {
        return m_ActiveTasks == 0 && m_Tasks.empty();
    });
}
```

If a task submitted to the pool calls `WaitAll()`, it acquires `m_QueueMutex` while holding it in the wait. Other tasks cannot be dequeued (the worker's main loop also locks `m_QueueMutex`), causing a deadlock. In the current codebase `WaitAll` is not called from tasks, but this is a correctness hazard for future use.

**Action:** Document that `WaitAll()` must not be called from a thread owned by the pool. Add a `NYON_ASSERT(!IsWorkerThread())` guard using a thread-local flag set by `WorkerThread()`.

---

### 8.2 🟡 `m_ActiveTasks` decrement has no fence ordering guarantee with condition variable notification

**File:** `engine/src/utils/ThreadPool.cpp`

```cpp
task();
if (--m_ActiveTasks == 0) {
    m_AllDoneCondition.notify_all();
}
```

`--m_ActiveTasks` uses an atomic decrement (assumed). However, if `m_ActiveTasks` is `std::atomic<size_t>` with default `memory_order_seq_cst`, and `WaitAll` reads it inside the condition variable predicate while holding `m_QueueMutex`, the lock provides the necessary sequencing. This is correct if and only if `m_ActiveTasks` is `std::atomic`. Confirm the declaration.

**Action:** Confirm `m_ActiveTasks` is declared `std::atomic<size_t>` in `ThreadPool.h`. Add this declaration if it is not.

---

### 8.3 🟡 `GetPendingTaskCount()` declared `const` but locks a non-`mutable` mutex

**File:** `engine/src/utils/ThreadPool.cpp`

```cpp
size_t ThreadPool::GetPendingTaskCount() const {
    std::lock_guard<std::mutex> lock(m_QueueMutex);  // m_QueueMutex must be mutable
```

For this to compile, `m_QueueMutex` must be declared `mutable std::mutex m_QueueMutex;`. Verify this in `ThreadPool.h`. If it is not `mutable`, this is a compilation error.

---

## 9. Debug Systems

### 9.1 🟡 Two parallel debug rendering systems with overlapping responsibility

Two systems can render physics debug visuals:

1. `Graphics::PhysicsDebugRenderer` (`engine/include/nyon/graphics/PhysicsDebugRenderer.h`) — a standalone class with rich flag-based rendering, drawing circles, polygons, capsules, segments, chains, manifolds, normals, CoM, velocity vectors.
2. `ECS::DebugRenderSystem` (`engine/include/nyon/ecs/systems/DebugRenderSystem.h`) — an ECS system that internally holds a `NyonDebugRenderer` and queues draw commands.

`PhysicsDebugRenderer` is more complete and more capable. `DebugRenderSystem` is the one actually wired into `ECSApplication`. They implement overlapping functionality (circle outline, polygon drawing) with different internal code paths. `ECS::DebugRenderSystem::DrawCapsuleShape` approximates a capsule as two circles + a line rather than using the GPU capsule shader that `PhysicsDebugRenderer` / `Renderer2D::DrawSolidCapsule` would use.

**Action:** Remove `DebugRenderSystem::NyonDebugRenderer` entirely. Delegate all draw calls from `DebugRenderSystem` to `Graphics::PhysicsDebugRenderer`. The ECS system becomes an orchestrator (iterates entities, determines colors) while `PhysicsDebugRenderer` handles all drawing.

---

### 9.2 🔵 `NYON_DEBUG_LOG` macro defined in multiple translation units

**Files:** `Application.cpp`, `ECSApplication.cpp`, `DebugRenderSystem.cpp`

All three define:

```cpp
#ifdef _DEBUG
#define NYON_DEBUG_LOG(x) std::cerr << x << std::endl
#else
#define NYON_DEBUG_LOG(x)
#endif
```

**Action:** Move to `engine/include/nyon/Debug.h` and include from each file.

---

## 10. Input System

### 10.1 🟡 `InputManager::IsMouseUp` returns `true` when window is null; `IsKeyUp` returns `false`

**File:** `engine/src/utils/InputManager.cpp`

```cpp
bool InputManager::IsKeyUp(int key) {
    ...
    if (s_Window == nullptr) { return false; }  // "key not up" when no window
    ...
}
bool InputManager::IsMouseUp(int button) {
    ...
    if (s_Window == nullptr) { return true; }   // "button is up" when no window — INCONSISTENT
    ...
}
```

Inconsistent null-window semantics. A mouse button being "up" when there is no window is arguably correct (no button can be pressed), but it is inconsistent with the key behaviour and will confuse users.

**Action:** Standardize: when no window, all keys and buttons are considered "not pressed" (up = true for all). `IsKeyUp` should return `true` when `s_Window == nullptr`.

---

### 10.2 🔵 `InputSystem` calls `Utils::InputManager::Update()` — but `Update()` copies key state

**File:** `engine/include/nyon/ecs/systems/InputSystem.h`

`InputSystem::Update()` calls `Utils::InputManager::Update()` which copies `s_CurrentKeys` → `s_PreviousKeys` via `memcpy`. This happens inside `SystemManager::Update()` which is called from `OnFixedUpdate`. If multiple systems call `Update()` more than once per frame due to substeps, key state is copied multiple times, losing "just pressed" events.

**Action:** Call `InputManager::Update()` exactly once per rendered frame from `Application::Run()` before the fixed-step loop, not from inside a system.

---

## 11. GPU Parallelisation Opportunities (CPU Fallbacks)

Per the requirement to move GPU-incompatible workloads to CPU parallel execution:

### 11.1 Particle lifecycle update → `ThreadPool`

The per-particle physics update (velocity integration, drag, gravity, lifetime decrement) is embarrassingly parallel. Once `ParticlePipelineSystem` is created (§3.5), partition the particle array into `numThreads` chunks and submit each chunk as a `ThreadPool` task.

### 11.2 Collision response impulse application → already parallel in spirit, needs SOA

The current velocity constraint solver is sequential (must be, for iterative solvers). However, the `IntegrateVelocities` step (applying accumulated forces) is embarrassingly parallel and already has a range-based overload `IntegrateVelocities(dt, start, end)`. This is already parallelised for `SolverBodies`. Ensure the parallel path is also taken for the particles.

### 11.3 Spatial hash construction for particle-particle broadphase → parallel insertion

Inserting N particles into a spatial hash can be done in parallel with a two-phase approach: first, each thread computes `(cellIndex, particleIndex)` pairs into a local buffer; then a single-threaded radix sort assembles the final structure. For 1M+ particles this is significant.

### 11.4 AABB update (broadphase tree) → parallel with read-only tree during query

`UpdateShapeAABB` per entity is currently sequential. It calls `CreateProxy`/`MoveProxy` on the `DynamicTree` — these mutate the tree so they cannot trivially be parallelized. However, computing AABBs from transforms can be done in parallel, staging the results into a batch, then applying all `MoveProxy` calls serially.

### 11.5 Polygon tessellation → CPU parallel prep before GPU upload

`DrawSolidPolygon` tessellates on the CPU into a vertex buffer then pushes to a GPU mapped buffer. For complex polygon scenes, tessellation of multiple polygons can be farmed to the `ThreadPool` with results written into thread-local staging arrays, then merged serially before the GPU upload.

---

## 12. Implementation Priority Order

Execute fixes in the following order to minimise regressions and maximise stability at each stage.

22. Implement collision filtering (`ShouldCollide`) (§2.5).
23. Implement `UpdateGroundedState` call in physics pipeline (§4.7).

### Stage 4 — Particle System Overhaul
24. Redesign `Particle` struct with `prevX/prevY`, `alpha`, `lifetime`, `userData` (§3.5-A).
25. Create `ParticleEmitterComponent` with developer hooks (§3.5-B).
26. Create `ParticlePipelineSystem` with `ThreadPool`-parallel update (§3.5-C, §11.1).
27. Fix `ParticleRenderSystem::Render` to use `alpha` interpolation (§3.2).
28. Fix `SetParticles` copy to pointer/span pattern (§3.3).
29. Integrate particle physics with engine `PhysicsWorldComponent` gravity (§3.4).
30. Fix spatial hash allocation in collision demo (§3.6).
31. Rename shadowing variables in demo collision handler (§3.7).

### Stage 5 — Thread Safety & Remaining Parallelism
32. Cache `world.gravity` before parallel threads in `ParallelVelocitySolving` (§4.2).
33. Audit and document `DynamicTree::Query` thread safety (§4.3).
34. Confirm `m_ActiveTasks` is `std::atomic<size_t>` (§8.2).
35. Fix `GetPendingTaskCount() const` mutable mutex (§8.3).
36. Add `IsWorkerThread` guard to `WaitAll` (§8.1).
37. Use `subStepCount` from `PhysicsWorldComponent` instead of ad hoc threshold (§4.8).
38. Validate `m_NumThreads > 0` before parallel division (§4.9).
39. Fix contact map key to include shape IDs (§4.10).
40. Parallelize AABB computation before `MoveProxy` (§11.4).

---

*End of Report. All line numbers reference the concatenated codebase file `daiyaanmuhammadfardeen-nyon-8a5edab282632443_12_.txt`.*
