# Nyon Engine — Comprehensive Bug Analysis Report

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Critical Bugs — Collision Resolution](#critical-bugs--collision-resolution)
3. [Critical Bugs — Physics Integration](#critical-bugs--physics-integration)
4. [Critical Bugs — System Architecture](#critical-bugs--system-architecture)
5. [High-Severity Issues](#high-severity-issues)
6. [Medium-Severity Issues](#medium-severity-issues)
7. [Low-Severity / Code Quality Issues](#low-severity--code-quality-issues)
8. [Architectural Limitations](#architectural-limitations)
9. [Per-System Breakdown](#per-system-breakdown)
10. [Recommended Fix Priority](#recommended-fix-priority)

---

## Executive Summary

The Nyon Engine is a 2D OpenGL game engine with an ECS (Entity-Component-System) architecture inspired by Box2D. The codebase has a solid structural foundation — the ECS primitives, renderer, and math types are generally well-formed — but the **physics pipeline is fundamentally broken in several compounding ways** that together produce exactly the symptoms described: objects tunneling at high velocity, "floaty" collisions at low velocity, and gravity-less-feeling responses on contact.

The root causes cluster into four categories:

- **Collision Detection / Resolution Conflict**: Two separate collision systems (`ContinuousCollisionSystem` and `CollisionSystem`) operate simultaneously and contradict each other's corrections.
- **CCD Implementation Errors**: The sweep-based TOI calculation uses sphere approximations, but the impulse resolution uses AABB logic — these are incompatible and produce wrong normals/depths.
- **System Initialization Race Condition**: Physics systems cache pointers to components at `Initialize()` time; dynamically spawned entities are invisible to those systems.
- **Transform/Physics Desynchronization**: The `TransformPhysicsSyncSystem` applies an additive velocity delta to the transform every frame instead of just tracking physics state — this double-integrates position.

What follows is a complete enumeration of every logic bug, architectural flaw, and improvement opportunity found in the repository.

---

## Critical Bugs — Collision Resolution

### CB-01 — Two Collision Systems Fight Each Other

**Files**: `CollisionSystem.h`, `ContinuousCollisionSystem.cpp`, `ECSApplication.cpp`

`ECSApplication::OnStart()` registers `InputSystem` then `RenderSystem`. `PhysicsDemoGame::OnECSStart()` then adds `PhysicsIntegrationSystem`, `ContinuousCollisionSystem`, `TransformPhysicsSyncSystem`, `CollisionPipelineSystem`, and `ConstraintSolverSystem`.

`CollisionSystem` is declared in `CollisionSystem.h` and is a fully self-contained inline implementation, but it is **never registered** in `PhysicsDemoGame`. However, `ContinuousCollisionSystem` IS registered. The problem is that `CollisionPipelineSystem` also detects collisions and tries to track contacts via its `ContactMap`, while `ContinuousCollisionSystem` independently resolves positional corrections and impulses. Both systems iterate over the same entities and modify the same `TransformComponent` and `PhysicsBodyComponent` data.

**Effect**: Corrections from `ContinuousCollisionSystem` are partially overwritten or compounded by `CollisionPipelineSystem`'s state, leading to jitter, incorrect final positions, and energy injection into the system.

**Fix**: Choose one collision resolution system. Either use `ContinuousCollisionSystem` alone (removing `CollisionPipelineSystem`'s resolution step) or build the pipeline system to completion and remove `ContinuousCollisionSystem`. The `CollisionPipelineSystem` should only detect and generate manifolds; resolution should be in `ConstraintSolverSystem`.

---

### CB-02 — TOI Calculation Uses Sphere Model, Resolution Uses AABB Model

**File**: `ContinuousCollisionSystem.cpp` — `PerformCCD()`

```cpp
toi = Physics::CollisionDetectionStrategy::CalculateTimeOfImpact(
    startPosA, endPosA, startPosB, endPosB, radiusA, radiusB);  // sphere TOI
```

Then immediately:

```cpp
ResolveCCDImpulse(...)  // uses AABB half-extents and axis-aligned normals
```

`CalculateTimeOfImpact` fits a sphere of radius `min(extentX, extentY)` around each AABB. For a 32×64 rectangle this sphere has radius 16, leaving half the shape unrepresented. The TOI result reflects the sphere collision moment, but the actual contact normal is then computed from AABB overlap — which reflects a *different* geometry and a *different* moment in time. The two are not consistent, so the normal direction and penetration depth fed into the impulse solver are both wrong.

**Effect**: Collision normals frequently point in the wrong direction. A box falling straight down onto a platform may receive a horizontal impulse. This explains the "collide like there is no gravity" symptom — the vertical velocity is not being cancelled because the normal is not pointing up.

**Fix**: Use a consistent geometry for both TOI and resolution. For AABB-AABB, implement a proper swept-AABB TOI (slab method) and derive the contact normal from the minimum-penetration axis at the exact TOI moment, not from a re-evaluated current-frame overlap.

---

### CB-03 — Penetration Depth Sign Convention Is Inverted

**File**: `ContinuousCollisionSystem.cpp` — `ResolveCCDImpulse()` and `ResolveCCDPositionalCorrection()`

```cpp
float overlapX = (halfSizeA.x + halfSizeB.x) - std::abs(delta.x);
float overlapY = (halfSizeA.y + halfSizeB.y) - std::abs(delta.y);
// ...
float penetrationDepth = (overlapX < overlapY) ? -overlapX : -overlapY;
// ...
if (penetrationDepth < -0.01f)  // only resolves when negative
{
    correction = normal * (std::abs(penetrationDepth) * BAUMGARTE_FACTOR);
```

`overlapX` and `overlapY` are positive when objects overlap (>0 = penetrating). The code then negates one of them to get `penetrationDepth`, making it negative when overlapping. The condition `penetrationDepth < -0.01f` then correctly triggers — but then `std::abs(penetrationDepth)` is used for the correction magnitude, which is fine. However, the normal direction is computed *before* checking penetration, using `delta.x < 0 ? -1 : 1`. This means the normal points **from B to A** (away from B), but the correction is then:

```cpp
transformA.position += correction * (bodyA.inverseMass / totalInvMass);
transformB.position -= correction * (bodyB.inverseMass / totalInvMass);
```

A is pushed in the direction of `normal` (away from B) and B is pushed in the opposite direction — which is **correct** for pushing them apart. But in `ResolveCCDCollision`, the impulse section has the opposite convention:

```cpp
bodyA.velocity += impulse * bodyA.inverseMass;   // impulse = normal * scalar
bodyB.velocity -= impulse * bodyB.inverseMass;
```

Here `normal` points from A to B (the delta.x sign check computes this as "if B is to the left, normal points left" — which actually points from A toward B's side). Applying `+impulse` to A's velocity pushes A toward B, and `-impulse` to B pushes B away from A. **This is backwards.** A should be repelled away from B.

**Effect**: At the moment of contact, both objects may be accelerated toward each other rather than away, causing them to pass through each other (tunneling) or explode apart wildly depending on which sign dominates.

**Fix**: Standardize the normal convention as "points from B to A" (i.e., the direction A should be pushed to separate). Then apply `+impulse * inverseMassA` to A's velocity and `-impulse * inverseMassB` to B's velocity.

---

### CB-04 — `ResolveCCDImpulse` Is Called Unconditionally Within TOI Branch

**File**: `ContinuousCollisionSystem.cpp` — `PerformCCD()`

```cpp
if (toi >= 0.0f && toi <= 1.0f)
{
    ResolveCCDImpulse(...);  // applies impulse without checking if objects are actually penetrating
    for (int iter = 0; iter < ITERATIONS; ++iter)
        ResolveCCDPositionalCorrection(...);
}
```

`CalculateTimeOfImpact` returns a value in [0,1] meaning "at this fraction of the frame the spheres touch." But at `toi = 1.0`, this means they touch at the very end of the frame and there may be zero penetration at the *start* of the frame. The function also returns `0.0f` for the "already overlapping" case. The `ResolveCCDImpulse` function then checks `if (penetrationDepth < -0.01f)` before applying anything, which helps somewhat — but it still runs all the AABB geometry and relative-velocity code unconditionally for objects that may only be grazing.

More critically, `CalculateTimeOfImpact` returns `-1.0f` when objects are moving apart or will never collide — but the check `toi >= 0.0f && toi <= 1.0f` will **not** catch -1, so that case is handled correctly. However, the function can return `0.0f` for currently-overlapping objects, and the code handles this with a debug log but still calls `ResolveCCDImpulse`, which is correct behavior — but then `ResolveCCDCollision` (a separate, longer function that's never called from `PerformCCD`) has its own `toi == 0.0f` branch. This dead code creates confusion about which path is actually exercised.

**Fix**: After computing TOI, move entities to the impact position before evaluating penetration. Only then compute the overlap and apply impulse. Currently the function evaluates penetration at the current frame's final positions, not at the TOI moment.

---

### CB-05 — Velocity Clamping Destroys Physics Realism

**File**: `ContinuousCollisionSystem.cpp`

```cpp
const float MAX_VELOCITY = 2000.0f;
if (velMagnitude > MAX_VELOCITY)
    bodyA.velocity = bodyA.velocity * (MAX_VELOCITY / velMagnitude);
```

This is applied *after* the impulse. It can silently cap physically correct large velocities (e.g., a heavy object accelerated by a large force). Worse, the cap in `ResolveCCDCollision` uses `MAX_VELOCITY = 100.0f` (an older value), while `ResolveCCDImpulse` uses `2000.0f`. These functions are both present and both called in some code paths, giving inconsistent energy removal.

**Fix**: Use a single, consistent velocity cap. Consider only capping in the integration step (`PhysicsIntegrationSystem`), not in the collision resolver, unless an impact genuinely requires capping (e.g., to prevent simulation instability).

---

### CB-06 — Ground Contact Detection Is Unreliable

**File**: `ContinuousCollisionSystem.cpp`

```cpp
bool isGroundContact = (normal.y < -0.7f && bodyB.isStatic);
```

`ResolveCCDImpulse` uses `normal.y < -0.7f` (expects normal to point upward, i.e., Y negative = up). But in `ResolveCCDCollision` the equivalent check is:

```cpp
bool isGroundContact = (normal.y > 0.8f && bodyB.isStatic);
```

These are opposite signs. One of them is wrong. The coordinate system uses Y-positive-downward (as stated in `PhysicsBodyComponent.h`). In that system, the ground is below the player. The normal from ground surface pointing toward the player should be `(0, -1)` — i.e., `normal.y = -1`. So `normal.y < -0.7f` is the correct form. The `> 0.8f` version in `ResolveCCDCollision` would treat a ceiling contact as a ground contact.

Since `ResolveCCDCollision` is never actually called from `PerformCCD` (see CB-07), only the `-0.7f` version matters in practice — but this is still a latent bug if `ResolveCCDCollision` is ever wired in.

---

### CB-07 — `ResolveCCDCollision` Is Dead Code

**File**: `ContinuousCollisionSystem.cpp` / `ContinuousCollisionSystem.h`

`ResolveCCDCollision` is a large, detailed function (the most complete collision resolution in the file) that is **never called from anywhere**. `PerformCCD` calls `ResolveCCDImpulse` and `ResolveCCDPositionalCorrection` instead. The fully-featured resolution path with debug logging, warm-start guards, and full impulse/friction/notification is simply unused dead code.

**Effect**: The most correct collision resolution code is never executed.

**Fix**: Replace the `ResolveCCDImpulse` + `ResolveCCDPositionalCorrection` calls in `PerformCCD` with `ResolveCCDCollision`, or refactor `ResolveCCDCollision` into the main path.

---

## Critical Bugs — Physics Integration

### CI-01 — `TransformPhysicsSyncSystem` Double-Integrates Position

**File**: `TransformPhysicsSyncSystem.cpp`

```cpp
void TransformPhysicsSyncSystem::SyncPhysicsToTransform(SyncEntity& entity)
{
    entity.transform->position = entity.transform->position + 
        (entity.physicsBody->velocity * m_InterpolationFactor);
    entity.transform->rotation = entity.transform->rotation + 
        (entity.physicsBody->angularVelocity * m_InterpolationFactor);
```

`m_InterpolationFactor` is initialized to `1.0f` and never set to `deltaTime`. This means every tick, the transform is advanced by `velocity * 1.0` — i.e., by `velocity` pixels per tick, regardless of the actual elapsed time. `PhysicsIntegrationSystem::IntegratePosition` also does:

```cpp
transform.position = transform.position + body.velocity * deltaTime;
```

So position is updated **twice per frame**: once by `PhysicsIntegrationSystem` with the correct `deltaTime`, and again by `TransformPhysicsSyncSystem` with a factor of 1.0. At 60 Hz, `deltaTime ≈ 0.0167`, so the sync system moves entities roughly **60× faster** than the integration system.

**Effect**: Objects appear to warp across the screen or immediately leave bounds. Collision detection never triggers because objects teleport past each other. This is likely the primary cause of tunneling at "high velocity" — even slow objects are moved 60 units per frame by this system.

**Fix**: Either remove `TransformPhysicsSyncSystem` entirely (position is already integrated by `PhysicsIntegrationSystem`) or change `m_InterpolationFactor` to represent `deltaTime` and ensure it is passed in correctly. The system's purpose — interpolation for rendering — should only apply in the render step, not the fixed update step.

---

### CI-02 — `PhysicsIntegrationSystem` Applies Gravity Even to Grounded Objects

**File**: `PhysicsIntegrationSystem.h`

```cpp
Math::Vector2 acceleration = gravity;
// ...
body.velocity = body.velocity + acceleration * deltaTime;
```

Gravity is applied unconditionally to all non-static, non-sleeping bodies. There is no check for whether the body is grounded. For a body resting on a platform, gravity will add downward velocity every frame, which the collision system must then cancel every frame. This works only if the collision system reliably cancels the velocity before integration compounds it. If the collision system misses one frame (due to sleeping, ordering, or tunneling), the object sinks into the platform.

The `isGrounded` flag exists in `PhysicsBodyComponent` but is never read by `PhysicsIntegrationSystem`. The stable grounded detection (`IsStablyGrounded()`) requires `groundedFrames >= 2`, but `groundedFrames` is reset to 0 at the start of each tick and only incremented during collision processing. If `PhysicsIntegrationSystem` runs *before* `CollisionSystem` (which it does — see system registration order), the velocity is updated before grounded state is established, causing perpetual sinking.

**Fix**: Either (a) apply gravity after collision detection and only to non-grounded bodies, or (b) use a "normal force" model where the collision system applies an equal-and-opposite contact force that cancels gravity for resting bodies. Option (b) is the Box2D approach and is architecturally cleaner.

---

### CI-03 — Sleep System Breaks Resting Object Simulation

**File**: `PhysicsIntegrationSystem.h` — `UpdateSleepState()`

```cpp
body.sleepTimer += deltaTime;
if (body.sleepTimer >= body.SLEEP_THRESHOLD)
{
    body.SetAwake(false);
    body.velocity = {0.0f, 0.0f};
    body.angularVelocity = 0.0f;
}
```

`SLEEP_THRESHOLD = 0.5f`. When a body rests on a surface, gravity continuously adds a small downward velocity. The collision system cancels it. So velocity oscillates near zero. The sleep system checks `LengthSquared > LINEAR_SLEEP_TOLERANCE^2` (tolerance = 0.01, so `0.0001`). The gravity contribution per frame at 60Hz is `98 * 0.0167 ≈ 1.63` pixels/sec, which is **vastly above** the sleep tolerance. The body will **never sleep** because gravity prevents it from being considered stationary.

If the collision system happens to zero the velocity perfectly (which it can't consistently do due to CB-01 through CB-06), the object might sleep — but then gravity wakes it again next frame, and the cycle restarts. In practice, objects on the ground will burn CPU every frame indefinitely.

**Fix**: Implement proper contact-based sleep: a body should only be considered for sleep if all its contact normals oppose gravity (i.e., it is truly supported), not just if its velocity is low.

---

### CI-04 — Drag Calculation Is Physically Incorrect

**File**: `PhysicsIntegrationSystem.h`

```cpp
if (body.drag > 0.0f && body.velocity.LengthSquared() > 0.0f)
{
    Math::Vector2 dragForce = body.velocity.Normalize() * (-body.drag * body.velocity.LengthSquared());
    acceleration = acceleration + dragForce * body.inverseMass;
}
```

`dragForce = -drag * v²` in the velocity direction. This is proportional to speed squared (aerodynamic drag), which is fine. However, it is then divided by mass (`* inverseMass`) in the acceleration accumulation — meaning the drag acceleration is `F/m = drag * v² / m`. For a unit-mass object this is fine, but for heavy objects the drag is reduced, which is physically wrong (aerodynamic drag acceleration is independent of mass). Additionally, `linearDamping` is *also* applied as `acceleration -= velocity * linearDamping`, which means both a velocity-proportional and a velocity-squared term are active simultaneously, creating a confusing mixed model.

---

## Critical Bugs — System Architecture

### CA-01 — Systems Cache Stale Component Pointers at Initialization

**Files**: `CollisionPipelineSystem.cpp`, `ConstraintSolverSystem.cpp`, `DebugRenderSystem.cpp`, `TransformPhysicsSyncSystem.cpp`

Every system that needs to iterate over entities does so by collecting raw pointers during `Initialize()`:

```cpp
// CollisionPipelineSystem::Initialize
for (auto entityId : bodyEntities)
{
    PhysicsBodyComponent* body = &componentStore.GetComponent<PhysicsBodyComponent>(entityId);
    m_Colliders.emplace_back(entityId, body, collider);
}
```

These pointers are stored in `m_Colliders`, `m_BodyComponents`, `m_Entities`, `m_SyncEntities`. They are **never updated**. When `SpawnRandomObject()` or `CreateBox()` creates a new entity after initialization, it is **invisible to all these systems**. The systems operate only on the entities that existed at startup.

Worse: `ComponentStore` uses `std::unordered_map<EntityID, T>` internally. When a new entity's component is inserted into the map, the map may **rehash**, invalidating all existing iterators and pointers. Any cached `PhysicsBodyComponent*` pointer becomes a dangling pointer after a new entity is added.

**Effect**: Crashes (use-after-free), missed collisions for dynamically spawned objects, and complete invisibility of runtime-spawned entities to physics systems.

**Fix**: Remove all pointer caching. Systems should query `ComponentStore` fresh each `Update()` call using `GetEntitiesWithComponent<T>()`. The ECS architecture is designed for this pattern — fighting it by caching pointers defeats the purpose and introduces undefined behavior.

---

### CA-02 — System Execution Order Is Physically Wrong

**File**: `PhysicsDemoGame.cpp` — `OnECSStart()`, `ECSApplication.cpp` — `OnStart()`

Registered order:
1. `InputSystem`
2. `RenderSystem`
3. `PhysicsIntegrationSystem`
4. `ContinuousCollisionSystem`
5. `TransformPhysicsSyncSystem`
6. `CollisionPipelineSystem`
7. `ConstraintSolverSystem`
8. `DebugRenderSystem`

Problems:
- **`RenderSystem` runs before physics is updated**, meaning it always renders the previous frame's state. This causes visual lag but not physics incorrectness.
- **`TransformPhysicsSyncSystem` runs between `ContinuousCollisionSystem` and `CollisionPipelineSystem`**, meaning it modifies transforms based on the post-CCD state before `CollisionPipelineSystem` has a chance to evaluate those transforms. This creates inconsistent state for `CollisionPipelineSystem`.
- **`ConstraintSolverSystem` runs last**, after everything else. It is supposed to solve velocity constraints *before* position integration — but integration already happened in step 3.
- **`DebugRenderSystem` calls `BeginScene`/`EndScene` internally** (a complete render pass) in the middle of the fixed-update loop, which is incorrect — it should render in the render step, not the physics step.

**Correct order should be**:
1. `InputSystem`
2. `PhysicsIntegrationSystem` (apply forces, integrate velocities)
3. `CollisionPipelineSystem` (broad + narrow phase, generate manifolds)
4. `ConstraintSolverSystem` (solve velocity/position constraints)
5. `ContinuousCollisionSystem` (CCD for fast-movers)
6. `TransformPhysicsSyncSystem` (sync physics state to transform — or remove entirely)
7. `RenderSystem` (render current state)
8. `DebugRenderSystem` (overlay debug info)

---

### CA-03 — `DebugRenderSystem` Calls `BeginScene`/`EndScene` in Fixed Update

**File**: `DebugRenderSystem.cpp`

```cpp
void DebugRenderSystem::Update(float deltaTime)
{
    Graphics::Renderer2D::BeginScene();
    // draw debug shapes...
    Graphics::Renderer2D::EndScene();
}
```

This runs inside `SystemManager::Update()`, which is called from `ECSApplication::OnFixedUpdate()`, which is called in the physics loop — **not** the render step. `BeginScene` clears the vertex buffer. `EndScene` calls `Flush` which submits geometry to the GPU via OpenGL. Calling OpenGL commands from the fixed-update thread (if threading is ever added) or at arbitrary fixed-update rates (possibly multiple times per render frame) is incorrect.

Additionally, `RenderSystem::Update()` also calls `BeginScene` and `EndScene`. So every fixed-update step, both systems issue GL draw calls, but only the last `glfwSwapBuffers` in `Application::Run()` presents the frame — meaning all the intermediate `EndScene` flushes are wasted draws that may not appear on screen at all.

---

### CA-04 — `CollisionPipelineSystem` Initializes With Stale Entity Lists, Then Queries Dynamic Tree With Entity-Level Granularity

**File**: `CollisionPipelineSystem.cpp`

The system registers one proxy per entity (using `shapeId = 0` for all shapes). The broad-phase tree is queried correctly, and `BroadPhaseCallback::QueryCallback` adds pairs to `m_ActivePairs`. But the `m_ShapeProxyMap` is keyed by `shapeId` (always 0) — meaning if two entities both have `shapeId = 0`, they share the same proxy slot and one will silently overwrite the other.

```cpp
void CollisionPipelineSystem::UpdateShapeAABB(uint32_t entityId, uint32_t shapeId, ...)
{
    auto proxyIt = m_ShapeProxyMap.find(shapeId);  // shapeId is always 0
```

Every entity uses `shapeId = 0`, so only the last entity to call `UpdateShapeAABB` will have its AABB registered in the tree. All others are silently dropped.

---

## High-Severity Issues

### HS-01 — `CollisionSystem` (inline in header) Is Registered Nowhere But Contains Boundary Enforcement

**File**: `CollisionSystem.h`

The boundary constraint logic (screen edges, top/bottom/left/right walls) lives inside `CollisionSystem::ApplyBoundaryConstraints()`. This system is never added to `SystemManager` in `PhysicsDemoGame`. Without it, objects can leave the screen bounds entirely, and the `groundedFrames` increment from screen-bottom contact never fires. The wall entities created by `CreateEnvironment()` are the only thing preventing out-of-bounds movement, but they rely on the collision system correctly resolving contacts — which is broken.

---

### HS-02 — `PhysicsBodyComponent::UpdateMassProperties()` Uses Wrong Inertia Formula

**File**: `PhysicsBodyComponent.h`

```cpp
inertia = mass * 0.1667f; // 1/6 approximation
```

This comment claims "1/6 approximation" but `1/6 ≈ 0.1667` only happens to be correct for a *unit square* (side=1). The moment of inertia for a rectangle is `I = m(w² + h²)/12`. With no width/height information at the component level (collider shape data is in `ColliderComponent`), this approximation is used for all bodies regardless of shape. A large 500×500 platform and a 10×10 box will have proportionally identical angular inertia, making angular dynamics physically wrong for all non-unit bodies.

---

### HS-03 — `CreateBox` Mass Calculation Is Arbitrary

**File**: `PhysicsDemoGame.cpp`

```cpp
body.mass = density * size.x * size.y * 0.001f;
```

The `0.001f` factor has no physical justification. For a 32×32 box with density 1.0, this gives mass `= 1 * 32 * 32 * 0.001 = 1.024 kg`. For a 1200×40 ground platform (density 0, `isStatic = true`) this gives 0, which is then handled by `UpdateMassProperties()` setting infinite mass — so ground works. But for a 30×30 interactive box vs. a 20×20 circle, the mass ratio is `0.9 / 1.257` — not what you'd expect from visual size ratios. Combined with the wrong inertia, angular responses are completely incorrect.

---

### HS-04 — Entity Destruction Does Not Remove Broad-Phase Proxies

**File**: `PhysicsDemoGame.cpp` — `RestartScene()`, `EntityManager.cpp`

```cpp
entityManager.DestroyEntity(entityId);
```

`DestroyEntity` removes the entity ID from active lists but does not notify any system. `CollisionPipelineSystem::m_ShapeProxyMap` retains proxy entries for destroyed entities. The `DynamicTree` still holds those proxies. When the tree queries those proxies, it may return entity IDs that no longer have valid components, causing `componentStore.GetComponent<>()` to hit an assert or return garbage.

---

### HS-05 — `ConstraintSolverSystem` Is a Skeleton — It Does Nothing Useful

**File**: `ConstraintSolverSystem.cpp`

```cpp
void ConstraintSolverSystem::InitializeConstraints()
{
    m_VelocityConstraints.clear();
    m_PositionConstraints.clear();
}
```

The constraint lists are cleared and never populated. `SolveVelocityConstraints()` loops over `m_VelocityConstraints` (always empty). `SolvePositionConstraints()` loops over `m_PositionConstraints` (always empty). The system burns CPU initializing `m_SolverBodies` and copying velocity data in and out, then does nothing with it.

The `IntegrateVelocities()` method inside `ConstraintSolverSystem` applies gravity *again* (in addition to `PhysicsIntegrationSystem`) and also applies a hardcoded `* 0.999f` damping:

```cpp
body.velocity = body.velocity + m_PhysicsWorld->gravity * dt;
body.velocity = body.velocity * 0.999f;
```

This means every physics tick, gravity is applied **twice** — once in `PhysicsIntegrationSystem` and once in `ConstraintSolverSystem`. This doubles the effective gravitational acceleration.

---

### HS-06 — `CollisionPhysics::ResolveCollision` Uses a Temporary `Utils::Physics::Body` Struct That Is Not Declared Anywhere Visible

**File**: `CollisionSystem.h`

```cpp
Utils::Physics::Body tempBodyA, tempBodyB;
tempBodyA.position = ...;
tempBodyA.velocity = bodyA.velocity;
```

`Utils::Physics::Body` is referenced but never defined in any included header. The `engine/src/utils/Physics.cpp` is intentionally empty ("moved to specialized modules"). There is no `Physics.h` included in `CollisionSystem.h`. This code would fail to compile. Since `CollisionSystem` is never registered, the compiler may optimize it away or it may not be included in the compilation unit — but if it were registered and compiled, it would be a hard compile error.

---

## Medium-Severity Issues

### MS-01 — `SyncTransformToPhysics` Computes Velocity From Position Delta Incorrectly

**File**: `TransformPhysicsSyncSystem.cpp`

```cpp
entity.physicsBody->velocity = (entity.transform->position - entity.lastPosition) / 
    (m_InterpolationFactor > 0.0f ? m_InterpolationFactor : 1.0f);
```

`m_InterpolationFactor` is `1.0f` (not deltaTime). So the derived velocity is `(positionDelta) / 1.0 = positionDelta`. For a teleportation of 100 pixels, the velocity becomes 100 pixels/tick — but since `PhysicsIntegrationSystem` multiplies by `deltaTime` (~0.0167), the resulting movement next frame would be `100 * 0.0167 ≈ 1.67` pixels, rapidly decaying. This makes teleportation via transform effectively impossible to use correctly.

---

### MS-02 — `CollisionPipelineSystem::Initialize` Is Called Before World Entity Exists

**File**: `PhysicsDemoGame.cpp`, `ECSApplication.cpp`

Systems are added via `systemManager.AddSystem()` in `PhysicsDemoGame::OnECSStart()`. `AddSystem()` calls `system->Initialize()` immediately. The `PhysicsWorldComponent` entity is created *after* all systems are added. So when `CollisionPipelineSystem::Initialize()` runs:

```cpp
const auto& worldEntities = componentStore.GetEntitiesWithComponent<PhysicsWorldComponent>();
if (!worldEntities.empty())
    m_PhysicsWorld = &componentStore.GetComponent<PhysicsWorldComponent>(worldEntities[0]);
```

`worldEntities` is empty. `m_PhysicsWorld` remains null. Then `Update()` early-returns:

```cpp
if (!m_PhysicsWorld) return;
```

So `CollisionPipelineSystem` never runs at all.

The same applies to `ConstraintSolverSystem`.

**Fix**: Create the `PhysicsWorldComponent` entity before registering systems.

---

### MS-03 — `PolygonShape::CalculateProperties()` Centroid Is an Average of Vertices, Not True Centroid

**File**: `ColliderComponent.h`

```cpp
for (const auto& vertex : vertices) {
    centroid = centroid + vertex;
}
centroid = centroid * (1.0f / vertices.size());
```

This is the **centroid of the vertex set** (unweighted average), not the **centroid of the polygon area**. For a convex polygon with unevenly-spaced vertices, these can differ significantly. The true centroid requires the shoelace-based area-weighted formula. For symmetric shapes (rectangles, regular polygons) this doesn't matter, but for irregular polygons it introduces angular dynamics errors.

---

### MS-04 — `PolygonShape::normals` Face Normals Are Not Normalized in `CalculateProperties`

**File**: `ColliderComponent.h`

```cpp
Math::Vector2 edge = vertices[next] - vertices[i];
Math::Vector2 normal = {-edge.y, edge.x};
float length = sqrt(normal.x * normal.x + normal.y * normal.y);
if (length > 0.0001f) {
    normal = normal * (1.0f / length);
    normals.push_back(normal);
}
```

The normalization is correct, but the outward-facing direction depends on the winding order of the vertices. If vertices are wound clockwise, `{-edge.y, edge.x}` points inward. If counter-clockwise, it points outward. No winding order check or enforcement is done. In `PhysicsDemoGame::CreateBox()`:

```cpp
boxShape.vertices = {
    {-halfWidth, -halfHeight},  // top-left
    {halfWidth, -halfHeight},   // top-right
    {halfWidth, halfHeight},    // bottom-right
    {-halfWidth, halfHeight}    // bottom-left
};
```

In screen-space (Y-down), this is counter-clockwise. `{-edge.y, edge.x}` for a rightward edge gives an upward normal — outward for a Y-down counter-clockwise polygon. This happens to be correct. But any game code that uses clockwise winding will produce inward normals, silently reversing all SAT collision responses.

---

### MS-05 — `PhysicsBodyComponent` `drag` and `linearDamping` Fields Overlap in Meaning

**File**: `PhysicsBodyComponent.h`, `PhysicsIntegrationSystem.h`

There are three separate damping/drag fields:
- `linearDamping` — subtracts `velocity * linearDamping` from acceleration
- `drag` — subtracts `velocity² * drag / mass` from acceleration  
- `maxSpeed` — a duplicate of `maxLinearSpeed` (both exist with the same default)

The documentation says `drag` is "air resistance coefficient" and `linearDamping` is "linear velocity decay (0–1)". But both are applied as accelerations, making them dimensionally inconsistent with their stated ranges. `linearDamping = 1.0` would add an acceleration of `-velocity` units/s², not actually decay velocity by 100%. The naming and range documentation is misleading.

---

### MS-06 — `InputManager::Update()` Loops Over All 512 GLFW Keys Every Frame

**File**: `InputManager.cpp`

```cpp
for (int i = 0; i < GLFW_KEY_LAST; i++)
    s_CurrentKeys[i] = (glfwGetKey(s_Window, i) == GLFW_PRESS);
```

`GLFW_KEY_LAST = 348`. This calls `glfwGetKey` 348 times per fixed-update tick. Since the fixed update runs at 60Hz and there may be multiple fixed-update steps per render frame (the accumulator pattern), this could fire hundreds of times per second. A callback-based approach (GLFW key callback + dirty flags) would be far more efficient.

---

### MS-07 — `Application::Run()` Fixed Update Loop Calls `ProcessInput()` Which Checks ESC Key

**File**: `Application.cpp`

```cpp
while (m_Accumulator >= FIXED_TIMESTEP)
{
    ProcessInput();  // checks GLFW_KEY_ESCAPE
    OnFixedUpdate(static_cast<float>(FIXED_TIMESTEP));
    m_Accumulator -= FIXED_TIMESTEP;
}
```

`ProcessInput()` is called once per physics substep. If the accumulator has built up 5 substeps (because the frame took 5× the fixed timestep), `ProcessInput()` is called 5 times before rendering. This is benign for key checking but means input state can be "consumed" multiple times for a single real key press. Combined with `InputManager::Update()` also being called inside `InputSystem::Update()`, key state may be updated mid-physics-step, causing inconsistent input reads across substeps.

---

### MS-08 — `ManifoldGenerator` Is Never Used

**Files**: `ManifoldGenerator.h`, `ManifoldGenerator.cpp`

The full manifold generation pipeline (circle-circle, circle-polygon, polygon-polygon) is implemented but never called by any system. `CollisionSystem` uses `Utils::CollisionPhysics` (a separate utility). `ContinuousCollisionSystem` uses inline AABB math. `CollisionPipelineSystem` has a placeholder comment where manifold generation should be. This represents dead code that duplicates functionality and adds compile time without contributing to runtime behavior.

---

### MS-09 — `ContactSolver` Is Also Dead Code

**File**: `ContactSolver.h`

`Physics::ContactSolver` declares `InitializeContacts`, `SolveVelocityConstraints`, and `SolvePositionConstraints` but provides no `.cpp` implementation visible in the repository. Even if it compiled, it is never called by any system.

---

## Low-Severity / Code Quality Issues

### LS-01 — Excessive Debug Logging in Hot Paths

**File**: `ContinuousCollisionSystem.cpp`, `ECSApplication.cpp`

```cpp
std::cerr << "[DEBUG] ECSApplication::OnFixedUpdate() called with delta time: " << deltaTime << std::endl;
```

This fires 60+ times per second. `std::endl` flushes the stream buffer on every call. Combined with the per-collision debug output in `ContinuousCollisionSystem`, this will dominate CPU time and make the engine appear slower than it is. All debug logging should be gated behind `#ifdef NYON_DEBUG` or a runtime flag.

---

### LS-02 — `Renderer2D::Init()` Called Twice (Once in `Application::Init()`, Once in `RenderSystem::Initialize()`)

**File**: `Application.cpp`, `RenderSystem.h`

`Application::Init()` calls `Graphics::Renderer2D::Init()`. Later, `RenderSystem::Initialize()` calls `Graphics::Renderer2D::Init()` again. The second call is safe because `Init()` checks for existing objects and deletes them before recreating — but this wastes initialization work and could cause a frame-zero flicker if anything was rendered between the two calls.

---

### LS-03 — `BehaviorComponent` Header Includes `<functional>` and `<vector>` but Vector Is Unused

**File**: `BehaviorComponent.h`

Minor include bloat; `std::vector` is not used in the component.

---

### LS-04 — `JointComponent` Uses a Raw `union` With Non-Trivial Members

**File**: `JointComponent.h`

The `union` contains `prismaticJoint.localAxisA` which is a `Math::Vector2`. `Vector2` has a non-trivial constructor. Using a union of structs with non-trivial members is undefined behavior in C++ unless carefully managed with placement new/explicit destruction. This will compile without warning but is technically UB.

---

### LS-05 — `AABB::RayCast` Uses `std::numeric_limits` Without Including `<limits>`

**File**: `DynamicTree.h`

`std::numeric_limits<float>::max()` and `std::numeric_limits<float>::epsilon()` are used in `AABB::RayCast` without an explicit `#include <limits>`. This compiles only because another included header transitively includes `<limits>`. It is a fragile dependency.

---

### LS-06 — `DynamicTree::Rebuild` Iterates by Index But Checks `height >= 0` Which Is Always True for `float`

**File**: `DynamicTree.cpp`

```cpp
if (m_nodes[i].height >= 0 && m_nodes[i].IsLeaf())
```

`height` is declared as `float`. `float >= 0` is almost always true (only NaN would fail). The intent was probably to check for allocated (non-free-list) nodes. Free nodes in the pool have their `parent` overwritten to point to the next free node, but `height` is not set to a sentinel value. This means `Rebuild()` may process free nodes as if they were valid proxies.

---

### LS-07 — `TransformComponent` Rotation Stored in Radians But Applied as Degrees

**File**: `PhysicsDemoGame.cpp`

```cpp
rampTransform.rotation = 25.0f;  // comment-free; is this degrees or radians?
```

`PhysicsIntegrationSystem` treats rotation as radians (wraps at 2π). If the ramp is set to `25.0f` radians (≈ 1432°, ≈ 352° net) this is clearly wrong. If intended as 25°, it should be `25.0f * (M_PI / 180.0f) ≈ 0.436f`. The `DebugRenderSystem::DrawPolygonShape` uses `cos(angle)` and `sin(angle)`, treating it as radians. The inconsistency means the ramp is rotated to an unintended angle.

---

### LS-08 — `PhysicsWorldComponent::callbacks` Functions Are Never Called in Normal Collision Flow

**File**: `PhysicsWorldComponent.h`, `CollisionPipelineSystem.cpp`

`beginContact` and `endContact` callbacks are invoked in `CollisionPipelineSystem::UpdateContacts()` — but as established in CA-01, this system never runs (null `m_PhysicsWorld`). Game code that registers callbacks via `SetBeginContactCallback()` will never receive events.

---

### LS-09 — `ColliderComponent::CalculateAABB` Does Not Account for Rotation

**File**: `ColliderComponent.h`

```cpp
case ShapeType::Polygon:
    for (const auto& vertex : polygon.vertices)
    {
        Math::Vector2 worldVertex = vertex + position;  // no rotation applied
```

Polygon vertices are offset by position only. If the entity has a non-zero `TransformComponent::rotation`, the AABB is computed for the unrotated polygon — which may not enclose the actual rotated shape at all. This makes broad-phase AABB detection incorrect for any rotated entity.

---

## Architectural Limitations

### AL-01 — No True Interpolation Between Physics Frames

`Application::Run()` computes `alpha = m_Accumulator / FIXED_TIMESTEP` and passes it to `OnInterpolateAndRender(alpha)`. `ECSApplication::OnInterpolateAndRender` does nothing — it doesn't store previous physics state or interpolate. `RenderSystem` renders current positions, not interpolated positions. The result is that objects jitter at the physics timestep rate rather than rendering smoothly at the display refresh rate.

**Fix**: Store `previousPosition` and `currentPosition` in `TransformComponent` (or a separate interpolation component). The render step should draw at `previousPosition + (currentPosition - previousPosition) * alpha`.

---

### AL-02 — `ComponentStore` Uses `unordered_map` Per Component Type — Not Cache-Friendly

**File**: `ComponentStore.h`

The comment says "Structure of Arrays pattern" but the implementation is actually "Array of Structures" per component type (`unordered_map<EntityID, T>`). True SoA would be `vector<T>` indexed by a dense entity index. The hash map causes cache misses on every component access during iteration. For physics simulations iterating thousands of entities per frame, this is a significant performance limitation.

---

### AL-03 — No Broad-Phase Filtering Based on Layer/Group in Practice

`ColliderComponent::Filter` implements category/mask bits and group indices. `Filter::ShouldCollide()` is fully implemented. But no collision system actually calls `ShouldCollide()` before generating contact pairs. All entities collide with all other entities regardless of filter settings.

---

### AL-04 — `DynamicTree` `height` Field Stored as `float` Instead of `int`

**File**: `DynamicTree.h`

```cpp
float height;
```

Tree height is always an integer. Using `float` wastes precision, requires `static_cast<int>()` at every comparison site, and is the root cause of LS-06. Should be `int32_t`.

---

### AL-05 — No Broad-Phase Island Detection or Sleeping Optimization

All dynamic bodies are processed every physics tick. Box2D's performance largely comes from grouping non-interacting bodies into separate islands and sleeping entire islands when stable. Nyon has `isAwake` but no island grouping. This means a scene with 100 resting objects still processes all 100×100 potential contact pairs every frame.

---

### AL-06 — No Separation Between Collision Detection and Collision Response

The engine has the concept (`CollisionPipelineSystem` detects, `ConstraintSolverSystem` resolves) but the actual resolution happens in `ContinuousCollisionSystem::PerformCCD` — bypassing the pipeline entirely. The pipeline and solver are skeletons. Unifying these into a coherent detect → manifold → solve → integrate pipeline is the fundamental architectural work needed.

---

## Per-System Breakdown

| System | Status | Primary Issue |
|---|---|---|
| `PhysicsIntegrationSystem` | Partially working | Double gravity (HS-05), no grounded check (CI-02) |
| `ContinuousCollisionSystem` | Broken | Wrong normals (CB-02), dead resolution path (CB-07), sign errors (CB-03) |
| `CollisionSystem` | Dead code | Never registered, references undefined types (HS-06) |
| `CollisionPipelineSystem` | Never runs | Null world pointer (MS-02), stale entity cache (CA-01), shapeId=0 conflict (CA-04) |
| `ConstraintSolverSystem` | Mostly dead | Double gravity (HS-05), empty constraint lists (HS-05) |
| `TransformPhysicsSyncSystem` | Harmful | Double-integrates position (CI-01), wrong interpolation factor |
| `RenderSystem` | Working | Renders uninterpolated positions (AL-01) |
| `DebugRenderSystem` | Incorrectly placed | Renders in physics loop, not render loop (CA-03) |
| `InputSystem` | Working | Inefficient polling (MS-06) |

---

## Recommended Fix Priority

### Priority 1 — Must Fix to Get Basic Physics Working

1. **Remove `TransformPhysicsSyncSystem`** from the system registration (CI-01). This single change will likely stop the teleportation behavior.
2. **Create `PhysicsWorldComponent` entity before adding systems** (MS-02). This enables `CollisionPipelineSystem` and `ConstraintSolverSystem` to actually run.
3. **Fix the normal direction convention** in `ContinuousCollisionSystem` (CB-03). Choose "B to A" or "A to B" and apply it consistently to both position correction and impulse.
4. **Wire `ResolveCCDCollision` into `PerformCCD`** (CB-07). This is the most complete resolution function in the file.
5. **Stop `ConstraintSolverSystem` from applying gravity** (HS-05). Remove `IntegrateVelocities()` from the solver — integration is already done by `PhysicsIntegrationSystem`.

### Priority 2 — Needed for Stable Collisions

6. **Make systems query entities dynamically** (CA-01). Remove all `Initialize()`-time pointer caching and query `ComponentStore` in `Update()`.
7. **Fix system execution order** (CA-02). Move `RenderSystem` and `DebugRenderSystem` out of the fixed-update loop.
8. **Fix AABB calculation to account for rotation** (LS-09).
9. **Unify collision geometry** — use AABB-AABB swept TOI or SAT throughout, not a sphere TOI with AABB resolution (CB-02).

### Priority 3 — Correctness Improvements

10. **Fix `PhysicsBodyComponent::UpdateMassProperties()`** to accept shape dimensions (HS-02).
11. **Add entity destruction notification to all systems** (HS-04).
12. **Implement proper `Filter::ShouldCollide()` checks** in collision systems (AL-03).
13. **Fix `PolygonShape::CalculateProperties()` centroid** to use area-weighted formula (MS-03).

### Priority 4 — Performance and Polish

14. Gate all `std::cerr` debug output behind a compile-time flag (LS-01).
15. Switch `InputManager` to GLFW callback model (MS-06).
16. Implement render interpolation using `alpha` from the game loop (AL-01).
17. Switch `ComponentStore` internals to dense arrays for cache performance (AL-02).
18. Implement island-based sleeping (AL-05).

---

*Report generated from static analysis of the Nyon Engine repository (master branch). All line references are approximate and based on the provided file listing.*
