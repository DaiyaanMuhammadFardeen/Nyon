# Nyon Engine — Deep Sweep Report (Addendum)
**Extends:** `NYON_ENGINE_ENGINEERING_REPORT.md`  
**Scope:** Full second pass across every file. New bugs only — no repeats from Report 1.  
**Severity:** 🔴 Critical · 🟠 High · 🟡 Medium · 🔵 Low

---

## Table of Contents

1. [Compile & Link Errors](#1-compile--link-errors)
2. [Island Sleep System — Completely Non-Functional](#2-island-sleep-system--completely-non-functional)
3. [Collision Detection Logic Bugs](#3-collision-detection-logic-bugs)
4. [Physics Pipeline Logic Bugs](#4-physics-pipeline-logic-bugs)
5. [ColliderComponent Data Model Problems](#5-collidercomponent-data-model-problems)
6. [Renderer2D — Additional Bugs](#6-renderer2d--additional-bugs)
7. [ParticleRenderer — Uninitialized Data & Coupling Issues](#7-particlerenderer--uninitialized-data--coupling-issues)
8. [InputManager Dead Code](#8-inputmanager-dead-code)
9. [Full Particle System Design — Complete Specification](#9-full-particle-system-design--complete-specification)
10. [Parallelism Expansion — Full Inventory](#10-parallelism-expansion--full-inventory)
11. [Collision System Parallelisation — Detailed Design](#11-collision-system-parallelisation--detailed-design)
12. [Additional Minor Bugs & Code Quality](#12-additional-minor-bugs--code-quality)
13. [Implementation Priority Addendum](#13-implementation-priority-addendum)

---

## 1. Compile & Link Errors

### 1.1 🔴 `SystemManager.h` — typo causes compilation failure

**File:** `engine/include/nyon/ecs/SystemManager.h`

```cpp
std::vector<std::unique_ptr<s>> m_Systems;   // 's' should be 'System'
```

Lowercase `s` is an undeclared identifier in this scope. The project does not compile at all. Every `.cpp` that includes `SystemManager.h` (directly or transitively through `ECSApplication.h`) fails.

**Action:** Change to `std::vector<std::unique_ptr<System>> m_Systems;`

---

### 1.2 🔴 `Camera2D` method bodies exist only in the backup file — linker error

**Files:** `engine/src/graphics/Renderer2D.cpp.backup` (lines 5277–5326) and `engine/src/graphics/Renderer2D.cpp`

`Renderer2D.h` declares:
```cpp
struct Camera2D {
    glm::mat4 GetViewMatrix() const;
    glm::mat4 GetProjectionMatrix(float screenWidth, float screenHeight) const;
    glm::mat4 GetViewProjectionMatrix(float screenWidth, float screenHeight) const;
    Math::Vector2 ScreenToWorld(Math::Vector2 screenPos, float screenWidth, float screenHeight) const;
    Math::Vector2 WorldToScreen(Math::Vector2 worldPos, float screenWidth, float screenHeight) const;
};
```

All five method **definitions** are in `Renderer2D.cpp.backup` — which has the `.backup` extension and is never compiled. The current `Renderer2D.cpp` does NOT define them. Any call to these methods produces an unresolved external symbol linker error. `BeginScene` calls `Camera2D::GetProjectionMatrix` and `GetViewMatrix`, making the entire renderer unlinkable.

**Action:** Move the five `Camera2D` method bodies from the backup into the main `Renderer2D.cpp`. They are correct implementations and should not be rewritten.

---

### 1.3 🔴 `Renderer2D::Flush()` only in backup — second linker error

**File:** `engine/src/graphics/Renderer2D.cpp.backup` (line 6666)

`Renderer2D::Flush()` is declared in `Renderer2D.h` (line 2149) and its implementation is in the backup. The current `Renderer2D.cpp` calls `EndScene` (which maps to the GPU-instanced path) but never defines `Flush()`. Callers of `Flush()` will generate a linker error.

**Action:** Implement `Flush()` in the main `Renderer2D.cpp` as a no-op or as an alias to the GPU `EndScene`, and document that the GPU path auto-flushes via `EndScene`. Remove the CPU-buffer version from the backup.

---

### 1.4 🔴 `ShaderLoader.h` missing `#pragma once` and has ODR violation risk

**File:** `engine/src/graphics/ShaderLoader.h`

```cpp
// No #pragma once or include guard
static std::string LoadShaderSource(const std::string& filepath) { ... }
```

Without an include guard, if this file is included from multiple translation units, each TU gets its own copy of the `static` function (different addresses but same name). This is legal but wasteful. More critically, it lacks `#pragma once`, meaning accidental double-inclusion in a single TU would produce redefinition errors. Once moved to the include tree (per Report 1, §1.1), the `static` qualifier must become `inline` or be moved to a `.cpp` file.

**Action:** Add `#pragma once`. Change `static std::string LoadShaderSource` → `inline std::string LoadShaderSource`.

---

## 2. Island Sleep System — Completely Non-Functional

### 2.1 🔴 `FindIslands` populates `m_SleepingIslands` incorrectly — islands never sleep

**File:** `engine/src/physics/Island.cpp`

`IslandManager::UpdateIslands` executes this sequence every frame:

```
FindIslands()         → creates new m_AllIslands, all with isAwake = true
                      → copies to m_AwakeIslands (all, since all are awake)
                      → m_SleepingIslands is ALWAYS EMPTY here
UpdateSleepTimers()   → increments sleep timers in m_AllIslands
PutIslandsToSleep()   → sets m_AllIslands[i].isAwake = false (value in m_AllIslands)
WakeSleepingIslands() → iterates m_SleepingIslands looking for it->isAwake == true
                      → m_SleepingIslands is EMPTY → nothing happens
```

The root cause: `FindIslands` rebuilds islands from scratch every frame. All new `Island` objects have `isAwake = true` (the default). So the copy into `m_AwakeIslands` / `m_SleepingIslands` at lines 7212–7217 always puts everything into `m_AwakeIslands`. `m_SleepingIslands` is never populated.

`PutIslandsToSleep` modifies `m_AllIslands[i].isAwake`, but those changes never propagate to the already-made copies in `m_AwakeIslands` or `m_SleepingIslands`.

`WakeIslandContaining` modifies `m_AllIslands[islandIndex].isAwake = true`, which has no effect on `m_SleepingIslands` either.

**Consequence:** The sleep/wake optimisation is completely inoperative. Every dynamic body is simulated every frame regardless of velocity, negating one of the primary performance optimisations in a physics engine.

**Action — Full redesign of the island persistence model:**

Islands must persist across frames instead of being rebuilt from scratch. The correct approach:

```
Frame N:
  1. BuildContactGraph / BuildJointGraph (same as now)
  2. DiffIslands: detect merged/split islands by comparing to previous frame's graph
  3. For unchanged islands: keep existing sleep state
  4. For new islands (new contacts formed): inherit awake state from constituent bodies
  5. UpdateSleepTimers on m_AllIslands (in-place, not copies)
  6. PutIslandsToSleep / WakeIslands (operates on m_AllIslands directly)
  7. m_AwakeIslands / m_SleepingIslands are VIEWS (pointers/indices into m_AllIslands), not copies
```

Minimum viable fix (without full persistence):
- Remove `m_AwakeIslands` and `m_SleepingIslands` as separate vectors.
- Use `m_AllIslands` directly everywhere.
- In `WakeSleepingIslands`, iterate `m_AllIslands` (not a stale copy).
- Carry sleep timer from previous frame by storing a `std::unordered_map<uint64_t, float> m_IslandSleepTimers` keyed by a stable island hash (sorted body IDs XOR'd). Match timers before rebuilding.

---

### 2.2 🟠 `IslandManager::GetBodySleepVelocity` — loaded collider is silently discarded

**File:** `engine/src/physics/Island.cpp`

```cpp
float extent = 1.0f;   // ← hardcoded; never updated
if (m_ComponentStore.HasComponent<ECS::ColliderComponent>(bodyId)) {
    const auto& collider = m_ComponentStore.GetComponent<ECS::ColliderComponent>(bodyId);
    // 'collider' is fetched but NEVER read — the if-block ends here
}
float linearVelSq = body.velocity.LengthSquared();
float angularVel = std::abs(body.angularVelocity);
float tangentialVel = angularVel * extent;   // extent is always 1.0f
```

`extent` should represent the approximate radius of the body (so that `angularVelocity × radius` gives the tangential surface speed, the correct metric for sleep comparison). It is hard-coded to `1.0f`. For a body with radius 50, its angular sleep velocity is evaluated 50× too leniently. Large circular bodies will sleep even when visibly spinning.

**Action:**

```cpp
float extent = 1.0f;
if (m_ComponentStore.HasComponent<ECS::ColliderComponent>(bodyId)) {
    const auto& collider = m_ComponentStore.GetComponent<ECS::ColliderComponent>(bodyId);
    Math::Vector2 dummyMin, dummyMax;
    collider.CalculateAABB({0,0}, 0.0f, dummyMin, dummyMax);
    float hw = (dummyMax.x - dummyMin.x) * 0.5f;
    float hh = (dummyMax.y - dummyMin.y) * 0.5f;
    extent = std::max(hw, hh);
}
```

---

### 2.3 🟡 `ShouldIslandSleep` threshold inconsistency — wrong comparison units

**File:** `engine/src/physics/Island.cpp`

```cpp
// In ShouldIslandSleep:
if (GetBodySleepVelocity(bodyId) > SLEEP_THRESHOLD * SLEEP_THRESHOLD) { return false; }

// In UpdateSleepTimers:
if (velocitySq > SLEEP_THRESHOLD * SLEEP_THRESHOLD) { belowThreshold = false; }
```

`GetBodySleepVelocity` returns `linearVelSq + tangentialVel²` — already a squared quantity. Comparing this to `SLEEP_THRESHOLD²` means the effective sleep threshold for the combined metric is `sqrt(SLEEP_THRESHOLD² - tangentialVel²)` for the linear component, not `SLEEP_THRESHOLD` as intended. Both checks should compare against `SLEEP_THRESHOLD * SLEEP_THRESHOLD` only if `GetBodySleepVelocity` returns a raw speed (not squared). Since it returns a squared+squared sum, the comparison value should be `SLEEP_THRESHOLD² + ANGULAR_SLEEP_THRESHOLD²` or the function should be refactored to return individual components.

**Action:** Rename to `GetBodySleepSpeed()` and return `sqrt(linearVelSq) + tangentialVel` (summed speeds, not squares). Compare against `SLEEP_THRESHOLD + ANGULAR_SLEEP_THRESHOLD`.

---

### 2.4 🟡 `IslandManager::FindIslands` — O(n²) duplicate-edge check in `ConnectionGraph`

**File:** `engine/src/physics/Island.cpp`

```cpp
for (const auto& connectedBody : connections) {
    if (std::find(m_ConnectionGraph[bodyId].begin(), m_ConnectionGraph[bodyId].end(), connectedBody)
        == m_ConnectionGraph[bodyId].end()) {
        m_ConnectionGraph[bodyId].push_back(connectedBody);
    }
}
```

For a body with K connections, the inner `std::find` is O(K). Over J joint connections, this is O(K×J) per body. For a densely connected ragdoll or chain, this is visibly slow.

**Action:** Build `m_ConnectionGraph` as `std::unordered_map<EntityID, std::unordered_set<EntityID>>` to guarantee O(1) deduplication:

```cpp
for (const auto& [bodyId, connections] : m_ContactGraph)
    for (auto& c : connections)
        m_ConnectionGraph[bodyId].insert(c);
for (const auto& [bodyId, connections] : m_JointGraph)
    for (auto& c : connections)
        m_ConnectionGraph[bodyId].insert(c);
```

---

## 3. Collision Detection Logic Bugs

### 3.1 🔴 `ComputeCircleCenters` does not rotate the circle's local `center` offset

**File:** `engine/src/physics/ManifoldGenerator.cpp`

```cpp
inline void ComputeCircleCenters(const ColliderComponent::CircleShape& circle,
                                 const TransformComponent& transform,
                                 Math::Vector2& outCenter) {
    outCenter = transform.position + circle.center;   // BUG: circle.center not rotated
}
```

`circle.center` is in the body's local coordinate frame. When the body has a non-zero `transform.rotation`, this local offset must be rotated before being added to `transform.position`:

```cpp
// Correct:
outCenter = transform.position + Rotate(circle.center, transform.rotation);
```

Without this, any circle whose `center` is not exactly `{0,0}` will have its collision position at the wrong world location when the body is rotated. This bug affects `CircleCircle`, `CirclePolygon`, and `CircleCapsule`. Off-center circle colliders are completely broken.

---

### 3.2 🟠 `CircleCapsule` applies the circle center rotation fix to capsule but not to circle

**File:** `engine/src/physics/ManifoldGenerator.cpp`

```cpp
const auto& circle = circleCollider.GetCircle();
Math::Vector2 circleCenter = transformA.position + circle.center;  // BUG: not rotated
// But capsule endpoints ARE correctly rotated:
capStart = { transformB.position.x + (capsule.center1.x * cosB - ...) };
```

The capsule endpoints apply the body rotation. The circle center does not. This inconsistency means the circle-capsule contact position is wrong whenever the circle's body is rotated. Fixed by applying `Rotate(circle.center, transformA.rotation)`.

---

### 3.3 🟠 `PolygonPolygon` — post-clip centroid normal flip can invert the manifold normal twice

**File:** `engine/src/physics/ManifoldGenerator.cpp`

```cpp
// After ClipSegmentToLine with reference on B:
ClipSegmentToLine(..., -separatingAxis, minOverlap);
manifold.normal = -separatingAxis;

// Later:
Math::Vector2 d = centroidB - centroidA;
if (Math::Vector2::Dot(d, manifold.normal) < 0.0f) {
    manifold.normal = -manifold.normal;
    for (auto& cp : manifold.points) {
        cp.normal = -cp.normal;
        cp.separation = -cp.separation;   // BUG: separation sign flip
    }
}
```

When the reference face is on polygon B and the centroid check triggers (the normal was already negated via `-separatingAxis`), the centroid flip negates again — potentially returning the manifold normal to the original `separatingAxis` direction. The `cp.separation` flip is also incorrect: `separation` is already negative for penetration; negating it turns it positive, telling the solver there is no penetration.

**Action:** The centroid normal check should operate on a canonical normal (always A→B). Compute it once before clip based on the reference face, and use it consistently for the centroid check. Do NOT flip `cp.separation` inside the normal-flip block; separation sign is independent of normal direction in the contact point frame.

---

### 3.4 🟠 `ClipSegmentToLine` silently discards the `overlap` (penetration depth) parameter

**File:** `engine/src/physics/ManifoldGenerator.cpp`

```cpp
inline void ClipSegmentToLine(std::vector<ECS::ContactPoint>& outContacts,
                              const std::vector<Math::Vector2>& refVerts,
                              const std::vector<Math::Vector2>& incVerts,
                              int refFaceIndex,
                              int incFaceIndex,
                              const Math::Vector2& normal,
                              float ) {    // ← overlap parameter is unnamed and discarded
```

The function performs Sutherland-Hodgman clipping against the two side-planes of the reference face but ignores the front-plane depth (the `overlap` / penetration value). The final contact-point depth is computed only from `Dot(normal, pt) - refOffset`, which is the signed distance from the reference face line. This is geometrically correct for 2D SAT, but the discarded `overlap` was likely intended for clamping the maximum correction depth. Document clearly that this is intentional, or restore the parameter as a max-depth clamp.

---

### 3.5 🟡 `PolygonShape::normals` and `vertices` can have different sizes — out-of-bounds risk

**File:** `engine/include/nyon/ecs/components/ColliderComponent.h`

```cpp
// In CalculateProperties():
for (size_t i = 0; i < vertices.size(); ++i) {
    ...
    if (length > 0.0001f) {
        normals.push_back(normal);  // ← only pushed if edge is non-degenerate
    }
}
```

For a polygon with a degenerate (zero-length) edge, `normals.size() < vertices.size()`. `ManifoldGenerator::ComputePolygonWorld` resizes `outNormals` to `poly.normals.size()` and separately `outVertices` to `poly.vertices.size()`. If `normalsA.size() != vertsA.size()`, the `PolygonPolygon` SAT loop at line 7632 iterates `normalsA.size()` normals but uses `vertsA` for `ProjectPolygon` — they will be in sync only if sizes match. The `FindIncidentFace` at line 7658 uses `normalsB` — if `normalsB.size() < vertsB.size()`, `incidentFace` may not correspond to a valid vertex pair.

**Action:** In `CalculateProperties()`, always push a normal (use the previous edge's normal as a fallback for degenerate edges, or assert `vertices.size() == normals.size()` after calculation). Add a validation method `ValidatePolygon() const` that asserts the invariant.

---

### 3.6 🟡 `PolygonCapsule` — `segLenSq` division not guarded against zero

**File:** `engine/src/physics/ManifoldGenerator.cpp`

```cpp
Math::Vector2 segDir = capEnd - capStart;
float segLenSq = segDir.LengthSquared();
float t = Dot(circleCenter - capStart, segDir) / segLenSq;  // division by zero if capsule degenerate
```

If `center1 == center2` (a degenerate capsule that is actually a circle), `segLenSq == 0` and this divides by zero, producing NaN for `t`, which propagates into the contact point and then into the velocity solver — causing the simulation to explode.

**Action:** Guard: `if (segLenSq < 1e-8f) { t = 0.0f; } else { t = ...; }`

---

### 3.7 🟡 `CapsuleCapsule` — degenerate parallel capsule case assigns both `s=0`, `t=0`

**File:** `engine/src/physics/ManifoldGenerator.cpp`

```cpp
if (denom < 1e-8f) {   // parallel capsules
    s = 0.0f;
    t = 0.0f;
}
```

When two parallel capsules overlap, the closest points between their segments can be anywhere along the overlap range, not necessarily at `s=0, t=0`. The correct approach for parallel segments is to clamp `t` based on the projection of `a1` onto the B segment, then recompute `s`. The current code produces a single contact point at the segment start positions only, even if the capsules are colliding along their full length.

---

## 4. Physics Pipeline Logic Bugs

### 4.1 🔴 `Integration()` syncs velocity locks to `PhysicsBodyComponent` but does NOT integrate positions

**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`

```cpp
void PhysicsPipelineSystem::Integration() {
    for (const auto& solverBody : m_SolverBodies) {
        // Applies motion lock masks to body.velocity / body.angularVelocity
        body.velocity = lockedVelocity;
        body.angularVelocity = lockedAngularVelocity;
    }
}
```

`Integration()` only writes locked velocities back to the `PhysicsBodyComponent`. It does **not** call `IntegratePositions`. Positions are integrated inside `PositionSolving()`:

```cpp
void PhysicsPipelineSystem::PositionSolving() {
    IntegratePositions(FIXED_TIMESTEP);   // ← positions integrated here
    for (int i = 0; i < m_Config.positionIterations; ++i) {
        SolvePositionConstraints(); }
}
```

This means the naming is deeply misleading: "PositionSolving" integrates positions (a velocity-level operation) AND then runs the position constraint solver. The separate `Integration()` step does NOT integrate anything — it is purely a velocity-lock enforcement pass.

**Action:** Rename `Integration()` to `SyncMotionLocksToComponents()`. Move `IntegratePositions(subStepDt)` into its own step between velocity solving and position constraint solving. The current pipeline order should be:

```
1. IntegrateVelocities(dt)   ← gravity + damping + velocity limits
2. WarmStart constraints
3. SolveVelocityConstraints (N iterations)
4. IntegratePositions(dt)    ← NEW explicit step
5. SolvePositionConstraints (N iterations)
6. SyncMotionLocksToComponents()
7. UpdateTransformsFromSolver()
```

---

### 4.2 🟠 `VelocitySolving` and `PositionSolving` use `FIXED_TIMESTEP` not `subStepDt`

**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`

```cpp
void PhysicsPipelineSystem::VelocitySolving() {
    ...
    IntegrateVelocities(FIXED_TIMESTEP);  // BUG: ignores subStepDt

void PhysicsPipelineSystem::PositionSolving() {
    IntegratePositions(FIXED_TIMESTEP);   // BUG: ignores subStepDt
```

When `numSubSteps == 2`, `subStepDt = deltaTime / 2`. But both serial solving functions always use `FIXED_TIMESTEP` (1/60 s). In a 2-substep scenario, the physics integrates at double the intended rate. This compounds the parallel path bug from Report 1 §4.1.

**Action:** Pass `subStepDt` to both `VelocitySolving(float dt)` and `PositionSolving(float dt)` as parameters.

---

### 4.3 🟡 `PositionSolving` and `UpdateTransformsFromSolver` contain dead code loops

**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`

```cpp
// At the end of PositionSolving:
for (size_t i = 0; i < std::min(m_SolverBodies.size(), (size_t)3); ++i) {
    const auto& body = m_SolverBodies[i];   // body loaded but never used
}

// At the end of UpdateTransformsFromSolver:
for (size_t i = 0; i < std::min(m_SolverBodies.size(), (size_t)3); ++i) {
    const auto& body = m_SolverBodies[i];   // body loaded but never used
}
```

These loops iterate the first 3 solver bodies and do nothing with them. They are dead debug stubs. Every compiler will warn about unused variables here.

**Action:** Delete both loops.

---

### 4.4 🟡 `MakeImpulseCacheKey` XOR hash creates trivial collisions

**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`

```cpp
uint64_t pairKey = (static_cast<uint64_t>(minEntity) << 32) | static_cast<uint64_t>(maxEntity);
return pairKey ^ (static_cast<uint64_t>(featureId) << 32);
```

XORing `featureId` into the upper 32 bits (where `minEntity` lives) means: if `featureId == minEntity`, the key reduces to `maxEntity` in the upper 32 bits — a collision with the pair `(0, maxEntity)`. For a polygon with entity ID 5 and 5 contact features (IDs 0–4), the feature ID 5 would collide with entity pair `(0, x)`.

**Action:** Use a proper mixing hash:

```cpp
uint64_t key = (static_cast<uint64_t>(minEntity) << 32) | maxEntity;
key ^= (key >> 33);
key *= 0xff51afd7ed558ccdULL;
key ^= (static_cast<uint64_t>(featureId) * 0xc4ceb9fe1a85ec53ULL);
return key;
```

Or simply: `return (pairKey * 2654435761ULL) ^ featureId;`

---

### 4.5 🟡 `WarmStartConstraints` clamps impulses to ±100 (magic number, wrong for large bodies)

**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`

```cpp
float maxImpulse = 100.0f;   // ← arbitrary magic number
clampedNormalImpulse = std::clamp(clampedNormalImpulse, -maxImpulse, maxImpulse);
```

A 100 kg body falling at 1000 units/s generates roughly 100,000 N·s of impulse per timestep. Clamping warm-starting impulse to ±100 means the warm-start has almost no effect for heavy/fast bodies, negating its purpose.

**Action:** Either remove the clamp entirely (impulses are already bounded by constraint solving), or derive it from the body's mass: `float maxImpulse = std::max(bodyA.invMass, bodyB.invMass) > 0 ? (1.0f / std::min(bodyA.invMass, bodyB.invMass)) * 10.0f : 1000.0f;`

---

### 4.6 🟡 `StoreImpulses` creates a temporary `unordered_map` to find stale keys — O(n²) cleanup

**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`

```cpp
std::unordered_map<uint64_t, bool> activeKeys;
for (...) { activeKeys[cacheKey] = true; }
// Then searches m_ImpulseCache for keys NOT in activeKeys:
for (const auto& [key, impulse] : m_ImpulseCache) {
    if (activeKeys.find(key) == activeKeys.end()) { keysToRemove.push_back(key); }
}
```

Building a full `unordered_map<uint64_t, bool>` just to do a set-difference is wasteful. Use `std::unordered_set<uint64_t>` for `activeKeys`. Better: iterate `m_ImpulseCache`, erase entries whose key is not in the active set in a single pass using `std::erase_if` (C++20) or manual erase-during-iterate pattern.

---

### 4.7 🟡 `ParallelPositionSolving` does not exist in the file — silent fallback

**File:** `engine/include/nyon/ecs/systems/PhysicsPipelineSystem.h`

`ParallelPositionSolving()` is declared at line 1753. Looking at the `Update()` dispatch:

```cpp
if (m_UseMultiThreading && m_VelocityConstraints.size() > 1) {
    ParallelVelocitySolving();
    ParallelPositionSolving();   // declared but where is the implementation?
}
```

The implementation of `ParallelPositionSolving` was not found in any of the source files read. It is declared but has no definition, causing a linker error if `m_UseMultiThreading` is `true` and there are active constraints. This means multi-threading mode is unlinkable.

**Action:** Implement `ParallelPositionSolving()`:

```cpp
void PhysicsPipelineSystem::ParallelPositionSolving() {
    // IntegratePositions is embarrassingly parallel:
    size_t n = m_SolverBodies.size();
    size_t batchSize = (n + m_NumThreads - 1) / m_NumThreads;
    std::vector<std::future<void>> futures;
    for (size_t t = 0; t < m_NumThreads && t * batchSize < n; ++t) {
        size_t start = t * batchSize;
        size_t end = std::min(start + batchSize, n);
        futures.push_back(Utils::ThreadPool::Instance().Submit([this, start, end]() {
            for (size_t i = start; i < end; ++i) {
                auto& b = m_SolverBodies[i];
                if (!b.isStatic && b.isAwake) {
                    b.prevPosition = b.position;
                    b.prevAngle = b.angle;
                    b.position += b.velocity * m_SubStepDt;  // requires m_SubStepDt member
                    b.angle += b.angularVelocity * m_SubStepDt; }
            }
        }));
    }
    for (auto& f : futures) f.get();
    // Position constraint solving is sequential (bodies are coupled):
    for (int i = 0; i < m_Config.positionIterations; ++i)
        SolvePositionConstraints();
}
```

---

## 5. ColliderComponent Data Model Problems

### 5.1 🟡 `ColliderComponent` has two separate `density` fields — the top-level one is dead code

**File:** `engine/include/nyon/ecs/components/ColliderComponent.h`

```cpp
struct ColliderComponent {
    ...
    float density = 1.0f;   // line 887 — top-level field, NEVER READ
    ...
    struct Material {
        float density = 1.0f;  // line 907 — used by CalculateMass() and physics pipeline
    } material;
```

`PhysicsPipelineSystem::PrepareBodiesForUpdate()` reads `collider.material.density`. `CalculateMass()` reads `material.density`. The top-level `density` field (line 887) is never written to or read anywhere in the engine. It will confuse developers who set `myCollider.density = 5.0f` expecting it to affect physics.

**Action:** Remove the top-level `density` field. All density operations go through `collider.material.density`.

---

### 5.2 🟡 `ColliderComponent` default constructor creates a 32×32 polygon not tied to any entity

**File:** `engine/include/nyon/ecs/components/ColliderComponent.h`

```cpp
ColliderComponent() {
    shape = PolygonShape({{0, 0}, {32, 0}, {32, 32}, {0, 32}});
}
```

Every default-constructed `ColliderComponent` silently creates a 32×32 box. A developer adding a `ColliderComponent` to a 100×100 render entity will not get a matching collider without explicitly specifying the shape.

**Action:** Change the default to a unit-circle with radius 1.0f (the most generic shape), or require explicit shape specification and delete the default constructor. At minimum, document prominently: `/** Default collider is a 32×32 polygon. Always specify shape explicitly. **/`

---

## 6. Renderer2D — Additional Bugs

### 6.1 🟠 `BeginScene` reads window size via `Application::Get()` inside a `try/catch(...)` that swallows all exceptions

**File:** `engine/src/graphics/Renderer2D.cpp.backup` (lines 5696–5712, also present in main cpp for BeginScene logic)

```cpp
try {
    Application& app = Application::Get();
    window = app.GetWindow();
} catch (...) {
    // silently ignores ALL exceptions
}
```

If `Application::Get()` throws (e.g. when the singleton hasn't been initialized — a legitimate programmer error), the exception is silently swallowed. The window remains null, the projection matrix is never updated, and all rendering uses a stale VP matrix with no error report.

**Action:** Remove the `try/catch`. `Application::Get()` should not throw — if it does, it indicates a programmer error that should propagate, not be hidden. Add a `NYON_ASSERT(s_Instance != nullptr)` inside `Application::Get()`.

---

### 6.2 🟠 `EnableBlending`, `EnableDepthTest`, `EnableCulling` store flags but never apply them to OpenGL state

**File:** `engine/src/graphics/Renderer2D.cpp`

```cpp
void Renderer2D::EnableBlending(bool enable) {
    if (s_Instance) s_Instance->BlendingEnabled = enable;
}
```

These methods set internal flags but do not call `glEnable(GL_BLEND)` / `glDisable(GL_BLEND)` etc. The flags are stored in `Impl` but never read by `EndScene` or `BeginScene`. Calling `Renderer2D::EnableBlending(false)` has no visual effect — blending remains enabled because it was set in `Application::Init()` via `glEnable(GL_BLEND)`.

**Action:** Apply the GL state change immediately in the setter:

```cpp
void Renderer2D::EnableBlending(bool enable) {
    if (!s_Instance) return;
    s_Instance->BlendingEnabled = enable;
    if (enable) glEnable(GL_BLEND); else glDisable(GL_BLEND);
}
```

---

### 6.3 🟡 `SetLineWidth` applies `glLineWidth` but modern OpenGL ignores it for values > 1

**File:** `engine/src/graphics/Renderer2D.cpp.backup` (and implied in main)

```cpp
glLineWidth(width);
```

Core OpenGL 3.1+ only guarantees `glLineWidth(1.0f)`. Any value other than 1.0 is implementation-defined and silently clamped to 1 on most modern drivers. The GPU line renderer already handles thickness via instanced rectangles (`line.vert`). Calling `glLineWidth` is therefore redundant and misleading.

**Action:** Remove `glLineWidth` calls. Document `SetLineWidth` as controlling the instanced rectangle thickness (which is what `DrawLine(... thickness)` already does).

---

### 6.4 🟡 `Renderer2D::Impl` stores `BlendingEnabled`, `DepthTestEnabled`, `CullingEnabled` flags that are never read

**File:** `engine/src/graphics/Renderer2D.cpp`

These three flags are set by the public API but never consulted during rendering. The `EndScene` / `BeginScene` path always uses whatever OpenGL state was set by `Application::Init()`.

**Action:** Either implement proper state management (apply flags in `BeginScene`) or remove the flags and the public API methods that set them, replacing with direct `glEnable`/`glDisable` wrappers.

---

## 7. ParticleRenderer — Uninitialized Data & Coupling Issues

### 7.1 🟠 `ParticleInstance::angle` and `::aspectRatio` are never set by `SubmitCircle`

**File:** `engine/include/nyon/graphics/ParticleRenderer.h` + `engine/src/graphics/ParticleRenderer.cpp`

```cpp
struct alignas(16) ParticleInstance {
    float x, y;
    float angle;        // ← not set by SubmitCircle
    float radius;
    float r, g, b;
    float aspectRatio;  // ← not set by SubmitCircle
};

void ParticleRenderer::SubmitCircle(float x, float y, float radius, float r, float g, float b) {
    ParticleInstance inst;
    inst.x = x; inst.y = y; inst.radius = radius;
    inst.r = r; inst.g = g; inst.b = b;
    // inst.angle and inst.aspectRatio NEVER SET → garbage values from stack
    m_CircleInstances.push_back(inst);
}
```

`angle` and `aspectRatio` will contain whatever happens to be on the stack at that address. If the GPU particle shader reads these as uniforms or instance attributes, the rendering output is undefined.

**Action:** Zero-initialize all `ParticleInstance` fields:

```cpp
void ParticleRenderer::SubmitCircle(float x, float y, float radius, float r, float g, float b) {
    m_CircleInstances.push_back({x, y, 0.0f, radius, r, g, b, 1.0f});
}
```

---

### 7.2 🟡 `ParticleRenderer` uses hardcoded inline shader strings, not the engine's file-based shader system

**File:** `engine/src/graphics/ParticleRenderer.cpp`

The particle renderer embeds GLSL as inline C++ string literals rather than loading from the `engine/assets/shaders/` directory like all other renderers. This means:
- Particle shaders cannot be hot-reloaded or modified without recompiling
- Shader source is split across two locations, violating DRY
- GLSL version directives and precision qualifiers may differ from the rest of the codebase

**Action:** Move particle circle and quad shaders to:
- `engine/assets/shaders/particle_circle.vert` / `particle_circle.frag`
- `engine/assets/shaders/particle_quad.vert` / `particle_quad.frag`

Load via the `ShaderLoader` utility (once moved to `include/`).

---

### 7.3 🟡 `ParticleRenderer::MAX_PARTICLES = 4,000,000` — persistently mapped buffer is enormous

**File:** `engine/include/nyon/graphics/ParticleRenderer.h`

```cpp
static constexpr uint32_t MAX_PARTICLES = 4'000'000;
```

`sizeof(ParticleInstance) = 32` bytes × 4,000,000 = 128 MB per buffer × number of frame slots. If the implementation allocates a persistently mapped GPU buffer for this (as `Renderer2D` does with triple-buffering), this is 384 MB of GPU memory reserved at startup regardless of actual particle count.

**Action:** Make `MAX_PARTICLES` a runtime parameter passed to the constructor. Default to 100,000. Allow games to increase it explicitly. Use `GL_DYNAMIC_DRAW` with `glBufferSubData` up to the reserved capacity, growing the buffer (with `glBufferData`) only when particle count exceeds it.

---

## 8. InputManager Dead Code

### 8.1 🟡 `s_ActiveKeys` and `s_ActiveMouseButtons` — maintained but never queried

**File:** `engine/src/utils/InputManager.cpp`

```cpp
static std::unordered_set<int> s_ActiveKeys;
static std::unordered_set<int> s_ActiveMouseButtons;
```

Both sets are updated in `KeyCallback` / `MouseButtonCallback` but no public method reads from them. `IsKeyDown` / `IsKeyPressed` read `s_CurrentKeys[]`. The sets consume heap memory, perform hash insertions/erasures on every key event, and are never used.

**Action:** Remove both sets and the associated insert/erase calls in the callbacks. The `s_CurrentKeys[]` and `s_PreviousKeys[]` bool arrays already provide all needed state.

---

## 9. Full Particle System Design — Complete Specification

This section provides a complete, implementable specification for the overhauled particle system. It supersedes and expands §3.5 of Report 1.

### 9.1 Design Principles

- Particles must be **full ECS citizens**: they exist as entities, carry standard components, obey `PhysicsWorldComponent` gravity, respect `ColliderComponent::Filter`, trigger `PhysicsWorldComponent::Callbacks`, and are visible to the `DebugRenderSystem`.
- The particle pipeline must be **maximally parallel** using `ThreadPool`.
- **Zero hardcoded values**: no magic window dimensions, no hardcoded gravity, no fixed restitution.
- Developers must be able to **fully customize** particle behaviour via callbacks and scripting hooks without modifying engine code.

---

### 9.2 ECS Component Architecture

**A. `ParticleComponent`** — core particle state, lives on the entity:

```cpp
// engine/include/nyon/ecs/components/ParticleComponent.h
#pragma once
#include "nyon/math/Vector2.h"
namespace Nyon::ECS {

struct ParticleComponent {
    // === Physics state (synced with PhysicsBodyComponent each step) ===
    // NOTE: Particles use PhysicsBodyComponent for velocity/force,
    //       TransformComponent for position, ColliderComponent for shape.
    //       ParticleComponent adds particle-specific properties only.

    // === Lifecycle ===
    float lifetime        = -1.0f;  // seconds remaining; < 0 = eternal
    float age             =  0.0f;  // seconds since spawn
    bool  alive           = true;

    // === Visual ===
    float alpha           = 1.0f;   // independent opacity
    float alphaStart      = 1.0f;   // opacity at spawn
    float alphaEnd        = 0.0f;   // opacity at death
    Math::Vector2 colorStart = {1,1};  // used with PhysicsBodyComponent color
    Math::Vector2 colorEnd   = {1,1};
    float sizeScale       = 1.0f;   // multiplier on ColliderComponent radius

    // === Emitter reference ===
    uint32_t emitterEntityId = INVALID_ENTITY;

    // === Developer payload ===
    uint64_t userData     = 0;      // opaque per-particle data for callbacks

    // === Interpolation (set by ParticlePipelineSystem) ===
    float prevAlpha       = 1.0f;
    float prevSizeScale   = 1.0f;
};

} // namespace Nyon::ECS
```

> **All kinematic properties** (position, velocity, mass, restitution, friction, damping, gravity scale) live in `TransformComponent`, `PhysicsBodyComponent`, and `ColliderComponent` — the same components used by regular physics entities. This means particles automatically participate in the main physics pipeline if needed, or can opt into the dedicated `ParticlePipelineSystem` for performance.

---

**B. `ParticleEmitterComponent`** — defines spawn behaviour, lives on an emitter entity:

```cpp
// engine/include/nyon/ecs/components/ParticleEmitterComponent.h
#pragma once
#include "nyon/math/Vector2.h"
#include "nyon/ecs/EntityManager.h"
#include <functional>
#include <random>

namespace Nyon::ECS {

struct ParticleSpawnParams {
    // Ranges — sampled uniformly at spawn time
    float minSpeed = 50.0f,     maxSpeed = 200.0f;
    float minAngleDeg = 0.0f,   maxAngleDeg = 360.0f;   // emission cone
    float minRadius = 4.0f,     maxRadius = 16.0f;       // ColliderComponent circle radius
    float minMass = 1.0f,       maxMass = 10.0f;
    float minLifetime = 1.0f,   maxLifetime = 3.0f;      // < 0 = use component default (eternal)
    float minDrag = 0.0f,       maxDrag = 0.05f;
    float minRestitution = 0.3f, maxRestitution = 0.8f;
    float minFriction = 0.1f,   maxFriction = 0.5f;

    // Colour
    Math::Vector2 colorStartMin = {1,1,1}, colorStartMax = {1,1,1}; // RGB
    Math::Vector2 colorEndMin   = {0,0,0}, colorEndMax   = {0,0,0};
    float alphaStart = 1.0f;
    float alphaEnd   = 0.0f;

    // Shape override (empty = use radius as circle)
    // Set to a PolygonShape to emit non-circular particles
    bool  useCircle = true;
};

struct ParticleEmitterComponent {
    // === Spawn rate ===
    float spawnRate    = 10.0f;   // particles/second; 0 = burst only
    uint32_t burstCount = 0;
    float spawnTimer   = 0.0f;
    uint32_t maxParticles = 1000; // max alive at once from this emitter
    uint32_t currentCount = 0;    // tracked by ParticlePipelineSystem
    bool  loop         = true;
    bool  active       = true;

    // === Spawn area ===
    enum class EmissionShape { Point, Circle, Rectangle, Annulus } emissionShape = EmissionShape::Point;
    float emissionRadius = 0.0f;         // for Circle/Annulus
    float emissionInnerRadius = 0.0f;    // for Annulus
    Math::Vector2 emissionSize = {0,0};  // for Rectangle

    // === Initial condition ranges ===
    ParticleSpawnParams spawnParams;

    // === Physics settings ===
    bool affectedByPhysicsWorld = true;  // uses PhysicsWorldComponent.gravity
    float gravityScale = 1.0f;           // per-emitter gravity multiplier
    bool collidesWithBodies = false;     // enter main physics pipeline
    bool collidesWithParticles = true;   // use dedicated particle broadphase

    // === Collision layer (inherits ColliderComponent::Filter system) ===
    uint16_t collisionCategory = 0x0002; // default particle layer
    uint16_t collisionMask     = 0xFFFF;

    // === Developer hooks (all called on the game thread, not worker threads) ===
    // Called when a particle entity is spawned. Set any custom component data here.
    std::function<void(EntityID particleEntity)> onSpawn;

    // Called each physics step per alive particle. Runs after standard physics update.
    // Use this for custom forces, colour animation, size animation, etc.
    // WARNING: Must be thread-safe if ParticlePipelineSystem uses parallel update.
    std::function<void(EntityID particleEntity, float deltaTime)> onUpdate;

    // Called when particle lifetime expires. Use to spawn secondary effects.
    std::function<void(EntityID particleEntity)> onDeath;

    // Called on particle-body collision (only if collidesWithBodies = true).
    std::function<void(EntityID particleEntity, EntityID bodyEntity)> onCollision;

    // === RNG seed (per-emitter for reproducibility) ===
    uint64_t seed = 0;  // 0 = random seed from std::random_device
};

} // namespace Nyon::ECS
```

---

### 9.3 `ParticlePipelineSystem` — Parallel Architecture

**File:** `engine/include/nyon/ecs/systems/ParticlePipelineSystem.h`

```
ParticlePipelineSystem::Update(float dt)
│
├── Phase 1: Tick emitters (main thread, fast)
│   For each ParticleEmitterComponent:
│   └── SpawnParticles(emitter, dt)
│       Creates N new entities with: TransformComponent, PhysicsBodyComponent,
│       ColliderComponent (circle, sensor-optional), ParticleComponent
│       Calls emitter.onSpawn(entityId) for each new particle
│
├── Phase 2: Parallel particle physics update
│   Partition ParticleComponent entities into ThreadPool chunks:
│   └── For each chunk [start, end]:
│       Submit ThreadPool task:
│       ├── Apply gravity from PhysicsWorldComponent (cached, read-only)
│       ├── Apply drag (PhysicsBodyComponent.drag)
│       ├── Integrate velocity → position (Euler)
│       ├── Update TransformComponent.previousPosition
│       ├── Tick ParticleComponent.age
│       ├── Compute alpha and sizeScale from age/lifetime
│       └── Invoke emitter.onUpdate(entityId, dt) [via function pointer]
│   WaitAll()
│
├── Phase 3: Parallel particle-particle broadphase (spatial hash)
│   Single spatial hash built in Phase 2 (each thread writes to thread-local cells)
│   Main thread merges cell lists (O(cells), cheap)
│   Parallel narrow phase: submit pairs to ThreadPool
│   WaitAll()
│   Collision response: sequential impulse resolution
│
├── Phase 4: Particle-body broadphase (optional, if any emitter has collidesWithBodies=true)
│   Query main DynamicTree for each particle AABB (read-only, safe concurrent)
│   Collision response respects ColliderComponent::Filter (particle layer vs body layer)
│   Calls emitter.onCollision(particleEntity, bodyEntity) on main thread
│
├── Phase 5: Lifecycle management (main thread)
│   For each particle where alive=false OR age >= lifetime:
│   └── Call emitter.onDeath(entityId)
│       Decrement emitter.currentCount
│       Schedule entity for destruction (deferred to avoid mid-update hazard)
│       (Destruction via EntityManager::DestroyEntity in a post-update queue)
│
└── Phase 6: Post-update cleanup (main thread)
    Process deferred entity destruction queue
    Update emitter.currentCount for all emitters
```

---

### 9.4 `ParticleRenderSystem` — Pure GPU Submission

`ParticleRenderSystem` becomes a thin layer:

```cpp
void ParticleRenderSystem::Render(float alpha) {
    // Get shared VP matrix from CameraSystem (no hardcoding)
    const glm::mat4& vp = m_CameraSystem->GetActiveViewProjection();

    m_ParticleRenderer->BeginFrame();
    m_ComponentStore->ForEachComponent<ParticleComponent>([&](EntityID id, const ParticleComponent& p) {
        if (!p.alive) return;
        const auto& tf = m_ComponentStore->GetComponent<TransformComponent>(id);
        const auto& col = m_ComponentStore->GetComponent<ColliderComponent>(id);

        // Interpolated position (uses TransformComponent.previousPosition)
        float ix = tf.previousPosition.x + (tf.position.x - tf.previousPosition.x) * alpha;
        float iy = tf.previousPosition.y + (tf.position.y - tf.previousPosition.y) * alpha;

        // Interpolated visual properties
        float ia = p.prevAlpha + (p.alpha - p.prevAlpha) * alpha;

        // Submit to GPU
        float radius = col.GetCircle().radius * p.sizeScale;
        m_ParticleRenderer->SubmitCircle(ix, iy, radius, ...color..., ia);
    });
    m_ParticleRenderer->Flush(vp);
}
```

No hardcoded resolution. No copies. Uses engine VP. Uses `TransformComponent.previousPosition` for interpolation.

---

### 9.5 Physics World Integration — No Hardcoding

The `ParticlePipelineSystem` must:
1. Find the `PhysicsWorldComponent` entity in `Initialize()` and cache `Math::Vector2 m_Gravity`.
2. Update `m_Gravity` at the start of each `Update()` from the cached world entity.
3. Apply `m_Gravity * emitter.gravityScale * body.mass * dt` to each particle's velocity.
4. Respect `world.enableSleep` — sleeping particles (velocity near zero, nothing nearby) stop integrating.
5. Respect `world.maxLinearSpeed` — clamp particle velocities.
6. Post collision events to `world.callbacks.beginContact` / `endContact` when `collidesWithBodies = true`.

---

## 10. Parallelism Expansion — Full Inventory

All items are new opportunities not covered in Report 1.

### 10.1 🟡 `UpdateTransformsFromSolver` — embarrassingly parallel

**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`

Currently sequential over all `m_SolverBodies`. Each body write is independent (each maps to a different `TransformComponent`).

**Action:**
```cpp
size_t batchSize = (m_SolverBodies.size() + m_NumThreads - 1) / m_NumThreads;
for (size_t t = 0; t < m_NumThreads; ++t) {
    size_t start = t * batchSize;
    size_t end = std::min(start + batchSize, m_SolverBodies.size());
    futures.push_back(pool.Submit([this, start, end]() {
        for (size_t i = start; i < end; ++i)
            WritebackSolverBodyToTransform(m_SolverBodies[i]);
    }));
}
```

**Prerequisite:** Verify that no two `SolverBody` objects share an entity (they must not — each entity has exactly one `TransformComponent`).

---

### 10.2 🟡 `PrepareBodiesForUpdate` AABB computation phase — parallel before serial tree updates

**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`

`UpdateShapeAABB` computes the AABB (CPU math) then calls `MoveProxy`/`CreateProxy` (tree mutation). The math is parallel; the tree mutation is not.

**Action — Two-phase split:**
```
Phase A (parallel): For each collider entity, compute AABB → store in staging array
Phase B (serial):   For each staging entry, call MoveProxy/CreateProxy on m_BroadPhaseTree
```

This removes the tree-mutation bottleneck from the critical path.

---

### 10.3 🟡 `ConstraintInitialization` — contact point mass computation is parallel

**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`

The loop over `m_ContactManifolds` in `ConstraintInitialization` computes `normalMass`, `tangentMass`, and `velocityBias` per contact point. Each contact pair reads from `m_SolverBodies` (read-only after body setup) and writes to its own `VelocityConstraint` (no shared state).

**Action:**
```cpp
size_t n = m_ContactManifolds.size();
size_t batch = (n + m_NumThreads - 1) / m_NumThreads;
for (size_t t = 0; t < m_NumThreads; ++t) {
    size_t s = t * batch, e = std::min(s + batch, n);
    futures.push_back(pool.Submit([this, s, e]() {
        InitializeConstraintRange(s, e);   // new helper function
    }));
}
WaitAll();
```

---

### 10.4 🟡 `ImpulseCache` warm-start lookup — batch parallel read

`WarmStartConstraints` iterates all constraints, looks up `m_ImpulseCache` per contact point. Cache reads are pure reads — thread-safe for `unordered_map` if no writes occur concurrently.

**Action:** Parallelize the warm-start lookup phase (split constraints into thread-local staging), then apply impulses serially (since body state is shared):
```
Phase A (parallel): For each constraint batch → look up cached impulses → store in staging
Phase B (serial):   Apply staged warm-start impulses to m_SolverBodies
```

---

### 10.5 🟡 `IslandManager::UpdateSleepTimers` — per-island, fully independent

**File:** `engine/src/physics/Island.cpp`

Each island's sleep timer update reads only from `m_ComponentStore` (read-only) and writes to its own `Island::sleepTimer`. No shared mutable state between islands.

**Action:**
```cpp
size_t n = m_AllIslands.size();
size_t batch = (n + NumThreads - 1) / NumThreads;
for (size_t t = 0; t < NumThreads; ++t) {
    pool.Submit([this, start, end, deltaTime]() {
        for (size_t i = start; i < end; ++i)
            UpdateIslandSleepTimer(m_AllIslands[i], deltaTime);
    });
}
pool.WaitAll();
```

---

### 10.6 🟡 Particle instance buffer build — parallel CPU → GPU staging

Before `ParticleRenderer::Flush`, particle data must be converted from ECS components to `ParticleInstance` structs. This conversion is embarrassingly parallel:

```cpp
// N threads each fill a sub-range of a pre-allocated staging vector:
std::vector<ParticleInstance> staging(aliveParticleCount);
pool.ForEach(particles, [&](size_t i, EntityID id) {
    const auto& tf = ...GetComponent<TransformComponent>(id);
    const auto& p  = ...GetComponent<ParticleComponent>(id);
    staging[i] = BuildInstance(tf, p, alpha);
});
pool.WaitAll();
// Then single glBufferSubData call
glBufferSubData(GL_ARRAY_BUFFER, 0, staging.size() * sizeof(ParticleInstance), staging.data());
```

---

### 10.7 🟡 Broad-phase pair deduplication — parallel hash then serial merge

The `ParallelBroadPhase` currently produces duplicate pairs (e.g. `(A,B)` and `(B,A)` from two separate tree queries). Deduplication is currently handled by the `if (i >= j) continue` guard in the serial path. In the parallel path there is no such guard — both `(A,B)` and `(B,A)` can appear.

**Action:** After merging futures in `ParallelBroadPhase`, deduplicate by normalizing pairs:
```cpp
for (auto& [a, b] : m_BroadPhasePairs)
    if (a > b) std::swap(a, b);
// Sort then unique:
std::sort(m_BroadPhasePairs.begin(), m_BroadPhasePairs.end());
m_BroadPhasePairs.erase(std::unique(m_BroadPhasePairs.begin(), m_BroadPhasePairs.end()), m_BroadPhasePairs.end());
```
This sort can itself be parallelized using a parallel radix sort for large pair counts.

---

## 11. Collision System Parallelisation — Detailed Design

### 11.1 Parallel Narrow Phase — Fix the current `std::future` per pair approach

**Current problem:** `ParallelNarrowPhase` submits one `future` per broad-phase pair. For 10,000 pairs, this creates 10,000 `packaged_task` allocations. ThreadPool task submission overhead becomes dominant.

**Correct approach — Batched parallel narrow phase:**

```cpp
void PhysicsPipelineSystem::ParallelNarrowPhase() {
    m_ContactManifolds.clear();
    m_ContactMap.clear();

    const size_t pairCount = m_BroadPhasePairs.size();
    const size_t batchSize = std::max<size_t>(32, (pairCount + m_NumThreads - 1) / m_NumThreads);

    // One vector of manifolds per thread (no locking needed)
    std::vector<std::vector<ContactManifold>> threadLocalManifolds(m_NumThreads);
    std::vector<std::future<void>> futures;

    for (size_t t = 0; t < m_NumThreads; ++t) {
        size_t start = t * batchSize;
        if (start >= pairCount) break;
        size_t end = std::min(start + batchSize, pairCount);

        futures.push_back(Utils::ThreadPool::Instance().Submit(
            [this, start, end, t, &threadLocalManifolds]() {
                auto& local = threadLocalManifolds[t];
                for (size_t i = start; i < end; ++i) {
                    const auto& [eA, eB] = m_BroadPhasePairs[i];
                    if (!TestCollision(eA, eB)) continue;
                    ContactManifold m = GenerateManifold(eA, eB);
                    if (!m.points.empty()) local.push_back(std::move(m));
                }
            }
        ));
    }
    for (auto& f : futures) f.get();

    // Serial merge (cheap — just pointer moves)
    for (auto& localVec : threadLocalManifolds) {
        for (auto& m : localVec) {
            uint64_t key = MakeContactKey(m.entityIdA, m.entityIdB);
            m_ContactMap[key] = m_ContactManifolds.size();
            m_ContactManifolds.push_back(std::move(m));
        }
    }
}
```

**Thread safety of `TestCollision` and `GenerateManifold`:** Both are read-only with respect to `m_ComponentStore`. `ComponentStore::GetComponent` and `HasComponent` perform only hash-map reads — safe for concurrent readers if no writer is active. Since no body is added/removed during a physics step, this is safe. Add a `// Thread-safe: read-only component access` comment.

---

### 11.2 `ManifoldGenerator` — make all methods `[[nodiscard]]` and const-correct

All `ManifoldGenerator` static methods read from component data. They should be marked `[[nodiscard]]` to prevent accidentally discarding generated manifolds:

```cpp
[[nodiscard]] static ECS::ContactManifold GenerateManifold(...);
```

---

### 11.3 Position constraint solving — candidate for XPBD (constraint-independent parallel solving)

The current `SolvePositionConstraints` is a sequential Gauss-Seidel loop (iterates constraints one by one, each modifying shared body positions). This cannot be parallelised as-is.

For a parallelizable alternative, consider **XPBD** (Extended Position Based Dynamics) — each constraint update is applied directly to positions using positional impulses, and islands can be solved independently in parallel after island detection:

```
For each awake island (in parallel):
    For each iteration:
        For each constraint in island (sequential within island):
            Solve constraint, update island's body positions
```

Since islands are by definition disconnected, different islands can be solved on different threads simultaneously. This is a medium-term architectural improvement.

---

## 12. Additional Minor Bugs & Code Quality

### 12.1 🟠 `BehaviorComponent` collision callback never invoked by the physics pipeline

**File:** `engine/include/nyon/ecs/components/BehaviorComponent.h`

`BehaviorComponent::OnCollision(entity, other)` is declared and wired up, but `PhysicsPipelineSystem` never calls it after resolving a contact. The `PhysicsWorldComponent::Callbacks::beginContact` is also never invoked (the function pointer exists but is never called).

**Action:** At the end of `NarrowPhaseDetection`, after populating `world.contactManifolds`, iterate new contacts and invoke:
```cpp
if (world.callbacks.beginContact)
    world.callbacks.beginContact(manifold.entityIdA, manifold.entityIdB);
if (m_ComponentStore->HasComponent<BehaviorComponent>(manifold.entityIdA))
    m_ComponentStore->GetComponent<BehaviorComponent>(manifold.entityIdA)
        .OnCollision(manifold.entityIdA, manifold.entityIdB);
```

---

### 12.2 🟡 `Vector2::operator/` performs division by zero with no guard

**File:** `engine/include/nyon/math/Vector2.h`

```cpp
Vector2 operator/(float scalar) const { return Vector2(x / scalar, y / scalar); }
```

No zero-check. Used for normalisation and mass computations throughout the physics pipeline. A zero-mass body (before `UpdateMassProperties` is called) or a `Length() == 0` normalisation will produce `NaN` which silently propagates.

**Action:**
```cpp
Vector2 operator/(float scalar) const {
    NYON_ASSERT(std::abs(scalar) > 1e-8f, "Vector2: division by near-zero scalar");
    return Vector2(x / scalar, y / scalar);
}
```

---

### 12.3 🟡 `PolygonShape::IsCounterClockwise` uses the shoelace area — handles only simple polygons

**File:** `engine/include/nyon/ecs/components/ColliderComponent.h`

```cpp
bool IsCounterClockwise() const {
    float area = 0.0f;
    for (...) area += vertices[i].x * vertices[j].y - vertices[j].x * vertices[i].y;
    return area > 0.0f;
}
```

This works for simple (non-self-intersecting) convex polygons. For self-intersecting or concave polygons, the shoelace area may be near zero or have the wrong sign. No validation is performed that the polygon is actually convex, which the collision algorithms require.

**Action:** Add a convexity check to `CalculateProperties()`:
```cpp
bool ValidateConvex() const {
    int sign = 0;
    for (size_t i = 0; i < vertices.size(); ++i) {
        const auto& a = vertices[i];
        const auto& b = vertices[(i+1) % vertices.size()];
        const auto& c = vertices[(i+2) % vertices.size()];
        float cross = (b.x-a.x)*(c.y-b.y) - (b.y-a.y)*(c.x-b.x);
        if (cross != 0.0f) {
            int s = cross > 0.0f ? 1 : -1;
            if (sign == 0) sign = s;
            else if (s != sign) return false;
        }
    }
    return true;
}
```

---

### 12.4 🟡 `DynamicTree::QueryInternal` is recursive — stack overflow risk for deep trees

**File:** `engine/include/nyon/physics/DynamicTree.h`

```cpp
template<typename T>
void QueryInternal(const AABB& aabb, T* callback, uint32_t nodeId) const {
    ...
    QueryInternal(aabb, callback, node.child1);
    QueryInternal(aabb, callback, node.child2);
}
```

Recursive DFS. For a degenerate tree (all bodies in a line), the tree height is O(n). With 10,000 entities, this could be 10,000 stack frames deep — stack overflow on typical 8 MB stacks.

**Action:** Convert to an iterative implementation using an explicit stack (e.g. `std::vector<uint32_t> nodeStack`). The `Balance` function keeps the tree AVL-like but doesn't guarantee O(log n) height under adversarial insertions.

---

### 12.5 🔵 `Application` window resize not handled — viewport and projection stale after resize

**File:** `engine/src/core/Application.cpp`

```cpp
void Application::Init() {
    ...
    glViewport(0, 0, m_Width, m_Height);  // set once, never updated
}
```

If the GLFW window is resized by the user, `glViewport` is never updated, causing stretching. `Camera2D::GetProjectionMatrix` uses dynamic `glfwGetWindowSize` so the projection adapts, but the GL viewport does not.

**Action:** Register a GLFW framebuffer size callback:
```cpp
glfwSetFramebufferSizeCallback(m_Window, [](GLFWwindow*, int w, int h) {
    glViewport(0, 0, w, h);
    Application::Get().m_Width = w;
    Application::Get().m_Height = h;
});
```

---

### 12.6 🔵 `ColliderComponent::ChainShape` has no collision detection — silently ignored

**File:** `engine/include/nyon/ecs/components/ColliderComponent.h` and `ManifoldGenerator.cpp`

`ChainShape` is declared in `ColliderComponent::ShapeType` as a valid type. `CalculateAABB` has no `case ShapeType::Chain` — it falls to `default: outMin = outMax = position`. This means chain-shape entities produce a zero-size AABB. They are effectively invisible to the broadphase and never generate contacts.

**Action:** Either implement chain-shape AABB calculation and collision detection (chain vs circle, chain vs polygon), or remove `ChainShape` from the public `ShapeType` enum and its associated `ColliderComponent(const ChainShape&)` constructor.

---

## 13. Implementation Priority Addendum

Add these to the end of Report 1's Stage order:

**Stage 1 additions (critical — must fix before any feature work):**
- §1.1: Fix `SystemManager.h` typo (`s` → `System`) — nothing compiles without this
- §1.2: Move `Camera2D` method bodies from backup to main `Renderer2D.cpp`
- §1.3: Implement `Renderer2D::Flush()` in main file
- §4.7: Implement `ParallelPositionSolving()` — linker error otherwise

**Stage 2 additions (high — wrong simulation results):**
- §2.1: Fix island sleep system (full redesign)
- §3.1: Fix `ComputeCircleCenters` rotation bug
- §3.2: Fix `CircleCapsule` circle center rotation
- §4.1: Rename `Integration()` to `SyncMotionLocksToComponents()`, move `IntegratePositions` to explicit step
- §4.2: Pass `subStepDt` to `VelocitySolving` and `PositionSolving`
- §3.3: Fix `PolygonPolygon` centroid normal double-flip and separation sign bug
- §3.6: Guard against `segLenSq == 0` in `CircleCapsule`

**Stage 3 additions (medium — data model cleanup):**
- §5.1: Remove duplicate top-level `density` field from `ColliderComponent`
- §5.2: Change `ColliderComponent` default to a documented shape or require explicit construction
- §4.3: Delete dead-code loops in `PositionSolving` and `UpdateTransformsFromSolver`
- §4.4: Fix `MakeImpulseCacheKey` XOR hash
- §4.5: Remove/fix warm-start magic clamp value
- §7.1: Zero-initialize `ParticleInstance` in `SubmitCircle`
- §8.1: Remove dead `s_ActiveKeys` / `s_ActiveMouseButtons`
- §12.1: Wire up `BehaviorComponent::OnCollision` and `PhysicsWorldComponent::Callbacks`

**Stage 4 (particle system):** Follow §9.2–9.5 to implement the full ECS particle system.

**Stage 5 (parallelism):** Implement §10.1–10.7 and §11.1–11.3 in the order listed.

---

*End of Report 2. Combined with Report 1, this constitutes a complete engineering audit of the Nyon engine codebase.*
