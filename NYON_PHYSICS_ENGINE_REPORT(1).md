# Nyon Physics Engine — Comprehensive Audit Report

> **Purpose:** This document is the authoritative guideline for engineers and AI agents to fix all bugs, architecture issues, physics correctness errors, and vulnerabilities found in the `daiyaanmuhammadfardeen-nyon` codebase. Every section contains the problem statement and the exact, complete fix to apply.

---

## Table of Contents

1. [Architecture & Dead Code](#1-architecture--dead-code)
2. [ECS / ComponentStore](#2-ecs--componentstore)
3. [Physics Integration](#3-physics-integration)
4. [Collision Detection — Broad Phase](#4-collision-detection--broad-phase)
5. [Collision Detection — Narrow Phase & Manifold Generation](#5-collision-detection--narrow-phase--manifold-generation)
6. [SAT Collision Detector](#6-sat-collision-detector)
7. [Constraint Solver](#7-constraint-solver)
8. [Continuous Collision Detection (CCD)](#8-continuous-collision-detection-ccd)
9. [Position Correction / Stabilization](#9-position-correction--stabilization)
10. [Island Manager](#10-island-manager)
11. [Rendering](#11-rendering)
12. [Input System](#12-input-system)
13. [Global Hardening & Cleanup Strategy](#13-global-hardening--cleanup-strategy)

---

## 1. Architecture & Dead Code

### 1.1 Duplicate Collision Systems

**Problem:** Two fully parallel collision pipelines exist simultaneously:
- `CollisionSystem` (legacy, O(N²) brute-force, hardcoded screen boundaries, frame-counter "grounded" state)
- `CollisionPipelineSystem` (modern, DynamicTree broad-phase, proper manifold pipeline)

Both are registered and run each tick. This causes double-resolution, corrupted impulse accumulation, and conflicting transform mutations.

**Fix:** Delete `CollisionSystem.h` entirely. Delete all references to it from `SystemManager`, `ECSApplication`, and any demo files. The `CollisionPipelineSystem` is the only collision system that should exist.

---

### 1.2 Duplicate Physics Body Component Headers

**Problem:** `LegacyPhysicsBodyComponent.h` and `LegacyColliderComponent.h` are present in the include tree, included in several translation units, and define overlapping types with `PhysicsBodyComponent` and `ColliderComponent`.

**Fix:** Delete both legacy header files and all `#include` directives referencing them. Grep for `LegacyPhysicsBodyComponent` and `LegacyColliderComponent` across the entire codebase and remove every occurrence.

---

### 1.3 `ResolveCCDImpulse` and `ResolveCCDPositionalCorrection` Are Dead

**Problem:** Both methods in `ContinuousCollisionSystem` contain only a `NYON_DEBUG_LOG("[WARNING] ... deprecated")` body. They are still declared in the header and called in zero places, but the signatures are part of the public API, creating confusion.

**Fix:** Remove both method declarations from `ContinuousCollisionSystem.h` and their stub definitions from `ContinuousCollisionSystem.cpp`.

---

### 1.4 `Physics.cpp` Is Orphaned

**Problem:** `engine/src/utils/Physics.cpp` exists in the source tree but has no corresponding header under `nyon/utils/` and is not referenced by any CMake target.

**Fix:** Either add a proper `Physics.h` and integrate it, or delete `Physics.cpp` if its contents have been superseded by `SATCollisionDetector` and `ManifoldGenerator`.

---

### 1.5 `RenderingDemo.cpp` Should Not Be in the Engine Library

**Problem:** `engine/src/graphics/RenderingDemo.cpp` is compiled into the engine static library. Demo/sample code must not be a dependency of the core engine.

**Fix:** Move `RenderingDemo.cpp` to a separate `demos/` or `samples/` CMake target that links against the engine. Remove it from `engine/CMakeLists.txt`.

---

### 1.6 `B2_UNUSED` Macro Used Without Box2D Dependency

**Problem:** `B2_UNUSED(...)` appears in `SATCollisionDetector.cpp` (`FindIncidentFace`, `ClipSegmentToLine`, `CCD::ComputeTOI`). This macro is Box2D-specific and is not defined anywhere in this codebase, causing undefined-macro warnings or compilation errors on strict compilers.

**Fix:** Replace every `B2_UNUSED(x)` with the portable form `(void)(x);`. Add a project-level macro if desired:
```cpp
// In a common header, e.g. nyon/core/Defines.h
#define NYON_UNUSED(x) (void)(x)
```
Then replace `B2_UNUSED` with `NYON_UNUSED`.

---

### 1.7 `std::cout` Debug Output Left in Production Code

**Problem:** Raw `std::cout` and `std::cerr` calls are scattered throughout production physics code (e.g., `ResolveCCDCollision`, `ConstraintSolverSystem`). These are not guarded by any compile-time flag in release builds.

**Fix:** Replace all unguarded `std::cout`/`std::cerr` with the project's `NYON_DEBUG_LOG` macro, which is already defined to be a no-op in non-debug builds. Audit every `.cpp` file in `src/physics/` and `src/ecs/systems/` for bare `std::cout`.

---

## 2. ECS / ComponentStore

### 2.1 O(N) Linear Scan on Every Component Access

**Problem:** `ComponentStore::GetComponent<T>()`, `HasComponent<T>()`, and `RemoveComponent<T>()` all iterate the entire `entityIds` vector linearly. With hundreds of entities this dominates CPU time each frame.

**Fix:** Add a secondary `std::unordered_map<EntityID, size_t> entityIndexMap` inside each `ComponentContainer<T>`. Maintain it in `AddComponent` (insert), `RemoveComponent` (erase), and `GetComponentByIndex` becomes index-direct. All lookups drop from O(N) to O(1) amortized.

```cpp
template<typename T>
struct ComponentContainer : public IComponentContainer {
    std::vector<T>        components;
    std::vector<EntityID> entityIds;
    std::vector<bool>     activeFlags;
    std::unordered_map<EntityID, size_t> indexMap; // NEW

    void AddComponent(EntityID entity, T&& component) {
        auto it = indexMap.find(entity);
        if (it != indexMap.end() && activeFlags[it->second]) {
            components[it->second] = std::forward<T>(component);
            return;
        }
        indexMap[entity] = components.size();
        components.push_back(std::forward<T>(component));
        entityIds.push_back(entity);
        activeFlags.push_back(true);
    }

    void RemoveComponent(EntityID entity) override {
        auto it = indexMap.find(entity);
        if (it != indexMap.end()) {
            activeFlags[it->second] = false;
            indexMap.erase(it);
        }
    }

    bool HasComponent(EntityID entity) const override {
        auto it = indexMap.find(entity);
        return it != indexMap.end() && activeFlags[it->second];
    }

    T& GetComponent(EntityID entity) {
        return components[indexMap.at(entity)];
    }
};
```

---

### 2.2 `RemoveComponent` Soft-Deletes Only — Memory Grows Forever

**Problem:** `RemoveComponent` sets `activeFlags[i] = false` but never reclaims the slot. Over time, the `components`, `entityIds`, and `activeFlags` vectors grow monotonically even when entities are destroyed. This is a memory leak.

**Fix:** Use swap-and-pop when removing:
```cpp
void RemoveComponent(EntityID entity) override {
    auto it = indexMap.find(entity);
    if (it == indexMap.end()) return;
    size_t idx = it->second;
    size_t last = components.size() - 1;
    if (idx != last) {
        // Move last element into the removed slot
        components[idx]  = std::move(components[last]);
        entityIds[idx]   = entityIds[last];
        activeFlags[idx] = activeFlags[last];
        indexMap[entityIds[idx]] = idx;
    }
    components.pop_back();
    entityIds.pop_back();
    activeFlags.pop_back();
    indexMap.erase(it);
}
```

---

### 2.3 `GetComponent` Returns a Static Dummy on Failure

**Problem:**
```cpp
assert(false);
static T dummy{};
return dummy;
```
In release builds, `assert` is stripped, so code proceeds to return a mutable reference to a shared static dummy. Any mutation via this reference corrupts the dummy for all future callers. This is undefined behaviour in practice.

**Fix:** After adding the `indexMap` (Section 2.1), this path is unreachable. Until then, replace the fallback with `std::terminate()` or throw `std::logic_error("ComponentStore: entity does not have component")`.

---

### 2.4 `GetEntitiesWithComponent` Returns a Temporary Vector Every Call

**Problem:** Called every frame inside `CollisionSystem` and `CollisionPipelineSystem` for multiple component types. Each call allocates and populates a `std::vector<EntityID>`.

**Fix:** Add a cached view/range that the caller can iterate without allocation:
```cpp
template<typename T>
const std::vector<EntityID>& GetEntityView() const; // backed by indexMap keys
```
Alternatively, expose `ForEachComponent<T>` for all iteration sites and remove `GetEntitiesWithComponent` from the hot path.

---

## 3. Physics Integration

### 3.1 Gravity Never Applied in `ConstraintSolverSystem::IntegrateVelocities`

**Problem:** The `IntegrateVelocities` method in `ConstraintSolverSystem` applies only damping. Gravity from `PhysicsWorldComponent` is never added to velocity, so bodies appear weightless unless gravity is applied elsewhere (there is no other place).

**Fix:**
```cpp
void ConstraintSolverSystem::IntegrateVelocities(float dt) {
    for (auto& body : m_SolverBodies) {
        if (body.isStatic || body.isKinematic) continue;
        // Apply gravity
        body.velocity += m_PhysicsWorld->gravity * dt;
        // Apply damping correctly (per-frame factor, not raw constant)
        float linearDampingFactor  = 1.0f / (1.0f + dt * m_PhysicsWorld->linearDamping);
        float angularDampingFactor = 1.0f / (1.0f + dt * m_PhysicsWorld->angularDamping);
        body.velocity        *= linearDampingFactor;
        body.angularVelocity *= angularDampingFactor;
    }
}
```

---

### 3.2 Damping Applied as a Raw Constant Multiplier

**Problem:** `body.velocity = body.velocity * 0.999f` and `body.angularVelocity = body.angularVelocity * 0.995f` are frame-rate-dependent. At 30 Hz the damping is half as strong as at 60 Hz.

**Fix:** Use the frame-rate-independent form shown above: `1.0f / (1.0f + dt * dampingCoefficient)`. Expose `linearDamping` and `angularDamping` as per-body fields in `PhysicsBodyComponent`.

---

### 3.3 Symplectic Euler Integration Order Is Wrong

**Problem:** In `ConstraintSolverSystem`, `IntegratePositions` adds `velocity * dt` to position, but this is called *before* `SolveVelocityConstraints` in some code paths. Correct symplectic Euler requires: apply forces → solve velocity constraints → integrate position. The current execution order is:
1. `IntegrateVelocities` (damping only — no gravity, see §3.1)
2. `InitializeConstraints`
3. `WarmStart`
4. `SolveVelocityConstraints`
5. `IntegratePositions`
6. `SolvePositionConstraints`

Step 1 must apply gravity *first*, then the rest follows. Confirm `IntegratePositions` always runs after `SolveVelocityConstraints`. If not, reorder.

---

### 3.4 `SolvePositionConstraints` Reads from `m_VelocityConstraints` for Position Solving

**Problem:** `SolvePositionConstraints` iterates `m_VelocityConstraints` (not `m_PositionConstraints`) to reconstruct contact geometry. This means it re-reads velocity-domain data (impulses, `rA`/`rB` computed for velocity) for position correction, which produces wrong contact points and wrong correction directions.

**Fix:** Populate and use `m_PositionConstraints` properly. Each `PositionConstraint` should store the local-space contact point and normal relative to each body so that world-space geometry can be recomputed from the *current* solver body positions during position iterations, exactly as Box2D does.

```cpp
// During SolvePositionConstraints:
for (auto& pc : m_PositionConstraints) {
    auto& bodyA = m_SolverBodies[pc.indexA];
    auto& bodyB = m_SolverBodies[pc.indexB];
    // Recompute world contact points from current positions + angles
    Math::Rotation2D rotA(bodyA.angle), rotB(bodyB.angle);
    Math::Vector2 rA = rotA * (pc.localPointA - bodyA.localCenter);
    Math::Vector2 rB = rotB * (pc.localPointB - bodyB.localCenter);
    Math::Vector2 normal = rotA * pc.localNormal;
    Math::Vector2 pointA = bodyA.position + rA;
    Math::Vector2 pointB = bodyB.position + rB;
    float separation = Math::Vector2::Dot(pointB - pointA, normal);
    // Apply Baumgarte position correction...
}
```

---

### 3.5 Angular Position Correction Multiplied by an Arbitrary 0.1 Factor

**Problem:**
```cpp
float angularCorrection = Math::Vector2::Cross(vcp.rA, correction) * vc.invIA * 0.1f;
```
The `0.1f` is unexplained magic. It under-corrects rotation, causing objects resting at an angle to slowly drift through floors.

**Fix:** Remove the arbitrary scale. The correct formulation (used by Box2D) is:
```cpp
float angularCorrectionA = invInertiaA * CrossScalar(rA, correctionImpulse);
bodyA.angle -= angularCorrectionA;
```
where `correctionImpulse` already incorporates the Baumgarte factor and mass weighting.

---

### 3.6 `TransformPhysicsSyncSystem` — Sync Direction Ambiguity

**Problem:** It is architecturally ambiguous whether the solver bodies are written back to `PhysicsBodyComponent` before or after `TransformComponent` is updated. If both happen in the wrong order, the rendered position lags one frame behind the physics position, and the interpolation in `TransformComponent::GetInterpolatedPosition` produces incorrect visual results.

**Fix:** Enforce a strict tick order and document it:
1. `TransformComponent::PrepareForUpdate()` — snapshot current position as `previousPosition`
2. Physics integration (all solver steps)
3. Write solver body results → `PhysicsBodyComponent` (velocity, angularVelocity)
4. Write solver body positions → `TransformComponent::position`
5. Render using `GetInterpolatedPosition(alpha)`

Enforce this order in `PhysicsPipelineSystem::Update`.

---

## 4. Collision Detection — Broad Phase

### 4.1 `CollisionSystem::ApplyBoundaryConstraints` Has Hardcoded Resolution

**Problem:**
```cpp
const float SCREEN_WIDTH  = 1280.0f;
const float SCREEN_HEIGHT = 720.0f;
const float PLAYER_SIZE   = 32.0f;
```
These magic constants are baked into a physics system. When any of these change (window resize, non-square objects) the physics breaks silently.

**Fix:** This entire system is eliminated per Section 1.1. The `BoundarySystem` (which properly reads world bounds from `PhysicsWorldComponent`) is the correct implementation.

---

### 4.2 `CollisionPipelineSystem` Does Not Account for Rotation in AABB Computation

**Problem:** `UpdateShapeAABB` computes the AABB at the *current* position but does not factor in `TransformComponent::rotation`. A rotated box has a larger axis-aligned bounding box than an unrotated one. Without this, the broad phase misses collisions for rotated shapes.

**Fix:** When computing the AABB for a rotated polygon, transform all vertices by the current rotation and then compute the min/max:
```cpp
void UpdateShapeAABB(uint32_t entityId, uint32_t shapeId,
                     ColliderComponent* collider,
                     const Math::Vector2& position,
                     float rotation) {
    // For polygon shapes:
    Math::Vector2 minPt = { FLT_MAX,  FLT_MAX};
    Math::Vector2 maxPt = {-FLT_MAX, -FLT_MAX};
    for (auto& v : collider->GetPolygon().vertices) {
        Math::Vector2 world = position + Rotate(v, rotation);
        minPt.x = std::min(minPt.x, world.x);
        minPt.y = std::min(minPt.y, world.y);
        maxPt.x = std::max(maxPt.x, world.x);
        maxPt.y = std::max(maxPt.y, world.y);
    }
    // Add fat AABB margin
    const float margin = 0.1f;
    Physics::AABB aabb{minPt - Math::Vector2{margin,margin},
                       maxPt + Math::Vector2{margin,margin}};
    m_BroadPhaseTree.MoveProxy(proxyId, aabb, bodyVelocity * dt);
}
```

---

### 4.3 `ContactPairHash` Is a Weak Hash with High Collision Rate

**Problem:**
```cpp
return ((hash(entityIdA)*31 + hash(entityIdB))*31 + hash(shapeIdA))*31 + hash(shapeIdB);
```
This polynomial hash with base 31 has poor avalanche and will cluster for sequential entity IDs, degrading the unordered map to linear scan.

**Fix:** Use a better mixing function:
```cpp
size_t operator()(const ContactPair& p) const noexcept {
    auto h = [](uint32_t v) -> size_t {
        v ^= v >> 16; v *= 0x45d9f3b; v ^= v >> 16; return v;
    };
    size_t seed = h(p.entityIdA);
    seed ^= h(p.entityIdB) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= h(p.shapeIdA)  + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= h(p.shapeIdB)  + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
}
```

---

## 5. Collision Detection — Narrow Phase & Manifold Generation

### 5.1 `ManifoldGenerator::GenerateManifold` Shape-Type Priority Is Wrong

**Problem:** The dispatch chain checks `Capsule` and `Segment` only *after* the `Polygon` pair, using `colliderA.GetType() == Capsule` checks in an `if-else` ladder without a `Circle vs Capsule` or `Polygon vs Capsule` branch. If entityA is a Circle and entityB is a Capsule, none of the first four conditions fire and the Capsule branch *does* run, but the `CapsuleCollision` implementation does not handle `Circle vs Capsule` correctly — it falls through to the catch-all return.

**Fix:** Replace the if-else ladder with a dispatch table keyed on `(typeA, typeB)` pairs:
```cpp
ContactManifold ManifoldGenerator::GenerateManifold(...) {
    using ST = ColliderComponent::ShapeType;
    ST tA = colliderA.GetType(), tB = colliderB.GetType();
    if (tA > tB) { std::swap(tA,tB); /* also swap all A/B args */ }
    if (tA==ST::Circle  && tB==ST::Circle)  return CircleCircle(...);
    if (tA==ST::Circle  && tB==ST::Polygon) return CirclePolygon(...);
    if (tA==ST::Circle  && tB==ST::Capsule) return CircleCapsule(...);
    if (tA==ST::Polygon && tB==ST::Polygon) return PolygonPolygon(...);
    if (tA==ST::Polygon && tB==ST::Capsule) return PolygonCapsule(...);
    if (tA==ST::Capsule && tB==ST::Capsule) return CapsuleCapsule(...);
    if (tA==ST::Segment || tB==ST::Segment) return SegmentCollision(...);
    return manifold; // touching=false
}
```
Implement missing pair functions (`CircleCapsule`, `PolygonCapsule`) using the closest-point-on-segment primitive already present in `ContinuousCollisionDetection::ClosestPointOnPolygon`.

---

### 5.2 `CirclePolygon` Does Not Rotate Polygon Vertices

**Problem:** In `ManifoldGenerator::CirclePolygon`, the polygon vertices from `PolygonShape::vertices` are stored in *local space*. When computing face normals and penetration, the code must transform vertices into world space using the polygon's transform rotation. Omitting the rotation gives wrong contact normals for any rotated polygon.

**Fix:** Before any projection or distance computation, apply the transform:
```cpp
// At the start of CirclePolygon:
std::vector<Math::Vector2> worldVerts;
std::vector<Math::Vector2> worldNormals;
ComputePolygonWorld(polygon, polyTransform, worldVerts, worldNormals);
// Then use worldVerts/worldNormals for all subsequent geometry
```
`ComputePolygonWorld` is already defined as a local helper in `ManifoldGenerator.cpp` but is not called in `CirclePolygon`. Call it.

---

### 5.3 `PolygonPolygon` Reference Face Selection Uses Wrong Axis

**Problem:** In `PolygonPolygon` SAT separation, `ProjectPolygon` in `ContinuousCollisionDetection` returns `maxProj` (the maximum projection value), not the minimum overlap. The separation computed as:
```cpp
separation = ProjectPolygon(vertsA, posA, normal) - ProjectPolygon(vertsB, posB, -normal);
```
is mixing max-projection of A on the axis with max-projection of B on the negated axis. The correct SAT separation for axis N is:
```
min_separation = max_projection(B on N) - max_projection(A on N)
```
when N points from A toward B. The current code double-counts and can report false negative separations, causing missed collisions.

**Fix:** Rewrite the SAT projection to return both min and max, then compute separation as:
```cpp
// On axis N (face normal of A pointing outward):
float sepA = ProjectMin(vertsB, posB, normal) - ProjectMax(vertsA, posA, normal);
// If sepA > 0: shapes are separated on this axis — early out
minSeparation = std::min(minSeparation, sepA);
```

---

### 5.4 Contact Point Position Formula Is Wrong for Circle vs Circle

**Problem:**
```cpp
cp.position = centerA + normal * (circleA.radius - 0.5f * penetration);
```
This attempts to place the contact at the midpoint of the overlap, but the formula is incorrect. The midpoint between the two circle surfaces along the collision normal is:
```
contactPoint = centerA + normal * circleA.radius    (on surface of A)
```
or equivalently:
```
contactPoint = (centerA + normal*circleA.radius + centerB - normal*circleB.radius) * 0.5f
```
**Fix:**
```cpp
cp.position = centerA + normal * circleA.radius;
// i.e. the contact is on the surface of circle A toward circle B
```

---

### 5.5 `manifold.localNormal` and `manifold.localPoint` Always Set to `{0,0}`

**Problem:** After every manifold is generated, `manifold.localNormal` and `manifold.localPoint` are left as zero vectors. These are used by `ConstraintSolverSystem` during position correction to re-derive the contact normal in local space — using `{0,0}` for them means position correction always pushes along the zero vector (i.e., does nothing).

**Fix:** After computing the world-space normal, convert it to body-A local space:
```cpp
manifold.localNormal = transformA.rotation.Inverse() * manifold.normal;
manifold.localPoint  = transformA.rotation.Inverse() * (manifold.points[0].position - transformA.position);
```

---

## 6. SAT Collision Detector

### 6.1 `ClipSegmentToLine` Only Clips Two Points — Cannot Handle Polygon Faces

**Problem:** The current implementation hardcodes exactly two input contact points (`vIn[0]` and `vIn[1]`). Sutherland-Hodgman clipping for polygon-polygon collision can have up to `2*N` candidate points. This means contact reduction silently produces incorrect manifolds for any polygon with more than 4 vertices.

**Fix:** Implement the full Sutherland-Hodgman loop:
```cpp
void SATCollisionDetector::ClipSegmentToLine(
        std::vector<ContactPoint>& vOut,
        const std::vector<ContactPoint>& vIn,
        const Math::Vector2& normal,
        float offset) {
    vOut.clear();
    if (vIn.empty()) return;
    for (size_t i = 0; i < vIn.size(); ++i) {
        const ContactPoint& c1 = vIn[i];
        const ContactPoint& c2 = vIn[(i + 1) % vIn.size()];
        float d1 = Math::Vector2::Dot(normal, c1.position) - offset;
        float d2 = Math::Vector2::Dot(normal, c2.position) - offset;
        if (d1 <= 0.0f) vOut.push_back(c1);
        if ((d1 < 0.0f) != (d2 < 0.0f)) {
            float t = d1 / (d1 - d2);
            ContactPoint cp = c1;
            cp.position = c1.position + (c2.position - c1.position) * t;
            vOut.push_back(cp);
        }
    }
}
```

---

### 6.2 `FindIncidentFace` Returns Only One Vertex Index — Needs Two Consecutive Vertices

**Problem:** `FindIncidentFace` finds `incidentIndex` (most anti-parallel face) but then sets:
```cpp
incidentIndex2 = (incidentIndex + 1) % vertsB.size();
```
This computes the *end* vertex of the incident face using `vertsB.size()` but the normal at `incidentIndex` corresponds to the edge from vertex `incidentIndex` to vertex `(incidentIndex+1) % n`. This is correct only when normals are stored in edge-order. If `normalsB` and `vertsB` have different sizes (possible when normals are cached differently), this produces wrong indices.

**Fix:** Assert that `normalsB.size() == vertsB.size()` at the start of `FindIncidentFace`. Add this invariant check to `PolygonShape::BuildNormals()` as well.

---

### 6.3 `SATCollisionDetector::CCD::ComputeTOI` Ignores Angular Velocities

**Problem:**
```cpp
B2_UNUSED(angularVelocityA);
B2_UNUSED(angularVelocityB);
```
Both angular velocities are completely ignored in the TOI computation. For fast-spinning bodies, this means tunnelling is not detected for the rotational component of motion.

**Fix:** Either implement a full GJK-based TOI that accounts for rotation, or at minimum conservatively expand the query radius by the expected rotational displacement:
```cpp
float extentA = /* half-diagonal of shapeA AABB */;
float extentB = /* half-diagonal of shapeB AABB */;
float rotationalMotionA = std::abs(angularVelocityA) * extentA * dt;
float rotationalMotionB = std::abs(angularVelocityB) * extentB * dt;
distance += rotationalMotionA + rotationalMotionB; // conservative expansion
```

---

### 6.4 `CCD::ConservativeAdvancement` Uses Incorrect `pointA`/`pointB` Arguments

**Problem:** The call site:
```cpp
fraction = ConservativeAdvancement(cp.position, cp.position, velocityA, velocityB, distance);
```
passes the same point (`cp.position`) for both `pointA` and `pointB`. Inside the function, `delta = pointB - pointA` is therefore the zero vector, so `direction` defaults to zero and `relativeSpeed` is computed as zero, causing the function to always return `1.0f` (no collision).

**Fix:**
```cpp
// The two points should be the contact locations on each body surface:
Math::Vector2 ptA = cp.position - cp.normal * (cp.separation * 0.5f); // on A surface
Math::Vector2 ptB = cp.position + cp.normal * (cp.separation * 0.5f); // on B surface
fraction = ConservativeAdvancement(ptA, ptB, velocityA, velocityB, -cp.separation);
```

---

## 7. Constraint Solver

### 7.1 Warm-Starting Applies Impulses With Wrong Sign Convention

**Problem:** In `ConstraintSolverSystem::WarmStart`:
```cpp
bodyA.velocity = bodyA.velocity - P * vc.invMassA;
```
The minus sign means body A is pushed *away* from the contact, which is wrong. Warm-starting should re-apply the cached impulse in the same direction it was originally applied (body A receives `-P`, body B receives `+P` when `P = normal * normalImpulse + tangent * tangentImpulse`).

The correct convention (matching Box2D): impulse P acts on B in the normal direction; A receives the reaction:
```cpp
// Body A: subtract impulse (reaction)
if (!bodyA.isStatic) {
    bodyA.velocity      -= P * vc.invMassA;
    bodyA.angularVelocity -= vc.invIA * CrossScalar(vcp.rA, P);
}
// Body B: add impulse
if (!bodyB.isStatic) {
    bodyB.velocity      += P * vc.invMassB;
    bodyB.angularVelocity += vc.invIB * CrossScalar(vcp.rB, P);
}
```
This is already the intent — but verify the sign of `P` is consistent with the normal direction convention used throughout the solver. The normal must always point from A to B. Audit `ManifoldGenerator` to ensure this invariant.

---

### 7.2 Friction Impulse Not Accumulated With Proper Clamping

**Problem:**
```cpp
float tangentImpulse = -vcp.tangentMass * vt;
float maxFriction = vc.friction * vcp.normalImpulse;
tangentImpulse = std::clamp(tangentImpulse, -maxFriction, maxFriction);
float oldTangentImpulse = vcp.tangentImpulse;
vcp.tangentImpulse = oldTangentImpulse + tangentImpulse;
tangentImpulse = vcp.tangentImpulse - oldTangentImpulse;
```
The clamping is applied to the *delta* impulse before accumulation, but the Coulomb friction cone constraint requires clamping the *accumulated* total:
```cpp
float newTangent = std::clamp(vcp.tangentImpulse + delta, -maxFriction, maxFriction);
float appliedDelta = newTangent - vcp.tangentImpulse;
vcp.tangentImpulse = newTangent;
// Apply appliedDelta to bodies
```
**Fix:** Replace the current pattern with accumulated clamping as shown above.

---

### 7.3 `StoreImpulses` Matches Manifolds by Entity ID Only — Ignores Shape IDs

**Problem:**
```cpp
if (manifold.entityIdA == bodyA.entityId && manifold.entityIdB == bodyB.entityId)
```
Two bodies can have multiple contact pairs (e.g., a box resting on a corner contacts two separate faces of a floor). With only entity ID matching, impulses from one face are written back to whichever manifold happens to match first, corrupting warm-start data.

**Fix:** Match by `(entityIdA, entityIdB, shapeIdA, shapeIdB)` tuple, exactly as `ContactPair` does.

---

### 7.4 `VelocityConstraint` Does Not Store Shape IDs for Impulse Caching

**Problem:** `VelocityConstraint` only stores `indexA` and `indexB` (solver body indices, not entity IDs, not shape IDs). `StoreImpulses` then has to reverse-lookup through `bodyA.entityId`, making it impossible to distinguish multiple collision pairs between the same two entities.

**Fix:** Add `uint32_t entityIdA, entityIdB, shapeIdA, shapeIdB` fields to `VelocityConstraint` and populate them from the contact manifold during `InitializeVelocityConstraints`.

---

### 7.5 `StabilizationSystem::ApplyWarmStarting` Mutates a `const` Vector

**Problem:**
```cpp
void StabilizationSystem::ApplyWarmStarting(
    const std::vector<PersistentContact>& persistentContacts,
    const std::vector<SolverBody>& solverBodies,   // <-- const
    ...
) {
    ...
    solverBodies[vc.indexA].linearVelocity += P * invMassA; // modifies const ref!
```
`solverBodies` is declared `const` but the function mutates it. This compiles only if the underlying storage is non-const and the reference is cast away — which is undefined behaviour. On some compilers this silently does nothing (warm-start has no effect).

**Fix:** Change the parameter to a non-const reference:
```cpp
void StabilizationSystem::ApplyWarmStarting(
    const std::vector<PersistentContact>& persistentContacts,
    std::vector<SolverBody>& solverBodies,  // non-const
    ...
)
```

---

## 8. Continuous Collision Detection (CCD)

### 8.1 `ContinuousCollisionSystem::PerformCCD` Falls Back to AABB-Only TOI

**Problem:** After computing predicted positions, the function calls `CollisionDetectionStrategy::CalculateTimeOfImpact` which internally does only AABB vs AABB intersection sweep. For non-rectangular shapes (circles, capsules, polygons) this is at best an over-approximation. For fast-spinning polygons, it produces spurious TOI values of 0.

**Fix:** Route the CCD call through `ContinuousCollisionDetection::PolygonPolygonCCD` (or the appropriate typed variant) which performs the actual shape-aware conservative advancement. The AABB-sweep result should only be used as a *pre-filter* (if AABBs don't sweep-intersect, skip expensive CCD), not as the final TOI.

---

### 8.2 `ResolveCCDCollision` Computes Normal From AABB Overlap, Not from Actual Shape

**Problem:**
```cpp
float overlapX = (halfSizeA.x + halfSizeB.x) - std::abs(delta.x);
float overlapY = (halfSizeA.y + halfSizeB.y) - std::abs(delta.y);
if (overlapX < overlapY) {
    normal = {delta.x > 0 ? -1.0f : 1.0f, 0.0f};
} else {
    normal = {0.0f, delta.y > 0 ? -1.0f : 1.0f};
}
```
AABB normals are always axis-aligned (`(±1,0)` or `(0,±1)`). For a tilted polygon hitting another polygon, the correct contact normal is oblique. Using the AABB normal sends the impulse in entirely the wrong direction.

**Fix:** After CCD positions the bodies at the TOI, call `ManifoldGenerator::GenerateManifold` to get the accurate contact normal and penetration depth. Use those for impulse resolution.

---

### 8.3 `penetrationDepth` Is Assigned Negative But Used Without Sign Awareness

**Problem:**
```cpp
penetrationDepth = -std::min(overlapX, overlapY);
```
Then later:
```cpp
if (penetrationDepth < -0.01f) {
    Math::Vector2 correction = normal * (std::abs(penetrationDepth) * BAUMGARTE_FACTOR);
```
The sign convention is inverted and recovered with `std::abs`, creating a double-negation that produces correct results only by accident. If the convention ever changes, this silently breaks.

**Fix:** Use a consistent convention: `penetrationDepth > 0` means overlap. Store it as a positive quantity and remove the `std::abs` call:
```cpp
penetrationDepth = std::min(overlapX, overlapY); // positive = overlap
if (penetrationDepth > 0.01f) {
    Math::Vector2 correction = normal * (penetrationDepth * BAUMGARTE_FACTOR);
```

---

### 8.4 CCD Does Not Handle Already-Overlapping Bodies Correctly

**Problem:**
```cpp
if (timeOfImpact == 0.0f) {
    transformA.position = startPosA;
    transformB.position = startPosB;
    std::cout << "[DEBUG] Handling already-overlapping case...";
}
```
Reverting to `startPosA`/`startPosB` restores the penetrating state. The correct handling is to run position correction (Baumgarte or split impulse) immediately, not to do nothing.

**Fix:** When `toi == 0.0f`, skip position advancement and proceed directly to `ManifoldGenerator` → impulse resolution with depenetration. The CCD position step is skipped; only the velocity/position correction step runs.

---

## 9. Position Correction / Stabilization

### 9.1 `ApplySplitImpulses` Sign of Separation Check Is Inverted

**Problem:**
```cpp
float C = std::max(pc.separation + baumgarteSlop, 0.0f);
float positionImpulse = -baumgarteBeta * C;
```
`C` is clamped to be `≥ 0`. A positive `C` means there *is* penetration to correct. Then `positionImpulse = -beta * C` is *negative*. The correction `P = normal * positionImpulse` is therefore in the *negative normal direction*, meaning bodies are pushed *into* each other instead of apart.

**Fix:**
```cpp
// separation < 0 means penetration. Correct when separation < -slop.
float C = std::min(pc.separation + baumgarteSlop, 0.0f); // <= 0 when penetrating
float positionImpulse = -baumgarteBeta * C; // now positive = push apart
positionImpulse = std::clamp(positionImpulse, 0.0f, maxPositionCorrection);
Math::Vector2 P = pc.normal * positionImpulse;
// Body A moves in -normal direction (pushed back)
solverBodies[pc.indexA].position -= P * invMassA;
// Body B moves in +normal direction (pushed forward)
solverBodies[pc.indexB].position += P * invMassB;
```

---

### 9.2 `StabilizationSystem::ComputeFeatureId` Ignores Contact Position

**Problem:** Feature ID is computed only from the contact normal (quantized). Two contact points on the same face but at different positions will have identical feature IDs, causing the warm-start cache to merge them into one persistent contact and lose one impulse.

**Fix:** Include the quantized local contact position in the hash:
```cpp
uint32_t StabilizationSystem::ComputeFeatureId(
        const ContactPoint& cp,
        const Math::Vector2& normal) {
    const float q = 10.0f;
    auto qi = [q](float v) { return static_cast<int32_t>(std::round(v * q)); };
    uint32_t hash = 0;
    auto mix = [&](uint32_t v) {
        hash ^= v + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    };
    mix(static_cast<uint32_t>(qi(normal.x)));
    mix(static_cast<uint32_t>(qi(normal.y)));
    mix(static_cast<uint32_t>(qi(cp.position.x)));
    mix(static_cast<uint32_t>(qi(cp.position.y)));
    return hash;
}
```

---

### 9.3 Warm-Start Impulse Is Scaled by `dtRatio` But Also by a Constant 0.9

**Problem:**
```cpp
float scaledNormalImpulse = pc.normalImpulse * warmStartScaling * dtRatio;
```
`warmStartScaling = 0.9f` is applied unconditionally on every tick. Over several ticks, this exponentially decays the accumulated impulse to zero, defeating the purpose of warm-starting. The decay should only occur when the timestep changes.

**Fix:** Remove the constant `warmStartScaling` factor. Scale only by `dtRatio` (which is `1.0` for constant timestep):
```cpp
float scaledNormalImpulse  = pc.normalImpulse  * dtRatio;
float scaledTangentImpulse = pc.tangentImpulse * dtRatio;
```

---

## 10. Island Manager

### 10.1 `BuildContactGraph` Connects All Non-Static Pairs Unconditionally

**Problem:**
```cpp
for (size_t i = 0; i < activeEntities.size(); ++i) {
    for (size_t j = i + 1; j < activeEntities.size(); ++j) {
        m_ContactGraph[i].push_back(j);
        m_ContactGraph[j].push_back(i);
    }
}
```
Every dynamic body is connected to every other dynamic body in the graph, regardless of whether they are actually in contact. This means all dynamic bodies end up in one giant island, preventing the sleep system from ever putting sub-groups to sleep.

**Fix:** Build the contact graph only from *active manifolds* in `PhysicsWorldComponent::contactManifolds`:
```cpp
void IslandManager::BuildContactGraph(const std::vector<ECS::EntityID>& activeEntities) {
    m_ContactGraph.clear();
    for (const auto& manifold : m_PhysicsWorld.contactManifolds) {
        if (!manifold.touching) continue;
        ECS::EntityID a = manifold.entityIdA;
        ECS::EntityID b = manifold.entityIdB;
        m_ContactGraph[a].push_back(b);
        m_ContactGraph[b].push_back(a);
    }
}
```

---

### 10.2 Island Sleep Check Does Not Account for Angular Velocity

**Problem:** `GetBodySleepVelocity` returns `body.velocity.Length()` (linear only). `ShouldIslandSleep` separately checks `body.angularVelocity` but only after `GetBodySleepVelocity`. The `UpdateSleepTimers` function uses only `GetBodySleepVelocity`, so a body spinning in place with zero linear velocity will go to sleep even though it is visibly rotating.

**Fix:** Combine both in a single metric inside `GetBodySleepVelocity`:
```cpp
float IslandManager::GetBodySleepVelocity(ECS::EntityID bodyId) const {
    const auto& body = m_ComponentStore.GetComponent<ECS::PhysicsBodyComponent>(bodyId);
    // Weighted sum: angular velocity contribution scaled by body extent
    float extent = /* body's half-diagonal extent */;
    return body.velocity.LengthSquared() + (body.angularVelocity * extent) * (body.angularVelocity * extent);
    // Compare squared to SLEEP_THRESHOLD^2 in callers
}
```

---

### 10.3 `BodyIslandMap` Index Goes Stale After `FindIslands` Moves Islands

**Problem:**
```cpp
m_BodyIslandMap[connectedBody] = m_AllIslands.size(); // captured by value before push_back
```
The index is assigned before `m_AllIslands.push_back(std::move(newIsland))`. After the push_back, `m_AllIslands.size()` has increased, so the stored index is `size - 1` (the just-pushed island). This is a coincidental off-by-one that happens to be correct *only* because the island is pushed immediately after the map entry is written — any refactor breaks this silently.

**Fix:** Capture the index explicitly:
```cpp
size_t islandIndex = m_AllIslands.size(); // before push_back
m_AllIslands.push_back(std::move(newIsland));
// Then in FloodFill, assign islandIndex to all bodies in the island
m_BodyIslandMap[bodyId] = islandIndex;
```

---

## 11. Rendering

### 11.1 `Renderer2D` Uses a Global Singleton With No Thread Safety

**Problem:** `Renderer2D::s_Instance` is a raw static pointer. All drawing functions check `if (!s_Instance)` with no mutex. If rendering is ever moved to a background thread (or called from a job system), this creates a data race.

**Fix:** For the current single-threaded architecture, document the singleton as not thread-safe and add a static assertion. For future proofing, convert to a passed-by-reference context object.

---

### 11.2 `DrawQuad` Does Not Apply Rotation

**Problem:** `DrawQuad` accepts an `origin` and `size` but no rotation angle. Entities with non-zero `TransformComponent::rotation` are drawn as axis-aligned rectangles.

**Fix:** Add a `float rotation = 0.0f` parameter:
```cpp
void Renderer2D::DrawQuad(const Math::Vector2& position,
                          const Math::Vector2& size,
                          const Math::Vector2& origin,
                          const Math::Vector3& color,
                          float rotation = 0.0f) {
    // Build 4 corners, rotate each around `position - origin`, then draw 2 triangles
    float c = std::cos(rotation), s = std::sin(rotation);
    auto rot = [&](Math::Vector2 p) -> Math::Vector2 {
        return {p.x*c - p.y*s, p.x*s + p.y*c};
    };
    Math::Vector2 corners[4] = {
        position + rot(-origin),
        position + rot({size.x - origin.x, -origin.y}),
        position + rot({size.x - origin.x, size.y - origin.y}),
        position + rot({-origin.x, size.y - origin.y})
    };
    // Push 2 triangles into QuadBuffer
}
```
Update `RenderSystem` to pass `transform.rotation` to `DrawQuad`.

---

### 11.3 `DrawSolidPolygon` Assumes Convex + CCW Winding — No Validation

**Problem:** `DrawSolidPolygon` uses a triangle-fan decomposition from `vertices[0]`. This is only correct for convex polygons with CCW winding. If a concave polygon is passed, the fan produces incorrect (overlapping) triangles with incorrect fill.

**Fix:** Add a convex check in debug builds. For concave polygons, use an ear-clipping triangulation. At minimum, document the convex precondition in the API and assert it:
```cpp
#ifndef NDEBUG
assert(IsConvex(vertices) && "DrawSolidPolygon requires a convex polygon");
#endif
```

---

### 11.4 `DrawSolidCapsule` Body Rectangle Uses Wrong Vertex Order

**Problem:**
```cpp
std::vector<Math::Vector2> bodyVerts = {
    center1 + side1, center1 + side2,
    center2 + side1, center2 + side2
};
// Triangles: 0,2,1 and 0,3,2
```
Vertices are ordered `[topA, bottomA, topB, bottomB]`. Triangle `(0,2,1)` = `(topA, topB, bottomA)` and `(0,3,2)` = `(topA, bottomB, topB)`. This produces a crossed (bow-tie) quad rather than a rectangle.

**Fix:** Use CCW winding:
```cpp
// Triangle 1: topA, topB, bottomA  →  should be topA, bottomA, bottomB
// Triangle 2: topA, bottomB, topB
// Correct CCW quads:
// v0=center1+side1 (top-left), v1=center2+side1 (top-right),
// v2=center2+side2 (bot-right), v3=center1+side2 (bot-left)
QuadBuffer.push_back({v0}); QuadBuffer.push_back({v1}); QuadBuffer.push_back({v2});
QuadBuffer.push_back({v0}); QuadBuffer.push_back({v2}); QuadBuffer.push_back({v3});
```

---

### 11.5 `DrawSector` and `DrawSolidSector` Are Identical

**Problem:** `DrawSolidSector` simply calls `DrawSector`:
```cpp
void Renderer2D::DrawSolidSector(...) {
    DrawSector(center, radius, angleStart, angleEnd, color, segments);
}
```
`DrawSector` renders using the `QuadBuffer` with filled triangles, so both functions are effectively the same. The `DrawSector` (outline) version should instead draw only the boundary lines (two radii + arc).

**Fix:**
```cpp
void Renderer2D::DrawSector(/* outline version */) {
    // Draw two radial lines
    Math::Vector2 start{center.x + radius*cos(angleStart), center.y + radius*sin(angleStart)};
    Math::Vector2 end  {center.x + radius*cos(angleEnd),   center.y + radius*sin(angleEnd)};
    DrawLine(center, start, color);
    DrawLine(center, end,   color);
    DrawArc(center, radius, angleStart, angleEnd, color, 1.0f, segments);
}
// Rename the filled version to DrawSolidSector and keep the triangle-fan code there.
```

---

### 11.6 `PI` Value Is a Truncated Literal

**Problem:** Throughout the renderer, `3.1415926535f` is used. This is only 10 significant digits. Use the standard constant:
```cpp
#include <numbers> // C++20
constexpr float PI = std::numbers::pi_v<float>;
// or for C++17:
constexpr float PI = 3.14159265358979323846f;
```

---

## 12. Input System

### 12.1 `IsKeyPressed` / `IsMousePressed` Break When `s_InputDirty` Is False

**Problem:** `InputManager::Update` copies `CurrentKeys` to `PreviousKeys` only when `s_InputDirty == true`. If no key events arrive for a frame, `PreviousKeys` equals `CurrentKeys`, which means `IsKeyPressed` (which checks `Current && !Previous`) returns `false` for a key that was pressed last frame and held. This is correct *only for held keys*. But if a key is pressed, `Update` is called with dirty=false (no new event), then `IsKeyPressed` sees `Current=true`, `Previous=false` (still from *two frames ago*) and reports a spurious "just pressed" on the second frame.

Actually the opposite bug is possible: if key is pressed (dirty=true, Previous updated to old state) → next frame no event (dirty=false, Previous NOT updated) → `IsKeyPressed` sees `Current=true, Previous=false` again → reports "pressed" for a second frame.

**Fix:** Always copy previous state at the start of each frame, unconditionally:
```cpp
void InputManager::Update() {
    memcpy(s_PreviousKeys,         s_CurrentKeys,         sizeof(s_CurrentKeys));
    memcpy(s_PreviousMouseButtons, s_CurrentMouseButtons, sizeof(s_CurrentMouseButtons));
    s_InputDirty = false;
    // GLFW callbacks update s_CurrentKeys asynchronously via event queue
}
```

---

### 12.2 `Init` Polls All Keys at Startup — GLFW Does Not Support This Reliably

**Problem:**
```cpp
for (int i = 0; i < GLFW_KEY_LAST; ++i) {
    if (glfwGetKey(window, i) == GLFW_PRESS) { ... }
}
```
`GLFW_KEY_LAST` is 348. Many key codes in the range `[0, 348]` are not valid GLFW key tokens. Calling `glfwGetKey` with an invalid key code returns `GLFW_RELEASE` on most platforms but is technically undefined. On Wayland backends, this loop has caused crashes.

**Fix:** Remove the startup poll loop. The initial state of all keys is correctly "not pressed"; GLFW callbacks will fire for any key that is down at window focus time.

---

## 13. Global Hardening & Cleanup Strategy

### 13.1 Remove All `assert` in Public-Facing Component Access

Replace `assert(HasComponent<T>(entity))` in `GetComponent` with a proper exception or logged error in release builds. Asserts are stripped in release and the downstream crash is far harder to debug.

### 13.2 Unify `SolverBody` Duplication

`Physics::SolverBody` (in `ConstraintSolver.h`) and `ECS::ConstraintSolverSystem::SolverBody` (defined inline in the system header) are two different structs with overlapping but not identical fields. Consolidate into one `Physics::SolverBody` used everywhere.

### 13.3 Move Magic Constants to `PhysicsWorldComponent`

The following magic numbers appear scattered across multiple files and must be centralized:

| Constant | Current Location | Move to |
|---|---|---|
| `FIXED_TIMESTEP = 1/60` | `Application.h` | `PhysicsWorldComponent` |
| `BAUMGARTE_FACTOR = 0.6f` | `ContinuousCollisionSystem.cpp` | `PhysicsWorldComponent::baumgarteBeta` |
| `ALLOWABLE_SLOP = 0.01f` | CCD, ConstraintSolver | `PhysicsWorldComponent::linearSlop` |
| `MAX_PENETRATION_CORRECTION = 50.0f` | CCD | `PhysicsWorldComponent` |
| `SLEEP_THRESHOLD` | Island.h | `PhysicsWorldComponent::sleepLinearThreshold` |
| `0.999f` / `0.995f` damping | ConstraintSolverSystem | `PhysicsBodyComponent::linearDamping/angularDamping` |

### 13.4 Replace Raw Pointer `m_PhysicsWorld` With a Reference or Checked Pointer

In `ConstraintSolverSystem`, `CollisionPipelineSystem`, and others, `m_PhysicsWorld` is a raw `PhysicsWorldComponent*` initialised to `nullptr`. Calls like `m_PhysicsWorld->velocityIterations` will crash if `Initialize` was not called. Either use a reference (cannot be null) or add a null check at the start of every `Update` method and assert/throw on null.

### 13.5 Do Not Store `fstream` / `chrono` Includes in `ManifoldGenerator.cpp`

`ManifoldGenerator.cpp` includes `<fstream>` and `<chrono>` but uses neither. Remove them.

### 13.6 Inertia Tensor Computation Must Be Shape-Aware

`PhysicsBodyComponent` stores `inverseMass` and `inverseInertia`. There is no code in the codebase that computes `inverseInertia` from the actual shape. For a solid rectangle:
```
I = (mass / 12) * (width² + height²)
```
For a solid circle:
```
I = 0.5 * mass * radius²
```
For a convex polygon (shoelace formula):
```
I = (mass / 6) * Σ|cross(v_i, v_{i+1})| * (dot(v_i,v_i) + dot(v_i,v_{i+1}) + dot(v_{i+1},v_{i+1}))
    ──────────────────────────────────────────────────────────────────────────────────────────────────
                         Σ|cross(v_i, v_{i+1})|
```

Add a `ColliderComponent::ComputeInertia(float mass) const` method that returns the correct moment of inertia, and call it during `PhysicsBodyComponent` initialization.

### 13.7 Circle Collider Must Offset Center by Transform — Currently Added at Manifold Time Only

`ColliderComponent::CircleShape::center` is an offset from the entity origin. In `ManifoldGenerator`, `ComputeCircleCenters` adds this offset at query time. But `CollisionPipelineSystem::UpdateShapeAABB` does not add the circle center offset when building the broad-phase AABB, causing misaligned AABBs for off-center circles.

**Fix:** In `UpdateShapeAABB`, for `ShapeType::Circle`:
```cpp
Math::Vector2 worldCenter = position + circle.center;
AABB aabb{worldCenter - Math::Vector2{radius, radius} - margin,
          worldCenter + Math::Vector2{radius, radius} + margin};
```

### 13.8 `OnFixedUpdate` Default Implementation Calls `OnUpdate`

In `Application.h`:
```cpp
virtual void OnFixedUpdate(float deltaTime) { OnUpdate(deltaTime); }
```
This means if a subclass overrides `OnUpdate` but not `OnFixedUpdate`, the update logic runs twice per fixed step: once through `OnFixedUpdate→OnUpdate` and once through the variable `OnUpdate` call in the game loop. The default should be an empty body.

**Fix:**
```cpp
virtual void OnFixedUpdate(float deltaTime) {}
```

---

## Summary Priority Table

| Priority | Section | Issue | Impact |
|---|---|---|---|
| P0 | 3.1 | Gravity never applied | Bodies weightless |
| P0 | 9.1 | Split impulse sign inverted | Bodies sink through floors |
| P0 | 7.2 | Friction impulse clamped wrong | Infinite friction / no friction |
| P0 | 1.1 | Dual collision systems | Double-resolution corruption |
| P0 | 10.1 | Island graph connects all bodies | Sleep system non-functional |
| P1 | 5.1 | Shape dispatch priority wrong | Capsule/Circle pairs undetected |
| P1 | 5.2 | CirclePolygon ignores rotation | Wrong normals for rotated shapes |
| P1 | 5.3 | SAT projection formula wrong | Missed/phantom collisions |
| P1 | 8.2 | CCD normal from AABB only | Wrong impulse direction |
| P1 | 8.4 | `ConservativeAdvancement` zero delta | CCD always returns no-collision |
| P1 | 7.1 | Warm-start sign error | Solver diverges on warm-start |
| P1 | 4.2 | AABB ignores rotation | Broad-phase misses on rotated bodies |
| P2 | 2.1 | O(N) component lookup | Performance |
| P2 | 2.2 | Memory never reclaimed | Memory leak |
| P2 | 3.2 | Frame-rate-dependent damping | Inconsistent simulation |
| P2 | 7.3 | Impulse cache keyed on entity only | Multi-contact warm-start wrong |
| P2 | 11.2 | DrawQuad ignores rotation | Visual desync from physics |
| P2 | 11.4 | Capsule rectangle crossed | Visual artifact |
| P3 | 1.6 | `B2_UNUSED` undefined macro | Compile error on strict compilers |
| P3 | 1.7 | `std::cout` in production | Log spam in release |
| P3 | 13.3 | Magic constants everywhere | Maintainability |

---

---

## 14. SystemManager

### 14.1 `m_Systems` Vector Has a Typo — Does Not Compile

**Problem:**
```cpp
std::vector<std::unique_ptr<s>> m_Systems;
```
`std::unique_ptr<s>` references an undefined type `s`. This is a clear LLM hallucination / transcription error. The code cannot compile as-is.

**Fix:**
```cpp
std::vector<std::unique_ptr<System>> m_Systems;
```

---

### 14.2 `GetSystem<T>` Uses `dynamic_cast` Every Call

**Problem:** `GetSystem<T>()` iterates all systems and `dynamic_cast`s each one. Since this may be called every frame (e.g., `ECSApplication::GetDebugRenderSystem()`), this is unnecessary overhead.

**Fix:** Add a `std::unordered_map<std::type_index, System*>` lookup table maintained in `AddSystem`. `GetSystem<T>` becomes a single map lookup:
```cpp
template<typename T>
T* GetSystem() {
    auto it = m_SystemLookup.find(typeid(T));
    return it != m_SystemLookup.end() ? static_cast<T*>(it->second) : nullptr;
}
```

---

## 15. Vector2 Math

### 15.1 `Vector2` Has a Spurious 3-Argument Constructor

**Problem:**
```cpp
Vector2(float x, float y, float z) : x(x), y(y) {}
```
The `z` parameter is silently discarded. This constructor exists nowhere in any standard math library and is an LLM hallucination. Any code calling `Vector2(a, b, c)` expecting a 3D vector will silently drop the z component.

**Fix:** Delete this constructor entirely. If 3D vectors are needed, use `Vector3`.

---

### 15.2 `Vector2::Length()` and `Vector3::Length()` Use Unqualified `sqrt`

**Problem:**
```cpp
float Length() const { return sqrt(x * x + y * y); }
```
`sqrt` without `std::` relies on a C-header `<math.h>` being included transitively. If only `<cmath>` is included (the C++ standard header), the name `sqrt` without `std::` is technically in the global namespace only by implementation courtesy, not guaranteed.

**Fix:** Replace all bare `sqrt`, `fabs`, `floor` calls with `std::sqrt`, `std::abs`, `std::floor` throughout both Vector headers.

---

### 15.3 `Vector2::Normalize()` Is Not `[[nodiscard]]` — Silent Discard

**Problem:** A common bug is `velocity.Normalize()` instead of `velocity = velocity.Normalize()`. With no `[[nodiscard]]`, the compiler silently drops the result.

**Fix:**
```cpp
[[nodiscard]] Vector2 Normalize() const { ... }
```
Apply `[[nodiscard]]` to all pure-return methods: `Normalize`, `Length`, `LengthSquared`, `Dot`, `Cross`.

---

### 15.4 `Vector2` Has No `DistanceSquared` Static Method — But Is Called in Codebase

**Problem:** `StabilizationSystem` calls `Math::Vector2::DistanceSquared(a, b)` but this method is not declared in `Vector2.h`. This is a compilation error.

**Fix:** Add the missing method:
```cpp
static float DistanceSquared(const Vector2& a, const Vector2& b) {
    float dx = a.x - b.x, dy = a.y - b.y;
    return dx * dx + dy * dy;
}
```

---

### 15.5 `Vector2` Has No `Rotation2D` Helper — But Is Referenced in `StabilizationSystem.h`

**Problem:** `StabilizationSystem` uses `Math::Rotation2D` (a 2×2 rotation matrix type) and calls `.Inverse()` on it. No such type is defined anywhere in the math headers. This is another compilation error from LLM hallucination.

**Fix:** Add a `Rotation2D` struct to `Vector2.h`:
```cpp
struct Rotation2D {
    float c, s; // cos, sin
    Rotation2D() : c(1.0f), s(0.0f) {}
    explicit Rotation2D(float angle) : c(std::cos(angle)), s(std::sin(angle)) {}
    Vector2 operator*(const Vector2& v) const {
        return {c * v.x - s * v.y, s * v.x + c * v.y};
    }
    Rotation2D Inverse() const { return {c, -s}; } // transpose = inverse for rotation
    // Note: stores c and s directly in constructor to avoid aliasing
    Rotation2D(float cos_val, float sin_val) : c(cos_val), s(sin_val) {}
};
```

---

## 16. PhysicsBodyComponent

### 16.1 `UpdateMassProperties` Resets `mass` to 0 When `isStatic` Is True

**Problem:**
```cpp
void UpdateMassProperties() {
    if (isStatic || isKinematic || mass <= 0.0f) {
        mass = 0.0f;  // <-- overwrites user-provided mass for static bodies
        inverseMass = 0.0f;
        ...
    }
}
```
If a body is static, its mass is zeroed. If it is later made dynamic (`isStatic = false`), its mass is permanently lost and `SetMass` must be called again. This is a state mutation surprise.

**Fix:** Preserve `mass` for static bodies; only zero `inverseMass` and `inverseInertia`:
```cpp
void UpdateMassProperties() {
    if (isStatic || isKinematic || mass <= 0.0f) {
        inverseMass = 0.0f;
        inverseInertia = 0.0f;
        // Do NOT zero mass — preserve it for when body becomes dynamic
    } else {
        inverseMass = 1.0f / mass;
        inverseInertia = (inertia > 0.0f) ? 1.0f / inertia : 0.0f;
    }
}
```

---

### 16.2 `inertia` and `inverseInertia` Default to 1.0 — Physically Wrong

**Problem:**
```cpp
float inertia = 1.0f;
float inverseInertia = 1.0f;
```
Default inertia of 1 for an arbitrary body is meaningless. Bodies created without explicitly setting inertia will have physically incorrect rotational responses. A box of mass 1 and size 32×32 should have `I = (1/12)*1*(32²+32²) ≈ 170`.

**Fix:** Default both to `0.0f`. Require that `ColliderComponent::CalculateInertiaForUnitDensity()` is called after body creation to populate the inertia from shape geometry. Add a convenience factory method to `PhysicsBodyComponent` or an `Initialize(ColliderComponent&)` method that auto-computes both mass and inertia.

---

### 16.3 `ApplyForceAtPoint` Uses `centerOfMass` Which Is Rarely Set

**Problem:**
```cpp
Math::Vector2 offset = point - centerOfMass;
torque += offset.x * forceVec.y - offset.y * forceVec.x;
```
`centerOfMass` defaults to `{0,0}`. Unless explicitly set, the torque computation is relative to the entity origin rather than the actual center of mass, giving wrong results for asymmetric shapes.

**Fix:** Compute `centerOfMass` from the collider shape centroid during body initialization (using `PolygonShape::centroid`).

---

### 16.4 `SLEEP_THRESHOLD` Is a Duration (0.5 seconds), But Is Compared Against a Speed Threshold

**Problem:**
```cpp
static constexpr float SLEEP_THRESHOLD = 0.5f;
...
if (body.sleepTimer >= body.SLEEP_THRESHOLD) {
    body.SetAwake(false);
```
`SLEEP_THRESHOLD` is used as a time-to-sleep duration here (correct). But in `IslandManager`:
```cpp
if (velocity > SLEEP_THRESHOLD) { belowThreshold = false; }
```
`SLEEP_THRESHOLD` is used as a velocity threshold. The two uses have incompatible units. This causes either sleep to never happen (if `0.5f` is too small as a velocity threshold) or bodies to sleep too quickly.

**Fix:** Rename and separate:
```cpp
static constexpr float TIME_TO_SLEEP = 0.5f;     // seconds
static constexpr float LINEAR_SLEEP_SPEED  = 0.05f; // units/s
static constexpr float ANGULAR_SLEEP_SPEED = 0.05f; // rad/s
```
Use `TIME_TO_SLEEP` for the timer comparison and `LINEAR_SLEEP_SPEED` for velocity thresholds.

---

## 17. ColliderComponent

### 17.1 Two Incompatible `ColliderComponent` Definitions in Same Namespace

**Problem:** Both `ColliderComponent.h` and `LegacyColliderComponent.h` define `struct ColliderComponent` in `namespace Nyon::ECS`. This is an **ODR (One Definition Rule) violation** that causes undefined behaviour at link time — even without `#include`-ing both files, if both translation units are linked together, the linker picks one definition arbitrarily.

**Fix:** Delete `LegacyColliderComponent.h` in full (covered also in §1.2), and ensure only one definition of `ColliderComponent` exists.

---

### 17.2 `CalculateAABB(position, outMin, outMax)` (No Rotation) Silently Passes `rotation=0`

**Problem:**
```cpp
void CalculateAABB(const Math::Vector2& position, Math::Vector2& outMin, Math::Vector2& outMax) const {
    CalculateAABB(position, 0.0f, outMin, outMax); // always uses rotation=0
}
```
This 2-argument overload is called from `PhysicsPipelineSystem::TestCollision` and `PhysicsPipelineSystem::GenerateManifold`. Both of those call sites have access to the entity's `TransformComponent::rotation` but don't pass it. As a result, the narrow-phase test and manifold generation for rotated shapes always compute the AABB at zero rotation, producing wrong overlap and wrong contact placement.

**Fix:** Remove the 2-argument overload entirely. All call sites must pass the rotation. Grep for `CalculateAABB(` and update all callers to pass `transform.rotation`.

---

### 17.3 `PolygonShape::CalculateProperties` Uses `signedArea` Without Halving for Centroid

**Problem:** The centroid formula used:
```cpp
float signedArea = 0.0f;
for (...) {
    float cross = vi.x * vj.y - vj.x * vi.y;
    signedArea += cross;  // This is 2 * area, not area
    centroid += (vi + vj) * cross;
}
centroid = centroid * (1.0f / (6.0f * (signedArea * 0.5f)));
```
`signedArea` accumulates the sum of cross products (not halved), so it equals `2 * area`. Then `signedArea * 0.5f` correctly recovers `area`. This happens to be correct, but is obfuscated. The inertia calculation in `CalculateInertiaForUnitDensity` (§17.4) doesn't follow the same pattern.

**Fix:** For clarity and correctness uniformity, rewrite as:
```cpp
float twoArea = 0.0f;
// ...
centroid *= 1.0f / (3.0f * twoArea); // direct formula
```

---

### 17.4 `CalculateInertiaForUnitDensity` for Polygon Uses Bounding-Box Approximation

**Problem:**
```cpp
case ShapeType::Polygon: {
    // Computes AABB of vertices, then uses (w² + h²) formula
    float w = max.x - min.x;
    float h = max.y - min.y;
    return area * (w * w + h * h) / 12.0f;
}
```
This is the formula for the inertia of a solid **rectangle**, not a general polygon. For any non-rectangular convex polygon (triangle, pentagon, etc.) this produces the wrong inertia.

**Fix:** Implement the correct polygon inertia using the shoelace-based formula:
```cpp
float CalculatePolygonInertia(float density) const {
    const auto& verts = GetPolygon().vertices;
    size_t n = verts.size();
    float numerator = 0.0f, denominator = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        const Math::Vector2& p0 = verts[i];
        const Math::Vector2& p1 = verts[(i + 1) % n];
        float cross = std::abs(Math::Vector2::Cross(p0, p1));
        numerator   += cross * (p0.x*p0.x + p0.x*p1.x + p1.x*p1.x
                              + p0.y*p0.y + p0.y*p1.y + p1.y*p1.y);
        denominator += cross;
    }
    float area = denominator * 0.5f;
    return density * area * numerator / (6.0f * denominator);
}
```

---

### 17.5 `CalculateInertiaForUnitDensity` for Circle Is Wrong

**Problem:**
```cpp
case ShapeType::Circle: {
    float r2 = circle.radius * circle.radius;
    return 0.5f * 3.14159f * r2 * r2; // = 0.5 * pi * r^4
}
```
This is missing a factor of `mass`. The moment of inertia of a solid disk is `I = 0.5 * mass * r²`. For unit density, `mass = pi * r²`, so `I = 0.5 * pi * r^4`. The formula is therefore numerically correct for unit density but only produces the right `inverseInertia` if divided by the actual mass during `UpdateMassProperties`. The code does not do this division — it stores the result directly as `inertia`.

**Fix:** The returned value must be the per-unit-mass inertia (`I/m = r²/2`), not the absolute inertia `I`:
```cpp
case ShapeType::Circle:
    return 0.5f * circle.radius * circle.radius; // I/m for solid disk
```
Then in the initialization:
```cpp
body.inertia = collider.CalculateInertiaForUnitDensity() * body.mass;
body.inverseInertia = (body.inertia > 0.0f) ? 1.0f / body.inertia : 0.0f;
```

---

## 18. PhysicsIntegrationSystem

### 18.1 Debug Log File Path Is Hardcoded to Developer's Local Machine

**Problem:**
```cpp
std::ofstream out("/home/daiyaan2002/Desktop/Projects/Nyon/.cursor/debug-b63f20.log", std::ios::app);
```
This is inside `IntegrateVelocity`, which runs every physics tick for every body. On any machine other than the developer's, this silently fails the `is_open()` check and does nothing — but it still constructs an `std::ofstream`, calls `time_point_cast`, and runs a `try/catch` on every single call.

**Fix:** Remove this entire logging block unconditionally. It is a developer debug artifact that should never have been committed. The surrounding `try/catch(...)` also swallows all exceptions silently, which is a separate code smell.

---

### 18.2 Gravity Is Disabled When Body Is "Stably Grounded"

**Problem:**
```cpp
if (!body.IsStablyGrounded()) {
    acceleration = gravity;
}
```
When a body is grounded (`groundedFrames >= 2`), gravity is not applied at all. This means any object that has been on the floor for 2+ frames has zero gravitational acceleration. If the floor beneath it disappears (e.g., in a platformer), the body will float until some other force or velocity causes `groundedFrames` to drop.

**Fix:** Always apply gravity. The constraint solver will prevent penetration and supply the reaction force. Suppressing gravity at the integration level breaks the contact constraint model — the normal force from the floor should cancel gravity, not the integration step:
```cpp
acceleration = gravity; // always
acceleration += body.force * body.inverseMass;
```

---

### 18.3 `transform.rotation` Is Wrapped With `6.28318530718f` Instead of `2*PI`

**Problem:**
```cpp
if (transform.rotation > 6.28318530718f)
    transform.rotation -= 6.28318530718f;
else if (transform.rotation < -6.28318530718f)
    transform.rotation += 6.28318530718f;
```
Rotation wrapping is unnecessary for a physics engine — angles can accumulate beyond 2π without issue in all standard formulas (sin/cos are periodic). Worse, the wrapping only handles one full revolution per frame. A body spinning at high angular velocity can jump from 6.0 to 7.0 radians in one step; the wrap subtracts one revolution, but if it spun 3 revolutions in a step (highly unlikely but possible) the clamping is wrong anyway.

**Fix:** Remove the rotation wrapping entirely. Use raw accumulation. If wrapping is needed for UI display, do it only in the debug renderer.

---

### 18.4 `ApplyBoundaryConstraints` Is Duplicated in Both `PhysicsIntegrationSystem` and `CollisionSystem`

**Problem:** The same hardcoded `1280×720` boundary logic with `PLAYER_SIZE = 32` appears in both `PhysicsIntegrationSystem::ApplyBoundaryConstraints` and `CollisionSystem::ApplyBoundaryConstraints`. Two independent boundary enforcement systems will fight each other.

**Fix:** Remove `ApplyBoundaryConstraints` from both systems. Use `BoundarySystem` exclusively and configure it from `PhysicsWorldComponent`.

---

### 18.5 `PhysicsIntegrationSystem` Includes `<fstream>` and `<chrono>` in Header

**Problem:**
```cpp
// PhysicsIntegrationSystem.h
#include <fstream>
#include <chrono>
```
These are pulled in transitively by every file that includes the system header. `<chrono>` is a heavy header.

**Fix:** Remove both includes from the header. They are only used by the debug logging block (§18.1) which should also be deleted.

---

## 19. `PhysicsPipelineSystem` Additional Issues

### 19.1 Gravity Hardcoded as `{0, 1000}` — Does Not Read From `PhysicsWorldComponent`

**Problem:**
```cpp
// VelocitySolving():
body.force += Math::Vector2{0.0f, 1000.0f};
```
This ignores `m_PhysicsWorld->gravity` entirely. The world gravity setting has no effect on the pipeline system.

**Fix:**
```cpp
body.force += m_PhysicsWorld->gravity * (1.0f / body.invMass); // F = mg
// or simply let IntegrateVelocities handle it:
body.velocity += m_PhysicsWorld->gravity * dt;
```

---

### 19.2 `GenerateManifold` Computes Separation as Sum of Half-Overlaps — Wrong Formula

**Problem:**
```cpp
point.separation = -(overlapMax.x - overlapMin.x) * 0.5f - (overlapMax.y - overlapMin.y) * 0.5f;
```
This sums half the X-overlap and half the Y-overlap. Penetration depth should be the minimum overlap on any axis (the SAT minimum penetration), not the average of all axes. This produces inflated penetration values and wrong correction magnitudes.

**Fix:** The `PhysicsPipelineSystem::GenerateManifold` is a stub AABB-centroid method. It must be replaced with a call to `ManifoldGenerator::GenerateManifold` (which already exists in the physics layer and handles all shape types properly).

---

### 19.3 `StoreImpulses` Decays Impulses by 0.9 Every Frame Without Reason

**Problem:**
```cpp
void PhysicsPipelineSystem::StoreImpulses() {
    for (auto& constraint : m_VelocityConstraints) {
        for (auto& point : constraint.points) {
            point.normalImpulse  *= 0.9f;
            point.tangentImpulse *= 0.9f;
        }
    }
}
```
Accumulated impulses for warm-starting should be preserved at full value (or scaled by `dtRatio`). Decaying them 10% per frame means after 10 frames the warm-start cache is essentially useless.

**Fix:** Remove the `*= 0.9f` scaling entirely. Store impulses directly. If scaling is desired when `dt` changes, use `scale = newDt / oldDt`.

---

### 19.4 `Integration()` Applies Additional 0.99 Damping After Solver Write-Back

**Problem:**
```cpp
void PhysicsPipelineSystem::Integration() {
    body.velocity      = solverBody.velocity;
    body.angularVelocity = solverBody.angularVelocity;
    body.velocity      *= 0.99f;  // extra damping
    body.angularVelocity *= 0.99f;
}
```
This adds yet another layer of frame-rate-dependent velocity damping on top of whatever the solver computes, on top of the damping in `ConstraintSolverSystem::IntegrateVelocities`. Three independent damping systems produce wildly unpredictable energy dissipation.

**Fix:** Remove all `*= 0.99f` or `*= 0.995f` raw multipliers from all systems. Use a single, properly formulated `linearDamping` coefficient applied once per frame in one place only (see §3.2).

---

### 19.5 `PositionConstraint::localCenterA/B` Declared as `float` Instead of `Vector2`

**Problem:**
```cpp
struct PositionConstraint {
    float localCenterA, localCenterB;  // BUG: should be Math::Vector2
    ...
};
```
A center of mass offset in 2D is a 2-component vector. Storing it as a single `float` means only one dimension (presumed X?) is captured. Any code using `localCenterA` for angular correction will be wrong.

**Fix:**
```cpp
struct PositionConstraint {
    Math::Vector2 localCenterA;
    Math::Vector2 localCenterB;
    ...
};
```

---

### 19.6 `UpdateShapeAABB` Passes `{0, 0}` as Body Velocity to `MoveProxy`

**Problem:**
```cpp
m_BroadPhaseTree.MoveProxy(it->second, fatAABB, {0.0f, 0.0f});
```
The `MoveProxy` displacement hint is always zero. The DynamicTree uses this displacement to predict the fat AABB expansion direction. Without it, the fat AABB is always symmetric, which is inefficient for fast-moving bodies (they exit the fat AABB every frame, causing constant tree mutations).

**Fix:** Pass the body's actual velocity-scaled displacement:
```cpp
if (m_ComponentStore->HasComponent<PhysicsBodyComponent>(entityId)) {
    const auto& body = m_ComponentStore->GetComponent<PhysicsBodyComponent>(entityId);
    Math::Vector2 displacement = body.velocity * FIXED_TIMESTEP;
    m_BroadPhaseTree.MoveProxy(it->second, fatAABB, displacement);
}
```

---

### 19.7 `BroadPhaseCallback::QueryCallback` Filters `otherEntityId >= entityId` — Wrong

**Problem:**
```cpp
if (otherEntityId >= entityId) return true;
```
This filters out the pair when `otherEntityId == entityId` (self-collision, correct) AND when `otherEntityId > entityId` (correct for dedup). But using `>=` means when `otherEntityId == entityId` AND when `otherEntityId > entityId`. The dedup intent is `>=` to avoid adding `(A,B)` when `(B,A)` will be found in A's query. However, this only works if entity IDs are dense and consistent. The correct idiomatic dedup check is simply `if (otherEntityId == entityId) return true;` and let the `ContactPair` hash handle deduplication, OR use strict `<` ordering: only add if `entityId < otherEntityId`.

**Fix:**
```cpp
// Only process pairs where entityId < otherEntityId (canonical ordering)
if (otherEntityId <= entityId) return true;
system->m_BroadPhasePairs.emplace_back(entityId, otherEntityId);
```

---

## 20. JointComponent

### 20.1 Joint Solver Is Completely Unimplemented

**Problem:** `JointComponent` defines `Distance`, `Revolute`, `Prismatic`, `Weld`, `Wheel`, and `Motor` joint types with full parameter structs. However, there is no `JointSolver` implementation anywhere in the codebase. `ConstraintSolverSystem` and `PhysicsPipelineSystem` have no code that reads `JointComponent` and generates joint constraints.

**Fix:** Implement a minimum viable `DistanceJoint` solver as the first joint type. A distance joint between two anchor points is a single constraint:
```
C = |worldAnchorB - worldAnchorA| - length = 0
```
The velocity constraint Jacobian and impulse resolution follow the standard formulation. Revolute and Prismatic joints follow from there. Until implemented, remove `JointComponent` from the public API or add a compile-time `static_assert(false, "JointComponent not implemented")` to prevent silent misuse.

---

### 20.2 `JointComponent` Constructor Computes `distanceJoint.length` from Anchors in Local Space

**Problem:**
```cpp
distanceJoint.length = (anchorB - anchorA).Length();
```
`anchorA` and `anchorB` are local-space anchors on each body. Their difference is not the world-space distance between them. For the initial rest length, the constructor should compute the world-space distance between the bodies' anchor points, which requires knowing the body positions at construction time.

**Fix:** Either document that `length` must be set explicitly by the caller after construction, or add a factory function that takes body positions:
```cpp
static JointComponent MakeDistance(uint32_t a, uint32_t b,
    const Math::Vector2& anchorA, const Math::Vector2& anchorB,
    const Math::Vector2& worldPosA, const Math::Vector2& worldPosB) {
    JointComponent j(Type::Distance, a, b, anchorA, anchorB);
    Math::Vector2 wA = worldPosA + anchorA;
    Math::Vector2 wB = worldPosB + anchorB;
    j.distanceJoint.length = (wB - wA).Length();
    return j;
}
```

---

## 21. `BoundarySystem`

### 21.1 `Boundary::GetPlanes()` Returns Signed Distances With Wrong Convention

**Problem:**
```cpp
planes[0] = { .normal = {1,0}, .point = minBounds, .distance = -minBounds.x };
planes[1] = { .normal = {-1,0}, .point = {maxBounds.x,...}, .distance = maxBounds.x };
```
For plane 0 (left wall, normal pointing right), the half-space equation should be `dot(normal, p) >= minBounds.x`, i.e., `distance = minBounds.x`. The stored value `distance = -minBounds.x` is the negation, which will produce inverted collision detection results.

**Fix:**
```cpp
planes[0] = { {1.f,0.f},  {minBounds.x, 0}, minBounds.x  }; // x >= minBounds.x
planes[1] = { {-1.f,0.f}, {maxBounds.x, 0}, -maxBounds.x }; // x <= maxBounds.x
planes[2] = { {0.f,1.f},  {0, minBounds.y}, minBounds.y  }; // y >= minBounds.y
planes[3] = { {0.f,-1.f}, {0, maxBounds.y}, -maxBounds.y }; // y <= maxBounds.y
```

---

## 22. `CollisionDetectionStrategy`

### 22.1 `GetApproximateSize` Returns Body Mass, Not Shape Size

**Problem:**
```cpp
static float GetApproximateSize(const ECS::PhysicsBodyComponent& body) {
    if (body.inverseMass > 0.0f) {
        return 1.0f / body.inverseMass; // this is just mass
    }
    return 50.0f;
}
```
This returns the body's mass, not its physical size. A heavy 1×1 pixel object and a light 1000×1000 meter object would return different "sizes" in ways that are physically reversed. A 1 kg small bullet and a 1 kg large box would appear the same size.

**Fix:** `GetApproximateSize` should not be on `PhysicsBodyComponent` at all — it should read from `ColliderComponent`. Change the signature:
```cpp
static float GetApproximateSize(const ECS::ColliderComponent& collider, const Math::Vector2& position) {
    Math::Vector2 min, max;
    collider.CalculateAABB(position, 0.0f, min, max);
    Math::Vector2 extents = (max - min) * 0.5f;
    return std::min(extents.x, extents.y); // smallest half-extent = tunnelling threshold
}
```

---

### 22.2 `CalculatePolygonTOI` Conservative Advancement Uses Total Displacement Speed, Not Approach Speed

**Problem:**
```cpp
float approachSpeed = (dPosA - dPosB).Length();
```
This is the magnitude of the *relative displacement vector*, which is the closing speed only when the bodies are moving directly toward each other. For bodies moving parallel or at an angle, this overestimates the approach speed, causing the TOI to converge faster than reality (tunnelling risk).

**Fix:** Project the relative velocity onto the separation axis:
```cpp
// Compute the separation axis (normalized line from posA to posB)
Math::Vector2 separationDir = (posB - posA);
float sepLen = separationDir.Length();
if (sepLen < 1e-6f) break;
separationDir = separationDir * (1.0f / sepLen);
// Approach speed is the component of relative velocity along the separation direction
float approachSpeed = Math::Vector2::Dot(dPosB - dPosA, separationDir);
if (approachSpeed <= 0.0f) break; // bodies diverging
```

---

## 23. `ContinuousCollisionDetection` (Physics Layer)

### 23.1 `CircleCircleCCD` Calls `ContinuousCollisionDetection::SolveQuadratic` Which Returns `false` When `a ≈ 0`

The quadratic `a = relVel.LengthSquared()` can be zero when relative velocity is zero. The code returns `false` immediately in that case. This is correct (no impact possible). But the `TOIResult.fraction` field is left at its default uninitialized value when `hit = false`. Callers must always check `result.hit` before using `result.fraction`.

**Fix:** Ensure `TOIResult` always zero-initializes `fraction` (it currently does via `float fraction;` with no initializer). Add `float fraction = 1.0f;` to the struct definition to make it safe to use without checking `hit`.

---

### 23.2 `CirclePolygonCCD` Ignores Polygon Rotation

**Problem:** `CirclePolygonCCD` takes polygon vertices and `polygonVelocity` but NOT a polygon position or orientation. The vertices are expected to be in world space. However, the method signature has `float polygonAngularVelocity` but the vertices passed are local-space. Angular velocity is ignored in the implementation (`B2_UNUSED(polygonAngularVelocity)` pattern). If the polygon is rotating, the swept volume is incorrect.

**Fix:** For an initial correct implementation, document that vertices must be pre-transformed to world space before calling, and note angular velocity is not handled. Long-term, use a conservative rotation sweep expansion (add the rotational motion contribution to the radius check).

---

## 24. `TransformPhysicsSyncSystem`

### 24.1 Stores Raw Pointers to Component Data — Invalidated on Component Resize

**Problem:**
```cpp
physicsBody = &componentStore.GetComponent<PhysicsBodyComponent>(entityId);
collider    = &componentStore.GetComponent<ColliderComponent>(entityId);
transform   = &componentStore.GetComponent<TransformComponent>(entityId);
```
These raw pointers are stored in `SyncEntity` and used every frame. But as shown in §2.1, `ComponentStore` stores components in `std::vector`. When new components are added (e.g., a new entity spawns), the vector may reallocate, invalidating ALL previously stored pointers. This is a dangling pointer bug.

**Fix:** Store entity IDs instead of raw pointers. Resolve the pointer each frame through `ComponentStore::GetComponent<T>(entityId)` (which after the §2.1 fix is O(1)):
```cpp
struct SyncEntity {
    uint32_t entityId;
    // No raw pointers
};
```

---

### 24.2 `m_DeltaTime = 0.016f` Hardcoded Default — Never Used Correctly

**Problem:** `m_DeltaTime` is set to a hardcoded `0.016f` as default, updated to `deltaTime` in `Update()`, but the value is never actually used in the sync logic. It is a dead field.

**Fix:** Remove `m_DeltaTime` and `m_InterpolationFactor` from the struct entirely.

---

## 25. `DebugRenderSystem`

### 25.1 `NyonDebugRenderer::DrawSolidCircle` Draws a Square, Not a Circle

**Problem:**
```cpp
void DebugRenderSystem::NyonDebugRenderer::DrawSolidCircle(...) {
    Math::Vector2 size(radius * 2.0f, radius * 2.0f);
    Math::Vector2 origin(radius, radius);
    Graphics::Renderer2D::DrawQuad(center, size, origin, color); // draws a quad!
}
```
This draws a square of size `2r × 2r` instead of a circle. All debug circle shapes (bodies, contact points, center of mass) will appear as squares.

**Fix:**
```cpp
Graphics::Renderer2D::DrawSolidCircle(center, radius, color, 16);
```

---

### 25.2 `DrawJoints`, `DrawContacts`, `DrawIslands` Are Empty Stubs

**Problem:** Three of the six `Draw*` methods are completely empty:
```cpp
void DebugRenderSystem::DrawJoints() {}
void DebugRenderSystem::DrawContacts() {}
void DebugRenderSystem::DrawIslands() {}
```
`m_DrawJoints = true` by default, so the system believes joints are being drawn but nothing happens.

**Fix:** Implement `DrawContacts` immediately — it is the most critical for debugging physics:
```cpp
void DebugRenderSystem::DrawContacts() {
    if (!m_PhysicsWorld) return;
    for (const auto& manifold : m_PhysicsWorld->contactManifolds) {
        if (!manifold.touching) continue;
        for (const auto& cp : manifold.points) {
            QueueCircle(cp.position, 3.0f, {1.f, 0.f, 0.f});
            QueueLine(cp.position, cp.position + manifold.normal * 10.f, {0.f, 1.f, 0.f});
        }
    }
}
```
Set `m_DrawJoints = false` until the joint solver is implemented.

---

### 25.3 `NyonDebugRenderer::QueueLine/QueueCircle/QueuePolygon/QueuePoint` Are All Empty

**Problem:** All four `Queue*` methods have empty bodies. The system queues commands via `DebugRenderSystem::QueueLine` (which works), but `NyonDebugRenderer::QueueLine` does nothing — the two are different code paths and the internal renderer's queue methods are dead code.

**Fix:** Remove the `NyonDebugRenderer` queue methods entirely. The outer `DebugRenderSystem::Queue*` methods (which push to `m_DebugCommands`) are the correct path.

---

## 26. `Camera2D`

### 26.1 Camera `GetViewMatrix` Applies Transformations in Wrong Order

**Problem:**
```cpp
view = glm::translate(view, -position);
view = glm::rotate(view, rotation, {0,0,1});
view = glm::scale(view, {zoom, zoom, 1});
```
GLM applies transformations right-to-left (column-major convention). The order should be: scale → rotate → translate. The current code applies translate first, meaning the translation is in un-scaled, un-rotated camera space — which is wrong for a rotating/zooming camera.

**Fix:** For a standard 2D camera where we want "world-to-camera" transform:
```cpp
glm::mat4 Camera2D::GetViewMatrix() const {
    glm::mat4 view = glm::mat4(1.0f);
    // TRS inverse: scale by zoom, rotate, then translate to -position
    view = glm::scale(view, glm::vec3(zoom, zoom, 1.0f));
    view = glm::rotate(view, -rotation, glm::vec3(0.0f, 0.0f, 1.0f));
    view = glm::translate(view, glm::vec3(-position.x, -position.y, 0.0f));
    return view;
}
```

---

### 26.2 `GetProjectionMatrix` Has `bottom` and `top` Swapped

**Problem:**
```cpp
float bottom = screenHeight / zoom;
float top = 0.0f;
return glm::ortho(left, right, bottom, top, nearPlane, farPlane);
```
`glm::ortho(left, right, bottom, top)` maps `bottom` to NDC -1 and `top` to NDC +1. With `bottom = screenHeight` and `top = 0`, the Y axis is flipped: world Y=0 is at screen top and Y=screenHeight is at screen bottom. This is intentional for screen-space coordinates but must be documented. If physics uses Y-up (gravity is typically negative Y), this will display everything upside down.

**Fix:** Define a coordinate system convention and enforce it. If Y is down in world space (gravity = `{0, 980}`), keep the current projection. If Y is up, swap:
```cpp
return glm::ortho(left, right, 0.0f, screenHeight/zoom, nearPlane, farPlane);
```
Document the chosen convention in a `COORDINATE_SYSTEM.md` and enforce it at `PhysicsWorldComponent::gravity` initialization.

---

### 26.3 `ScreenToWorld` and `WorldToScreen` Don't Account for Viewport Normalization

**Problem:**
```cpp
glm::vec4 worldPos = glm::inverse(GetViewProjectionMatrix(...)) * glm::vec4(screenPos.x, screenPos.y, 0, 1);
```
`screenPos` is in pixel coordinates [0, screenWidth] × [0, screenHeight]. The inverse VP matrix expects NDC coordinates [-1, 1]. The division into NDC space is missing:
```
ndcX = (2 * screenX / screenWidth) - 1
ndcY = 1 - (2 * screenY / screenHeight)  // or flip depending on convention
```
Without this, mouse picking and any world-space interaction from screen coordinates will be completely wrong.

**Fix:**
```cpp
Math::Vector2 Camera2D::ScreenToWorld(Math::Vector2 screenPos, float sw, float sh) const {
    float ndcX = (2.0f * screenPos.x / sw) - 1.0f;
    float ndcY = 1.0f - (2.0f * screenPos.y / sh);
    glm::vec4 world = glm::inverse(GetViewProjectionMatrix(sw, sh)) * glm::vec4(ndcX, ndcY, 0.0f, 1.0f);
    return {world.x, world.y};
}
```

---

## 27. Shader

### 27.1 Shader Uses GLSL `#version 460 core` — Incompatible With Many Systems

**Problem:** `#version 460 core` requires OpenGL 4.6. macOS supports only up to OpenGL 4.1. Any CI machine or developer laptop running macOS will fail to compile the shader.

**Fix:** Use `#version 330 core` as a baseline. All features used (basic attributes, uniforms, `fragColor`) are available in GLSL 3.30.

---

### 27.2 Fragment Shader Applies a Fake Lighting Model That Produces Incorrect Colors

**Problem:**
```glsl
if (length(v_Normal) > 0.001) {
    float lightIntensity = 0.8 + 0.2 * abs(v_Normal.y);
    color *= lightIntensity;
}
```
All vertices are submitted with `nx = ny = 0` (see the `Vertex` struct default constructor). `length(v_Normal)` is always `0`, so this branch never executes. It is dead shader code left from an LLM attempt at a lighting system.

**Fix:** Remove the lighting branch from the fragment shader. If lighting is desired in the future, it must be based on actual geometry normals provided per-vertex.

---

## 28. `ConstraintSolverSystem` (ECS-Layer) vs `ConstraintSolver` (Physics-Layer) Duplication

**Problem:** Both `ConstraintSolverSystem` (in `engine/src/ecs/systems/`) and `ConstraintSolver` (in `engine/src/physics/`) implement their own:
- `SolverBody` struct
- `VelocityConstraint` / `VelocityConstraintPoint` struct
- Warm-start, velocity solve, position solve logic

Additionally, `PhysicsPipelineSystem` has a *third* inline copy of the same structs and solver logic. There are therefore three independent solver implementations, none of which is the reference, and all of which have different bugs.

**Fix:** Delete `ConstraintSolverSystem` as a standalone ECS system. Delete the inline solver inside `PhysicsPipelineSystem`. Use `Physics::ConstraintSolver` as the single source of truth, called from `PhysicsPipelineSystem`. This eliminates two-thirds of the solver duplication and all the corresponding bugs.

---

## 29. `ECSApplication`

### 29.1 `OnFixedUpdate` Is `final` — Prevents Subclasses From Adding Fixed-Step Logic

**Problem:**
```cpp
void OnFixedUpdate(float deltaTime) override final;
```
`ECSApplication::OnFixedUpdate` runs ECS systems at the fixed step and is declared `final`. Any user application that extends `ECSApplication` cannot add their own fixed-step physics logic without overriding `OnECSUpdate` (which only runs the user-facing update, not at fixed step).

**Fix:** Provide a protected override point:
```cpp
void OnFixedUpdate(float deltaTime) override final {
    // Run ECS physics
    OnECSFixedUpdate(deltaTime); // new user hook
}
protected:
virtual void OnECSFixedUpdate(float deltaTime) {}
```

---

### 29.2 `OnInterpolateAndRender` Calls `GetDebugRenderSystem()` Which Calls `dynamic_cast` Every Frame

**Problem:** `GetDebugRenderSystem()` is called in the render loop and uses `dynamic_cast<DebugRenderSystem*>` via `SystemManager::GetSystem<T>`. This is a runtime type check every rendered frame.

**Fix:** Cache the `DebugRenderSystem*` pointer during `Initialize`, after the §14.2 fix is applied.

---

## 30. Updated Priority Table (Additional Issues)

| Priority | Section | Issue | Impact |
|---|---|---|---|
| P0 | 14.1 | `m_Systems` typo — does not compile | Compile failure |
| P0 | 15.4 | `DistanceSquared` missing — does not compile | Compile failure |
| P0 | 15.5 | `Rotation2D` missing — does not compile | Compile failure |
| P0 | 17.1 | Two `ColliderComponent` definitions — ODR violation | Undefined behavior |
| P0 | 18.1 | Debug log writes to hardcoded path every physics tick | Performance / crash on other machines |
| P0 | 19.1 | Gravity hardcoded as 1000, ignores world config | Wrong physics |
| P0 | 26.3 | `ScreenToWorld` skips NDC conversion | Mouse input completely broken |
| P1 | 15.1 | `Vector2(x,y,z)` silently drops z | Data corruption |
| P1 | 16.2 | Default inertia = 1.0 is physically wrong | Wrong rotational response |
| P1 | 17.4 | Polygon inertia uses rectangle formula | Wrong rotation for all non-box shapes |
| P1 | 17.5 | Circle inertia formula wrong (returns absolute I) | Wrong rotation for circles |
| P1 | 18.2 | Gravity disabled for grounded bodies | Objects float when floor removed |
| P1 | 19.2 | Manifold uses AABB centroid, not shape-accurate | Wrong contact normal/position |
| P1 | 19.3 | Warm-start cache decayed 10%/frame | Warm-start non-functional |
| P1 | 24.1 | Sync system stores raw component pointers | Dangling pointer / crash on spawn |
| P1 | 25.1 | Debug circle renders as square | Incorrect debug visualization |
| P1 | 26.1 | Camera view matrix transform order wrong | Incorrect world rendering |
| P1 | 26.2 | Projection Y-axis convention undocumented/possibly inverted | Upside-down world |
| P2 | 17.2 | `CalculateAABB` no-rotation overload ignores rotation | AABB wrong for rotated shapes in pipeline |
| P2 | 20.1 | Joint solver entirely unimplemented | All joint types silently fail |
| P2 | 22.1 | `GetApproximateSize` returns mass, not size | CCD strategy incorrect |
| P2 | 28 | Three duplicate solver implementations | Bug triplication |
| P2 | 27.1 | GLSL 4.6 — incompatible with macOS | Will not run on macOS |
| P3 | 15.2 | Unqualified `sqrt` in vector headers | Non-portable |
| P3 | 15.3 | `Normalize` not `[[nodiscard]]` | Silent dropped result |
| P3 | 18.3 | Rotation wrapping unnecessary | Harmless but wasteful |
| P3 | 21.1 | Boundary plane distance sign wrong | Wrong boundary reflection direction |
| P3 | 25.2 | DrawJoints/Contacts/Islands are empty stubs | Debug features silently do nothing |
| P3 | 27.2 | Dead lighting code in fragment shader | Dead code |

---

---

## 31. `Application.cpp` — Debug Noise Everywhere

### 31.1 [P3] Every Lifecycle Event Spams `std::cerr`

**Problem:** `Application::Init()`, `Application::Run()`, `Application::Close()`, `Application::~Application()` all contain unconditional `std::cerr << "[DEBUG] ..."` statements, including printing the raw `GLFWwindow*` pointer value. This fires in production builds and leaks internal state to stderr.

**Fix:** Wrap every `std::cerr << "[DEBUG]"` line in `#ifdef _DEBUG` / `#endif`, or replace with the `NYON_DEBUG_LOG` macro already defined in `ECSApplication.cpp`. Remove the pointer-printing statement (`m_Window << std::endl`) entirely:
```cpp
#ifdef _DEBUG
    std::cerr << "[DEBUG] GLFW window created successfully" << std::endl;
#endif
```

---

### 31.2 [P1] `Application::OnFixedUpdate` Default Implementation Calls `OnUpdate`

**Problem:**
```cpp
virtual void OnFixedUpdate(float deltaTime) { OnUpdate(deltaTime); }
```
The base `Application` class makes `OnFixedUpdate` silently alias `OnUpdate`. A subclass that overrides only `OnUpdate` (expecting it to be the variable-rate per-frame hook) will unintentionally run its `OnUpdate` code at fixed rate too, typically getting double-execution. The two hooks should have distinct, non-delegating defaults.

**Fix:**
```cpp
virtual void OnFixedUpdate(float deltaTime) {}  // no delegation
virtual void OnUpdate(float deltaTime) {}
```

---

## 32. `ECSApplication.cpp` — Render System Called Twice Per Frame

### 32.1 [P1] `OnInterpolateAndRender` Calls `renderSystem->Update(0.0f)` But `OnFixedUpdate` Already Ran All Systems

**Problem:**
```cpp
void ECSApplication::OnFixedUpdate(float deltaTime) {
    m_SystemManager.Update(deltaTime);   // runs ALL systems including RenderSystem
}
void ECSApplication::OnInterpolateAndRender(float alpha) {
    renderSystem->Update(0.0f);          // runs RenderSystem AGAIN
}
```
`SystemManager::Update` iterates all registered systems, including `RenderSystem`. Then `OnInterpolateAndRender` explicitly calls `RenderSystem::Update` a second time with `deltaTime = 0`. This means rendering runs twice per fixed step: once during physics, once during the render hook. All draw calls are issued twice each frame.

**Fix:** Either remove `RenderSystem` from the `SystemManager` update loop (only trigger it from `OnInterpolateAndRender`), or have `SystemManager::Update` skip systems tagged as render-only:
```cpp
// In ECSApplication::OnFixedUpdate — update only non-render systems
m_SystemManager.UpdatePhysicsSystems(deltaTime);

// In ECSApplication::OnInterpolateAndRender — update render systems only
renderSystem->SetInterpolationAlpha(alpha);
renderSystem->Update(0.0f);
```

---

### 32.2 [P2] `OnInterpolateAndRender` Calls `renderSystem->Update(0.0f)` Even When `m_ECSInitialized == false`

**Problem:**
```cpp
ECS::RenderSystem* renderSystem = m_SystemManager.GetSystem<ECS::RenderSystem>();
if (renderSystem) {
    renderSystem->SetInterpolationAlpha(alpha);
}
if (m_ECSInitialized) {
    renderSystem->Update(0.0f);   // m_ECSInitialized check doesn't guard GetSystem above
}
```
`GetSystem` is called unconditionally before the `m_ECSInitialized` guard. If `GetSystem` returns null (e.g., called before `OnStart`), the `renderSystem->Update` is skipped, but `SetInterpolationAlpha` is still called on the null pointer — undefined behaviour.

**Fix:**
```cpp
if (m_ECSInitialized) {
    ECS::RenderSystem* renderSystem = m_SystemManager.GetSystem<ECS::RenderSystem>();
    if (renderSystem) {
        renderSystem->SetInterpolationAlpha(alpha);
        renderSystem->Update(0.0f);
    }
}
```

---

## 33. `ComponentStore.h` — O(N) Linear Scan on Every Component Access

### 33.1 [P2] `GetComponent<T>`, `HasComponent<T>`, `RemoveComponent<T>` Are All O(N)

**Problem:** Every component lookup does a full linear scan of `entityIds`:
```cpp
for (size_t i = 0; i < container.entityIds.size(); ++i) {
    if (container.entityIds[i] == entity && container.activeFlags[i]) ...
}
```
For a scene with 1,000 bodies, each frame of the physics pipeline calls `GetComponent` dozens of times per entity per system, yielding O(N²) behavior in the solver hot path.

**Fix:** Add a `std::unordered_map<EntityID, size_t> entityIndex` inside `ComponentContainer<T>` that maps entity IDs to their slot index:
```cpp
template<typename T>
struct ComponentContainer : public IComponentContainer {
    std::vector<T> components;
    std::vector<EntityID> entityIds;
    std::vector<bool> activeFlags;
    std::unordered_map<EntityID, size_t> entityIndex; // NEW

    void AddComponent(EntityID entity, T&& component) {
        auto it = entityIndex.find(entity);
        if (it != entityIndex.end() && activeFlags[it->second]) {
            components[it->second] = std::forward<T>(component);
            return;
        }
        entityIndex[entity] = components.size();
        components.push_back(std::forward<T>(component));
        entityIds.push_back(entity);
        activeFlags.push_back(true);
    }
    T& GetComponent(EntityID entity) {
        return components[entityIndex.at(entity)];
    }
    ...
};
```
Note: `RemoveComponent` must also update the index map (swap-and-pop idiom is ideal here).

---

### 33.2 [P1] `GetComponent<T>` Returns a `static T dummy{}` on Not-Found Instead of Asserting

**Problem:**
```cpp
assert(false);
static T dummy{};
return dummy;   // returns a shared mutable default!
```
The fallthrough after `assert(false)` returns a `static` (single shared instance across all calls) default object. In release builds (assert disabled), all failed lookups return the same object, and all callers will silently mutate the same dummy instance. This is a silent data corruption bug in release.

**Fix:** Replace the unreachable dummy with `std::terminate()` or throw:
```cpp
// After the loop with no match:
std::terminate(); // or throw std::runtime_error("entity not found");
```

---

## 34. `CollisionPipelineSystem.cpp` — Contact Pair Key Bug

### 34.1 [P1] `ContactPair.shapeIdB` Is Set to `otherEntityId` Instead of `shapeId`

**Problem:**
```cpp
ContactPair pair{entityId, otherEntityId, shapeId, otherEntityId};  // BUG: last field
```
The fourth field of `ContactPair` is `shapeIdB` but is initialized with `otherEntityId`. This means all contact pair keys store the entity ID in the shape ID slot. When there are multiple shapes per entity (not yet implemented, but the data structure supports it), every contact will hash incorrectly and be treated as a unique new contact every frame, preventing warm-starting and triggering `beginContact` callbacks on every tick.

**Fix:**
```cpp
// In BroadPhaseCallback::QueryCallback:
uint32_t otherShapeId = otherEntityId; // placeholder until multi-shape support
ContactPair pair{entityId, otherEntityId, shapeId, otherShapeId};
```
More correctly, the broad phase should track per-shape proxy IDs separately from entity IDs. Until then, canonicalize: `shapeIdB = otherEntityId` consistently so the hash is at least deterministic.

---

### 34.2 [P2] `ProcessNarrowPhase` Also Uses `entityId` as `shapeId` in `BroadPhaseCallback`

**Problem:**
```cpp
callback.shapeId = entityIdA;  // entityId used as shapeId
```
Same issue as 34.1 but in `ProcessNarrowPhase`. This perpetuates the entity-as-shape-ID confusion throughout the contact map.

**Fix:** Define a proper `shapeId` per entity (even if just `entityId * 1 + 0` for the zeroth shape) and store it separately from the entity ID.

---

## 35. `ConstraintSolverSystem.cpp` — Gravity Never Applied

### 35.1 [P0] `IntegrateVelocities` Applies Damping But No Gravity

**Problem:**
```cpp
void ConstraintSolverSystem::IntegrateVelocities(float dt) {
    for (auto& body : m_SolverBodies) {
        if (body.isStatic || body.isKinematic) continue;
        body.velocity = body.velocity * 0.999f;         // damping only
        body.angularVelocity = body.angularVelocity * 0.995f;
        // GRAVITY IS NEVER ADDED
    }
}
```
The `ConstraintSolverSystem`'s `IntegrateVelocities` never applies gravity from `m_PhysicsWorld->gravity`. Gravity is only applied in the `Physics::ConstraintSolver::IntegrateVelocities` (the lower-level solver), but `ConstraintSolverSystem` does not use that solver — it uses its own inline integration. So in all ECS-path simulations, objects fall only due to the separate `PhysicsIntegrationSystem`, which also has gravity bugs (§18.2 of the original report), making gravity doubly broken.

**Fix:**
```cpp
void ConstraintSolverSystem::IntegrateVelocities(float dt) {
    const Math::Vector2 gravity = m_PhysicsWorld ? m_PhysicsWorld->gravity : Math::Vector2{0.0f, 9.81f};
    for (auto& body : m_SolverBodies) {
        if (body.isStatic || body.isKinematic) continue;
        if (body.invMass > 0.0f) {
            body.velocity = body.velocity + gravity * dt;
        }
        body.velocity = body.velocity * 0.999f;
        body.angularVelocity = body.angularVelocity * 0.995f;
    }
}
```

---

### 35.2 [P1] `ConstraintSolverSystem::SolvePositionConstraints` Uses Velocity Constraint Data for Position Correction

**Problem:** `SolvePositionConstraints` iterates over `m_VelocityConstraints` (not `m_PositionConstraints`) and uses `vcp.rA`/`vcp.rB` from velocity constraint points to compute position correction. The `m_PositionConstraints` vector (which is filled in `InitializeConstraints`) is completely unused. This means position correction is using velocity contact offsets, not the local contact reference points intended for position solving.

**Fix:** Iterate `m_PositionConstraints` in `SolvePositionConstraints` and use the `localPoints[j]` stored there:
```cpp
void ConstraintSolverSystem::SolvePositionConstraints() {
    for (int it = 0; it < m_PhysicsWorld->positionIterations; ++it) {
        for (auto& pc : m_PositionConstraints) {  // use PositionConstraints, not VelocityConstraints
            auto& bodyA = m_SolverBodies[pc.indexA];
            auto& bodyB = m_SolverBodies[pc.indexB];
            for (int j = 0; j < pc.pointCount; ++j) {
                Math::Vector2 rA = pc.localPoints[j] - (bodyA.position + bodyA.localCenter);
                Math::Vector2 rB = pc.localPoints[j] - (bodyB.position + bodyB.localCenter);
                // ... standard Baumgarte correction using rA, rB ...
            }
        }
    }
}
```

---

### 35.3 [P2] `ConstraintSolverSystem.cpp` Includes `<fstream>` and `<chrono>` Without Using Them

**Problem:** Both headers are included but not referenced anywhere in the translation unit. This is leftover from the debug logging block that was (partially) removed.

**Fix:** Remove both includes:
```cpp
// DELETE these two lines:
#include <fstream>
#include <chrono>
```

---

## 36. `Island.cpp` — `BuildContactGraph` Connects All Body Pairs Regardless of Actual Contact

### 36.1 [P0] `BuildContactGraph` Connects Every Non-Static Body to Every Other Non-Static Body

**Problem:**
```cpp
void IslandManager::BuildContactGraph(const std::vector<ECS::EntityID>& activeEntities) {
    for (size_t i = 0; i < activeEntities.size(); ++i) {
        ...
        for (size_t j = i + 1; j < activeEntities.size(); ++j) {
            ...
            m_ContactGraph[activeEntities[i]].push_back(activeEntities[j]);
            m_ContactGraph[activeEntities[j]].push_back(activeEntities[i]);
        }
    }
}
```
This is a complete graph — it adds edges between **every** pair of non-static dynamic bodies, regardless of whether they are actually touching. As a result, `FloodFill` puts all dynamic bodies into a single island. No body ever falls asleep independently because there will always be "connected" bodies keeping it awake.

**Fix:** Build the contact graph from actual contact manifolds, not from the entity list:
```cpp
void IslandManager::BuildContactGraph(const std::vector<ECS::EntityID>& activeEntities) {
    m_ContactGraph.clear();
    // Read actual contacts from ComponentStore (PhysicsWorldComponent::contactManifolds)
    if (!m_ComponentStore.HasComponent<ECS::PhysicsWorldComponent>(/* world entity */))
        return;
    const auto& manifolds = /* get contactManifolds */;
    for (const auto& manifold : manifolds) {
        if (!manifold.touching) continue;
        m_ContactGraph[manifold.entityIdA].push_back(manifold.entityIdB);
        m_ContactGraph[manifold.entityIdB].push_back(manifold.entityIdA);
    }
}
```
This is one of the most critical correctness bugs in the entire engine because it breaks sleep, performance, and energy conservation for all multi-body scenes.

---

### 36.2 [P1] `FindIslands` Copies `m_AllIslands` Into `m_AwakeIslands` / `m_SleepingIslands` By `std::move` But Leaves `m_AllIslands` In Valid-But-Empty State

**Problem:**
```cpp
for (auto& island : m_AllIslands) {
    if (island.isAwake) {
        m_AwakeIslands.push_back(std::move(island));  // island is now empty
    }
}
// m_AllIslands still has the same number of elements — just all moved-from (empty)
```
After the loop, `m_AllIslands` contains moved-from `Island` objects with empty `bodyIds`. If any code later uses `m_AllIslands` (e.g., `WakeIslandContaining` which indexes into it), the indices in `m_BodyIslandMap` now point to empty-but-valid objects, not the awake/sleeping copies. `WakeIslandContaining` will then set `isAwake = true` on an empty shell rather than the real island.

**Fix:** Either use indices into a single authoritative list (don't maintain three lists), or clear `m_AllIslands` after the partition and accept that `m_BodyIslandMap` must reference into `m_AwakeIslands`/`m_SleepingIslands` by a unified ID.

---

## 37. `ManifoldGenerator.cpp` — Multiple Issues

### 37.1 [P0] `CapsuleCollision` and `SegmentCollision` Are Empty Stubs — Always Return `touching = false`

**Problem:**
```cpp
ContactManifold ManifoldGenerator::CapsuleCollision(...) {
    manifold.touching = false;
    ...
    return manifold;  // always false; capsule collisions never detected
}
ContactManifold ManifoldGenerator::SegmentCollision(...) {
    manifold.touching = false;
    ...
    return manifold;  // same
}
```
Any entity with a `CapsuleShape` or `SegmentShape` collider will never generate contact manifolds. They will fall through all other objects silently. The `SATCollisionDetector::DetectCapsuleCollision` implementation exists and is correct — `ManifoldGenerator` just doesn't delegate to it.

**Fix:** Delegate to `SATCollisionDetector` (which is already the correct implementation path for other shapes):
```cpp
ContactManifold ManifoldGenerator::CapsuleCollision(...) {
    SATCollisionDetector detector;
    const auto& capsuleA = colliderA.GetCapsule();
    // build variant for other shape
    std::variant<...> otherShape;
    if (colliderB.GetType() == ColliderComponent::ShapeType::Circle)
        otherShape = colliderB.GetCircle();
    else if (colliderB.GetType() == ColliderComponent::ShapeType::Polygon)
        otherShape = colliderB.GetPolygon();
    else
        otherShape = colliderB.GetCapsule();
    return detector.DetectCapsuleCollision(entityIdA, entityIdB, capsuleA, otherShape, transformA, transformB);
}
```

---

### 37.2 [P1] `PolygonPolygon` in `ManifoldGenerator` Only Generates One Contact Point

**Problem:** The `ManifoldGenerator::PolygonPolygon` implementation finds the MTD axis via SAT, picks support points from each polygon, and averages them into a single contact:
```cpp
Math::Vector2 contactPoint = (supportA + supportB) * 0.5f;
manifold.points.push_back(cp);  // only one point
```
A stable stacking configuration typically needs 2 contact points (both corners of the resting edge). With a single averaged contact point, objects will wobble and spin instead of resting stably. The `SATCollisionDetector::DetectPolygonPolygon` already implements proper Sutherland-Hodgman clipping to generate up to 2 correct contact points.

**Fix:** Replace `ManifoldGenerator::PolygonPolygon`'s body with a delegation to `SATCollisionDetector::DetectPolygonPolygon`:
```cpp
ContactManifold ManifoldGenerator::PolygonPolygon(..., ContactManifold& manifold) {
    SATCollisionDetector detector;
    return detector.DetectPolygonPolygon(
        entityIdA, entityIdB, shapeIdA, shapeIdB,
        polyA, polyB, transformA, transformB);
}
```

---

### 37.3 [P2] `ManifoldGenerator.cpp` Includes `<fstream>` and `<chrono>` Without Using Them

Same leftover debug include issue as §35.3. **Fix:** Remove both.

---

### 37.4 [P1] `GenerateManifold` Dispatch Priority: Capsule Check Is Last — Falls Through If One Side Is Circle/Polygon

**Problem:**
```cpp
if (Circle × Circle) ...
if (Circle × Polygon) ...
if (Polygon × Circle) ...
if (Polygon × Polygon) ...
if (Capsule on either side) ...  // only reached if NOT circle or polygon
if (Segment on either side) ...
```
A `Capsule × Circle` pair satisfies `colliderA.GetType() == Circle` first and falls into `CirclePolygon` (which then crashes or produces garbage because the polygon `GetPolygon()` will return a default/empty polygon for a circle collider). Capsule/Segment checks must come first.

**Fix:** Reorder dispatch: check for Capsule/Segment *before* the circle/polygon pairs, and handle all hybrid combinations explicitly:
```cpp
if (capsule on either side) → CapsuleCollision(...)
if (segment on either side) → SegmentCollision(...)
if (circle × circle) → ...
if (circle × polygon or polygon × circle) → ...
if (polygon × polygon) → ...
```

---

## 38. `SATCollisionDetector.cpp` — Undefined Macro

### 38.1 [P0] `B2_UNUSED` Macro Is Undefined

**Problem:**
```cpp
B2_UNUSED(vertsA);          // in FindIncidentFace
B2_UNUSED(clipEdgeIndex);   // in ClipSegmentToLine
B2_UNUSED(angularVelocityA); // in CCD::ComputeTOI
B2_UNUSED(angularVelocityB);
```
`B2_UNUSED` is a Box2D macro (`(void)(x)`) that is not defined anywhere in the Nyon codebase. This will cause a compilation error ("identifier 'B2_UNUSED' is undefined") on any clean build.

**Fix:** Add the macro definition to a shared internal header (e.g., `nyon/core/Macros.h`) and include it:
```cpp
#define NYON_UNUSED(x) (void)(x)
```
Then replace all `B2_UNUSED(x)` calls with `NYON_UNUSED(x)`.

---

### 38.2 [P1] `SATCollisionDetector::ComputePolygonWorld` Takes `PolygonShape` by Value Via `polygon.size()` / `polygon[i]` — Direct Subscript on `struct`

**Problem:**
```cpp
for (size_t i = 0; i < polygon.size(); ++i) {
    const auto& v = polygon[i];
```
`ColliderComponent::PolygonShape` is a struct with a `std::vector<Math::Vector2> vertices` member. The struct itself does not have `size()` or `operator[]`. This will fail to compile unless `PolygonShape` implicitly converts to `vector<Vector2>` or has those operators.

**Fix:** Access the `vertices` member explicitly:
```cpp
for (size_t i = 0; i < polygon.vertices.size(); ++i) {
    const auto& v = polygon.vertices[i];
```

---

## 39. `PhysicsPipeline.cpp` — Multiple Issues

### 39.1 [P0] `UpdateDynamicTree` Calls `componentStore.GetComponent<T>(entityId)` Returning a Raw Pointer — API Mismatch

**Problem:**
```cpp
auto* transform = componentStore.GetComponent<ECS::TransformComponent>(entityId);
if (!transform) return;
```
`ComponentStore::GetComponent<T>` returns a `T&` reference, not a `T*` pointer. Assigning it to `auto*` will fail to compile, and the null-check `if (!transform)` is meaningless for a reference.

**Fix:** Use a reference and check with `HasComponent` first, or change the API to return `T*`:
```cpp
if (!componentStore.HasComponent<ECS::TransformComponent>(entityId)) return;
auto& transform = componentStore.GetComponent<ECS::TransformComponent>(entityId);
```
Alternatively, add a `TryGetComponent<T>` method returning `T*` (nullptr on missing) to `ComponentStore`.

---

### 39.2 [P1] `UpdateDynamicTree` Computes AABB Using `transform->rotation * vertex` But `float * Vector2` Is Undefined

**Problem:**
```cpp
Math::Vector2 worldVertex = transform->position + transform->rotation * (vertex * transform->scale);
```
`transform->rotation` is a `float` (angle in radians). Multiplying a `float` by a `Math::Vector2` is only valid if `operator*(float, Vector2)` is defined, which it likely is via scalar-multiply overload. However, it does not apply the rotation — it scales the vertex by the angle value (a float), not by a rotation matrix. This is a physics-breaking bug: polygon AABBs are not axis-aligned with respect to the body's actual rotation.

**Fix:**
```cpp
float cosA = std::cos(transform->rotation);
float sinA = std::sin(transform->rotation);
Math::Vector2 scaled = vertex * transform->scale;
Math::Vector2 rotated{
    scaled.x * cosA - scaled.y * sinA,
    scaled.x * sinA + scaled.y * cosA
};
Math::Vector2 worldVertex = transform->position + rotated;
```

---

### 39.3 [P0] `SolveConstraints` Calls `componentStore.GetComponents<ECS::TransformComponent>()` — Method Does Not Exist

**Problem:**
```cpp
auto persistentContacts = m_StabilizationSystem.UpdatePersistentContacts(
    validContacts,
    *componentStore.GetComponents<ECS::TransformComponent>());  // DOES NOT EXIST
```
`ComponentStore` has no `GetComponents<T>()` method (which would presumably return a pointer to the internal vector). This will not compile.

**Fix:** Build a `std::vector<TransformComponent>` indexed by entity ID before calling `UpdatePersistentContacts`, or change `UpdatePersistentContacts` to take a `ComponentStore&` reference:
```cpp
std::vector<ECS::TransformComponent> transforms;
// collect transforms in order matching solverBodies...
auto persistentContacts = m_StabilizationSystem.UpdatePersistentContacts(validContacts, transforms);
```

---

### 39.4 [P0] `WriteBackToComponents` Assigns `Math::Rotation2D(transform->rotation)` to `transform->rotation` (a `float`)

**Problem:**
```cpp
transform->rotation = solverBody.angle;
transform->rotation = Math::Rotation2D(transform->rotation);  // Rotation2D → float ???
```
`transform->rotation` is a `float`. `Math::Rotation2D` is a struct (confirmed by its use in `StabilizationSystem`). Assigning a struct to a float is a compile error (or, if `Rotation2D` has an implicit float conversion, a silent precision loss). This line appears to be a copy-paste artifact from `StabilizationSystem::ApplySplitImpulses`.

**Fix:** Remove the second assignment entirely:
```cpp
transform->rotation = solverBody.angle;  // just this line
```

---

### 39.5 [P1] `SolveConstraints` Iterates `m_Contacts` (NarrowPhaseContact) But Builds `validContacts` From a Redundant Local Filter

**Problem:**
```cpp
m_FinalContacts.insert(m_FinalContacts.end(), m_Contacts.begin(), m_Contacts.end());
std::vector<ContactManifold> validContacts;
for (const auto& contact : m_Contacts) {
    if (contact.toi <= 1.0f && contact.manifold.touching) {
        validContacts.push_back(contact.manifold);
    }
}
```
`m_FinalContacts` is used for sleep waking in `ApplyStabilization`, but `validContacts` is a subset of `m_Contacts`. The duplicate append into `m_FinalContacts` means all contacts (valid or not) become boundary+narrow combined, making `ApplyStabilization`'s wake logic wake bodies from stale/invalid contacts.

**Fix:** Only insert `validContacts` (manifolds with `toi ≤ 1` and `touching = true`) into `m_FinalContacts`, not all of `m_Contacts`.

---

### 39.6 [P2] `BroadPhaseQueryCallback::QueryCallback` Never Calls `AddOverlap` — Pairs Are Never Generated

**Problem:**
```cpp
bool BroadPhaseQueryCallback::QueryCallback(uint32_t nodeIdA, uint32_t userDataA) {
    m_CurrentIdA = userDataA;
    m_NodeIdA = nodeIdA;
    return true;  // just stores the current ID — never adds a pair!
}
```
`AddOverlap` is a separate method that adds pairs but is never called from the tree query path. The `DynamicTree::Query` callback interface calls `QueryCallback` for every overlapping node, but this implementation only saves the last one. No broad phase pairs are ever generated.

**Fix:** Add pair generation inside `QueryCallback`:
```cpp
bool QueryCallback(uint32_t nodeId, uint32_t userData) {
    if (userData <= m_CurrentIdA) return true; // dedup
    BroadPhasePair pair;
    pair.entityIdA = m_CurrentIdA;
    pair.entityIdB = userData;
    pair.shapeIdA = 0;
    pair.shapeIdB = 0;
    pair.aabbA = m_Tree.GetFatAABB(m_NodeIdA);
    pair.aabbB = m_Tree.GetFatAABB(nodeId);
    m_Pairs.push_back(pair);
    return true;
}
```

---

## 40. `StabilizationSystem.cpp` — Multiple Issues

### 40.1 [P0] `UpdatePersistentContacts` Calls `transformA.rotation.Inverse()` But `TransformComponent::rotation` Is a `float`

**Problem:**
```cpp
Math::Vector2 localPointA = transformA.rotation.Inverse() * (contactPoint.position - transformA.position);
```
`TransformComponent::rotation` is a `float` (angle in radians). Floats don't have `.Inverse()` or `operator*(Vector2)`. This will not compile.

**Fix:**
```cpp
float invRotA = -transformA.rotation;  // inverse of rotation angle
float cosInv = std::cos(invRotA), sinInv = std::sin(invRotA);
Math::Vector2 d = contactPoint.position - transformA.position;
Math::Vector2 localPointA = { d.x * cosInv - d.y * sinInv, d.x * sinInv + d.y * cosInv };
```

---

### 40.2 [P0] `UpdatePersistentContacts` Calls `FindMatchingContact` With 8 Arguments But `StabilizationSystem::FindMatchingContact` (Private) Takes Only 5

**Problem:**
```cpp
// Call site (8 args):
PersistentContact* cachedContact = FindMatchingContact(
    m_PersistentContacts,
    manifold.entityIdA, manifold.entityIdB,
    manifold.shapeIdA, manifold.shapeIdB,
    featureId, localPointA, localPointB, localPointTolerance);

// Declaration (header, 5 args):
int FindMatchingContact(uint32_t entityIdA, uint32_t entityIdB,
                        uint32_t featureId, const Math::Vector2& localPoint) const;
```
The call site passes 9 arguments (including `m_PersistentContacts`), the header declares 5, and the `.cpp` definition takes 8. None of these match each other. This will not compile and represents a deeply inconsistent interface.

**Fix:** Align the signature across header and implementation:
```cpp
// Consistent signature:
PersistentContact* FindMatchingContact(
    uint32_t entityIdA, uint32_t entityIdB,
    uint32_t shapeIdA, uint32_t shapeIdB,
    uint32_t featureId,
    const Math::Vector2& localPointA,
    const Math::Vector2& localPointB,
    float tolerance);
// Remove the contacts vector param — use m_PersistentContacts directly.
```

---

### 40.3 [P0] `UpdatePersistentContacts` Calls `ComputeFeatureId(contactPoint, manifold.normal)` But `ComputeFeatureId` Is Not Declared in the Header

**Problem:** `ComputeFeatureId` is defined in `StabilizationSystem.cpp` but does not appear in `StabilizationSystem.h`. This is a missing declaration → compile error in any TU that calls it.

**Fix:** Add to `StabilizationSystem.h`:
```cpp
private:
    static uint32_t ComputeFeatureId(const ContactPoint& cp, const Math::Vector2& normal);
```

---

### 40.4 [P0] `ApplyWarmStarting` Compares Entity IDs Against Solver Body Indices

**Problem:**
```cpp
if ((pc.entityIdA == vc.indexA && pc.entityIdB == vc.indexB) || ...)
```
`pc.entityIdA` is an entity ID (e.g., entity 42). `vc.indexA` is a solver body index (e.g., index 3). These are entirely different number spaces. The comparison will virtually never match (unless by coincidence), so warm-starting via `StabilizationSystem` is completely broken.

**Fix:** Match by entity ID: `vc` must store entity IDs, not just indices. Add `uint32_t entityIdA, entityIdB` to `VelocityConstraint` and populate them in `InitializeVelocityConstraints`:
```cpp
vc.entityIdA = manifold.entityIdA;
vc.entityIdB = manifold.entityIdB;
```
Then in `ApplyWarmStarting`:
```cpp
if ((pc.entityIdA == vc.entityIdA && pc.entityIdB == vc.entityIdB) || ...)
```

---

### 40.5 [P0] `ApplyWarmStarting` Uses Undefined Constants `kInvalidBodyIndex`

**Problem:**
```cpp
if (vc.indexA != kInvalidBodyIndex) { ...
if (vc.indexB != kInvalidBodyIndex) { ...
```
`kInvalidBodyIndex` is not defined anywhere in the Nyon codebase. This will not compile.

**Fix:** Replace with `UINT32_MAX` (consistent with the `ConstraintSolver.cpp` convention) or define:
```cpp
static constexpr uint32_t kInvalidBodyIndex = UINT32_MAX;
```
in a shared `PhysicsConstants.h`.

---

### 40.6 [P1] `ApplyWarmStarting` Applies Impulses Directly to `solverBodies[vc.indexA].linearVelocity` But `solverBodies` Is Passed by `const` Reference

**Problem:**
```cpp
void StabilizationSystem::ApplyWarmStarting(
    const std::vector<PersistentContact>& persistentContacts,
    const std::vector<SolverBody>& solverBodies,   // const!
    std::vector<VelocityConstraint>& velocityConstraints,
    float dtRatio) {
    ...
    solverBodies[vc.indexA].linearVelocity += P * invMassA;  // writes to const!
```
`solverBodies` is `const` but the function writes to it. This does not compile.

**Fix:** Change the parameter to non-const:
```cpp
void ApplyWarmStarting(
    const std::vector<PersistentContact>& persistentContacts,
    std::vector<SolverBody>& solverBodies,   // non-const
    std::vector<VelocityConstraint>& velocityConstraints,
    float dtRatio);
```

---

### 40.7 [P1] `ApplySplitImpulses` Uses `pc.normal` and `pc.rA`/`pc.rB` But `PositionConstraint` Has No Such Fields

**Problem:**
```cpp
Math::Vector2 P = pc.normal * positionImpulse;
solverBodies[pc.indexA].angle -= Math::Vector2::Cross(pc.rA, P) * invInertiaA;
```
`Physics::PositionConstraint` has fields `localPointA`, `localPointB`, `localNormal`, `separation`, `minSeparation` — it has no `normal` or `rA`/`rB` fields. This does not compile.

**Fix:** Use the correct field names:
```cpp
Math::Vector2 P = pc.localNormal * positionImpulse;
Math::Vector2 rA = pc.localPointA;  // approximate
solverBodies[pc.indexA].angle -= Math::Vector2::Cross(rA, P) * invInertiaA;
```

---

### 40.8 [P0] `ApplySplitImpulses` Assigns `Math::Rotation2D(angle)` to `SolverBody::rotation` But `SolverBody` Has No `rotation` Field

**Problem:**
```cpp
solverBodies[pc.indexA].rotation = Math::Rotation2D(solverBodies[pc.indexA].angle);
```
`Physics::SolverBody` has no `rotation` field — it has `angle` (float), `deltaPosition`, `deltaRotation`. This does not compile.

**Fix:** Remove the line entirely; the angle integration is handled elsewhere. If a `Rotation2D` cache is desired, it must first be added to `SolverBody`.

---

## 41. `ConstraintSolver.cpp` — Friction Hardcoded

### 41.1 [P1] `InitializeVelocityConstraints` Hardcodes `friction = 0.3f` and `restitution = 0.0f`

**Problem:**
```cpp
vc.friction = 0.3f;      // ignores manifold.friction
vc.restitution = 0.0f;   // ignores manifold.restitution
```
The `ContactManifold` passed in has `friction` and `restitution` fields already computed by `CollisionPipelineSystem::UpdateContacts` (geometric mean of the two bodies' materials). These are ignored; every contact uses 0.3 friction and 0.0 restitution regardless of the materials set on the `PhysicsBodyComponent`.

**Fix:**
```cpp
vc.friction = manifold.friction;
vc.restitution = manifold.restitution;
```
And apply restitution in the velocity bias:
```cpp
float relativeVelocityAlongNormal = ...; // vn at constraint setup
vcp.bias = -vc.restitution * std::min(relativeVelocityAlongNormal, 0.0f);
```

---

## 42. `InputManager.cpp` — `IsKeyUp` Returns `true` When Window Is Null

### 42.1 [P3] `IsKeyUp` with Null Window Returns `true` — Misleading Default

**Problem:**
```cpp
bool InputManager::IsKeyUp(int key) {
    if (key < 0 || key >= GLFW_KEY_LAST) return false;
    if (s_Window == nullptr) return true;  // "key is up" when no window?
    return !s_CurrentKeys[key];
}
```
When no window is initialized, `IsKeyUp` returns `true` for all keys (treating the absence of a window as "all keys released"). This is inconsistent with `IsKeyDown` (returns `false`) and `IsKeyPressed` (returns `false`) in the same null-window case. Code that checks `IsKeyUp(SPACE)` during loading before the window exists will falsely believe the key was just released.

**Fix:** Return `false` consistently for all key query methods when `s_Window == nullptr`:
```cpp
if (s_Window == nullptr) return false;
```

---

### 42.2 [P2] `InputManager::Update` Only Copies State When `s_InputDirty` — Frame Coherence Broken

**Problem:**
```cpp
void InputManager::Update() {
    if (s_InputDirty) {
        memcpy(s_PreviousKeys, s_CurrentKeys, sizeof(s_CurrentKeys));
        s_InputDirty = false;
    }
}
```
`s_InputDirty` is set to `true` only when a key event fires during that frame. If no key events fire (a common case), `s_PreviousKeys` is never updated. After 2+ frames of no input, `s_PreviousKeys` still reflects the state from the last event. `IsKeyPressed` (`current && !previous`) will report a key as still "just pressed" across many frames — a stuck-key ghost input.

**Fix:** Always copy state in `Update()`, regardless of `s_InputDirty`:
```cpp
void InputManager::Update() {
    if (s_Window == nullptr) return;
    memcpy(s_PreviousKeys, s_CurrentKeys, sizeof(s_CurrentKeys));
    memcpy(s_PreviousMouseButtons, s_CurrentMouseButtons, sizeof(s_CurrentMouseButtons));
    // s_InputDirty can be removed entirely
}
```

---

## 43. `DynamicTree.cpp` — `Rebuild` Is O(N²) Free-List Walk

### 43.1 [P3] `Rebuild` Detects Free Nodes With an O(N×M) Nested Walk

**Problem:**
```cpp
void DynamicTree::Rebuild(bool fullRebuild) {
    for (uint32_t i = 0; i < m_nodes.size(); ++i) {
        bool isFreeNode = false;
        uint32_t freeIndex = m_freeList;
        while (freeIndex != TreeNode::NULL_NODE) {  // walk entire free list for every node
            if (freeIndex == i) { isFreeNode = true; break; }
            freeIndex = m_nodes[freeIndex].parent;
        }
        ...
    }
}
```
For N nodes and M free nodes, this is O(N×M). For a busy scene it can be O(N²).

**Fix:** Build a `std::unordered_set<uint32_t>` of free nodes once before the loop:
```cpp
std::unordered_set<uint32_t> freeSet;
uint32_t f = m_freeList;
while (f != TreeNode::NULL_NODE) { freeSet.insert(f); f = m_nodes[f].parent; }
for (uint32_t i = 0; i < m_nodes.size(); ++i) {
    if (freeSet.count(i) == 0 && m_nodes[i].IsLeaf()) proxies.push_back(i);
}
```

---

## 44. `ConstraintSolver.cpp` — `SolveVelocityConstraint` Computes Bias Inside Solver Loop

### 44.1 [P2] Baumgarte Bias Recomputed Every Iteration

**Problem:**
```cpp
void ConstraintSolver::SolveVelocityConstraint(VelocityConstraint& vc) {
    for (auto& vcp : vc.points) {
        float C = std::max(vcp.separation + m_BaumgarteSlop, 0.0f);
        vcp.bias = -m_BaumgarteBeta * C / m_TimeStep;  // recomputed every iteration
        float lambda = -vcp.normalMass * (vn + vcp.bias);
```
The Baumgarte bias only depends on the initial separation, not on velocity changes during iteration. Recomputing it inside the velocity iteration loop artificially increases the position correction per iteration, leading to energy injection (jitter). The bias should be computed once during `InitializeVelocityConstraints` and stored, not recomputed each iteration.

**Fix:** Move bias computation to `InitializeVelocityConstraints`:
```cpp
// During initialization:
float C = std::max(cp.separation + m_BaumgarteSlop, 0.0f);
vcp.bias = -m_BaumgarteBeta * C / m_TimeStep;

// In SolveVelocityConstraint — use stored bias, don't recompute:
float lambda = -vcp.normalMass * (vn + vcp.bias);
```

---

## 45. Complete Updated Priority Table (All Newly Found Issues)

| Priority | Section | Issue |
|---|---|---|
| P0 | 31.2 | `OnFixedUpdate` default delegates to `OnUpdate` — double execution |
| P0 | 35.1 | `ConstraintSolverSystem::IntegrateVelocities` never applies gravity |
| P0 | 36.1 | `BuildContactGraph` connects all bodies — all form one island |
| P0 | 37.1 | `CapsuleCollision` and `SegmentCollision` are empty stubs |
| P0 | 38.1 | `B2_UNUSED` macro undefined — compilation error |
| P0 | 38.2 | `polygon.size()` / `polygon[i]` — invalid subscript on struct |
| P0 | 39.1 | `GetComponent` returns `T&` but is assigned to `T*` — compile error |
| P0 | 39.3 | `GetComponents<T>()` does not exist — compile error |
| P0 | 39.4 | `Rotation2D` assigned to `float` — type mismatch compile error |
| P0 | 39.6 | `BroadPhaseQueryCallback::QueryCallback` never generates pairs |
| P0 | 40.1 | `rotation.Inverse()` called on `float` — compile error |
| P0 | 40.2 | `FindMatchingContact` called with wrong argument count — compile error |
| P0 | 40.3 | `ComputeFeatureId` not declared — compile error |
| P0 | 40.5 | `kInvalidBodyIndex` undefined — compile error |
| P0 | 40.7 | `PositionConstraint` has no `normal`/`rA`/`rB` fields — compile error |
| P0 | 40.8 | `SolverBody` has no `rotation` field — compile error |
| P1 | 32.1 | `RenderSystem::Update` called twice per frame |
| P1 | 32.2 | Null pointer dereference in `OnInterpolateAndRender` before guard |
| P1 | 34.1 | `ContactPair.shapeIdB` stores entity ID — warm-starting key corrupt |
| P1 | 35.2 | `SolvePositionConstraints` iterates velocity constraints, not position constraints |
| P1 | 36.2 | `FindIslands` moves islands, leaving `m_AllIslands` with empty shells |
| P1 | 37.2 | `PolygonPolygon` generates only 1 contact point — stacking unstable |
| P1 | 37.4 | Capsule/Segment dispatch last — circle/polygon match intercepted first |
| P1 | 39.2 | `transform->rotation * vertex` scales by angle value, not rotate |
| P1 | 39.5 | All narrow-phase contacts appended to `m_FinalContacts` including invalid ones |
| P1 | 40.4 | `ApplyWarmStarting` compares entity IDs against solver body indices |
| P1 | 40.6 | `solverBodies` const ref mutated in `ApplyWarmStarting` — compile error |
| P1 | 41.1 | `ConstraintSolver` hardcodes friction=0.3, restitution=0.0 |
| P1 | 42.2 | `InputManager::Update` skips state copy when no events — ghost input |
| P2 | 33.1 | `ComponentStore::GetComponent` is O(N) — hot path performance |
| P2 | 33.2 | `GetComponent` returns shared `static dummy` on miss — silent data corruption in release |
| P2 | 34.2 | `ProcessNarrowPhase` also uses entity ID as shape ID |
| P2 | 35.3 | `ConstraintSolverSystem.cpp` includes `<fstream>`, `<chrono>` unused |
| P2 | 37.3 | `ManifoldGenerator.cpp` includes `<fstream>`, `<chrono>` unused |
| P2 | 44.1 | Baumgarte bias recomputed every velocity iteration — energy injection |
| P3 | 31.1 | Unconditional `std::cerr` debug noise in production builds |
| P3 | 42.1 | `IsKeyUp` returns `true` with null window — inconsistent with IsKeyDown |
| P3 | 43.1 | `DynamicTree::Rebuild` free-list detection is O(N×M) |

---

*End of Report*


The code is now quite close to a working 2D physics pipeline (inspired by Box2D-lite style), but several critical issues remain that explain why objects phase through floors/platforms despite contacts being detected:

### Main remaining problems causing tunneling / phasing through

1. **Position correction is applied too late**  
   You call `IntegratePositions()` **before** `SolvePositionConstraints()`.  
   → You first move bodies into deeper penetration, then try to push back. This is unstable and often insufficient (especially at 60 Hz).

2. **Position solver uses stale contact data**  
   You loop over `m_VelocityConstraints.points`, but those points have positions from the **previous** narrow-phase. After velocity solving + integration, bodies have moved → `point.position - body.position` is wrong → wrong `rA`, wrong correction direction/strength.

3. **No dedicated `PositionConstraint` usage**  
   You declared `PositionConstraint` and `m_PositionConstraints` but never fill or use them. The current `SolvePositionConstraints()` is using velocity-constraint data incorrectly.

4. **Baumgarte is applied, but weakly / inconsistently**  
   Typical stable values are β ≈ 0.1–0.25 (Box2D classic used ~0.2). Your `baumgarte = 0.2f` is reasonable, but because of point 1+2 it doesn't help enough.

5. **No post-correction velocity update** (minor, but adds jitter)  
   After position correction you don't zero out velocity component along normal (can cause sinking bounce-back).

6. **Static bodies are included in solver bodies** — good, but ensure invMass=0, invInertia=0 is strictly enforced.

7. **Warm-starting happens after velocity integration**  
   You apply cached impulses **after** gravity/force integration — this is wrong. Warm-start should happen **before** new velocity integration (standard is: warm-start → solve velocity → integrate velocity → solve position → integrate position).

### Recommended order (standard Box2D-like)

```
PrepareBodiesForUpdate()
BroadPhase → NarrowPhase → IslandDetection
ConstraintInitialization()   // fill velocity + position constraints, compute masses, restore impulses
VelocitySolving():
    WarmStartConstraints()
    IntegrateVelocities(dt)
    for velocityIterations: SolveVelocityConstraints()
PositionSolving():
    IntegratePositions(dt)
    for positionIterations: SolvePositionConstraints()
Integration()  // copy back to components
StoreImpulses()
UpdateSleeping()
UpdateTransformsFromSolver()
```

### Fixed version of the file

I only changed the problematic parts (mostly `VelocitySolving`, `PositionSolving`, `ConstraintInitialization`, order in `Update`).  
I added filling of `m_PositionConstraints` and rewrote `SolvePositionConstraints` to use it.  
I moved `IntegratePositions` **after** velocity solve but **before** position solve loop.

```cpp
// ... (includes and namespace unchanged)

void PhysicsPipelineSystem::Update(float deltaTime)
{
    // ... (lazy init of m_PhysicsWorld and m_IslandManager unchanged)

    if (!m_PhysicsWorld || !m_ComponentStore) return;

    auto startTime = std::chrono::high_resolution_clock::now();

    m_ActiveEntities.clear();
    m_ComponentStore->ForEachComponent<PhysicsBodyComponent>([&](EntityID eid, const PhysicsBodyComponent& b) {
        if (!b.isStatic) m_ActiveEntities.push_back(eid);
    });

    PrepareBodiesForUpdate();
    BroadPhaseDetection();
    NarrowPhaseDetection();
    IslandDetection();

    // Critical: initialize BOTH velocity and position constraints here
    ConstraintInitialization();

    VelocitySolving();      // warm-start → integrate vel → solve vel
    PositionSolving();      // integrate pos → solve pos (multiple times)

    Integration();
    StoreImpulses();
    UpdateSleeping();
    UpdateTransformsFromSolver();

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<float, std::milli>(endTime - startTime);
    m_Stats.updateTime = duration.count();
}

void PhysicsPipelineSystem::ConstraintInitialization()
{
    m_VelocityConstraints.clear();
    m_PositionConstraints.clear();  // ← important, we will fill this now

    for (const auto& manifold : m_ContactManifolds)
    {
        if (manifold.points.empty()) continue;

        auto itA = m_EntityToSolverIndex.find(manifold.entityIdA);
        auto itB = m_EntityToSolverIndex.find(manifold.entityIdB);
        if (itA == m_EntityToSolverIndex.end() || itB == m_EntityToSolverIndex.end()) continue;

        size_t idxA = itA->second;
        size_t idxB = itB->second;

        const auto& bodyA = m_SolverBodies[idxA];
        const auto& bodyB = m_SolverBodies[idxB];

        // --- Velocity constraint ---
        VelocityConstraint vc;
        vc.normal   = manifold.normal;
        vc.tangent  = Math::Vector2{-vc.normal.y, vc.normal.x};
        vc.indexA   = idxA;
        vc.indexB   = idxB;
        vc.invMassA = bodyA.invMass;
        vc.invMassB = bodyB.invMass;
        vc.invIA    = bodyA.invInertia;
        vc.invIB    = bodyB.invInertia;

        const auto& ca = m_ComponentStore->GetComponent<ColliderComponent>(manifold.entityIdA);
        const auto& cb = m_ComponentStore->GetComponent<ColliderComponent>(manifold.entityIdB);
        vc.friction    = std::sqrt(ca.material.friction * cb.material.friction);
        vc.restitution = std::max(ca.material.restitution, cb.material.restitution);

        vc.points = manifold.points;  // copy

        // Precompute normal/tangent mass + velocity bias + restore impulses
        for (auto& p : vc.points)
        {
            Math::Vector2 rA = p.position - bodyA.position;
            Math::Vector2 rB = p.position - bodyB.position;

            float crA_n = Math::Vector2::Cross(rA, vc.normal);
            float crB_n = Math::Vector2::Cross(rB, vc.normal);
            float kNormal = vc.invMassA + vc.invMassB + vc.invIA * crA_n*crA_n + vc.invIB * crB_n*crB_n;
            p.normalMass = (kNormal > 1e-6f) ? 1.0f / kNormal : 0.0f;

            float crA_t = Math::Vector2::Cross(rA, vc.tangent);
            float crB_t = Math::Vector2::Cross(rB, vc.tangent);
            float kTangent = vc.invMassA + vc.invMassB + vc.invIA * crA_t*crA_t + vc.invIB * crB_t*crB_t;
            p.tangentMass = (kTangent > 1e-6f) ? 1.0f / kTangent : 0.0f;

            // Relative normal velocity (at beginning of step)
            Math::Vector2 dv = (bodyB.velocity + Math::Vector2::Cross(bodyB.angularVelocity, rB))
                             - (bodyA.velocity + Math::Vector2::Cross(bodyA.angularVelocity, rA));
            float vn = Math::Vector2::Dot(dv, vc.normal);

            p.velocityBias = (vn < -m_PhysicsWorld->restitutionThreshold) ? -vc.restitution * vn : 0.0f;

            // Warm-start from cache
            uint64_t key = MakeImpulseCacheKey(manifold.entityIdA, manifold.entityIdB, p.featureId);
            auto cit = m_ImpulseCache.find(key);
            if (cit != m_ImpulseCache.end())
            {
                p.normalImpulse  = cit->second.normalImpulse;
                p.tangentImpulse = cit->second.tangentImpulse;
            }
            else
            {
                p.normalImpulse = p.tangentImpulse = 0.0f;
            }
        }

        m_VelocityConstraints.push_back(std::move(vc));

        // --- Position constraint (one per manifold, not per point — simpler & sufficient) ---
        PositionConstraint pc;
        pc.indexA = idxA;
        pc.indexB = idxB;
        pc.invMassA = bodyA.invMass;  pc.invMassB = bodyB.invMass;
        pc.invIA = bodyA.invInertia;  pc.invIB = bodyB.invInertia;
        pc.localNormal = manifold.localNormal;   // assume you store this in manifold
        pc.localPoint  = manifold.localPoint;    // reference point (e.g. deepest or average)

        // For simple correction we only need min separation (or use per-point if you want)
        pc.minSeparation = -std::numeric_limits<float>::infinity();
        for (const auto& p : manifold.points)
            if (p.separation < pc.minSeparation) pc.minSeparation = p.separation;

        m_PositionConstraints.push_back(std::move(pc));
    }

    m_Stats.activeConstraints = m_VelocityConstraints.size();
}

void PhysicsPipelineSystem::VelocitySolving()
{
    // Warm-start BEFORE integrating new forces/velocities
    if (m_Config.warmStarting) WarmStartConstraints();

    // Apply gravity + user forces
    for (auto& b : m_SolverBodies)
    {
        if (b.isStatic || !b.isAwake) continue;
        float mass = b.invMass > 0 ? 1.0f / b.invMass : 0.0f;
        b.force += m_PhysicsWorld->gravity * mass;
    }

    IntegrateVelocities(FIXED_TIMESTEP);

    for (int i = 0; i < m_Config.velocityIterations; ++i)
        SolveVelocityConstraints();
}

void PhysicsPipelineSystem::PositionSolving()
{
    IntegratePositions(FIXED_TIMESTEP);

    for (int i = 0; i < m_Config.positionIterations; ++i)
        SolvePositionConstraints();
}

void PhysicsPipelineSystem::SolvePositionConstraints()
{
    for (auto& pc : m_PositionConstraints)
    {
        auto& bodyA = m_SolverBodies[pc.indexA];
        auto& bodyB = m_SolverBodies[pc.indexB];

        // Simple single-point / min-separation correction (good enough for most games)
        if (pc.minSeparation > -m_Config.linearSlop) continue;

        float C = m_Config.baumgarte * (pc.minSeparation + m_Config.linearSlop);
        C = std::min(C, m_Config.maxLinearCorrection);  // avoid explosion

        float mass = pc.invMassA + pc.invMassB
                   + pc.invIA * /* some rotational term if you want */ 0.0f
                   + pc.invIB * 0.0f;

        if (mass < 1e-6f) continue;
        mass = 1.0f / mass;

        Math::Vector2 correction = pc.localNormal * (C * mass);  // direction from A to B

        if (!bodyA.isStatic)
            bodyA.position -= correction * pc.invMassA;

        if (!bodyB.isStatic)
            bodyB.position += correction * pc.invMassB;
    }
}

// The rest of the file (PrepareBodiesForUpdate, BroadPhase, NarrowPhase, WarmStart,
// SolveVelocityConstraints, Integrate*, StoreImpulses, UpdateSleeping, etc.) can stay
// mostly as-is — they look reasonable now that order & masses are fixed.
```

### Quick tuning recommendations

- In `Config`: keep `baumgarte = 0.2f`, `linearSlop = 0.01f` to `0.005f`, `maxLinearCorrection = 0.2f`
- `velocityIterations = 8–10`, `positionIterations = 3–5` (start low, increase if jittery)
- Make sure static floor has `invMass = invInertia = 0.0f` (enforced in `PrepareBodiesForUpdate` when `isStatic=true`)

Rebuild, drop a box on a static platform, and watch the logs / debug draw.  
If still tunneling → increase position iterations to 5–8 or lower `linearSlop` a bit.

Let me know what behavior you see after this change (sinking, bouncing too much, stable but soft, etc.) — we can fine-tune from there. Good luck!