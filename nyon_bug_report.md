# Nyon Physics Engine — Comprehensive Bug Report & Implementation Plan

**Version:** 1.0  
**Date:** March 2026  
**Scope:** Full codebase deep-dive. All issues from the reported symptom list investigated.

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Bug Severity Legend](#severity-legend)
3. [BUG-001 — Player Quad Never Jumps](#bug-001--player-quad-never-jumps)
4. [BUG-002 — Polygon Spins Uncontrollably (Energy Injection via Stale Warm-Start)](#bug-002--polygon-spins-uncontrollably)
5. [BUG-003 — Silent Mass & Inertia Override Each Frame](#bug-003--silent-mass--inertia-override-each-frame)
6. [BUG-004 — Polygon Phases Through Platform (Missing Angular Position Correction)](#bug-004--polygon-phases-through-platform)
7. [BUG-005 — Ball Double-Bounce: Gravity Appears to Double After Second Bounce](#bug-005--ball-double-bounce)
8. [BUG-006 — Circles Spin Erratically and Never Settle](#bug-006--circles-spin-erratically)
9. [BUG-007 — Sliding and Rolling Friction Appear Non-Functional](#bug-007--sliding-and-rolling-friction-non-functional)
10. [BUG-008 — SAT Manifold Geometrically Unstable for Rotating Polygons](#bug-008--sat-manifold-unstable-for-rotating-polygons)
11. [Implementation Plan: GJK/EPA Replacement for PolygonPolygon](#implementation-plan-gjkepa)
12. [Implementation Plan: Proper Rolling Friction](#implementation-plan-rolling-friction)
13. [Priority Fix Order](#priority-fix-order)

---

## Executive Summary

Eight distinct bugs were found through full source analysis. They decompose into three systemic root causes:

1. **Wrong sign convention on contact normals** — the manifold normal points A→B (into the body), but multiple call sites treat it as pointing away from the surface. This single sign error is responsible for both the jump failing and incorrect restitution behavior.

2. **Silent per-frame mass/inertia override** — `PrepareBodiesForUpdate()` silently recalculates and overwrites mass and inertia from collider density on every physics step, overriding any user-set values. This corrupts warm-starting, friction scaling, and jump forces.

3. **Meaningless contact feature IDs** — `ClipSegmentToLine` never assigns `featureId` to its output `ContactPoint`s. They default to zero and get sequentially renumbered by array index. The impulse cache then warm-starts geometrically wrong impulses onto wrong contacts every frame, continuously injecting energy and producing runaway spin.

All other symptoms (phasing, double-bounce, erratic circles, broken friction) cascade from these three root causes combined with two missing physics terms: angular position correction in the Baumgarte solver, and angular damping in the integrator.

---

## Severity Legend

| Level | Meaning |
|-------|---------|
| **CRITICAL** | Breaks core engine behavior entirely. Affects every entity. |
| **HIGH** | Significantly degrades simulation quality or player experience. |
| **MEDIUM** | Noticeable artifact; architectural problem that compounds over time. |
| **LOW** | Minor inconsistency or missing polish. |

---

## BUG-001 — Player Quad Never Jumps

**Severity:** CRITICAL  
**File:** `game/simple-physics-demo/src/SimplePhysicsDemo.cpp`  
**Function:** `IsPlayerGrounded()`  
**Symptom:** W key never causes the player quad to leave the ground regardless of how it is pressed.

### Root Cause

In the physics pipeline, the manifold normal for all shape pairs is defined to point **FROM entity A TOWARD entity B**. When the player quad is entity A and the platform is entity B, the normal vector points **downward** (from the player quad into the platform). This is the correct convention for the impulse solver — it resolves penetration in the A→B direction.

`IsPlayerGrounded()` reads this normal and tries to check if there is a floor beneath the player:

```cpp
// SimplePhysicsDemo.cpp — IsPlayerGrounded() — CURRENT BUGGY CODE
const Math::Vector2 up{0.0f, 1.0f};

Math::Vector2 contactNormal = isPlayerA ? manifold.normal : -manifold.normal;
float dotUp = Math::Vector2::Dot(contactNormal, up);
if (dotUp > 0.7f) {
    // check separation...
    return true;
}
```

When the player **is** entity A:
- `contactNormal = manifold.normal` = the **downward** vector (A→B)
- `dot(DOWN, UP)` = `-1.0`
- The condition `dotUp > 0.7f` is **never satisfied**
- The function **always returns false**

The jump line `body.velocity.y += PLAYER_JUMP_FORCE / body.mass` is gated behind `IsPlayerGrounded()`, so the jump **never fires**.

The code author's intent was correct — they wanted to flip the normal depending on which role the player plays in the pair — but the flip is applied to the wrong case. When the player is A, you need to **negate** to get the floor-to-player upward direction, not use it as-is.

### Exact Fix

```cpp
// SimplePhysicsDemo.cpp — IsPlayerGrounded() — FIXED
// manifold.normal points FROM A TOWARD B (i.e., from player into platform when player is A).
// To get the "surface normal pointing up toward player" direction:
//   - If player is A: negate (flip A→B downward to B→A upward)
//   - If player is B: use as-is (A→B points upward from platform toward player)

Math::Vector2 contactNormal = isPlayerA ? -manifold.normal : manifold.normal;
float dotUp = Math::Vector2::Dot(contactNormal, up);
if (dotUp > 0.7f) {
    for (const auto& pt : manifold.points) {
        if (pt.separation < 1.0f) {
            return true;
        }
    }
}
```

This one character change (`-manifold.normal` instead of `manifold.normal` for the A case) is all that is needed to make the jump work.

### Secondary Contributing Issue

Even with the above fix, the jump force computation uses `body.mass`:

```cpp
body.velocity.y += PLAYER_JUMP_FORCE / body.mass;
```

If BUG-003 (mass override) has corrupted `body.mass` in a prior frame, the jump force will be wrong. Fix BUG-003 first, or at minimum verify that `body.mass` is 2.0 at the time of the jump (see BUG-003 below).

---

## BUG-002 — Polygon Spins Uncontrollably

**Severity:** CRITICAL  
**Files:** `engine/src/physics/ManifoldGenerator.cpp`, `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`  
**Functions:** `ClipSegmentToLine()`, `GenerateManifold()`, `WarmStartConstraints()`  
**Symptom:** When polygon rotation is unlocked, the quad immediately begins spinning and accelerates without bound.

### Root Cause

The warm-starting mechanism is injecting **false energy** into the system on every single frame. The cause is that all PolygonPolygon contact points are assigned `featureId = 0` and then renumbered `1, 2, ...` by array index with no geometric meaning.

**Step 1 — ClipSegmentToLine never sets featureId:**

```cpp
// ManifoldGenerator.cpp — ClipSegmentToLine() — CURRENT BUGGY CODE
ECS::ContactPoint cp{};
cp.position      = pt;
cp.normal        = normal;
cp.separation    = sep;
cp.normalImpulse = 0.0f;
// cp.featureId is NEVER SET — defaults to 0 for every contact point
outContacts.push_back(cp);
```

**Step 2 — GenerateManifold renumbers them sequentially:**

```cpp
// PhysicsPipelineSystem.cpp — GenerateManifold() — CURRENT BUGGY CODE
contactPoint.featureId = point.featureId != 0 ? point.featureId : (i + 1);
// Result: contact[0] gets featureId=1, contact[1] gets featureId=2
// These IDs carry zero geometric meaning.
```

**Step 3 — WarmStartConstraints blindly applies cached impulse for featureId=1:**

On frame N: polygon is at angle θ. featureId=1 corresponds to contact point at geometric position P1. The solver computes `normalImpulse = X` for this point. This gets cached under key `(entityA, entityB, featureId=1)`.

On frame N+1: polygon has rotated slightly to angle θ+dθ. A NEW contact is generated, but it still gets featureId=1 (just by array position). WarmStartConstraints looks up the cache for featureId=1, finds `normalImpulse = X` from last frame, and applies it to what is now a **geometrically different contact point**.

This is not just wrong — it actively injects torque because the cached impulse is applied at a wrong geometric offset from the centroid. Every frame adds angular momentum. The polygon spins faster and faster.

### Exact Fix

Assign feature IDs that **encode the actual geometry** — specifically the reference face index and the incident face index — so the cache correctly matches contacts that correspond to the same geometric feature across frames.

**Fix 1 — ClipSegmentToLine: add featureId encoding**

Update the function to accept and encode the face indices:

```cpp
// ManifoldGenerator.cpp — ClipSegmentToLine() — FIXED
// Encode stable feature ID: high 16 bits = reference face, low 16 bits = incident face + vertex offset
// Add a parameter 'int incVertexOffset' (0 or 1) to distinguish the two clipped vertices.

inline void ClipSegmentToLine(
    std::vector<ECS::ContactPoint>& outContacts,
    const std::vector<Math::Vector2>& refVerts,
    const std::vector<Math::Vector2>& incVerts,
    int refFaceIndex,
    int incFaceIndex,
    const Math::Vector2& normal,
    float /* unused */)
{
    // ... existing clipping logic ...

    for (const auto& pt : finalClip) {
        float sep = Dot(normal, pt) - refOffset;
        if (sep <= 0.0f) {
            ECS::ContactPoint cp{};
            cp.position      = pt;
            cp.normal        = normal;
            cp.separation    = sep;
            cp.normalImpulse = 0.0f;
            cp.tangentImpulse = 0.0f;

            // FIXED: encode geometrically stable feature ID
            // Use the index within outContacts as the vertex offset (0 or 1)
            uint32_t vertexOffset = static_cast<uint32_t>(outContacts.size());
            cp.featureId = (static_cast<uint32_t>(refFaceIndex) << 16)
                         | ((static_cast<uint32_t>(incFaceIndex) & 0xFF) << 8)
                         | (vertexOffset & 0xFF);

            outContacts.push_back(cp);
        }
    }
}
```

**Fix 2 — Remove the fallback renumbering in GenerateManifold:**

```cpp
// PhysicsPipelineSystem.cpp — GenerateManifold() — FIXED
// Trust the featureId set by the manifold generator. Only use index fallback
// for shapes that legitimately have no feature concept (e.g. circle-circle = always 0).
contactPoint.featureId = point.featureId; // Do NOT override with (i+1)
```

With geometrically meaningful IDs, the warm-start cache correctly maps previous impulses to the same physical contact feature. Energy is conserved and the spin is damped naturally by friction.

---

## BUG-003 — Silent Mass & Inertia Override Each Frame

**Severity:** CRITICAL  
**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`  
**Function:** `PrepareBodiesForUpdate()`  
**Symptom:** Mass and inertia set explicitly in entity creation code are silently overwritten during the first (and potentially subsequent) physics step(s), corrupting all downstream calculations.

### Root Cause

```cpp
// PhysicsPipelineSystem.cpp — PrepareBodiesForUpdate() — CURRENT BUGGY CODE
if (!body.isStatic && m_ComponentStore->HasComponent<ColliderComponent>(entityId)) {
    const auto& collider = m_ComponentStore->GetComponent<ColliderComponent>(entityId);
    float area            = collider.CalculateArea();
    float density         = collider.material.density;
    float calculatedMass  = area * density;

    if (body.mass <= 0.0f || std::abs(body.mass - calculatedMass) > 0.01f) {
        body.SetMass(calculatedMass);  // ← silently overrides user-set mass
    }

    float unitInertia        = collider.CalculateInertiaPerUnitMass();
    float calculatedInertia  = unitInertia * calculatedMass;

    if (body.inertia <= 0.0f || std::abs(body.inertia - calculatedInertia) > 0.01f) {
        body.SetInertia(calculatedInertia);  // ← silently overrides user-set inertia
    }
}
```

**Concrete example with the player quad:**

- `body.mass` explicitly set to `2.0f` in `CreatePlayerQuad()`
- `body.inertia` explicitly set to `collider.CalculateInertiaPerUnitMass() * body.mass`
- In the demo, `area = 50*50 = 2500`, `density = 0.0008`, so `calculatedMass = 2500 * 0.0008 = 2.0`
- `|2.0 - 2.0| = 0.0` — happens to not trigger the override for this specific entity

However, for circles with random radii (e.g. radius=20): `area = π*400 ≈ 1256.6`, `calculatedMass ≈ 1.005`. If `body.mass` was set to `2.0`, then `|2.0 - 1.005| = 0.995 > 0.01` → **override fires on frame 1**. This changes the mass mid-step, which means the warm-start impulse cache from before the override is now based on the wrong mass. The impulse applied via `velocity += P * inverseMass` uses the NEW mass, but the stored `normalImpulse` magnitude was computed under the OLD mass. Result: one frame of wrong velocity change → visible velocity spike → erratic bounce/spin.

### Exact Fix

Add an `explicitMass` flag to `PhysicsBodyComponent`:

```cpp
// PhysicsBodyComponent.h — ADDED FIELDS
bool massIsExplicit    = false;  // true when mass is user-set; skip auto-calculation
bool inertiaIsExplicit = false;  // true when inertia is user-set; skip auto-calculation

void SetMass(float newMass) {
    mass           = newMass;
    massIsExplicit = true;  // mark as user-controlled
    UpdateMassProperties();
}

void SetInertia(float newInertia) {
    inertia           = newInertia;
    inertiaIsExplicit = true;
    inverseInertia    = (inertia > 0.0f) ? 1.0f / inertia : 0.0f;
}
```

Gate the auto-calculation in `PrepareBodiesForUpdate()`:

```cpp
// PhysicsPipelineSystem.cpp — PrepareBodiesForUpdate() — FIXED
if (!body.isStatic && m_ComponentStore->HasComponent<ColliderComponent>(entityId)) {
    if (!body.massIsExplicit) {
        float area           = collider.CalculateArea();
        float calculatedMass = area * collider.material.density;
        if (body.mass <= 0.0f || std::abs(body.mass - calculatedMass) > 0.01f) {
            body.SetMass(calculatedMass);
        }
    }
    if (!body.inertiaIsExplicit) {
        float unitInertia       = collider.CalculateInertiaPerUnitMass();
        float calculatedInertia = unitInertia * body.mass;
        if (body.inertia <= 0.0f || std::abs(body.inertia - calculatedInertia) > 0.01f) {
            body.SetInertia(calculatedInertia);
        }
    }
}
```

Update entity creation in the demo to use `SetMass()` so the explicit flag is properly set:

```cpp
// SimplePhysicsDemo.cpp — CreatePlayerQuad() and CreateSpawnedCircle() — FIXED
// Replace: body.mass = 2.0f;
// With:
body.SetMass(2.0f);

// Replace: body.inertia = collider.CalculateInertiaPerUnitMass() * body.mass;
// With:
body.SetInertia(collider.CalculateInertiaPerUnitMass() * body.mass);
```

---

## BUG-004 — Polygon Phases Through Platform

**Severity:** HIGH  
**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`  
**Function:** `SolvePositionConstraints()`  
**Symptom:** A spinning polygon progressively sinks into and eventually passes through the platform.

### Root Cause

`SolvePositionConstraints()` applies Baumgarte position correction (pushes overlapping bodies apart) but **only corrects linear position**. It never corrects the angle. For a rotating polygon, angular penetration is never resolved, so the same corner re-penetrates every frame.

```cpp
// PhysicsPipelineSystem.cpp — SolvePositionConstraints() — CURRENT BUGGY CODE
Math::Vector2 P = constraint.normal * (C * mass);

if (!bodyA.isStatic && !bodyB.isStatic) {
    bodyA.position -= P * constraint.invMassA;
    bodyB.position += P * constraint.invMassB;
    // bodyA.angle and bodyB.angle are NEVER MODIFIED
}
else if (!bodyA.isStatic) {
    bodyA.position -= P * constraint.invMassA;
    // bodyA.angle is NEVER MODIFIED
}
else if (!bodyB.isStatic) {
    bodyB.position += P * constraint.invMassB;
    // bodyB.angle is NEVER MODIFIED
}
```

For a box spinning at, say, 5 rad/s: in one step it rotates by `5 * 1/60 ≈ 0.083 rad ≈ 4.8°`. A 50×50 box corner can be up to 35px from center, so the corner can move `35 * sin(4.8°) ≈ 2.9px` per frame purely from rotation. Without angular correction, the solver pushes the box up by the penetration depth but doesn't account for the angular component, so the corner re-penetrates in the next frame. Combined with BUG-002 (energy injection), the spin grows each frame, the re-penetration grows, and eventually the accumulated error exceeds the solver's ability to correct, causing the phasing-through.

### Exact Fix

Add angular position correction using the contact arm cross product, matching Box2D's position solver:

```cpp
// PhysicsPipelineSystem.cpp — SolvePositionConstraints() — FIXED
Math::Vector2 P = constraint.normal * (C * mass);

if (!bodyA.isStatic && !bodyB.isStatic) {
    bodyA.position -= P * constraint.invMassA;
    bodyA.angle    -= constraint.invIA * Math::Vector2::Cross(rA, P);  // ADDED
    bodyB.position += P * constraint.invMassB;
    bodyB.angle    += constraint.invIB * Math::Vector2::Cross(rB, P);  // ADDED
    correctionsApplied++;
}
else if (!bodyA.isStatic) {
    bodyA.position -= P * constraint.invMassA;
    bodyA.angle    -= constraint.invIA * Math::Vector2::Cross(rA, P);  // ADDED
    correctionsApplied++;
}
else if (!bodyB.isStatic) {
    bodyB.position += P * constraint.invMassB;
    bodyB.angle    += constraint.invIB * Math::Vector2::Cross(rB, P);  // ADDED
    correctionsApplied++;
}
```

The corrected `bodyA.angle` and `bodyB.angle` are already written back to the transform in `UpdateTransformsFromSolver()` via `solverBody.angle`, so no additional changes are needed there.

Note: `rA` and `rB` are already computed earlier in the loop as:
```cpp
Math::Vector2 rA = point.position - worldCentroidA;
Math::Vector2 rB = point.position - worldCentroidB;
```
These are the correct contact arm vectors from centroid to contact point.

---

## BUG-005 — Ball Double-Bounce

**Severity:** HIGH  
**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`  
**Function:** `ConstraintInitialization()`  
**Symptom:** First bounce looks correct. Second bounce and beyond appear as if gravity doubled — the ball falls faster than expected and bounces chaotically.

### Root Cause

`restitutionThreshold` is initialized to `0.0f` in `PhysicsWorldComponent`. This means **every** contact, including near-zero approach velocities during resting contact, gets the bounce restitution bias applied.

```cpp
// PhysicsPipelineSystem.cpp — ConstraintInitialization() — CURRENT BUGGY CODE
float restitutionThreshold = 0.0f;  // fetched from PhysicsWorldComponent
if (vRel < -restitutionThreshold) {  // true for ANY approach velocity, including 0.001
    point.velocityBias = -vc.restitution * vRel;  // bounce bias applied to resting contacts too
}
else {
    point.velocityBias = 0.0f;
}
```

**What happens on the second bounce:**

The ball approaches at `v2 = e * v1` (after first bounce with restitution `e`). The velocity bias is computed as `-e * v2 = -e^2 * v1`. But during the resting phase between frame contacts — when the ball is technically still touching the platform but at near-zero relative velocity — the condition `vRel < 0` still fires (any tiny settling speed counts), and `velocityBias` gets a tiny non-zero value. 

With `e = 0.5` and gravity continuously pulling the ball down, the solver sees an approach velocity from gravity's contribution (`g * dt ≈ 9.8 * 0.016 ≈ 0.16 px/s per frame in normalized units`). It applies bounce bias `-0.5 * 0.16 = 0.08` upward. This fights the normal contact force and causes the constraint to oscillate. Visually the ball appears to accelerate downward faster (because the constraint is fighting itself on the wrong side of the threshold).

### Exact Fix

Set a non-zero restitution threshold when initializing the physics world:

```cpp
// SimplePhysicsDemo.cpp (or wherever PhysicsWorldComponent is created) — FIXED
world.restitutionThreshold = 100.0f;  // pixels/second — only bounce if approaching faster than this
```

This means contacts where the approach velocity is less than 100 px/s (slow, resting contact) will use `velocityBias = 0`, which is the pure non-penetration constraint — effectively a normal force with no bounce. Only fast impacts trigger restitution. Tune between `50.0f` and `200.0f` depending on the scale of your world coordinates.

---

## BUG-006 — Circles Spin Erratically

**Severity:** HIGH  
**File:** `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`  
**Function:** `IntegrateVelocities()`  
**Symptom:** Spawned circles acquire spin on first contact and continue spinning faster and faster. The spin never decays even when the circle is sitting still on a surface.

### Root Cause

`angularDamping` defaults to `0.0f` and **is never applied in the integrator**.

```cpp
// PhysicsPipelineSystem.cpp — IntegrateVelocities() — CURRENT BUGGY CODE
for (auto& body : m_SolverBodies) {
    if (body.isStatic || !body.isAwake) continue;

    body.velocity       += (body.force  * body.invMass)    * dt;
    body.angularVelocity += (body.torque * body.invInertia) * dt;

    // No damping of any kind is applied here.
    // body.angularDamping field exists but is never used.
    body.force  = Math::Vector2{0.0f, 0.0f};
    body.torque = 0.0f;
}
```

When a circle hits a surface at an angle, the tangential friction impulse at the contact point creates a torque around the center: `τ = Cross(r_contact, F_tangential)`. With zero damping this spin is permanent. BUG-005 (zero restitution threshold) compounds this: multiple micro-contact frames per resting contact each add a small tangential impulse, adding small increments of spin each frame. Over many frames this produces the observed erratic spinning behavior.

Note: `PhysicsBodyComponent` has `float angularDamping = 0.0f` declared at line 848, but it is purely decorative — it is never read by any system.

### Exact Fix

Apply exponential damping in `IntegrateVelocities()`. Exponential decay (`pow(1-d, dt)`) is numerically stable for any timestep and naturally handles variable damping coefficients:

```cpp
// PhysicsPipelineSystem.cpp — IntegrateVelocities() — FIXED
for (auto& body : m_SolverBodies) {
    if (body.isStatic || !body.isAwake) continue;

    body.velocity        += (body.force  * body.invMass)    * dt;
    body.angularVelocity += (body.torque * body.invInertia) * dt;

    // Apply damping — read linearDamping and angularDamping from the body component
    // These need to be fetched from PhysicsBodyComponent (not just SolverBody)
    float linearDamp  = 0.0f;
    float angularDamp = 0.0f;
    if (m_ComponentStore->HasComponent<PhysicsBodyComponent>(body.entityId)) {
        const auto& pbody = m_ComponentStore->GetComponent<PhysicsBodyComponent>(body.entityId);
        linearDamp  = pbody.drag;           // existing field
        angularDamp = pbody.angularDamping; // existing field, currently always 0
    }

    if (linearDamp  > 0.0f) body.velocity        *= std::pow(1.0f - linearDamp,  dt);
    if (angularDamp > 0.0f) body.angularVelocity *= std::pow(1.0f - angularDamp, dt);

    body.force  = Math::Vector2{0.0f, 0.0f};
    body.torque = 0.0f;
}
```

Set reasonable defaults in the demo for circles:

```cpp
// SimplePhysicsDemo.cpp — CreateSpawnedCircle() — ADDED
body.angularDamping = 0.05f;  // gentle continuous spin decay
body.drag           = 0.01f;  // small linear air resistance
```

Also add `linearDamping`/`drag` to `SolverBody` struct if you want to avoid per-body component lookups in the hot loop. The existing `drag` field in `PhysicsBodyComponent` is not currently being copied into `SolverBody` in `PrepareBodiesForUpdate()`.

---

## BUG-007 — Sliding and Rolling Friction Non-Functional

**Severity:** HIGH  
**Files:** `game/simple-physics-demo/src/SimplePhysicsDemo.cpp`, `engine/src/ecs/systems/PhysicsPipelineSystem.cpp`  
**Functions:** `HandlePlayerInput()`, `IntegrateVelocities()`  
**Symptom:** Entities slide without deceleration. Circles roll without any rolling resistance.

### Root Cause: Sliding Friction

The Coulomb friction constraint in `SolveVelocityConstraints()` is correctly implemented — it computes a tangential impulse and clamps it to `mu * normalImpulse`. However, for the player quad, this friction is circumvented because the player's horizontal velocity is **directly set** every frame, overriding whatever friction computed:

```cpp
// SimplePhysicsDemo.cpp — HandlePlayerInput() — CURRENT BUGGY CODE
if (horizontal != 0.0f) {
    body.velocity.x = horizontal * PLAYER_MOVE_SPEED;  // hard override, friction cannot fight this
}
else {
    body.velocity.x *= 0.9f;  // manual damping, not friction
}
```

When the velocity is directly set, the relative tangential velocity at the contact is controlled entirely by the assignment, not by the friction constraint. The friction impulse computed by the solver is applied, but then immediately overwritten on the next `HandlePlayerInput()` call.

### Root Cause: Rolling Friction

Rolling resistance (a torque opposing rotation proportional to normal force and rolling radius) is **not implemented at all**. The tangential constraint handles sliding friction (opposing translational slip at the contact point) but not rotational resistance. These are separate physical phenomena.

### Exact Fix: Sliding Friction (Player)

Replace direct velocity assignment with force-based movement:

```cpp
// SimplePhysicsDemo.cpp — HandlePlayerInput() — FIXED
constexpr float MOVE_FORCE = 5000.0f;  // tune to feel; higher = snappier response

if (horizontal != 0.0f) {
    body.ApplyForce({ horizontal * MOVE_FORCE, 0.0f });
    // Cap horizontal speed to prevent indefinite acceleration
    if (std::abs(body.velocity.x) > PLAYER_MOVE_SPEED) {
        body.velocity.x = std::copysign(PLAYER_MOVE_SPEED, body.velocity.x);
    }
} else {
    // When no input, let friction do the work — but apply extra drag for responsive stopping
    body.velocity.x *= 0.80f;
}
```

This allows the friction constraint to push back against the applied force, producing realistic deceleration when the player stops on a surface.

### Exact Fix: Rolling Friction

Add a rolling resistance torque in `IntegrateVelocities()` for bodies in contact:

```cpp
// PhysicsPipelineSystem.cpp — IntegrateVelocities() — ADDED rolling resistance
// Rolling resistance coefficient Cr for rubber-on-hard-surface ≈ 0.01
// Torque = -sign(omega) * Cr * mass * gravity * radius
// Simplified as velocity-proportional damping when no normal force is available here:

// Add to PhysicsBodyComponent:
// float rollingFriction = 0.0f;   // rolling resistance coefficient

// In IntegrateVelocities(), after angularVelocity integration:
if (body.rollingFriction > 0.0f && std::abs(body.angularVelocity) > 0.001f) {
    float resistanceTorque = -body.angularVelocity * body.rollingFriction;
    body.angularVelocity  += resistanceTorque * body.invInertia * dt;
}
```

A more physically accurate approach ties rolling resistance to the normal impulse from contact (see the Rolling Friction implementation plan at the end of this document).

---

## BUG-008 — SAT Manifold Unstable for Rotating Polygons

**Severity:** MEDIUM  
**File:** `engine/src/physics/ManifoldGenerator.cpp`  
**Function:** `PolygonPolygon()`  
**Symptom:** Even with BUG-002 fixed, the contact normal for a fast-rotating polygon can flip 180° between frames, causing impulse instability and jitter. For deep penetration the SAT gives wrong reference axes.

### Root Cause

The current SAT implementation finds the minimum-overlap separating axis and uses it as the collision normal. For a polygon rotating fast:

1. The minimum-overlap face can swap from face N to face N+1 between frames (when their overlap values are close), causing the manifold normal to flip 180°.
2. `ClipSegmentToLine` only clips the **two endpoints of the incident face** against the **side planes of the reference face**. For a box on a flat surface this produces 2 contacts, which is correct. But for edge cases (vertex-vertex contacts, deep penetration) the clipping produces wrong positions.
3. When penetration is large (> half the body size), SAT's minimum overlap axis is no longer the MTv (minimum translation vector) and can point in the wrong direction entirely.

GJK (Gilbert-Johnson-Keerthi) + EPA (Expanding Polytope Algorithm) resolves all of these cases by computing the exact MTv from the Minkowski difference of the two shapes. The normal is always the minimum translation direction and does not flip unless the geometry fundamentally changes.

### Recommendation

Replace `PolygonPolygon` with GJK/EPA. See the full implementation plan below.

---

## Implementation Plan: GJK/EPA

### Overview

GJK determines whether two convex shapes intersect by iteratively building a simplex (triangle in 2D) in the Minkowski difference space. If the simplex contains the origin, the shapes overlap. EPA then expands this simplex to find the minimum penetration vector.

### Phase 1: Support Functions

Each shape needs a support function that returns the farthest point in a given direction. Add these as methods or free functions in `ManifoldGenerator.cpp`:

```cpp
// ManifoldGenerator.cpp — ADDED

// Support function for a convex polygon (already in world space)
Math::Vector2 SupportPolygon(const std::vector<Math::Vector2>& worldVerts, const Math::Vector2& dir) {
    float maxDot = -std::numeric_limits<float>::infinity();
    Math::Vector2 best{};
    for (const auto& v : worldVerts) {
        float d = Dot(v, dir);
        if (d > maxDot) { maxDot = d; best = v; }
    }
    return best;
}

// Support function for a circle
Math::Vector2 SupportCircle(const Math::Vector2& center, float radius, const Math::Vector2& dir) {
    float len = dir.Length();
    if (len < 1e-8f) return center;
    return center + (dir / len) * radius;
}

// Minkowski difference support: support(A, dir) - support(B, -dir)
Math::Vector2 MinkowskiSupport(
    const std::vector<Math::Vector2>& vertsA,
    const std::vector<Math::Vector2>& vertsB,
    const Math::Vector2& dir)
{
    return SupportPolygon(vertsA, dir) - SupportPolygon(vertsB, {-dir.x, -dir.y});
}
```

### Phase 2: GJK Loop

```cpp
struct GJKSimplex {
    Math::Vector2 points[3];
    int count = 0;

    void Add(const Math::Vector2& p) {
        points[count++] = p;
    }
};

// Returns true if the two convex polygon vertex sets overlap.
// Fills 'simplex' with the final simplex for EPA input.
bool GJK(const std::vector<Math::Vector2>& vertsA,
         const std::vector<Math::Vector2>& vertsB,
         GJKSimplex& simplex)
{
    Math::Vector2 dir = {1.0f, 0.0f};  // initial direction
    Math::Vector2 support = MinkowskiSupport(vertsA, vertsB, dir);
    simplex.Add(support);
    dir = {-support.x, -support.y};  // toward origin

    for (int iter = 0; iter < 64; ++iter) {
        support = MinkowskiSupport(vertsA, vertsB, dir);
        if (Dot(support, dir) < 0.0f) return false;  // no overlap
        simplex.Add(support);

        // DoSimplex: check if origin is in/near simplex; update dir
        if (DoSimplex(simplex, dir)) return true;  // origin is inside simplex
    }
    return false;
}
```

`DoSimplex` handles the 2-point (line) and 3-point (triangle) cases in 2D, returning the new search direction toward the origin or confirming containment. This is the standard GJK subroutine (see Erin Catto's GDC 2010 slides for the reference implementation).

### Phase 3: EPA

```cpp
// Returns the minimum penetration depth and collision normal.
struct EPAResult {
    Math::Vector2 normal;
    float depth;
};

EPAResult EPA(const std::vector<Math::Vector2>& vertsA,
              const std::vector<Math::Vector2>& vertsB,
              GJKSimplex& simplex)
{
    // Expand simplex into a polygon (polytope)
    std::vector<Math::Vector2> polytope(simplex.points, simplex.points + simplex.count);

    for (int iter = 0; iter < 64; ++iter) {
        // Find the edge of the polytope closest to the origin
        float minDist = std::numeric_limits<float>::infinity();
        int   minIdx  = 0;
        Math::Vector2 minNormal;

        for (int i = 0; i < (int)polytope.size(); ++i) {
            int j = (i + 1) % polytope.size();
            Math::Vector2 edge = polytope[j] - polytope[i];
            Math::Vector2 normal = Normalize({edge.y, -edge.x});  // outward normal
            float dist = Dot(normal, polytope[i]);
            if (dist < 0.0f) { normal = -normal; dist = -dist; }
            if (dist < minDist) { minDist = dist; minNormal = normal; minIdx = j; }
        }

        // Support in direction of closest edge normal
        Math::Vector2 support = MinkowskiSupport(vertsA, vertsB, minNormal);
        float sDist = Dot(minNormal, support);

        if (std::abs(sDist - minDist) < 1e-4f) {
            return { minNormal, sDist };  // converged
        }
        polytope.insert(polytope.begin() + minIdx, support);
    }

    return { {0.0f, 1.0f}, 0.0f };  // fallback
}
```

### Phase 4: Contact Manifold from EPA Result

Once EPA returns the MTv normal and depth, generate the contact manifold by finding the incident edge on each shape:

```cpp
ECS::ContactManifold ManifoldGenerator::PolygonPolygonGJK(
    /* ... same signature as existing PolygonPolygon ... */)
{
    std::vector<Math::Vector2> vertsA, normalsA, vertsB, normalsB;
    ComputePolygonWorld(polyA, transformA, vertsA, normalsA);
    ComputePolygonWorld(polyB, transformB, vertsB, normalsB);

    GJKSimplex simplex;
    if (!GJK(vertsA, vertsB, simplex)) {
        return manifold;  // no collision
    }

    EPAResult epa = EPA(vertsA, vertsB, simplex);
    manifold.normal = epa.normal;

    // Find reference and incident faces using the EPA normal
    int refFace = FindIncidentFace(normalsA, epa.normal);   // most aligned with normal
    int incFace = FindIncidentFace(normalsB, -epa.normal);  // most aligned with -normal

    // Clip incident edge against reference face side planes (existing ClipSegmentToLine)
    ClipSegmentToLine(manifold.points, vertsA, vertsB, refFace, incFace, epa.normal, epa.depth);

    // Set separation from EPA depth
    for (auto& cp : manifold.points) {
        cp.separation = -epa.depth;
    }

    manifold.touching = !manifold.points.empty();
    return manifold;
}
```

Replace the call in `GenerateManifold` for the `Polygon-Polygon` case:

```cpp
if (tA == ST::Polygon && tB == ST::Polygon) {
    return PolygonPolygonGJK(entityIdA, entityIdB, shapeIdA, shapeIdB,
                             colliderA.GetPolygon(), colliderB.GetPolygon(),
                             transformA, transformB, manifold);
}
```

Keep the existing SAT path as a fast-reject pre-check (AABB overlap, already handled by broad phase) to avoid running full GJK on every pair.

---

## Implementation Plan: Rolling Friction

Rolling friction is most accurately implemented as a constraint torque tied to the normal contact force. Here is the recommended approach:

### Step 1: Add rolling friction coefficient to ColliderComponent material

```cpp
// ColliderComponent.h — PhysicsMaterial struct — ADDED
struct PhysicsMaterial {
    float density     = 1.0f;
    float friction    = 0.5f;
    float restitution = 0.0f;
    float rollingFriction = 0.01f;  // NEW: rolling resistance coefficient (dimensionless)
};
```

### Step 2: Compute rolling friction in VelocityConstraint initialization

In `ConstraintInitialization()`, alongside the existing friction/restitution blending:

```cpp
// Combine rolling friction coefficients (geometric mean like sliding friction)
vc.rollingFriction = std::sqrt(colliderA.material.rollingFriction * colliderB.material.rollingFriction);
```

Add `float rollingFriction` to the `VelocityConstraint` struct.

### Step 3: Apply rolling resistance torque in SolveVelocityConstraints

After the tangential (sliding) friction impulse is applied:

```cpp
// PhysicsPipelineSystem.cpp — SolveVelocityConstraints() — ADDED rolling friction block

// Rolling resistance: a torque opposing angular velocity, proportional to normal force.
// Only applies when there is a normal impulse (body is in contact).
// τ_resist = -Cr * r * normalImpulse * sign(omega)
// Applied as an angular impulse capped by rolling friction coefficient.

float Cr = constraint.rollingFriction;
if (Cr > 0.0f && point.normalImpulse > 0.0f) {
    // For circles: use the actual radius. For polygons: approximate as half the min extent.
    // For now pass rolling friction radius via the body or compute from collider.
    float radius = 1.0f;  // TODO: retrieve from collider

    float maxRollingImpulse = Cr * point.normalImpulse * radius;

    if (!bodyA.isStatic) {
        float wSign  = (bodyA.angularVelocity > 0.0f) ? 1.0f : -1.0f;
        float dOmegA = -wSign * maxRollingImpulse * constraint.invIA;
        // Clamp so it doesn't flip the spin direction
        if (std::abs(dOmegA) > std::abs(bodyA.angularVelocity))
            dOmegA = -bodyA.angularVelocity;
        bodyA.angularVelocity += dOmegA;
    }
    if (!bodyB.isStatic) {
        float wSign  = (bodyB.angularVelocity > 0.0f) ? 1.0f : -1.0f;
        float dOmegB = -wSign * maxRollingImpulse * constraint.invIB;
        if (std::abs(dOmegB) > std::abs(bodyB.angularVelocity))
            dOmegB = -bodyB.angularVelocity;
        bodyB.angularVelocity += dOmegB;
    }
}
```

---

## Priority Fix Order

Apply fixes in this order to avoid compounding issues:

| Priority | Bug ID | Why This Order |
|----------|--------|----------------|
| 1 | BUG-003 | Mass override corrupts all other fixes. Fix first so subsequent fixes run with correct mass. |
| 2 | BUG-002 | Energy injection is the most destabilizing. Must stop before evaluating other dynamics. |
| 3 | BUG-001 | Jump is completely broken. Fix the sign and verify grounded detection with debug logging. |
| 4 | BUG-005 | Fix restitution threshold to get stable bouncing before diagnosing friction. |
| 5 | BUG-006 | Enable angular damping. Circles should settle after fixing #5. |
| 6 | BUG-004 | Angular position correction. Polygons should stop phasing now that spin energy is controlled. |
| 7 | BUG-007 | Friction fixes. These are only meaningful once the above stabilize the simulation. |
| 8 | BUG-008 | GJK/EPA. Major architectural change; defer until all numerical bugs are fixed. |

---

## Quick Reference: All Changed Files

| File | Change |
|------|--------|
| `PhysicsBodyComponent.h` | Add `massIsExplicit`, `inertiaIsExplicit`, update `SetMass()`/`SetInertia()`. Add `rollingFriction` to material. |
| `PhysicsPipelineSystem.cpp` — `PrepareBodiesForUpdate()` | Gate mass override behind `massIsExplicit` flag. |
| `PhysicsPipelineSystem.cpp` — `ConstraintInitialization()` | Copy `rollingFriction`. |
| `PhysicsPipelineSystem.cpp` — `SolveVelocityConstraints()` | Add rolling resistance torque block. |
| `PhysicsPipelineSystem.cpp` — `SolvePositionConstraints()` | Add angular correction `body.angle -= invI * Cross(r, P)`. |
| `PhysicsPipelineSystem.cpp` — `IntegrateVelocities()` | Apply `angularDamping` and `drag` exponential decay. |
| `PhysicsPipelineSystem.cpp` — `GenerateManifold()` | Remove sequential `featureId` renumbering. |
| `ManifoldGenerator.cpp` — `ClipSegmentToLine()` | Assign geometrically encoded `featureId`. |
| `ManifoldGenerator.cpp` | Add GJK/EPA functions and `PolygonPolygonGJK()`. |
| `SimplePhysicsDemo.cpp` — `IsPlayerGrounded()` | Flip sign: `isPlayerA ? -manifold.normal : manifold.normal`. |
| `SimplePhysicsDemo.cpp` — `HandlePlayerInput()` | Replace direct velocity set with `ApplyForce`. |
| `SimplePhysicsDemo.cpp` — `CreatePlayerQuad()` | Use `body.SetMass()` and `body.SetInertia()`. |
| `SimplePhysicsDemo.cpp` — `CreateSpawnedCircle()` | Use `body.SetMass()`. Set `angularDamping`, `drag`. |
| `PhysicsWorldComponent.h` (or demo init) | Set `world.restitutionThreshold = 100.0f`. |

---

---

# GPU Particle Rendering — Full Implementation Plan

**Scope:** Replace the current single-threaded CPU batch renderer (`Renderer2D`) with a fully GPU-driven instanced rendering pipeline capable of rendering millions of particles per frame at 60+ fps. This plan is written against your existing OpenGL 4.6 core profile context and `Renderer2D` architecture.

---

## Table of Contents (GPU Section)

1. [Current Renderer Analysis & Bottlenecks](#current-renderer-analysis--bottlenecks)
2. [Architecture Overview: CPU vs GPU Driven](#architecture-overview-cpu-vs-gpu-driven)
3. [Phase 1 — Instanced Rendering for Circles and Quads](#phase-1--instanced-rendering-for-circles-and-quads)
4. [Phase 2 — GPU Particle Buffer with Persistent Mapping](#phase-2--gpu-particle-buffer-with-persistent-mapping)
5. [Phase 3 — Compute Shader Particle Simulation](#phase-3--compute-shader-particle-simulation)
6. [Phase 4 — GPU-Side Culling with Indirect Draw](#phase-4--gpu-side-culling-with-indirect-draw)
7. [Phase 5 — Spatial Hashing on GPU for Collision Broadphase](#phase-5--spatial-hashing-on-gpu)
8. [Shader Code Reference](#shader-code-reference)
9. [Integration with Existing ECS and PhysicsPipelineSystem](#integration-with-existing-ecs)
10. [Memory Layout and Budget](#memory-layout-and-budget)
11. [Build Order](#build-order)

---

## Current Renderer Analysis & Bottlenecks

Before designing the replacement, it is important to be exact about what the current renderer does and where each bottleneck sits.

### What Renderer2D currently does

Looking at `Renderer2D.cpp` and the `Impl` struct:

- Maintains two `std::vector<Vertex>` CPU buffers: `QuadBuffer` (triangles) and `LineBuffer` (lines).
- Every shape drawn (quad, circle, polygon) calls a function like `DrawQuad()` or `DrawSolidCircle()` which pushes vertices into these vectors **on the CPU** — one vertex per element, with position, color, UV, and normal baked in at the time of the draw call.
- At `EndScene()` → `Flush()`, the entire CPU buffer is uploaded to the GPU via `glBufferData(GL_DYNAMIC_DRAW)` — a full buffer re-upload every frame.
- A single `glDrawArrays()` call renders everything.

There is **no geometry shader, no instancing, no indirect draw, no compute**. The VAO/VBO pipeline is basic immediate-mode emulation inside modern OpenGL.

### Where every millisecond goes

| Stage | Cost | Why it is slow |
|-------|------|----------------|
| `DrawSolidCircle()` — CPU loop | O(N × segments) | For 32 segments and 1M circles = 32M vertex struct writes on CPU per frame. |
| `std::vector::push_back` churn | O(N) allocations | Cache-unfriendly; `Vertex` is 40 bytes; 1M circles = 1.28 GB/s of CPU-side writes. |
| `glBufferData(GL_DYNAMIC_DRAW)` | Full PCIe upload | Re-uploads the **entire** vertex buffer every frame even if nothing moved. At 40 bytes × 32M vertices = 1.28 GB per frame. PCIe 3.0 x16 = ~16 GB/s, so just this transfer costs ~80ms. |
| `BeginScene()` `QuadBuffer.clear()` | Clears 40M entries | Destructs the vector contents, not just sets size to 0. |
| Single-threaded CPU geometry generation | No parallelism | All tessellation (circle segments, polygon fans) runs on one thread serially. |

At 1000 particles the current approach works fine. At 100,000 it becomes sluggish. At 1,000,000 it is physically impossible to maintain 60fps on any hardware.

---

## Architecture Overview: CPU vs GPU Driven

### CPU-driven (current)
```
CPU: generate all geometry → push to vector → upload to GPU → draw
GPU: receives finished triangles, shades them
```

### GPU-driven (target)
```
CPU: upload per-instance data once (position, angle, color, radius) → dispatch indirect draw
GPU: generates geometry in vertex shader → culls in compute shader → draws only visible
```

The key insight is that **a circle at world position (x, y) with radius r and color (r,g,b) is fully described by 7 floats**. You do not need to send 32 × 3 = 96 vertices per circle. You send 7 floats and let the vertex shader compute the 96 vertex positions on-GPU, in parallel, for all circles simultaneously.

This collapses the per-circle CPU work from 96 × 40 byte struct writes = **3840 bytes** down to **28 bytes** of instance data.

For 1 million circles:
- Current approach: 3.84 GB of CPU writes + 3.84 GB PCIe upload
- GPU-driven approach: 28 MB of instance data upload, **137× less data**

---

## Phase 1 — Instanced Rendering for Circles and Quads

This is the foundation. Every subsequent phase builds on this. It replaces the `DrawSolidCircle` and `DrawQuad` vertex-generation loops entirely.

### 1.1 — Define the Instance Data Layout

Create a new header: `engine/include/nyon/graphics/ParticleRenderer.h`

```cpp
// engine/include/nyon/graphics/ParticleRenderer.h

#pragma once
#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include <glad/glad.h>
#include <vector>
#include <cstdint>

namespace Nyon::Graphics {

    // Per-instance data uploaded to the GPU.
    // Kept tightly packed: 32 bytes per particle — fits exactly in one cache line half.
    struct alignas(16) ParticleInstance {
        float x, y;          // world position (8 bytes)
        float angle;         // rotation in radians (4 bytes)
        float radius;        // for circles; half-extent for quads (4 bytes)
        float r, g, b;       // color (12 bytes)
        float aspectRatio;   // width/height for quads; 1.0 for circles (4 bytes)
        // Total: 32 bytes
    };

    static_assert(sizeof(ParticleInstance) == 32, "ParticleInstance must be 32 bytes");

    class ParticleRenderer {
    public:
        // Maximum number of particles. 4M = 128 MB of instance data.
        static constexpr uint32_t MAX_PARTICLES = 4'000'000;
        // Circle mesh resolution. 16 sides is enough for smooth appearance at small sizes.
        static constexpr int CIRCLE_SEGMENTS = 16;
        // Quad uses 6 vertices (2 triangles).
        static constexpr int QUAD_VERTEX_COUNT = 6;

        ParticleRenderer();
        ~ParticleRenderer();

        // Non-copyable, non-movable (owns GPU resources).
        ParticleRenderer(const ParticleRenderer&) = delete;
        ParticleRenderer& operator=(const ParticleRenderer&) = delete;

        void Init();
        void Shutdown();

        // Called once per frame before submitting any particles.
        void BeginFrame();

        // Submit a circle instance. Batched until Flush().
        void SubmitCircle(float x, float y, float radius, float r, float g, float b);

        // Submit a quad instance. aspectRatio = width / height.
        void SubmitQuad(float x, float y, float halfExtent, float angle,
                        float r, float g, float b, float aspectRatio = 1.0f);

        // Upload all pending instances and issue instanced draw calls.
        void Flush(const glm::mat4& viewProjection);

        uint32_t GetLastFrameCircleCount()  const { return m_LastCircleCount;  }
        uint32_t GetLastFrameQuadCount()    const { return m_LastQuadCount;    }

    private:
        void BuildCircleMesh();
        void BuildQuadMesh();
        void SetupInstanceBuffer();
        void SetupShaders();

        // Circle VAO
        GLuint m_CircleVAO      = 0;
        GLuint m_CircleMeshVBO  = 0;   // static mesh vertices (unit circle)
        GLuint m_CircleInstVBO  = 0;   // per-instance data (dynamic, persistently mapped)

        // Quad VAO
        GLuint m_QuadVAO        = 0;
        GLuint m_QuadMeshVBO    = 0;   // static mesh vertices (unit quad)
        GLuint m_QuadInstVBO    = 0;   // per-instance data

        // Shader programs
        GLuint m_CircleShader   = 0;
        GLuint m_QuadShader     = 0;

        // CPU-side staging buffers (written this frame, then uploaded)
        std::vector<ParticleInstance> m_CircleInstances;
        std::vector<ParticleInstance> m_QuadInstances;

        // Stats
        uint32_t m_LastCircleCount = 0;
        uint32_t m_LastQuadCount   = 0;
    };

} // namespace Nyon::Graphics
```

### 1.2 — Build the Unit Circle Mesh (static, uploaded once)

The mesh is a **triangle fan** for a unit circle (radius = 1.0). The vertex shader will scale it by `radius` per instance:

```cpp
// engine/src/graphics/ParticleRenderer.cpp

void ParticleRenderer::BuildCircleMesh() {
    // Triangle fan: center vertex + CIRCLE_SEGMENTS+1 perimeter vertices
    // = CIRCLE_SEGMENTS triangles, each with 3 vertices = CIRCLE_SEGMENTS*3 vertices total
    const int vertCount = CIRCLE_SEGMENTS * 3;
    std::vector<float> verts;
    verts.reserve(vertCount * 2);  // only XY; color/radius from instance data

    const float step = 2.0f * 3.14159265f / static_cast<float>(CIRCLE_SEGMENTS);
    for (int i = 0; i < CIRCLE_SEGMENTS; ++i) {
        float a0 = step * i;
        float a1 = step * (i + 1);
        // Center
        verts.push_back(0.0f); verts.push_back(0.0f);
        // Perimeter point 0
        verts.push_back(std::cos(a0)); verts.push_back(std::sin(a0));
        // Perimeter point 1
        verts.push_back(std::cos(a1)); verts.push_back(std::sin(a1));
    }

    glGenVertexArrays(1, &m_CircleVAO);
    glGenBuffers(1, &m_CircleMeshVBO);

    glBindVertexArray(m_CircleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_CircleMeshVBO);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);

    // Attribute 0: mesh XY (vec2), from mesh VBO
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}
```

### 1.3 — Setup the Instance Buffer with Vertex Attribute Divisor

```cpp
void ParticleRenderer::SetupInstanceBuffer() {
    // --- Circle instance VBO ---
    glBindVertexArray(m_CircleVAO);

    glGenBuffers(1, &m_CircleInstVBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_CircleInstVBO);
    // Allocate GPU memory for MAX_PARTICLES instances, no data yet
    glBufferData(GL_ARRAY_BUFFER,
                 MAX_PARTICLES * sizeof(ParticleInstance),
                 nullptr, GL_DYNAMIC_DRAW);

    // Attribute 1: instance position (xy) — offset 0
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(ParticleInstance), (void*)offsetof(ParticleInstance, x));
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);  // advance once per INSTANCE, not per vertex

    // Attribute 2: instance angle + radius (zw packed as vec2)
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(ParticleInstance), (void*)offsetof(ParticleInstance, angle));
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2, 1);

    // Attribute 3: instance color (rgb)
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(ParticleInstance), (void*)offsetof(ParticleInstance, r));
    glEnableVertexAttribArray(3);
    glVertexAttribDivisor(3, 1);

    // Attribute 4: aspectRatio
    glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, sizeof(ParticleInstance), (void*)offsetof(ParticleInstance, aspectRatio));
    glEnableVertexAttribArray(4);
    glVertexAttribDivisor(4, 1);

    glBindVertexArray(0);
    // (Repeat symmetrically for m_QuadVAO / m_QuadInstVBO)
}
```

The `glVertexAttribDivisor(attrib, 1)` call is the key that turns this into instanced rendering. It tells OpenGL: advance this attribute once per draw instance, not once per vertex. Every vertex in triangle fan #5 reads the same `(x, y, radius, color)` for instance #5.

### 1.4 — The Flush Call

```cpp
void ParticleRenderer::Flush(const glm::mat4& viewProjection) {
    if (!m_CircleInstances.empty()) {
        // Upload only the used portion — glBufferSubData, NOT glBufferData
        glBindBuffer(GL_ARRAY_BUFFER, m_CircleInstVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        m_CircleInstances.size() * sizeof(ParticleInstance),
                        m_CircleInstances.data());

        glUseProgram(m_CircleShader);
        glUniformMatrix4fv(glGetUniformLocation(m_CircleShader, "u_VP"),
                           1, GL_FALSE, &viewProjection[0][0]);

        glBindVertexArray(m_CircleVAO);
        // glDrawArraysInstanced: draw the CIRCLE_SEGMENTS*3 mesh vertices,
        // repeated m_CircleInstances.size() times
        glDrawArraysInstanced(GL_TRIANGLES,
                              0,
                              CIRCLE_SEGMENTS * 3,
                              static_cast<GLsizei>(m_CircleInstances.size()));
        glBindVertexArray(0);

        m_LastCircleCount = static_cast<uint32_t>(m_CircleInstances.size());
        m_CircleInstances.clear();
    }
    // (Repeat for quads)
}
```

A single `glDrawArraysInstanced` call with `instanceCount = 1,000,000` is processed entirely in parallel by the GPU's shader cores. The GPU has thousands of cores and processes all 1 million instances concurrently.

---

## Phase 2 — GPU Particle Buffer with Persistent Mapping

Phase 1 still does a `glBufferSubData` upload every frame, which stalls the GPU pipeline if the driver synchronizes the CPU and GPU on that buffer. For millions of particles updating every frame, this is the next bottleneck.

The solution is **persistent buffer mapping with triple buffering**: the CPU writes to a persistently mapped GPU buffer with no stall. A fence ensures the CPU does not overwrite a section the GPU is still reading.

### 2.1 — Allocate a Persistently Mapped Buffer

Requires OpenGL 4.4+ (`GL_ARB_buffer_storage`). Your context is 4.6, so this is available.

```cpp
// engine/src/graphics/ParticleRenderer.cpp — Init()

constexpr int NUM_BUFFERS = 3;  // triple-buffering
const GLsizeiptr totalSize = MAX_PARTICLES * sizeof(ParticleInstance) * NUM_BUFFERS;

// Flags: MAP_WRITE (CPU writes) | MAP_PERSISTENT (stays mapped across draw calls)
//        | MAP_COHERENT (writes visible to GPU without explicit flush)
const GLbitfield storageFlags = GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT;
const GLbitfield mapFlags     = GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT;

glGenBuffers(1, &m_CircleInstVBO);
glBindBuffer(GL_ARRAY_BUFFER, m_CircleInstVBO);
glBufferStorage(GL_ARRAY_BUFFER, totalSize, nullptr, storageFlags);  // NOT glBufferData

m_MappedCircleBuffer = static_cast<ParticleInstance*>(
    glMapBufferRange(GL_ARRAY_BUFFER, 0, totalSize, mapFlags)
);
// m_MappedCircleBuffer is now a permanent CPU pointer into GPU memory.
// Never call glUnmapBuffer on it until Shutdown().

// Initialize fences to null
for (int i = 0; i < NUM_BUFFERS; ++i) {
    m_Fences[i] = nullptr;
}
```

### 2.2 — Triple-Buffer Write Pattern

```cpp
// engine/src/graphics/ParticleRenderer.cpp

void ParticleRenderer::BeginFrame() {
    // Advance to the next buffer slot
    m_CurrentBufferIdx = (m_CurrentBufferIdx + 1) % NUM_BUFFERS;

    // Wait for the GPU to finish reading the slot we're about to write into.
    // This stall should be zero if the CPU is not ahead of the GPU by 3 frames.
    if (m_Fences[m_CurrentBufferIdx] != nullptr) {
        GLsync fence = m_Fences[m_CurrentBufferIdx];
        while (true) {
            GLenum result = glClientWaitSync(fence, GL_SYNC_FLUSH_COMMANDS_BIT, 1'000'000);
            if (result == GL_ALREADY_SIGNALED || result == GL_CONDITION_SATISFIED) break;
            // If timeout, keep waiting — in practice this should never spin more than once
        }
        glDeleteSync(fence);
        m_Fences[m_CurrentBufferIdx] = nullptr;
    }

    // The write pointer for this frame points to this buffer slot's region
    m_WritePtr = m_MappedCircleBuffer + (m_CurrentBufferIdx * MAX_PARTICLES);
    m_WriteCount = 0;
}

void ParticleRenderer::SubmitCircle(float x, float y, float radius,
                                    float r, float g, float b) {
    if (m_WriteCount >= MAX_PARTICLES) return;
    // Direct write into GPU memory — no copy, no intermediate vector
    ParticleInstance& inst = m_WritePtr[m_WriteCount++];
    inst.x = x;  inst.y = y;
    inst.angle = 0.0f;
    inst.radius = radius;
    inst.r = r;  inst.g = g;  inst.b = b;
    inst.aspectRatio = 1.0f;
}

void ParticleRenderer::Flush(const glm::mat4& viewProjection) {
    if (m_WriteCount == 0) return;

    // Issue the draw call pointing at this buffer slot's offset
    const GLintptr offset = m_CurrentBufferIdx * MAX_PARTICLES * sizeof(ParticleInstance);

    // Rebind the VAO's instance attribute to start at offset
    glBindVertexArray(m_CircleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_CircleInstVBO);
    // Update the attribute pointer to the current buffer slot's offset
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(ParticleInstance),
                          (void*)(offset + offsetof(ParticleInstance, x)));
    // ... repeat for other attributes with the same offset base ...

    glUseProgram(m_CircleShader);
    glUniformMatrix4fv(/* u_VP */, 1, GL_FALSE, &viewProjection[0][0]);
    glDrawArraysInstanced(GL_TRIANGLES, 0, CIRCLE_SEGMENTS * 3, m_WriteCount);

    // Place a fence so we know when the GPU is done with this slot
    m_Fences[m_CurrentBufferIdx] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);

    m_LastCircleCount = m_WriteCount;
    m_WriteCount = 0;
    glBindVertexArray(0);
}
```

With persistent mapping, `SubmitCircle()` is literally a struct field assignment into a pointer — it is as fast as writing to a `std::vector` but the data is already in GPU-accessible memory. There is no `glBufferSubData`, no PCIe transfer stall.

---

## Phase 3 — Compute Shader Particle Simulation

Once particles live permanently in a GPU buffer (Phase 2), the physics integration loop for simple particles (gravity, velocity, lifetime) can be moved entirely onto the GPU. This frees the CPU entirely for complex rigid-body physics (your existing island solver) while the GPU handles millions of visual-only particles.

This introduces a clear split:
- **CPU + Island solver:** rigid bodies, joints, collision detection (existing `PhysicsPipelineSystem`)
- **GPU compute shader:** visual particles — smoke, sparks, debris, dust — that do not need full collision

### 3.1 — Particle State Buffer

Define the particle state that lives on the GPU as a Shader Storage Buffer Object (SSBO):

```glsl
// particles.glsl (shared include)

struct Particle {
    vec2  position;       // 8 bytes
    vec2  velocity;       // 8 bytes
    vec4  color;          // 16 bytes (rgba + alpha for fade)
    float lifetime;       // seconds remaining (4 bytes)
    float maxLifetime;    // total lifetime for fade calculation (4 bytes)
    float radius;         // (4 bytes)
    float angularVel;     // spin (4 bytes)
    float angle;          // current rotation (4 bytes)
    float _pad0;          // padding to 16-byte alignment (4 bytes)
    float _pad1;
    float _pad2;
    // Total: 64 bytes per particle
};
```

```cpp
// engine/include/nyon/graphics/GPUParticleSystem.h

struct GPUParticle {
    float px, py;          // position
    float vx, vy;          // velocity
    float r, g, b, a;      // color + alpha
    float lifetime;
    float maxLifetime;
    float radius;
    float angularVel;
    float angle;
    float pad0, pad1, pad2;
};
static_assert(sizeof(GPUParticle) == 64, "GPUParticle must be 64 bytes");
```

Allocate the SSBO on the GPU:

```cpp
// engine/src/graphics/GPUParticleSystem.cpp — Init()

constexpr uint32_t MAX_GPU_PARTICLES = 4'000'000;

glGenBuffers(1, &m_ParticleSSBO);
glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_ParticleSSBO);
glBufferStorage(GL_SHADER_STORAGE_BUFFER,
                MAX_GPU_PARTICLES * sizeof(GPUParticle),
                nullptr,
                GL_DYNAMIC_STORAGE_BIT);  // CPU can call glBufferSubData to spawn particles
glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_ParticleSSBO);  // binding = 0

// Atomic counter for active particle count
glGenBuffers(1, &m_CounterBuffer);
glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, m_CounterBuffer);
glBufferStorage(GL_ATOMIC_COUNTER_BUFFER, sizeof(uint32_t), nullptr,
                GL_DYNAMIC_STORAGE_BIT | GL_MAP_READ_BIT);
glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, 0, m_CounterBuffer);
```

### 3.2 — Simulation Compute Shader

```glsl
// engine/assets/shaders/particle_simulate.comp
#version 460 core

layout(local_size_x = 256, local_size_y = 1, local_size_z = 1) in;

struct Particle {
    vec2  position;
    vec2  velocity;
    vec4  color;
    float lifetime;
    float maxLifetime;
    float radius;
    float angularVel;
    float angle;
    float _pad0, _pad1, _pad2;
};

layout(std430, binding = 0) buffer ParticleBuffer {
    Particle particles[];
};

// Output: compact list of alive particle indices for the render pass
layout(std430, binding = 1) buffer AliveBuffer {
    uint aliveIndices[];
};

layout(binding = 0, offset = 0) uniform atomic_uint u_AliveCount;

uniform float u_DeltaTime;
uniform vec2  u_Gravity;        // e.g. (0, -980)
uniform uint  u_TotalParticles; // total slots including dead ones

void main() {
    uint idx = gl_GlobalInvocationID.x;
    if (idx >= u_TotalParticles) return;

    Particle p = particles[idx];

    // Skip dead particles
    if (p.lifetime <= 0.0) return;

    // Integrate velocity and position
    p.velocity  += u_Gravity * u_DeltaTime;
    p.position  += p.velocity * u_DeltaTime;
    p.angle     += p.angularVel * u_DeltaTime;
    p.lifetime  -= u_DeltaTime;

    // Fade alpha based on remaining lifetime
    float lifeFraction = clamp(p.lifetime / p.maxLifetime, 0.0, 1.0);
    p.color.a = lifeFraction;

    // Write back updated state
    particles[idx] = p;

    // If still alive, append to alive list for rendering
    if (p.lifetime > 0.0) {
        uint slot = atomicCounterIncrement(u_AliveCount);
        aliveIndices[slot] = idx;
    }
}
```

Dispatch this shader every frame:

```cpp
// engine/src/graphics/GPUParticleSystem.cpp — Simulate()

void GPUParticleSystem::Simulate(float dt, const Math::Vector2& gravity) {
    // Reset alive counter to 0
    uint32_t zero = 0;
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, m_CounterBuffer);
    glBufferSubData(GL_ATOMIC_COUNTER_BUFFER, 0, sizeof(uint32_t), &zero);

    glUseProgram(m_SimulateShader);
    glUniform1f(glGetUniformLocation(m_SimulateShader, "u_DeltaTime"), dt);
    glUniform2f(glGetUniformLocation(m_SimulateShader, "u_Gravity"), gravity.x, gravity.y);
    glUniform1ui(glGetUniformLocation(m_SimulateShader, "u_TotalParticles"), m_TotalAllocated);

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_ParticleSSBO);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_AliveSSBO);
    glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, 0, m_CounterBuffer);

    // Dispatch enough groups to cover all particles (256 threads per group)
    uint32_t groups = (m_TotalAllocated + 255) / 256;
    glDispatchCompute(groups, 1, 1);

    // Memory barrier: ensure the SSBO writes from compute are visible
    // to the subsequent vertex shader reads
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT);
}
```

The `local_size_x = 256` means each compute shader workgroup processes 256 particles in parallel. For 4 million particles: 4,000,000 / 256 = 15,625 workgroups dispatched in one `glDispatchCompute` call. On a modern GPU with 512 compute units, all 15,625 groups run concurrently — the entire 4M particle simulation takes roughly 0.3–1ms on current hardware.

### 3.3 — Render Pass After Simulation

Instead of reading back the particle positions to the CPU, read them directly from the SSBO in the vertex shader using the alive index list:

```glsl
// engine/assets/shaders/particle_render.vert
#version 460 core

// The unit circle mesh (local space, radius 1.0)
layout(location = 0) in vec2 a_LocalPos;

// The alive indices buffer — one uint per alive particle
layout(std430, binding = 1) readonly buffer AliveBuffer {
    uint aliveIndices[];
};

// The full particle state buffer
layout(std430, binding = 0) readonly buffer ParticleBuffer {
    // (same Particle struct as compute shader)
};

uniform mat4 u_VP;

out vec4 v_Color;
out vec2 v_LocalPos;  // for circle SDF anti-aliasing

void main() {
    // gl_InstanceID = which alive particle we are drawing
    uint particleIdx = aliveIndices[gl_InstanceID];
    // Read this particle's state directly from SSBO
    Particle p = particles[particleIdx];

    // Build per-instance transform
    float c = cos(p.angle);
    float s = sin(p.angle);
    vec2 rotated = vec2(
        a_LocalPos.x * c - a_LocalPos.y * s,
        a_LocalPos.x * s + a_LocalPos.y * c
    );
    vec2 worldPos = p.position + rotated * p.radius;

    gl_Position = u_VP * vec4(worldPos, 0.0, 1.0);
    v_Color     = p.color;   // includes faded alpha
    v_LocalPos  = a_LocalPos;
}
```

```glsl
// engine/assets/shaders/particle_render.frag
#version 460 core

in vec4 v_Color;
in vec2 v_LocalPos;

out vec4 fragColor;

void main() {
    // SDF-based circle: discard fragments outside radius 1.0
    float dist = length(v_LocalPos);
    if (dist > 1.0) discard;

    // Smooth anti-aliased edge using SDF
    float alpha = 1.0 - smoothstep(0.9, 1.0, dist);
    fragColor = vec4(v_Color.rgb, v_Color.a * alpha);
}
```

The fragment shader `discard` at `dist > 1.0` makes the circle perfectly round even though the mesh is a polygon. The `smoothstep` at the edge provides subpixel antialiasing for free.

---

## Phase 4 — GPU-Side Culling with Indirect Draw

At 4 million particles, even the `glDrawArraysInstanced` call iterates through all 4M instances in the vertex shader. If the camera can only see 100,000 particles (the rest are off-screen), you are running the vertex shader 4M times when only 100K produce visible fragments. GPU-side frustum culling removes the off-screen work entirely.

### 4.1 — Culling Compute Shader

This runs before the render pass and compacts alive particles into a draw-indirect buffer. Only particles inside the view frustum (or camera bounds in 2D: a rectangle) are written to the final draw list.

```glsl
// engine/assets/shaders/particle_cull.comp
#version 460 core
layout(local_size_x = 256) in;

// Input: list of alive particle indices (from simulation pass)
layout(std430, binding = 1) readonly buffer AliveBuffer {
    uint aliveIndices[];
};

layout(std430, binding = 0) readonly buffer ParticleBuffer {
    Particle particles[];
};

// Output: indices that passed culling
layout(std430, binding = 2) buffer VisibleBuffer {
    uint visibleIndices[];
};

// DrawArraysIndirectCommand — written by compute, read by GPU draw call
layout(std430, binding = 3) buffer DrawIndirectBuffer {
    uint  count;         // number of instances
    uint  primCount;     // 1 (unused here, repurposed as instance count)
    uint  first;         // 0
    uint  baseInstance;  // 0
};

layout(binding = 0, offset = 0) uniform atomic_uint u_VisibleCount;

// 2D camera bounds in world space
uniform vec2 u_CamMin;   // bottom-left of visible area
uniform vec2 u_CamMax;   // top-right of visible area
uniform uint u_AliveCount;

void main() {
    uint idx = gl_GlobalInvocationID.x;
    if (idx >= u_AliveCount) return;

    uint particleIdx = aliveIndices[idx];
    Particle p = particles[particleIdx];

    // 2D AABB cull: is particle AABB overlapping camera bounds?
    vec2 pMin = p.position - vec2(p.radius);
    vec2 pMax = p.position + vec2(p.radius);
    if (pMax.x < u_CamMin.x || pMin.x > u_CamMax.x ||
        pMax.y < u_CamMin.y || pMin.y > u_CamMax.y) {
        return;  // off-screen, discard
    }

    // Passed cull — append to visible list
    uint slot = atomicCounterIncrement(u_VisibleCount);
    visibleIndices[slot] = particleIdx;
}
```

After this pass, write the visible count into the indirect draw command:

```cpp
// After culling pass, update the indirect draw command count
// This is done with a tiny compute shader or a glCopyBufferSubData trick:
// Read u_VisibleCount from the atomic counter and write it to DrawIndirectBuffer.count
// (Can be done in the same culling shader at the end with a barrier + single thread write.)
```

### 4.2 — Indirect Draw Call

```cpp
// engine/src/graphics/GPUParticleSystem.cpp — Render()

void GPUParticleSystem::Render(const glm::mat4& vp, const Math::Vector2& camMin, const Math::Vector2& camMax) {
    // --- Culling pass ---
    glUseProgram(m_CullShader);
    glUniform2f(glGetUniformLocation(m_CullShader, "u_CamMin"), camMin.x, camMin.y);
    glUniform2f(glGetUniformLocation(m_CullShader, "u_CamMax"), camMax.x, camMax.y);
    glUniform1ui(glGetUniformLocation(m_CullShader, "u_AliveCount"), m_AliveCount);

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_AliveSSBO);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_VisibleSSBO);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, m_IndirectBuffer);
    glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, 0, m_VisibleCounterBuffer);

    glDispatchCompute((m_AliveCount + 255) / 256, 1, 1);
    glMemoryBarrier(GL_COMMAND_BARRIER_BIT | GL_SHADER_STORAGE_BARRIER_BIT);

    // --- Render pass ---
    glUseProgram(m_RenderShader);
    glUniformMatrix4fv(glGetUniformLocation(m_RenderShader, "u_VP"), 1, GL_FALSE, &vp[0][0]);

    glBindVertexArray(m_CircleVAO);
    glBindBuffer(GL_DRAW_INDIRECT_BUFFER, m_IndirectBuffer);

    // The GPU reads instance count from the indirect buffer — CPU never knows the count
    glDrawArraysIndirect(GL_TRIANGLES, nullptr);

    glBindVertexArray(0);
    glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
}
```

`glDrawArraysIndirect` reads its parameters (instance count, vertex count) directly from a GPU buffer. The CPU never needs to read back the visible count. The entire path — simulate → cull → render — runs on the GPU without any CPU involvement or synchronization.

---

## Phase 5 — Spatial Hashing on GPU for Collision Broadphase

Once you have millions of particles moving on the GPU, you may want cheap particle-particle collision (e.g. for fluid, crowd, or sand simulations). The existing `DynamicTree` broadphase runs entirely on the CPU and cannot handle millions of particles. GPU spatial hashing solves this.

### 5.1 — Concept

Divide the world into a uniform grid of cells. For each particle, compute which cell it occupies via:

```glsl
ivec2 cell = ivec2(floor(position / cellSize));
uint cellHash = (cell.x * 73856093u) ^ (cell.y * 19349663u);
uint bucket   = cellHash % NUM_BUCKETS;
```

This is a hash function. Use it to sort particles into buckets in a single compute pass, then in a second pass check only particles in the same or neighboring buckets for collision.

### 5.2 — Three-pass Algorithm

**Pass 1 — Count:** Each particle writes its bucket ID. Count how many particles fall in each bucket.

```glsl
// particle_hash_count.comp
layout(local_size_x = 256) in;

layout(std430, binding = 0) readonly buffer ParticleBuffer { Particle particles[]; };
layout(std430, binding = 1) writeonly buffer BucketBuffer  { uint bucketIds[]; };
layout(std430, binding = 2) buffer CountBuffer             { uint counts[]; };  // per-bucket count

uniform float u_CellSize;
uniform uint  u_NumBuckets;
uniform uint  u_NumParticles;

void main() {
    uint idx = gl_GlobalInvocationID.x;
    if (idx >= u_NumParticles) return;
    vec2 pos = particles[idx].position;
    ivec2 cell = ivec2(floor(pos / u_CellSize));
    uint hash = (uint(cell.x) * 73856093u) ^ (uint(cell.y) * 19349663u);
    uint bucket = hash % u_NumBuckets;
    bucketIds[idx] = bucket;
    atomicAdd(counts[bucket], 1u);
}
```

**Pass 2 — Prefix Sum:** Convert per-bucket counts to start offsets. This can be done with a GPU parallel prefix sum (scan) algorithm, or for simplicity, a CPU readback of the count buffer followed by `std::exclusive_scan` for up to 65536 buckets.

**Pass 3 — Scatter:** Write each particle into its bucket slot using the prefix offsets.

**Pass 4 — Collision Check:** For each alive particle, check all 9 neighboring cells (3×3 grid). For each neighbor particle in those buckets, test distance < r1 + r2 and compute collision response impulse.

```glsl
// particle_collide.comp
layout(local_size_x = 256) in;
// (bindings to ParticleBuffer, BucketBuffer, OffsetBuffer, CountBuffer)

void main() {
    uint idx = gl_GlobalInvocationID.x;
    if (idx >= u_NumParticles) return;
    Particle self = particles[idx];

    ivec2 myCell = ivec2(floor(self.position / u_CellSize));

    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            ivec2 neighborCell = myCell + ivec2(dx, dy);
            uint hash   = (uint(neighborCell.x) * 73856093u) ^ (uint(neighborCell.y) * 19349663u);
            uint bucket = hash % u_NumBuckets;
            uint start  = offsets[bucket];
            uint end    = start + counts[bucket];

            for (uint j = start; j < end; ++j) {
                uint otherIdx = sortedIndices[j];
                if (otherIdx == idx) continue;
                Particle other = particles[otherIdx];
                vec2 delta = other.position - self.position;
                float dist = length(delta);
                float minDist = self.radius + other.radius;
                if (dist < minDist && dist > 0.001) {
                    vec2 normal = delta / dist;
                    float overlap = minDist - dist;
                    // Simple elastic impulse for equal masses
                    float dvn = dot(other.velocity - self.velocity, normal);
                    if (dvn < 0.0) {
                        float impulse = dvn * 0.5;  // restitution = 0.5
                        particles[idx].velocity   += normal * impulse;
                        // Note: two particles writing each other's velocities causes races.
                        // Use a two-pass read-accumulate-write pattern or accept the race
                        // as an approximation (common in GPU particle systems).
                    }
                    // Positional correction
                    particles[idx].position -= normal * (overlap * 0.5);
                }
            }
        }
    }
}
```

For a spatially uniform distribution of 4M particles in a 1280×720 world with cell size = 10px: 128×72 = 9216 cells, average ~434 particles per cell. The inner loop checks at most 9 × 434 = 3906 pairs per particle. This is extremely cache-friendly when particles are sorted by bucket.

---

## Shader Code Reference

### Circle Vertex Shader (Phase 1 — Instanced, CPU-driven instances)

```glsl
// engine/assets/shaders/circle_instanced.vert
#version 460 core

layout(location = 0) in vec2 a_LocalPos;       // unit circle mesh
layout(location = 1) in vec2 a_InstPos;         // instance: world position
layout(location = 2) in vec2 a_InstAngleRadius; // instance: (angle, radius)
layout(location = 3) in vec3 a_InstColor;        // instance: rgb
layout(location = 4) in float a_InstAspect;      // instance: aspect ratio

uniform mat4 u_VP;

out vec3 v_Color;
out vec2 v_LocalPos;

void main() {
    float angle  = a_InstAngleRadius.x;
    float radius = a_InstAngleRadius.y;

    float c = cos(angle);
    float s = sin(angle);
    // Scale local pos by radius and aspect, then rotate, then translate
    vec2 scaled = vec2(a_LocalPos.x * radius * a_InstAspect,
                       a_LocalPos.y * radius);
    vec2 rotated = vec2(scaled.x * c - scaled.y * s,
                        scaled.x * s + scaled.y * c);
    vec2 world = a_InstPos + rotated;

    gl_Position = u_VP * vec4(world, 0.0, 1.0);
    v_Color     = a_InstColor;
    v_LocalPos  = a_LocalPos;
}
```

### Circle Fragment Shader (SDF anti-aliasing)

```glsl
// engine/assets/shaders/circle_instanced.frag
#version 460 core

in vec3 v_Color;
in vec2 v_LocalPos;
out vec4 fragColor;

void main() {
    float d     = length(v_LocalPos);
    if (d > 1.0) discard;
    float alpha = 1.0 - smoothstep(0.85, 1.0, d);
    // Optional: add specular highlight for 3D look
    float light = 1.0 - 0.3 * d;
    fragColor = vec4(v_Color * light, alpha);
}
```

### Quad Vertex Shader (instanced with rotation)

```glsl
// engine/assets/shaders/quad_instanced.vert
#version 460 core

layout(location = 0) in vec2 a_LocalPos;       // unit quad: (-1,-1) to (1,1)
layout(location = 1) in vec2 a_InstPos;
layout(location = 2) in vec2 a_InstAngleRadius;
layout(location = 3) in vec3 a_InstColor;
layout(location = 4) in float a_InstAspect;

uniform mat4 u_VP;
out vec3 v_Color;

void main() {
    float angle  = a_InstAngleRadius.x;
    float radius = a_InstAngleRadius.y;

    float c = cos(angle); float s = sin(angle);
    vec2 scaled  = vec2(a_LocalPos.x * radius * a_InstAspect, a_LocalPos.y * radius);
    vec2 rotated = vec2(scaled.x*c - scaled.y*s, scaled.x*s + scaled.y*c);

    gl_Position = u_VP * vec4(a_InstPos + rotated, 0.0, 1.0);
    v_Color = a_InstColor;
}
```

---

## Integration with Existing ECS

### Where to plug in

The existing render flow is:

```
ECSApplication::OnInterpolateAndRender(alpha)
  → m_RenderSystem->Update(alpha)   (iterates components, calls Renderer2D::DrawQuad/DrawCircle)
  → m_DebugRenderSystem->Update()
```

The cleanest integration without breaking the existing `RenderSystem` is to add a `GPUParticleSystem` alongside it:

```cpp
// ECSApplication.h — ADDED
std::unique_ptr<Graphics::GPUParticleSystem> m_GPUParticleSystem;
std::unique_ptr<Graphics::ParticleRenderer>  m_ParticleRenderer;
```

```cpp
// ECSApplication.cpp — OnStart()
m_ParticleRenderer  = std::make_unique<Graphics::ParticleRenderer>();
m_ParticleRenderer->Init();
m_GPUParticleSystem = std::make_unique<Graphics::GPUParticleSystem>();
m_GPUParticleSystem->Init(4'000'000);
```

```cpp
// ECSApplication.cpp — OnFixedUpdate() — physics tick
// Run GPU particle simulation at the same fixed timestep as physics
m_GPUParticleSystem->Simulate(deltaTime, world.gravity);
```

```cpp
// ECSApplication.cpp — OnInterpolateAndRender()
// 1. Draw rigid bodies (existing — CPU generated, small count, fine as-is)
m_RenderSystem->Update(alpha);

// 2. Submit GPU particle render (after physics bodies, before debug)
glm::mat4 vp = /* from camera */;
Math::Vector2 camMin{0.0f, 0.0f};
Math::Vector2 camMax{1280.0f, 720.0f};
m_GPUParticleSystem->Render(vp, camMin, camMax);

// 3. Debug overlay (existing)
m_DebugRenderSystem->Update();
```

The existing `Renderer2D` continues to handle rigid body rendering (quads, polygons, circles for your ~10-100 rigid bodies). The `GPUParticleSystem` handles visual-only particles (sparks, dust, debris, fluid) at millions of instances. These two pipelines are fully independent.

### Spawning Particles from C++ (CPU → GPU)

When a collision event occurs, spawn visual particles from the CPU:

```cpp
// In PhysicsPipelineSystem or a new ParticleSpawnSystem:

void SpawnCollisionSparks(const ContactManifold& manifold,
                          GPUParticleSystem& gps) {
    const Math::Vector2& cp = manifold.points[0].position;
    constexpr int SPARK_COUNT = 32;

    for (int i = 0; i < SPARK_COUNT; ++i) {
        float angle = (i / float(SPARK_COUNT)) * 2.0f * 3.14159f;
        float speed = 100.0f + static_cast<float>(rand() % 200);
        GPUParticle p{};
        p.px = cp.x;  p.py = cp.y;
        p.vx = std::cos(angle) * speed;
        p.vy = std::sin(angle) * speed;
        p.r = 1.0f;  p.g = 0.8f;  p.b = 0.2f;  p.a = 1.0f;
        p.lifetime    = 0.3f + static_cast<float>(rand() % 100) / 200.0f;
        p.maxLifetime = p.lifetime;
        p.radius      = 2.0f + static_cast<float>(rand() % 4);
        p.angularVel  = (rand() % 10 - 5) * 2.0f;
        gps.SpawnParticle(p);
    }
}
```

`GPUParticleSystem::SpawnParticle()` writes into a CPU-side spawn queue. At the beginning of the next `Simulate()` call, the spawn queue is uploaded to the GPU SSBO via `glBufferSubData` into free (dead) particle slots. This is the only CPU→GPU transfer in the hot path.

---

## Memory Layout and Budget

| Buffer | Count | Per Element | Total |
|--------|-------|-------------|-------|
| `ParticleSSBO` (positions, velocities, color, lifetime) | 4,000,000 | 64 bytes | 256 MB |
| `AliveSSBO` (indices of alive particles) | 4,000,000 | 4 bytes | 16 MB |
| `VisibleSSBO` (indices that passed culling) | 4,000,000 | 4 bytes | 16 MB |
| `DrawIndirectBuffer` | 1 command | 16 bytes | negligible |
| `CircleInstVBO` (persistent, triple-buffered) | 4M × 3 | 32 bytes | 384 MB (shared with SSBO path; use one or the other) |
| `SpatialHash CountBuffer` | 65,536 buckets | 4 bytes | 256 KB |
| `SpatialHash OffsetBuffer` | 65,536 buckets | 4 bytes | 256 KB |
| `SortedIndices` | 4,000,000 | 4 bytes | 16 MB |

**Recommended budget for most setups:** Use 1M particles (not 4M) for the SSBO path — this brings total GPU memory usage to ~80 MB, which fits comfortably in 4 GB VRAM alongside framebuffers and other scene data.

**GPU frame time budget (estimated for RTX 3060 class GPU):**

| Pass | ~Time |
|------|-------|
| Simulate compute (1M particles, 256 threads/group) | ~0.4ms |
| Cull compute | ~0.1ms |
| Spatial hash (3 passes) | ~0.8ms |
| Collision compute | ~1.5ms |
| Render (instanced draw, 16-segment circles) | ~0.6ms |
| **Total** | **~3.4ms** (leaves ~12.6ms for rigid bodies and other rendering at 60fps) |

---

## Build Order

Implement in this order. Each phase produces visible improvement and is independently testable:

| Phase | Deliverable | Benefit |
|-------|-------------|---------|
| 1 | `ParticleRenderer` with instanced draw, CPU-side instance buffer | ~50–100× fewer CPU cycles for rendering. Drop-in replacement for `DrawSolidCircle` / `DrawQuad`. |
| 2 | Persistent buffer mapping, triple-buffering, fences | Eliminates PCIe stalls. Enables smooth 100K+ particle updates. |
| 3 | `GPUParticleSystem` + simulation compute shader | Moves all simple particle physics off the CPU. Enables 1M+ visual particles. |
| 4 | Culling compute shader + `glDrawArraysIndirect` | Ensures only visible particles consume vertex shader invocations. |
| 5 | Spatial hash broadphase compute | Enables particle-particle interaction (fluid, sand, crowd) at GPU scale. |

Phase 1 alone will produce a dramatic visible improvement for your current demo. Phases 2–4 unlock the million-particle target. Phase 5 is only needed if you want particle-particle physics rather than just visual particles.

---

## New Files Summary (GPU Rendering)

| File | Purpose |
|------|---------|
| `engine/include/nyon/graphics/ParticleRenderer.h` | Phase 1–2: instanced renderer class definition |
| `engine/src/graphics/ParticleRenderer.cpp` | Phase 1–2: implementation |
| `engine/include/nyon/graphics/GPUParticleSystem.h` | Phase 3–5: GPU particle system class |
| `engine/src/graphics/GPUParticleSystem.cpp` | Phase 3–5: implementation |
| `engine/assets/shaders/circle_instanced.vert` | Instanced circle vertex shader |
| `engine/assets/shaders/circle_instanced.frag` | Circle SDF fragment shader |
| `engine/assets/shaders/quad_instanced.vert` | Instanced quad vertex shader |
| `engine/assets/shaders/particle_simulate.comp` | Simulation compute shader |
| `engine/assets/shaders/particle_cull.comp` | Frustum culling compute shader |
| `engine/assets/shaders/particle_render.vert` | SSBO-driven render vertex shader |
| `engine/assets/shaders/particle_render.frag` | Particle fragment shader |
| `engine/assets/shaders/particle_hash_count.comp` | Spatial hash count pass |
| `engine/assets/shaders/particle_collide.comp` | Particle-particle collision compute |

---

---

# Renderer2D CPU-to-GPU Migration Plan

**Scope:** Drop-in GPU replacement for `Renderer2D`. The **public API in `Renderer2D.h` does not change at all**. No call sites in `RenderSystem`, `DebugRenderSystem`, `PhysicsDebugRenderer`, or the demo change. You only rewrite the `Impl` struct internals and the static method bodies inside `Renderer2D.cpp`. Every `DrawQuad`, `DrawSolidCircle`, `DrawLine`, `DrawCapsule`, etc. call continues to work identically from the outside.

---

## Why This Is the Right Approach

The `Renderer2D` uses a clean pimpl pattern. The public class in the header owns nothing except a `static std::unique_ptr<Impl> s_Instance`. The entire implementation is hidden inside the `Impl` struct in `Renderer2D.cpp`. This means the entire GPU rewrite is **contained to one file**. You do not need to touch any header, any system, any ECS component, or any demo code.

The constraint to solve is: the static methods like `DrawQuad(position, size, origin, color, rotation)` cannot change their signature. The GPU rewrite must accept those same arguments and route them into GPU-efficient batches internally.

---

## Current Bottleneck Map (Per Draw Call)

Before designing the fix, here is exactly what the current code does inside `Renderer2D.cpp` for each call type, and how much CPU work each one performs:

| Call | Current CPU work per call | Vertices pushed |
|------|--------------------------|-----------------|
| `DrawQuad` | 4× `cos/sin`, 4 corner rotations, 6 struct writes | 6 × 40 bytes = 240 bytes |
| `DrawSolidCircle` (32 seg) | 32× `cos/sin` pairs, 96 struct writes | 96 × 40 bytes = 3,840 bytes |
| `DrawCircle` (32 seg) | 64× `cos/sin`, 64 struct writes | 64 × 40 bytes = 2,560 bytes |
| `DrawLine` (thick) | 4 vector ops, calls `DrawSolidPolygon` | 6 × 40 bytes = 240 bytes |
| `DrawLine` (thin) | 2 struct writes | 2 × 40 bytes = 80 bytes |
| `DrawSolidPolygon` (N verts) | (N-2) triangle fans | (N-2)×3 × 40 bytes |
| `DrawSolidCapsule` (16 seg) | 32× `cos/sin` + 6 rect verts | (96 + 6) × 40 bytes = 4,080 bytes |
| `DrawSolidCapsule` (16 seg) | 32× `cos/sin` + 6 rect verts | (96 + 6) × 40 bytes ≈ 4,080 bytes |
| `FlushBuffer` (each flush) | `glBufferData(GL_DYNAMIC_DRAW)` — **full re-allocation** | Entire buffer re-uploaded |

The `glBufferData` call inside `FlushBuffer` is particularly destructive. It calls `GL_DYNAMIC_DRAW` which tells the driver to allocate a new GPU buffer each call, copy data in, and orphan the old allocation. For 10,000 draw calls per frame this creates 10,000 driver-side allocations.

---

## Migration Strategy: Three Tiers

Not every draw call benefits equally from instancing. The migration is split into three tiers prioritized by return on investment:

**Tier 1 — Full instancing (maximum gain, implement first):**
- `DrawSolidCircle` / `DrawCircle` — currently the most expensive per call
- `DrawQuad` — used for every rigid body and most game objects

**Tier 2 — Instanced line quads (good gain):**
- `DrawLine` / `DrawSegment` — converts both thin and thick lines into a single instanced path
- `DrawSolidCapsule` / `DrawCapsule` — eliminates all per-capsule `cos/sin` loops

**Tier 3 — Persistent mapped polygon buffer (modest gain, keeps CPU tessellation):**
- `DrawSolidPolygon` / `DrawPolygon` — arbitrary convex polygons cannot be instanced with a fixed mesh; keep CPU fan tessellation but eliminate `glBufferData` re-allocation by using a persistent mapped buffer
- `DrawArc`, `DrawSector`, `DrawEllipse`, `DrawSolidEllipse`, `DrawSolidSector` — infrequent debug shapes; same persistent buffer treatment

---

## New `Impl` Structure

Replace the entire `Renderer2D::Impl` struct. The new version has one instance buffer and one VAO per shape type.

```cpp
// Renderer2D.cpp — new Impl struct (replaces the existing one entirely)

struct Renderer2D::Impl {
    // -----------------------------------------------------------------------
    // Shared state (unchanged from original)
    // -----------------------------------------------------------------------
    bool Initialized    = false;
    bool GLAvailable    = false;
    bool BlendingEnabled  = true;
    bool DepthTestEnabled = false;
    bool CullingEnabled   = false;
    float CurrentLineWidth = 1.0f;
    Camera2D CurrentCamera;
    glm::mat4 ViewMatrix       = glm::mat4(1.0f);
    glm::mat4 ProjectionMatrix = glm::mat4(1.0f);

    // -----------------------------------------------------------------------
    // Per-instance data structs — tightly packed, GPU-aligned
    // -----------------------------------------------------------------------

    // 40 bytes — fits in one cache line with the next field
    struct alignas(8) QuadInstance {
        float px, py;       // world position (pivot)
        float sx, sy;       // size
        float ox, oy;       // origin offset
        float angle;        // rotation in radians
        float r, g, b;      // color
    };
    static_assert(sizeof(QuadInstance) == 40);

    // 28 bytes
    struct alignas(4) CircleInstance {
        float cx, cy;       // center
        float radius;
        float r, g, b;      // color
        float outlined;     // 0.0 = filled, 1.0 = outline ring
    };
    static_assert(sizeof(CircleInstance) == 28);

    // 32 bytes
    struct alignas(8) LineInstance {
        float x0, y0;       // start point
        float x1, y1;       // end point
        float r, g, b;      // color
        float thickness;    // world-space thickness
    };
    static_assert(sizeof(LineInstance) == 32);

    // 36 bytes
    struct alignas(4) CapsuleInstance {
        float cx0, cy0;     // center1
        float cx1, cy1;     // center2
        float radius;
        float r, g, b;      // color
        float outlined;     // 0.0 = filled, 1.0 = outline
    };
    static_assert(sizeof(CapsuleInstance) == 36);

    // -----------------------------------------------------------------------
    // Tier 1: Quad GPU objects
    // -----------------------------------------------------------------------
    GLuint QuadMeshVBO = 0;   // static unit quad mesh (6 verts, uploaded once)
    GLuint QuadInstVBO = 0;   // instance buffer (persistently mapped)
    GLuint QuadVAO     = 0;
    GLuint QuadShader  = 0;
    GLint  QuadVP_Loc  = -1;

    QuadInstance* QuadInstPtr   = nullptr;  // persistent map pointer
    uint32_t      QuadInstCount = 0;
    static constexpr uint32_t MAX_QUADS = 131072;  // 128K quads = ~5MB

    // -----------------------------------------------------------------------
    // Tier 1: Circle GPU objects
    // -----------------------------------------------------------------------
    GLuint CircleMeshVBO = 0;  // static circle triangle fan (16 tris × 3 = 48 verts)
    GLuint CircleInstVBO = 0;
    GLuint CircleVAO     = 0;
    GLuint CircleShader  = 0;
    GLint  CircleVP_Loc  = -1;

    CircleInstance* CircleInstPtr   = nullptr;
    uint32_t        CircleInstCount = 0;
    static constexpr uint32_t MAX_CIRCLES       = 524288;  // 512K circles = ~14MB
    static constexpr int      CIRCLE_SEG        = 16;      // 16-sided mesh; SDF smooths the edge

    // -----------------------------------------------------------------------
    // Tier 2: Line GPU objects
    // -----------------------------------------------------------------------
    GLuint LineMeshVBO = 0;   // static unit rectangle (6 verts: two triangles)
    GLuint LineInstVBO = 0;
    GLuint LineVAO     = 0;
    GLuint LineShader  = 0;
    GLint  LineVP_Loc  = -1;

    LineInstance* LineInstPtr   = nullptr;
    uint32_t      LineInstCount = 0;
    static constexpr uint32_t MAX_LINES = 262144;  // 256K lines = ~8MB

    // -----------------------------------------------------------------------
    // Tier 2: Capsule GPU objects
    // -----------------------------------------------------------------------
    GLuint CapsuleMeshVBO = 0;  // static unit capsule mesh (generated once)
    GLuint CapsuleInstVBO = 0;
    GLuint CapsuleVAO     = 0;
    GLuint CapsuleShader  = 0;
    GLint  CapsuleVP_Loc  = -1;

    CapsuleInstance* CapsuleInstPtr   = nullptr;
    uint32_t         CapsuleInstCount = 0;
    static constexpr uint32_t MAX_CAPSULES   = 65536;
    static constexpr int      CAPSULE_SEG    = 8;

    // -----------------------------------------------------------------------
    // Tier 3: Polygon passthrough buffer (CPU fan, persistent map)
    // -----------------------------------------------------------------------
    // Two separate buffers: filled triangles (GL_TRIANGLES) and outline (GL_LINES)
    GLuint PolyFillVAO = 0, PolyFillVBO = 0;
    GLuint PolyLineVAO = 0, PolyLineVBO = 0;
    GLuint PolyShader  = 0;   // simple passthrough, same as original shader
    GLint  PolyVP_Loc  = -1;

    // Persistently mapped: CPU writes world-space Vertex structs directly here
    // We keep the Vertex struct for polygons since they're already tessellated in world space
    Vertex*   PolyFillPtr   = nullptr;
    Vertex*   PolyLinePtr   = nullptr;
    uint32_t  PolyFillCount = 0;
    uint32_t  PolyLineCount = 0;
    static constexpr uint32_t MAX_POLY_VERTS = 65536;  // 64K polygon vertices each

    // -----------------------------------------------------------------------
    // Triple-buffer fences (one per buffer type, cycling index 0/1/2)
    // -----------------------------------------------------------------------
    static constexpr int NUM_FRAMES = 3;
    int   CurrentFrame = 0;
    GLsync QuadFences[NUM_FRAMES]    = {};
    GLsync CircleFences[NUM_FRAMES]  = {};
    GLsync LineFences[NUM_FRAMES]    = {};
    GLsync CapsuleFences[NUM_FRAMES] = {};
    GLsync PolyFences[NUM_FRAMES]    = {};

    // Helper: wait for the current frame's fence then advance
    void WaitFence(GLsync* fences, int idx) {
        if (fences[idx]) {
            glClientWaitSync(fences[idx], GL_SYNC_FLUSH_COMMANDS_BIT, 100'000'000);
            glDeleteSync(fences[idx]);
            fences[idx] = nullptr;
        }
    }
    void PlaceFence(GLsync* fences, int idx) {
        fences[idx] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    }

    // -----------------------------------------------------------------------
    // Internal helpers (shader compile, etc.) — same as original
    // -----------------------------------------------------------------------
    GLuint CompileShader(GLenum type, const char* src);
    GLuint CreateProgram(const char* vsSrc, const char* fsSrc);
    bool   CheckGLFunctionsLoaded();

    // -----------------------------------------------------------------------
    // Setup methods (called once in Init)
    // -----------------------------------------------------------------------
    void SetupQuadPipeline();
    void SetupCirclePipeline();
    void SetupLinePipeline();
    void SetupCapsulePipeline();
    void SetupPolygonPipeline();

    // Helper: allocate a persistently mapped SSBO/VBO triple-buffer
    // Returns the base pointer. The full buffer is NUM_FRAMES × singleSize bytes.
    void* AllocatePersistentBuffer(GLuint& vbo, GLsizeiptr singleSize);

    // View/projection update (same as original)
    void UpdateProjectionMatrix(float w, float h);
    void UpdateViewMatrix();
};
```

---

## Persistent Buffer Helper

Every instance buffer uses the same allocation pattern. Extract it into a helper so each pipeline setup calls it once:

```cpp
// Renderer2D.cpp

void* Renderer2D::Impl::AllocatePersistentBuffer(GLuint& vbo, GLsizeiptr singleSize) {
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    const GLbitfield flags = GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT;
    // Allocate NUM_FRAMES slots so triple-buffering never stalls
    glBufferStorage(GL_ARRAY_BUFFER, singleSize * NUM_FRAMES, nullptr, flags);

    void* ptr = glMapBufferRange(GL_ARRAY_BUFFER, 0, singleSize * NUM_FRAMES, flags);
    return ptr;  // stay mapped forever; unmap only in Shutdown
}
```

---

## Pipeline Setup: Quads (Tier 1)

```cpp
void Renderer2D::Impl::SetupQuadPipeline() {
    // --- Static mesh: unit quad with normalized UVs (0,0)→(1,1) ---
    // These represent local-space corner positions before instance transform.
    // X maps to size.x, Y maps to size.y; origin offset is applied in VS.
    const float mesh[] = {
        0.0f, 0.0f,   1.0f, 0.0f,   1.0f, 1.0f,   // triangle 0
        0.0f, 0.0f,   1.0f, 1.0f,   0.0f, 1.0f,   // triangle 1
    };
    glGenBuffers(1, &QuadMeshVBO);
    glBindBuffer(GL_ARRAY_BUFFER, QuadMeshVBO);
    glBufferStorage(GL_ARRAY_BUFFER, sizeof(mesh), mesh, 0);  // immutable

    // --- VAO ---
    glGenVertexArrays(1, &QuadVAO);
    glBindVertexArray(QuadVAO);

    // Attrib 0: local XY from mesh VBO — one per vertex, no divisor
    glBindBuffer(GL_ARRAY_BUFFER, QuadMeshVBO);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // --- Instance buffer (persistently mapped, triple-buffered) ---
    QuadInstPtr = static_cast<QuadInstance*>(
        AllocatePersistentBuffer(QuadInstVBO, MAX_QUADS * sizeof(QuadInstance))
    );

    glBindBuffer(GL_ARRAY_BUFFER, QuadInstVBO);
    const GLsizei stride = sizeof(QuadInstance);

    // Attrib 1: instance position (px, py) — divisor 1
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(QuadInstance, px));
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);

    // Attrib 2: instance size (sx, sy)
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(QuadInstance, sx));
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2, 1);

    // Attrib 3: instance origin (ox, oy)
    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(QuadInstance, ox));
    glEnableVertexAttribArray(3);
    glVertexAttribDivisor(3, 1);

    // Attrib 4: instance angle
    glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(QuadInstance, angle));
    glEnableVertexAttribArray(4);
    glVertexAttribDivisor(4, 1);

    // Attrib 5: instance color (r, g, b)
    glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(QuadInstance, r));
    glEnableVertexAttribArray(5);
    glVertexAttribDivisor(5, 1);

    glBindVertexArray(0);

    // --- Shader ---
    QuadShader = CreateProgram(QUAD_VERT_SRC, QUAD_FRAG_SRC);
    QuadVP_Loc = glGetUniformLocation(QuadShader, "u_VP");
}
```

---

## Pipeline Setup: Circles (Tier 1)

```cpp
void Renderer2D::Impl::SetupCirclePipeline() {
    // --- Static mesh: triangle fan for unit circle ---
    // 16 segments × 3 vertices = 48 vertices, each just a vec2 local position.
    // The fragment shader uses SDF to make the circle perfectly round.
    std::vector<float> mesh;
    mesh.reserve(CIRCLE_SEG * 3 * 2);
    const float step = 2.0f * 3.14159265f / CIRCLE_SEG;
    for (int i = 0; i < CIRCLE_SEG; ++i) {
        mesh.push_back(0.0f); mesh.push_back(0.0f);  // center
        mesh.push_back(std::cos(step * i));      mesh.push_back(std::sin(step * i));
        mesh.push_back(std::cos(step * (i+1)));  mesh.push_back(std::sin(step * (i+1)));
    }

    glGenBuffers(1, &CircleMeshVBO);
    glBindBuffer(GL_ARRAY_BUFFER, CircleMeshVBO);
    glBufferStorage(GL_ARRAY_BUFFER, mesh.size() * sizeof(float), mesh.data(), 0);

    glGenVertexArrays(1, &CircleVAO);
    glBindVertexArray(CircleVAO);

    // Attrib 0: local XY from mesh (local position on unit circle)
    glBindBuffer(GL_ARRAY_BUFFER, CircleMeshVBO);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Instance buffer
    CircleInstPtr = static_cast<CircleInstance*>(
        AllocatePersistentBuffer(CircleInstVBO, MAX_CIRCLES * sizeof(CircleInstance))
    );
    glBindBuffer(GL_ARRAY_BUFFER, CircleInstVBO);
    const GLsizei stride = sizeof(CircleInstance);

    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(CircleInstance, cx));
    glEnableVertexAttribArray(1); glVertexAttribDivisor(1, 1);  // center xy

    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(CircleInstance, radius));
    glEnableVertexAttribArray(2); glVertexAttribDivisor(2, 1);  // radius

    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(CircleInstance, r));
    glEnableVertexAttribArray(3); glVertexAttribDivisor(3, 1);  // color rgb

    glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(CircleInstance, outlined));
    glEnableVertexAttribArray(4); glVertexAttribDivisor(4, 1);  // 0=filled 1=outline

    glBindVertexArray(0);
    CircleShader = CreateProgram(CIRCLE_VERT_SRC, CIRCLE_FRAG_SRC);
    CircleVP_Loc = glGetUniformLocation(CircleShader, "u_VP");
}
```

---

## Pipeline Setup: Lines (Tier 2)

```cpp
void Renderer2D::Impl::SetupLinePipeline() {
    // Unit rectangle: X = 0..1 (along line direction), Y = -0.5..+0.5 (across thickness)
    // The vertex shader stretches X to line length and scales Y by thickness.
    const float mesh[] = {
        0.0f, -0.5f,   1.0f, -0.5f,   1.0f,  0.5f,   // tri 0
        0.0f, -0.5f,   1.0f,  0.5f,   0.0f,  0.5f,   // tri 1
    };
    glGenBuffers(1, &LineMeshVBO);
    glBindBuffer(GL_ARRAY_BUFFER, LineMeshVBO);
    glBufferStorage(GL_ARRAY_BUFFER, sizeof(mesh), mesh, 0);

    glGenVertexArrays(1, &LineVAO);
    glBindVertexArray(LineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, LineMeshVBO);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    LineInstPtr = static_cast<LineInstance*>(
        AllocatePersistentBuffer(LineInstVBO, MAX_LINES * sizeof(LineInstance))
    );
    glBindBuffer(GL_ARRAY_BUFFER, LineInstVBO);
    const GLsizei stride = sizeof(LineInstance);

    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(LineInstance, x0));
    glEnableVertexAttribArray(1); glVertexAttribDivisor(1, 1);  // start point

    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(LineInstance, x1));
    glEnableVertexAttribArray(2); glVertexAttribDivisor(2, 1);  // end point

    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(LineInstance, r));
    glEnableVertexAttribArray(3); glVertexAttribDivisor(3, 1);  // color

    glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(LineInstance, thickness));
    glEnableVertexAttribArray(4); glVertexAttribDivisor(4, 1);  // thickness

    glBindVertexArray(0);
    LineShader = CreateProgram(LINE_VERT_SRC, LINE_FRAG_SRC);
    LineVP_Loc = glGetUniformLocation(LineShader, "u_VP");
}
```

---

## Rewriting the Static Draw Methods

These are the only code changes visible outside `Impl`. They live in the existing static method bodies.

### `DrawQuad` (was: 6 CPU vertex writes + `cos/sin`)

```cpp
// Renderer2D.cpp — DrawQuad — AFTER
void Renderer2D::DrawQuad(const Math::Vector2& position,
                          const Math::Vector2& size,
                          const Math::Vector2& origin,
                          const Math::Vector3& color,
                          float rotation) {
    if (!s_Instance || !s_Instance->Initialized) return;
    if (s_Instance->QuadInstCount >= Impl::MAX_QUADS) return;

    // Write one instance into the persistently mapped buffer.
    // The write pointer already points to this frame's slot.
    Impl::QuadInstance& inst = s_Instance->QuadInstPtr[
        s_Instance->CurrentFrame * Impl::MAX_QUADS + s_Instance->QuadInstCount++
    ];
    inst.px = position.x;  inst.py = position.y;
    inst.sx = size.x;      inst.sy = size.y;
    inst.ox = origin.x;    inst.oy = origin.y;
    inst.angle = rotation;
    inst.r = color.x;  inst.g = color.y;  inst.b = color.z;
}
```

That is the entire new `DrawQuad`. One struct fill. No `cos`, no `sin`, no loop, no vector push. The vertex shader now does the rotation.

### `DrawSolidCircle` and `DrawCircle` (was: 96 + 64 CPU vertex writes)

```cpp
void Renderer2D::DrawSolidCircle(const Math::Vector2& center, float radius,
                                  const Math::Vector3& color, int /*segments*/) {
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    if (s_Instance->CircleInstCount >= Impl::MAX_CIRCLES) return;

    Impl::CircleInstance& inst = s_Instance->CircleInstPtr[
        s_Instance->CurrentFrame * Impl::MAX_CIRCLES + s_Instance->CircleInstCount++
    ];
    inst.cx = center.x;  inst.cy = center.y;
    inst.radius = radius;
    inst.r = color.x;  inst.g = color.y;  inst.b = color.z;
    inst.outlined = 0.0f;  // filled
}

void Renderer2D::DrawCircle(const Math::Vector2& center, float radius,
                             const Math::Vector3& color, int /*segments*/) {
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    if (s_Instance->CircleInstCount >= Impl::MAX_CIRCLES) return;

    Impl::CircleInstance& inst = s_Instance->CircleInstPtr[
        s_Instance->CurrentFrame * Impl::MAX_CIRCLES + s_Instance->CircleInstCount++
    ];
    inst.cx = center.x;  inst.cy = center.y;
    inst.radius = radius;
    inst.r = color.x;  inst.g = color.y;  inst.b = color.z;
    inst.outlined = 1.0f;  // outline ring via SDF in frag shader
}
```

Note the `int segments` parameter is accepted but ignored. The GPU circle uses a fixed 16-sided mesh that the SDF fragment shader rounds perfectly regardless. The caller's `segments` argument continues to compile without warnings — it is just not used.

### `DrawLine` (was: two separate paths — thin GL_LINES push, thick calls DrawSolidPolygon)

```cpp
void Renderer2D::DrawLine(const Math::Vector2& start, const Math::Vector2& end,
                           const Math::Vector3& color, float thickness) {
    if (!s_Instance || !s_Instance->Initialized) return;
    if (s_Instance->LineInstCount >= Impl::MAX_LINES) return;

    // Clamp minimum thickness so very thin lines still have a 1px wide quad
    const float t = std::max(thickness, 1.0f);

    Impl::LineInstance& inst = s_Instance->LineInstPtr[
        s_Instance->CurrentFrame * Impl::MAX_LINES + s_Instance->LineInstCount++
    ];
    inst.x0 = start.x;  inst.y0 = start.y;
    inst.x1 = end.x;    inst.y1 = end.y;
    inst.r = color.x;  inst.g = color.y;  inst.b = color.z;
    inst.thickness = t;
}
```

The two-path split (thin vs. thick) is completely gone. Both go through the same instanced line quad. The vertex shader handles all thicknesses uniformly.

### `DrawSolidCapsule` and `DrawCapsule` (was: 100+ vertex writes per call)

```cpp
void Renderer2D::DrawSolidCapsule(const Math::Vector2& c1, const Math::Vector2& c2,
                                   float radius, const Math::Vector3& color, int /*seg*/) {
    if (!s_Instance || !s_Instance->Initialized || radius <= 0.0f) return;
    if (s_Instance->CapsuleInstCount >= Impl::MAX_CAPSULES) return;

    Impl::CapsuleInstance& inst = s_Instance->CapsuleInstPtr[
        s_Instance->CurrentFrame * Impl::MAX_CAPSULES + s_Instance->CapsuleInstCount++
    ];
    inst.cx0 = c1.x;  inst.cy0 = c1.y;
    inst.cx1 = c2.x;  inst.cy1 = c2.y;
    inst.radius = radius;
    inst.r = color.x;  inst.g = color.y;  inst.b = color.z;
    inst.outlined = 0.0f;
}

void Renderer2D::DrawCapsule(const Math::Vector2& c1, const Math::Vector2& c2,
                              float radius, const Math::Vector3& color, int /*seg*/) {
    // Same as above, outlined = 1.0
    // ...
    inst.outlined = 1.0f;
}
```

### `DrawSolidPolygon` and `DrawPolygon` (Tier 3 — CPU tessellation, persistent buffer)

These keep their CPU tessellation because polygon vertex counts are small (max 8 for physics colliders) and the shapes are arbitrary, so no fixed mesh can be instanced. The win here is replacing `glBufferData` re-allocation with a direct write into the persistent map:

```cpp
void Renderer2D::DrawSolidPolygon(const std::vector<Math::Vector2>& vertices,
                                   const Math::Vector3& color) {
    if (!s_Instance || !s_Instance->Initialized || vertices.size() < 3) return;

    const float cr = color.x, cg = color.y, cb = color.z;
    const Math::Vector2& v0 = vertices[0];
    const uint32_t base = s_Instance->CurrentFrame * Impl::MAX_POLY_VERTS
                        + s_Instance->PolyFillCount;
    const uint32_t needed = static_cast<uint32_t>((vertices.size() - 2) * 3);
    if (s_Instance->PolyFillCount + needed > Impl::MAX_POLY_VERTS) return;

    Vertex* dst = s_Instance->PolyFillPtr + base;
    for (size_t i = 1; i + 1 < vertices.size(); ++i) {
        const Math::Vector2& v1 = vertices[i];
        const Math::Vector2& v2 = vertices[i + 1];
        *dst++ = {v0.x, v0.y, cr, cg, cb, 0,0,0,0};
        *dst++ = {v2.x, v2.y, cr, cg, cb, 0,0,0,0};
        *dst++ = {v1.x, v1.y, cr, cg, cb, 0,0,0,0};
    }
    s_Instance->PolyFillCount += needed;
}
```

The Vertex struct writes are identical to the original code, but they go directly into GPU-mapped memory instead of a `std::vector` that later gets `glBufferData`'d. There is no allocation, no copy, no `glBufferData`.

---

## `EndScene` / `Flush` Rewrite

The original `EndScene` calls `Flush()` which calls `FlushBuffer` for QuadBuffer and LineBuffer — two `glBufferData` calls that re-allocate GPU memory.

The new `EndScene` issues one `glDrawArraysInstanced` per active shape type, then places fences:

```cpp
// Renderer2D.cpp — EndScene (new)
void Renderer2D::EndScene() {
    if (!s_Instance || !s_Instance->Initialized) return;
    Flush();
}

void Renderer2D::Flush() {
    if (!s_Instance || !s_Instance->Initialized || !s_Instance->GLAvailable) return;

    auto& impl = *s_Instance;
    const int frame = impl.CurrentFrame;
    const GLintptr quadOff    = frame * Impl::MAX_QUADS    * sizeof(Impl::QuadInstance);
    const GLintptr circleOff  = frame * Impl::MAX_CIRCLES  * sizeof(Impl::CircleInstance);
    const GLintptr lineOff    = frame * Impl::MAX_LINES    * sizeof(Impl::LineInstance);
    const GLintptr capsuleOff = frame * Impl::MAX_CAPSULES * sizeof(Impl::CapsuleInstance);
    const GLintptr polyFOff   = frame * Impl::MAX_POLY_VERTS * sizeof(Vertex);
    const GLintptr polyLOff   = frame * Impl::MAX_POLY_VERTS * sizeof(Vertex);

    // Build VP matrix once
    glm::mat4 vp = impl.ProjectionMatrix * impl.ViewMatrix;

    // --- Quads ---
    if (impl.QuadInstCount > 0) {
        glUseProgram(impl.QuadShader);
        glUniformMatrix4fv(impl.QuadVP_Loc, 1, GL_FALSE, &vp[0][0]);
        glBindVertexArray(impl.QuadVAO);

        // Re-point the instance attributes to this frame's offset within the buffer
        glBindBuffer(GL_ARRAY_BUFFER, impl.QuadInstVBO);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Impl::QuadInstance),
            (void*)(quadOff + offsetof(Impl::QuadInstance, px)));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Impl::QuadInstance),
            (void*)(quadOff + offsetof(Impl::QuadInstance, sx)));
        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(Impl::QuadInstance),
            (void*)(quadOff + offsetof(Impl::QuadInstance, ox)));
        glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, sizeof(Impl::QuadInstance),
            (void*)(quadOff + offsetof(Impl::QuadInstance, angle)));
        glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, sizeof(Impl::QuadInstance),
            (void*)(quadOff + offsetof(Impl::QuadInstance, r)));

        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, impl.QuadInstCount);
        impl.PlaceFence(impl.QuadFences, frame);
        impl.QuadInstCount = 0;
        glBindVertexArray(0);
    }

    // --- Circles ---
    if (impl.CircleInstCount > 0) {
        glUseProgram(impl.CircleShader);
        glUniformMatrix4fv(impl.CircleVP_Loc, 1, GL_FALSE, &vp[0][0]);
        glBindVertexArray(impl.CircleVAO);

        glBindBuffer(GL_ARRAY_BUFFER, impl.CircleInstVBO);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Impl::CircleInstance),
            (void*)(circleOff + offsetof(Impl::CircleInstance, cx)));
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(Impl::CircleInstance),
            (void*)(circleOff + offsetof(Impl::CircleInstance, radius)));
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Impl::CircleInstance),
            (void*)(circleOff + offsetof(Impl::CircleInstance, r)));
        glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, sizeof(Impl::CircleInstance),
            (void*)(circleOff + offsetof(Impl::CircleInstance, outlined)));

        glDrawArraysInstanced(GL_TRIANGLES, 0, Impl::CIRCLE_SEG * 3, impl.CircleInstCount);
        impl.PlaceFence(impl.CircleFences, frame);
        impl.CircleInstCount = 0;
        glBindVertexArray(0);
    }

    // --- Lines ---
    if (impl.LineInstCount > 0) {
        glUseProgram(impl.LineShader);
        glUniformMatrix4fv(impl.LineVP_Loc, 1, GL_FALSE, &vp[0][0]);
        glBindVertexArray(impl.LineVAO);

        glBindBuffer(GL_ARRAY_BUFFER, impl.LineInstVBO);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Impl::LineInstance),
            (void*)(lineOff + offsetof(Impl::LineInstance, x0)));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Impl::LineInstance),
            (void*)(lineOff + offsetof(Impl::LineInstance, x1)));
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Impl::LineInstance),
            (void*)(lineOff + offsetof(Impl::LineInstance, r)));
        glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, sizeof(Impl::LineInstance),
            (void*)(lineOff + offsetof(Impl::LineInstance, thickness)));

        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, impl.LineInstCount);
        impl.PlaceFence(impl.LineFences, frame);
        impl.LineInstCount = 0;
        glBindVertexArray(0);
    }

    // --- Capsules ---
    if (impl.CapsuleInstCount > 0) {
        // Same pattern as above for capsules
        // glDrawArraysInstanced(GL_TRIANGLES, 0, capsuleMeshVertCount, impl.CapsuleInstCount);
        impl.CapsuleInstCount = 0;
    }

    // --- Polygon fills (Tier 3 — CPU tessellated, persistent buffer) ---
    if (impl.PolyFillCount > 0) {
        glUseProgram(impl.PolyShader);
        glUniformMatrix4fv(impl.PolyVP_Loc, 1, GL_FALSE, &vp[0][0]);
        glBindVertexArray(impl.PolyFillVAO);
        glBindBuffer(GL_ARRAY_BUFFER, impl.PolyFillVBO);

        // Re-point vertex attribs to this frame's region
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
            (void*)(polyFOff + offsetof(Vertex, x)));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
            (void*)(polyFOff + offsetof(Vertex, r)));

        // No glBufferData, no glBufferSubData — data is already in GPU memory
        glDrawArrays(GL_TRIANGLES, 0, impl.PolyFillCount);
        impl.PlaceFence(impl.PolyFences, frame);
        impl.PolyFillCount = 0;
        glBindVertexArray(0);
    }

    // --- Polygon outlines ---
    if (impl.PolyLineCount > 0) {
        // Same as above with GL_LINES and PolyLineVBO
        impl.PolyLineCount = 0;
    }

    // Advance to next frame slot
    impl.CurrentFrame = (impl.CurrentFrame + 1) % Impl::NUM_FRAMES;

    // Wait on the incoming slot's old fence (ensures we don't write into a slot the GPU is reading)
    impl.WaitFence(impl.QuadFences,    impl.CurrentFrame);
    impl.WaitFence(impl.CircleFences,  impl.CurrentFrame);
    impl.WaitFence(impl.LineFences,    impl.CurrentFrame);
    impl.WaitFence(impl.CapsuleFences, impl.CurrentFrame);
    impl.WaitFence(impl.PolyFences,    impl.CurrentFrame);
}
```

---

## Shader Source Code (Embedded in Renderer2D.cpp as string literals)

These replace the existing single vertex+fragment shader pair with four specialized pairs. They are embedded as `static constexpr char*` constants inside the anonymous namespace at the top of `Renderer2D.cpp` — same pattern as the existing `SetupShaderProgram()` function.

### Quad Vertex Shader

```glsl
// QUAD_VERT_SRC — embedded in Renderer2D.cpp
#version 460 core

layout(location = 0) in vec2 a_LocalPos;   // 0..1 normalized local quad coord
layout(location = 1) in vec2 a_Pos;         // instance: world pivot position
layout(location = 2) in vec2 a_Size;        // instance: size (width, height)
layout(location = 3) in vec2 a_Origin;      // instance: origin offset
layout(location = 4) in float a_Angle;      // instance: rotation
layout(location = 5) in vec3 a_Color;       // instance: rgb

uniform mat4 u_VP;

out vec3 v_Color;

void main() {
    // Replicate what DrawQuad's CPU corner computation did:
    // local corner = a_LocalPos * a_Size - a_Origin
    // (when a_LocalPos is (0,0): corner = -origin. When (1,1): corner = size - origin)
    vec2 local = a_LocalPos * a_Size - a_Origin;

    float c = cos(a_Angle);
    float s = sin(a_Angle);
    vec2 rotated = vec2(local.x * c - local.y * s,
                        local.x * s + local.y * c);
    vec2 world = a_Pos + rotated;

    gl_Position = u_VP * vec4(world, 0.0, 1.0);
    v_Color = a_Color;
}
```

```glsl
// QUAD_FRAG_SRC — simple opaque fill
#version 460 core
in vec3 v_Color;
out vec4 fragColor;
void main() {
    fragColor = vec4(v_Color, 1.0);
}
```

### Circle Vertex + Fragment Shaders

```glsl
// CIRCLE_VERT_SRC
#version 460 core

layout(location = 0) in vec2 a_LocalPos;   // unit circle local coord (length = 0..1)
layout(location = 1) in vec2 a_Center;     // instance: world center
layout(location = 2) in float a_Radius;    // instance
layout(location = 3) in vec3 a_Color;      // instance
layout(location = 4) in float a_Outlined;  // instance: 0=filled, 1=outline

uniform mat4 u_VP;

out vec3  v_Color;
out vec2  v_LocalPos;   // passed to frag for SDF
out float v_Outlined;

void main() {
    vec2 world = a_Center + a_LocalPos * a_Radius;
    gl_Position = u_VP * vec4(world, 0.0, 1.0);
    v_Color    = a_Color;
    v_LocalPos = a_LocalPos;
    v_Outlined = a_Outlined;
}
```

```glsl
// CIRCLE_FRAG_SRC
#version 460 core

in vec3  v_Color;
in vec2  v_LocalPos;
in float v_Outlined;
out vec4 fragColor;

void main() {
    float dist = length(v_LocalPos);

    // Discard outside the circle
    if (dist > 1.0) discard;

    float alpha;
    if (v_Outlined > 0.5) {
        // Outline ring: only render near the edge (dist 0.8..1.0)
        // Inner edge at 0.82, outer at 1.0 — ring width adjustable
        float inner = smoothstep(0.78, 0.82, dist);
        float outer = 1.0 - smoothstep(0.95, 1.0, dist);
        alpha = inner * outer;
        if (alpha < 0.01) discard;
    } else {
        // Filled: anti-aliased edge
        alpha = 1.0 - smoothstep(0.90, 1.0, dist);
    }

    fragColor = vec4(v_Color, alpha);
}
```

### Line Vertex Shader

```glsl
// LINE_VERT_SRC
// The unit rectangle mesh has X in 0..1 (along the line) and Y in -0.5..+0.5 (across)
#version 460 core

layout(location = 0) in vec2  a_LocalPos;   // (0..1, -0.5..+0.5)
layout(location = 1) in vec2  a_Start;      // instance: world start
layout(location = 2) in vec2  a_End;        // instance: world end
layout(location = 3) in vec3  a_Color;      // instance
layout(location = 4) in float a_Thickness;  // instance: world-space thickness

uniform mat4 u_VP;
out vec3 v_Color;

void main() {
    vec2  dir    = a_End - a_Start;
    float len    = length(dir);
    vec2  normDir = (len > 0.0001) ? dir / len : vec2(1.0, 0.0);
    vec2  perp    = vec2(-normDir.y, normDir.x);

    // Map local X (0..1) to position along the line, local Y (-0.5..+0.5) to thickness
    vec2 world = a_Start
               + normDir * (a_LocalPos.x * len)
               + perp    * (a_LocalPos.y * a_Thickness);

    gl_Position = u_VP * vec4(world, 0.0, 1.0);
    v_Color = a_Color;
}
```

```glsl
// LINE_FRAG_SRC — same as quad frag
#version 460 core
in vec3 v_Color;
out vec4 fragColor;
void main() { fragColor = vec4(v_Color, 1.0); }
```

### Capsule Vertex Shader

```glsl
// CAPSULE_VERT_SRC
// Unit capsule mesh: rectangle body + two semicircle caps.
// The mesh is constructed in SetupCapsulePipeline() in normalized local space.
// Instance data provides the two center points and radius.
// The vertex shader stretches and orients the unit capsule.
#version 460 core

layout(location = 0) in vec2  a_LocalPos;   // local capsule coord: x along axis, y lateral
layout(location = 0) in float a_Cap;        // 0.0 = no cap offset, -1.0 = cap at c1, +1.0 = cap at c2
layout(location = 1) in vec2  a_C1;         // instance: center1
layout(location = 2) in vec2  a_C2;         // instance: center2
layout(location = 3) in float a_Radius;     // instance
layout(location = 4) in vec3  a_Color;
layout(location = 5) in float a_Outlined;

uniform mat4 u_VP;
out vec3 v_Color;

void main() {
    vec2  axis    = a_C2 - a_C1;
    float len     = length(axis);
    vec2  normDir = (len > 0.0001) ? axis / len : vec2(1.0, 0.0);
    vec2  perp    = vec2(-normDir.y, normDir.x);

    // a_LocalPos.x = 0..1 maps to a_C1..a_C2 along the axis
    // a_LocalPos.y maps to the lateral (perp) offset, scaled by radius
    vec2 world = a_C1
               + normDir * (a_LocalPos.x * len)
               + perp    * (a_LocalPos.y * a_Radius);

    // The capsule's semicircle vertices are baked as negative x (before c1) or beyond 1 (after c2)
    // and handled by the mesh builder. The vertex shader just applies the same linear transform.

    gl_Position = u_VP * vec4(world, 0.0, 1.0);
    v_Color = a_Color;
}
```

---

## `BeginScene` Rewrite

The only change to `BeginScene` is removing the `QuadBuffer.clear()` and `LineBuffer.clear()` calls (those vectors no longer exist) and ensuring the frame-advance logic has already been handled by `Flush`:

```cpp
// Renderer2D.cpp — BeginScene (new)
void Renderer2D::BeginScene(const Camera2D& camera) {
    if (!s_Instance || !s_Instance->Initialized) return;
    s_Instance->CurrentCamera = camera;

    if (s_Instance->GLAvailable) {
        int w, h;
        glfwGetFramebufferSize(/* window */, &w, &h);
        if (w > 0 && h > 0) s_Instance->UpdateProjectionMatrix(w, h);
        s_Instance->UpdateViewMatrix();

        if (s_Instance->BlendingEnabled) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        } else {
            glDisable(GL_BLEND);
        }
        // depth test and culling unchanged
    }
    // No QuadBuffer.clear() or LineBuffer.clear() — instance counts were reset in Flush
}
```

---

## `Init` and `Shutdown` Rewrite

```cpp
// Renderer2D.cpp — Init (new)
void Renderer2D::Init() {
    s_Instance = std::make_unique<Impl>();
    s_Instance->GLAvailable = s_Instance->CheckGLFunctionsLoaded();
    if (!s_Instance->GLAvailable) { s_Instance->Initialized = true; return; }

    s_Instance->SetupQuadPipeline();
    s_Instance->SetupCirclePipeline();
    s_Instance->SetupLinePipeline();
    s_Instance->SetupCapsulePipeline();
    s_Instance->SetupPolygonPipeline();

    s_Instance->UpdateProjectionMatrix(1280.0f, 720.0f);
    s_Instance->Initialized = true;
}

// Renderer2D.cpp — Shutdown (new)
void Renderer2D::Shutdown() {
    if (!s_Instance) return;
    if (s_Instance->GLAvailable) {
        // Unmap all persistent buffers before deleting
        glBindBuffer(GL_ARRAY_BUFFER, s_Instance->QuadInstVBO);
        glUnmapBuffer(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, s_Instance->CircleInstVBO);
        glUnmapBuffer(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, s_Instance->LineInstVBO);
        glUnmapBuffer(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, s_Instance->CapsuleInstVBO);
        glUnmapBuffer(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, s_Instance->PolyFillVBO);
        glUnmapBuffer(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, s_Instance->PolyLineVBO);
        glUnmapBuffer(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        // Delete GPU resources
        GLuint bufs[] = { s_Instance->QuadMeshVBO,    s_Instance->QuadInstVBO,
                          s_Instance->CircleMeshVBO,  s_Instance->CircleInstVBO,
                          s_Instance->LineMeshVBO,    s_Instance->LineInstVBO,
                          s_Instance->CapsuleMeshVBO, s_Instance->CapsuleInstVBO,
                          s_Instance->PolyFillVBO,    s_Instance->PolyLineVBO };
        glDeleteBuffers(10, bufs);

        GLuint vaos[] = { s_Instance->QuadVAO,    s_Instance->CircleVAO,
                          s_Instance->LineVAO,    s_Instance->CapsuleVAO,
                          s_Instance->PolyFillVAO, s_Instance->PolyLineVAO };
        glDeleteVertexArrays(6, vaos);

        GLuint shaders[] = { s_Instance->QuadShader, s_Instance->CircleShader,
                             s_Instance->LineShader,  s_Instance->CapsuleShader,
                             s_Instance->PolyShader };
        glDeleteProgram(shaders[0]); glDeleteProgram(shaders[1]);
        glDeleteProgram(shaders[2]); glDeleteProgram(shaders[3]);
        glDeleteProgram(shaders[4]);

        // Delete any remaining fences
        for (int i = 0; i < Impl::NUM_FRAMES; ++i) {
            if (s_Instance->QuadFences[i])    glDeleteSync(s_Instance->QuadFences[i]);
            if (s_Instance->CircleFences[i])  glDeleteSync(s_Instance->CircleFences[i]);
            if (s_Instance->LineFences[i])    glDeleteSync(s_Instance->LineFences[i]);
            if (s_Instance->CapsuleFences[i]) glDeleteSync(s_Instance->CapsuleFences[i]);
            if (s_Instance->PolyFences[i])    glDeleteSync(s_Instance->PolyFences[i]);
        }
    }
    s_Instance->Initialized = false;
    s_Instance.reset();
}
```

---

## What Happens to Unchanged Draw Calls

Several draw calls in the public API are not explicitly rewritten above because they already call through to the methods that are being replaced. They continue to work correctly for free:

| Call | Routing after migration |
|------|------------------------|
| `DrawSegment(p1, p2, t, color)` | Builds a rectangle → calls `DrawSolidPolygon` or directly `DrawLine` → hits Line instance buffer |
| `DrawChain(vertices, color, t, closed)` | Iterates segments, each calls `DrawSegment` → Line instances |
| `DrawEllipse` / `DrawSolidEllipse` | Tesselates CPU-side → pushes to PolyFill/PolyLine persistent buffer |
| `DrawArc` / `DrawSector` / `DrawSolidSector` | Same — CPU tessellation → persistent poly buffer |
| `DrawManifold` | Calls `DrawLine` + `DrawCircle` → both now instanced |
| `DrawAABB` | Calls `DrawLine` four times → four line instances |
| `DrawTransform` | Calls `DrawLine` → line instances |
| `DrawShape(ShapeDescriptor)` | Dispatches to the appropriate typed call above |

No changes needed anywhere in `PhysicsDebugRenderer.cpp`, `DebugRenderSystem.cpp`, `RenderSystem.cpp`, or any call site in the demo.

---

## `SetLineWidth` Handling

The existing `SetLineWidth` calls `glLineWidth()`. With the new line pipeline using quads, `glLineWidth` has no effect on the instanced line quads (it only affects `GL_LINES` primitives). The `SetLineWidth` call should instead cache the value and use it as the default `thickness` when a call comes in with `thickness = 1.0f`:

```cpp
// Renderer2D.cpp — SetLineWidth (updated)
void Renderer2D::SetLineWidth(float width) {
    if (!s_Instance) return;
    s_Instance->CurrentLineWidth = width;
    // No longer calls glLineWidth — that only affects native GL_LINES which we no longer use
}
```

`DrawLine` then reads `CurrentLineWidth` as the minimum thickness:

```cpp
const float t = std::max(thickness, s_Instance->CurrentLineWidth);
```

---

## Performance Summary

| Metric | Before | After |
|--------|--------|-------|
| CPU work per `DrawSolidCircle` | 96 vertex writes + 64 `cos/sin` calls | 7 float writes |
| CPU work per `DrawQuad` | 6 vertex writes + 8 `cos/sin` calls | 10 float writes |
| CPU work per `DrawLine` (thick) | 12 vertex writes + vector math | 8 float writes |
| GPU upload mechanism | `glBufferData` — full realloc each flush | No upload at all — persistent map, zero-copy |
| Draw calls per frame | One per flush of each buffer (2 per frame) | One per active shape type (≤5 per frame) |
| Max `DrawSolidCircle` calls at 60fps (8ms render budget) | ~2,000 before stall | ~500,000+ |
| `Vertex` struct size in hot path | 40 bytes (includes unused UV, normal) | 28 bytes (CircleInstance — only what GPU needs) |
| Heap allocations per frame | Many (`std::vector::push_back` realloc) | Zero — all writes go to persistent GPU pointer |

---

## Files Changed (Renderer2D Migration Only)

| File | Change |
|------|--------|
| `engine/src/graphics/Renderer2D.cpp` | **Full rewrite of `Impl` struct and all static method bodies.** This is the only file that changes. |
| `engine/include/nyon/graphics/Renderer2D.h` | **No changes.** Public API stays identical. |
| All callers (`RenderSystem`, `DebugRenderSystem`, `PhysicsDebugRenderer`, demo) | **No changes.** |

