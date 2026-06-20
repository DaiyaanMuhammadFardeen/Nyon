## 3. Physics and Collisions

This guide shows how to set up a physics world, add rigid bodies and colliders to entities, and let the engine's physics pipeline simulate motion and collisions.

### 3.1. Physics overview

Key physics-related types:

- **`Nyon::ECS::PhysicsWorldComponent`**
  - Global simulation settings: gravity, time step, solver iterations, sleeping, and debug flags.
  - Designed to be attached to a single "world" entity.

- **`Nyon::ECS::PhysicsBodyComponent`**
  - Per-entity rigid body properties: mass, velocity, forces, body type (static/kinematic/dynamic), etc.

- **`Nyon::ECS::ColliderComponent`**
  - Per-entity collider shape (circle, polygon, capsule, segment, chain, composite) with material properties and filtering.

- **`Nyon::ECS::PhysicsPipelineSystem`**
  - A unified system that performs broad-phase, narrow-phase, constraint solving, and integration each fixed step.

You usually need:

1. One entity with `PhysicsWorldComponent`.
2. One or more entities with `TransformComponent`, `PhysicsBodyComponent`, and `ColliderComponent`.
3. A registered `PhysicsPipelineSystem` so the simulation actually runs.

### 3.2. Creating the physics world entity

In `SimpleDemoGame::OnECSStart`, create a dedicated world entity and attach `PhysicsWorldComponent`:

```cpp
#include "nyon/ecs/components/PhysicsWorldComponent.h"

void SimpleDemoGame::OnECSStart()
{
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    // Create world entity with PhysicsWorldComponent
    Nyon::ECS::EntityID worldEntity = entities.CreateEntity();

    Nyon::ECS::PhysicsWorldComponent world;
    world.gravity               = { 0.0f, -980.0f };  // Pixels/s², Y-up
    world.timeStep              = 1.0f / 60.0f;       // 60 Hz physics
    world.velocityIterations    = 8;
    world.positionIterations    = 3;
    world.enableSleep           = true;
    world.enableWarmStarting    = true;
    world.linearSlop            = 0.05f;              // Pixels (not Box2D metres)
    world.maxLinearCorrection   = 2.0f;               // Pixels per step

    // Optional: enable debug drawing of physics data
    world.SetDebugDraw(
        /*shapes*/   true,
        /*joints*/   false,
        /*aabbs*/    false,
        /*contacts*/ true,
        /*islands*/  false
    );

    components.AddComponent(worldEntity, std::move(world));
}
```

### 3.3. Registering the physics pipeline system

By default, `ECSApplication` registers input and rendering systems. To run physics you must add `PhysicsPipelineSystem` yourself. Add it in `OnECSStart` **after** creating the world entity:

```cpp
#include "nyon/ecs/systems/PhysicsPipelineSystem.h"

void SimpleDemoGame::OnECSStart()
{
    // ... create world entity first (section 3.2) ...

    // Register the physics pipeline system
    auto& systemManager = GetSystemManager();
    systemManager.AddSystem(std::make_unique<Nyon::ECS::PhysicsPipelineSystem>());

    // Now everything with TransformComponent + PhysicsBodyComponent + ColliderComponent
    // will be simulated every fixed step.
}
```

You can retrieve and configure the pipeline later:

```cpp
auto* physicsSystem = systemManager.GetSystem<Nyon::ECS::PhysicsPipelineSystem>();
if (physicsSystem)
{
    auto config = physicsSystem->GetConfig();
    config.velocityIterations  = 8;
    config.positionIterations  = 3;
    config.warmStarting        = true;
    config.linearSlop          = 0.05f;
    config.maxLinearCorrection = 2.0f;
    physicsSystem->SetConfig(config);
}
```

> **Note**: Systems have not been registered yet when `OnECSStart` runs. If you need to configure the pipeline based on the world component's settings, do it on the first `OnECSFixedUpdate` tick or use the pipeline's own `Config` struct.

### 3.4. Creating a static ground

A static body is one that never moves but can collide with dynamic bodies. Combine:

- `TransformComponent` – world-space position/scale.
- `PhysicsBodyComponent` – with `isStatic = true`.
- `ColliderComponent` – with a polygon or circle shape.

Example: a wide ground platform near the bottom of the screen.

```cpp
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"

void SimpleDemoGame::OnECSStart()
{
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    // Ground entity
    auto ground = entities.CreateEntity();

    // Transform: place near the bottom of the screen
    Nyon::ECS::TransformComponent t;
    t.position         = { 640.0f, 100.0f };   // Near bottom of 720px screen
    t.previousPosition = t.position;
    t.scale            = { 1.0f, 1.0f };

    // Static body (infinite mass)
    Nyon::ECS::PhysicsBodyComponent body;
    body.isStatic = true;
    body.UpdateMassProperties();  // Sets inverseMass = 0, inverseInertia = 0

    // Box collider using a polygon (vertices centered on origin)
    using Nyon::Math::Vector2;
    Nyon::ECS::ColliderComponent::PolygonShape boxShape({
        { -400.0f, -25.0f },
        {  400.0f, -25.0f },
        {  400.0f,  25.0f },
        { -400.0f,  25.0f }
    });

    Nyon::ECS::ColliderComponent collider(boxShape);
    collider.material.friction    = 0.6f;
    collider.material.restitution = 0.2f;

    components.AddComponent(ground, std::move(t));
    components.AddComponent(ground, std::move(body));
    components.AddComponent(ground, std::move(collider));
}
```

### 3.5. Creating dynamic falling boxes

Dynamic bodies have finite mass and respond to forces and collisions:

```cpp
void SimpleDemoGame::OnECSStart()
{
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    // Create a stack of boxes
    for (int i = 0; i < 5; ++i)
    {
        auto box = entities.CreateEntity();

        Nyon::ECS::TransformComponent t;
        t.position         = { 640.0f, 200.0f - i * 60.0f };
        t.previousPosition = t.position;

        Nyon::ECS::PhysicsBodyComponent body;
        body.mass = 1.0f;
        body.SetInertia(1.0f);  // Simple inertia for box
        body.UpdateMassProperties();
        body.SetAwake(true);

        using Nyon::Math::Vector2;
        Nyon::ECS::ColliderComponent::PolygonShape boxShape({
            { -25.0f, -25.0f },
            {  25.0f, -25.0f },
            {  25.0f,  25.0f },
            { -25.0f,  25.0f }
        });

        Nyon::ECS::ColliderComponent collider(boxShape);
        collider.material.friction    = 0.4f;
        collider.material.restitution = 0.1f;

        components.AddComponent(box, std::move(t));
        components.AddComponent(box, std::move(body));
        components.AddComponent(box, std::move(collider));
    }
}
```

With the physics world and `PhysicsPipelineSystem` active, these boxes will fall under gravity and collide with the ground.

### 3.6. Using circle colliders

Circle colliders are created with `ColliderComponent::CircleShape`:

```cpp
// Circle shape
Nyon::ECS::ColliderComponent::CircleShape circleShape;
circleShape.center = { 0.0f, 0.0f };
circleShape.radius = 25.0f;

Nyon::ECS::ColliderComponent collider(circleShape);
collider.material.friction    = 0.4f;
collider.material.restitution = 0.8f;
```

Or using the convenience constructor:

```cpp
Nyon::ECS::ColliderComponent collider(25.0f);  // Radius = 25
```

For proper physics, set the inertia from the collider shape:

```cpp
float area = collider.CalculateArea();
body.SetMass(2.0f);
body.SetInertia(collider.CalculateInertiaPerUnitMass() * body.mass);
body.UpdateMassProperties();
```

### 3.7. Sensors and collision filtering

`ColliderComponent` includes:

- **Sensors**:
  - `collider.isSensor = true;` – the collider detects overlaps but does not generate physical impulses.

- **Filtering**:
  - `collider.filter.categoryBits` – which categories this collider belongs to.
  - `collider.filter.maskBits` – which categories it can collide with.
  - `collider.filter.groupIndex` – positive forces collision, negative forbids.

Example: a trigger that only detects the player:

```cpp
Nyon::ECS::ColliderComponent::PolygonShape triggerShape({
    { -50.0f, -50.0f },
    {  50.0f, -50.0f },
    {  50.0f,  50.0f },
    { -50.0f,  50.0f }
});
Nyon::ECS::ColliderComponent trigger(triggerShape);
trigger.isSensor                    = true;
trigger.filter.categoryBits         = 0x0002;
trigger.filter.maskBits             = 0x0001; // Only category 1 (player)
```

### 3.8. Collision callbacks and contact manifolds

You can register callbacks on `PhysicsWorldComponent` for event-driven gameplay:

```cpp
auto& world = components.GetComponent<Nyon::ECS::PhysicsWorldComponent>(worldEntity);

world.SetBeginContactCallback(
    [](uint32_t entityA, uint32_t entityB)
    {
        // Called when two entities first touch
    }
);

world.SetEndContactCallback(
    [](uint32_t entityA, uint32_t entityB)
    {
        // Called when contact ends
    }
);
```

Available callbacks: `beginContact`, `endContact`, `preSolve`, `postSolve`, `jointBreak`, `sensorBegin`, `sensorEnd`.

Contact manifolds are available in `world.contactManifolds` each frame:

```cpp
for (const auto& manifold : world.contactManifolds)
{
    // manifold.entityIdA, manifold.entityIdB – the two colliding entities
    // manifold.normal – contact normal (from A to B)
    // manifold.points[] – detailed contact points
    // manifold.friction, manifold.restitution
    // manifold.touching – whether shapes are currently touching

    for (const auto& pt : manifold.points)
    {
        // pt.position    – world position of contact
        // pt.separation  – negative = penetration depth
        // pt.normalImpulse, pt.tangentImpulse
    }
}
```

Manifolds are cleared each frame and repopulated by the physics pipeline.

### 3.9. PhysicsBodyComponent details

Key properties:

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `velocity` | `Vector2` | `{0,0}` | Linear velocity (pixels/s) |
| `force` | `Vector2` | `{0,0}` | Accumulated force |
| `mass` | `float` | `1.0` | Mass (0 = infinite/static) |
| `inverseMass` | `float` | `1.0` | Cached 1/mass |
| `inertia` | `float` | `1.0` | Moment of inertia |
| `inverseInertia` | `float` | `1.0` | Cached 1/inertia |
| `friction` | `float` | `0.1` | Friction coefficient |
| `restitution` | `float` | `0.0` | Bounciness |
| `drag` | `float` | `0.0` | Air resistance |
| `angularVelocity` | `float` | `0.0` | Angular velocity (rad/s) |
| `torque` | `float` | `0.0` | Accumulated torque |
| `isStatic` | `bool` | `false` | Immovable body |
| `isKinematic` | `bool` | `false` | User-controlled motion |
| `isBullet` | `bool` | `false` | CCD enabled |
| `isAwake` | `bool` | `true` | Active simulation |
| `allowSleep` | `bool` | `true` | Can sleep when at rest |

Key methods:

```cpp
void ApplyForce(const Math::Vector2& f);              // Add force
void ApplyForceAtPoint(const Math::Vector2& f,
                       const Math::Vector2& point);   // Force at offset (generates torque)
void ApplyTorque(float t);                             // Add torque
void ApplyLinearImpulse(const Math::Vector2& impulse); // Instant velocity change
void ApplyAngularImpulse(float impulse);               // Instant angular change
void SetMass(float m);                                 // Set mass (marks explicit)
void SetInertia(float i);                              // Set inertia (marks explicit)
void UpdateMassProperties();                           // Recompute from flags
void SetAwake(bool awake);                              // Wake/sleep body
void ClearForces();                                    // Zero force and torque
bool IsDynamic() const;                                // !isStatic && !isKinematic
```

### 3.10. Summary

- Attach `PhysicsWorldComponent` to a dedicated world entity.
- Register `PhysicsPipelineSystem` via `GetSystemManager().AddSystem(...)`.
- Give entities `TransformComponent + PhysicsBodyComponent + ColliderComponent` to make them physical.
- Use `body.isStatic = true` for immovable objects.
- Set mass, inertia, friction, and restitution for each body.
- Collision callbacks and contact manifolds enable event-driven gameplay.

Next, proceed to **`04-rendering-and-debug-views.md`** to see how to visualize your entities and physics world.
