## 3. Physics and Collisions

This guide shows how to set up a physics world, add rigid bodies and colliders to entities, and let the engine’s physics pipeline simulate motion and collisions.

### 3.1. Physics overview

Key physics–related types:

- **`Nyon::ECS::PhysicsWorldComponent`**
  - Global simulation settings: gravity, time step, solver iterations, sleeping, and debug flags.
  - Designed to be attached to a single “world” entity.
- **`Nyon::ECS::PhysicsBodyComponent`**
  - Per–entity rigid body properties: mass, velocity, forces, damping, body type (static/kinematic/dynamic), etc.
- **`Nyon::ECS::ColliderComponent`**
  - Per–entity collider shape (circle, polygon, capsule, segment, chain, composite) and material/filtering.
- **`Nyon::ECS::PhysicsPipelineSystem`**
  - A unified system that performs broad–phase, narrow–phase, constraint solving, and integration each fixed step.

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

    // 1) Create world entity with PhysicsWorldComponent
    Nyon::ECS::EntityID worldEntity = entities.CreateEntity();

    Nyon::ECS::PhysicsWorldComponent world;
    world.gravity = { 0.0f, 980.0f };    // pixels / s^2, positive Y is down
    world.timeStep = 1.0f / 60.0f;       // 60 Hz physics
    world.enableSleep = true;            // allow bodies to sleep when at rest

    // Optional: enable debug drawing of physics data
    world.SetDebugDraw(
        /*shapes*/   true,
        /*joints*/   false,
        /*aabbs*/    false,
        /*contacts*/ true,
        /*islands*/  false
    );

    components.AddComponent(worldEntity, std::move(world));

    // (Setup bodies and colliders below; see sections 3.3+)
}
```

The physics systems discover this component through the `ComponentStore` and use it to drive the entire simulation.

### 3.3. Registering the physics pipeline system

By default, `ECSApplication` registers input and rendering systems. To run physics you should add `PhysicsPipelineSystem` yourself in `OnECSStart`:

```cpp
#include "nyon/ecs/systems/PhysicsPipelineSystem.h"

void SimpleDemoGame::OnECSStart()
{
    // ... create world entity first ...

    // 2) Register the unified physics pipeline system
    auto& systemManager = GetSystemManager();
    systemManager.AddSystem(std::make_unique<Nyon::ECS::PhysicsPipelineSystem>());

    // Now the physics world + bodies/colliders will be simulated every fixed step.
}
```

You can later retrieve and configure the system:

```cpp
auto* physicsSystem = systemManager.GetSystem<Nyon::ECS::PhysicsPipelineSystem>();
if (physicsSystem)
{
    auto config = physicsSystem->GetConfig();
    config.velocityIterations  = 8;
    config.positionIterations  = 3;
    config.warmStarting        = true;
    physicsSystem->SetConfig(config);
}
```

### 3.4. Creating a static ground

A static body is one that never moves but can collide with dynamic bodies. Combine:

- `TransformComponent` – world–space position/scale.
- `PhysicsBodyComponent` – with `isStatic = true`.
- `ColliderComponent` – with a polygon or circle shape.

Example: a wide ground platform.

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
    Nyon::ECS::TransformComponent transform;
    transform.position = { 640.0f, 650.0f };
    transform.scale    = { 1.0f, 1.0f };

    // Static body (infinite mass)
    Nyon::ECS::PhysicsBodyComponent body;
    body.isStatic = true;
    body.UpdateMassProperties();

    // Box collider using a polygon
    using Nyon::Math::Vector2;
    Nyon::ECS::ColliderComponent::PolygonShape boxShape({
        { -400.0f, -20.0f },
        {  400.0f, -20.0f },
        {  400.0f,  20.0f },
        { -400.0f,  20.0f }
    });

    Nyon::ECS::ColliderComponent collider(boxShape);
    collider.material.friction    = 0.8f;
    collider.material.restitution = 0.0f; // not bouncy

    components.AddComponent(ground, std::move(transform));
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

    // Example: create a stack of boxes
    for (int i = 0; i < 5; ++i)
    {
        auto box = entities.CreateEntity();

        Nyon::ECS::TransformComponent transform;
        transform.position = { 640.0f, 200.0f - i * 60.0f };

        Nyon::ECS::PhysicsBodyComponent body;
        body.mass     = 1.0f;
        body.isStatic = false;
        body.UpdateMassProperties(); // computes inverseMass/inverseInertia

        using Nyon::Math::Vector2;
        Nyon::ECS::ColliderComponent::PolygonShape boxShape({
            { -25.0f, -25.0f },
            {  25.0f, -25.0f },
            {  25.0f,  25.0f },
            { -25.0f,  25.0f }
        });

        Nyon::ECS::ColliderComponent collider(boxShape);
        collider.material.friction    = 0.4f;
        collider.material.restitution = 0.1f; // slightly bouncy

        components.AddComponent(box, std::move(transform));
        components.AddComponent(box, std::move(body));
        components.AddComponent(box, std::move(collider));
    }
}
```

With the physics world and `PhysicsPipelineSystem` active, these boxes will fall under gravity and collide with the ground.

### 3.6. Sensors and filtering

`ColliderComponent` includes:

- **Sensors**:
  - `collider.isSensor = true;` – the collider will detect overlaps but not generate physical impulses.
- **Filtering**:
  - `collider.filter.categoryBits` – which category this collider belongs to.
  - `collider.filter.maskBits` – which categories it can collide with.
  - `collider.filter.groupIndex` – to force or forbid collisions within a group.

Example: a trigger that only detects the player:

```cpp
Nyon::ECS::ColliderComponent triggerCollider(boxShape);
triggerCollider.isSensor = true;
triggerCollider.filter.categoryBits = 0x0002; // trigger category
triggerCollider.filter.maskBits     = 0x0001; // only collide with category 1 (e.g., player)
```

Physics contact/sensor callbacks are configured on `PhysicsWorldComponent::callbacks` if you need event–driven gameplay (e.g., begin/end contact).

### 3.7. Summary

- Use a single `PhysicsWorldComponent` on a dedicated entity to control global physics.
- Give entities `TransformComponent + PhysicsBodyComponent + ColliderComponent` to make them physical.
- Register `PhysicsPipelineSystem` so the engine actually steps the simulation each fixed update.

Next, proceed to **`04-rendering-and-debug-views.md`** to see how to visualize your entities and physics world.

