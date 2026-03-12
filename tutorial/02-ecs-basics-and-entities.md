## 2. ECS Basics and Entities

In Nyon, games are built on an Entity–Component–System (ECS) architecture. This guide explains the main ECS concepts and how to create and manage entities from your `ECSApplication`–derived game class.

### 2.1. Key ECS types

- **`Nyon::ECS::EntityID`** – A simple integer ID that represents a game object.
- **`Nyon::ECS::EntityManager`** – Creates, destroys, and tracks entities.
- **`Nyon::ECS::ComponentStore`** – Stores component data in a cache–friendly Structure–of–Arrays layout.
- **`Nyon::ECS::SystemManager`** – Owns all systems and runs them each fixed update.

Your `ECSApplication`–derived class exposes:

- `GetEntityManager()`
- `GetComponentStore()`
- `GetSystemManager()`

You will typically use these inside `OnECSStart`, `OnECSFixedUpdate`, and `OnECSUpdate`.

### 2.2. Creating your first entity

The simplest ECS entity is one that just has a transform (position, rotation, scale). Nyon provides `Nyon::ECS::TransformComponent`:

```cpp
#include "nyon/ecs/components/TransformComponent.h"
```

In `SimpleDemoGame::OnECSStart`:

```cpp
void SimpleDemoGame::OnECSStart()
{
    auto& entities = GetEntityManager();
    auto& components = GetComponentStore();

    // Create a single entity
    Nyon::ECS::EntityID player = entities.CreateEntity();

    // Give it a transform at the center of the screen
    Nyon::ECS::TransformComponent transform;
    transform.position = { 640.0f, 360.0f }; // screen center in pixels
    transform.scale    = { 1.0f, 1.0f };

    components.AddComponent(player, std::move(transform));
}
```

This creates an entity that can later be rendered or simulated. By itself, the transform does not move; systems and gameplay logic will do that.

### 2.3. Adding your own lightweight components

You can define custom components to store game–specific data. For example, a simple tag and health component:

```cpp
// In include/Components.h (your own header)
#pragma once

struct PlayerTag {};

struct HealthComponent
{
    int maxHealth = 100;
    int currentHealth = 100;
};
```

Attach them to entities like any other component:

```cpp
#include "Components.h"

void SimpleDemoGame::OnECSStart()
{
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    auto player = entities.CreateEntity();

    components.AddComponent(player, Nyon::ECS::TransformComponent{{640.0f, 360.0f}});
    components.AddComponent(player, PlayerTag{});
    components.AddComponent(player, HealthComponent{});
}
```

There is no central registration step: `ComponentStore` will automatically create storage for any component type you use.

### 2.4. Querying components

`ComponentStore` provides convenience methods for working with components:

- `HasComponent<T>(entity)` – whether a given entity has a component.
- `GetComponent<T>(entity)` – reference to an existing component (non–const or const).
- `GetEntitiesWithComponent<T>()` – vector of entities that have a specific component.
- `ForEachComponent<T>(func)` – iterate over all components of a given type.

Example: apply damage to every entity with `HealthComponent`:

```cpp
auto& components = GetComponentStore();

components.ForEachComponent<HealthComponent>(
    [](Nyon::ECS::EntityID entity, HealthComponent& health)
    {
        health.currentHealth -= 1; // simple test logic
    }
);
```

### 2.5. Lifecycle: where to put ECS logic

For `ECSApplication`, the main hooks are:

- **`OnECSStart()`**
  - Called once after ECS is initialized.
  - Use this to create entities and set up the scene.
- **`OnECSFixedUpdate(float deltaTime)`**
  - Called on a fixed timestep (typically 60 Hz).
  - Ideal for physics–related logic and deterministic game rules.
- **`OnECSUpdate(float deltaTime)`**
  - Called each fixed step after systems are updated.
  - Good for game logic that depends on the latest physics results.

Internally, `SystemManager.Update(deltaTime)` is invoked from `OnFixedUpdate`, so any systems you register (and built–in ones like input and rendering) run automatically there.

### 2.6. Summary

- Use `GetEntityManager()` to create and destroy entities.
- Use `GetComponentStore()` to attach data to entities and query it.
- Keep world creation in `OnECSStart`, deterministic rules in `OnECSFixedUpdate`, and higher–level gameplay in `OnECSUpdate`.

Next, go to **`03-physics-and-collisions.md`** to give your entities bodies, colliders, and a physics world so they can move and collide.

