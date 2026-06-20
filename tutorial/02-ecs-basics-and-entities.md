## 2. ECS Basics and Entities

In Nyon, games are built on an Entity–Component–System (ECS) architecture. This guide explains the main ECS concepts and how to create and manage entities from your `ECSApplication`-derived game class.

### 2.1. Key ECS types

- **`Nyon::ECS::EntityID`** – A `uint32_t` that represents a game object.
- **`Nyon::ECS::EntityManager`** – Creates, destroys, and tracks entities.
- **`Nyon::ECS::ComponentStore`** – Stores component data in a cache-friendly Structure-of-Arrays layout.
- **`Nyon::ECS::SystemManager`** – Owns all systems and runs them each fixed update.

Your `ECSApplication`-derived class exposes these accessors:

```cpp
ECS::EntityManager&   GetEntityManager();
ECS::ComponentStore&  GetComponentStore();
ECS::SystemManager&   GetSystemManager();
```

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
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    // Create a single entity
    Nyon::ECS::EntityID player = entities.CreateEntity();

    // Give it a transform at the center of the screen
    Nyon::ECS::TransformComponent transform;
    transform.position         = { 640.0f, 360.0f }; // screen center in pixels
    transform.previousPosition = transform.position; // initialize for interpolation
    transform.scale            = { 1.0f, 1.0f };

    components.AddComponent(player, std::move(transform));
}
```

The `TransformComponent` struct has the following fields:

```cpp
struct TransformComponent {
    Math::Vector2 position         = {0.0f, 0.0f};  // Current position
    Math::Vector2 previousPosition = {0.0f, 0.0f};  // Position from last frame (for interpolation)
    Math::Vector2 scale           = {1.0f, 1.0f};
    float rotation                = 0.0f;            // Radians
    float previousRotation        = 0.0f;

    // Call before physics updates to capture current state for interpolation
    void PrepareForUpdate();

    // Get interpolated position for smooth rendering
    Math::Vector2 GetInterpolatedPosition(float alpha) const;

    // Get interpolated rotation for smooth rendering
    float GetInterpolatedRotation(float alpha) const;
};
```

Convenience constructors are available:

```cpp
TransformComponent();                              // Default
TransformComponent(const Math::Vector2& pos);       // Position only
TransformComponent(const Math::Vector2& pos,
                   const Math::Vector2& scl);       // Position + scale
TransformComponent(const Math::Vector2& pos,
                   const Math::Vector2& scl,
                   float rot);                      // Position + scale + rotation
```

### 2.3. Adding your own lightweight components

You can define custom components to store game-specific data. For example, a simple tag and health component:

```cpp
// In include/Components.h (your own header)
#pragma once

struct PlayerTag {};

struct HealthComponent
{
    int maxHealth     = 100;
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

There is no central registration step: `ComponentStore` automatically creates storage for any component type the first time you use it.

### 2.4. Querying components

`ComponentStore` provides convenience methods for working with components:

- `HasComponent<T>(entity)` – whether a given entity has a component.
- `GetComponent<T>(entity)` – reference to an existing component (non-const or const).
- `GetEntitiesWithComponent<T>()` – vector of entities that have a specific component.
- `ForEachComponent<T>(func)` – iterate over all components of a given type.
- `RemoveComponent<T>(entity)` – remove a component from an entity.
- `GetComponentCount<T>()` – number of active components of a given type.

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

The `ForEachComponent` callback receives the `EntityID` and a reference to the component. Iteration order is deterministic (insertion order).

### 2.5. Lifecycle: where to put ECS logic

For `ECSApplication`, the main hooks are:

- **`OnECSStart()`**
  - Called once after ECS is initialized (before any systems are registered).
  - Use this to create entities, add components, and set up the scene.
  - **Note**: systems have not been registered yet when this runs. Any system-level configuration must be deferred to the first `OnECSFixedUpdate` tick.

- **`OnECSFixedUpdate(float deltaTime)`**
  - Called on a fixed timestep (60 Hz, `deltaTime ≈ 1/60`).
  - Ideal for physics-related logic: reading input, applying forces, spawning entities.
  - This is the main gameplay tick.

- **`OnECSUpdate(float deltaTime)`**
  - Called each fixed step **after** all systems are updated.
  - Good for higher-level game logic that depends on the latest physics results: timers, score management, AI decisions.

Internally, `ECSApplication` wires these hooks:

```
Application::OnStart()              (final) → calls OnECSStart()
Application::OnFixedUpdate(dt)      (final) → calls SystemManager::Update(dt),
                                               then OnECSUpdate(dt),
                                               then OnECSFixedUpdate(dt)
Application::OnInterpolateAndRender(alpha) (final) → renders with interpolation
```

### 2.6. Systems overview

While you can put all logic in your `ECSApplication` hooks, for larger projects you can register custom systems via `GetSystemManager()`:

```cpp
GetSystemManager().AddSystem(std::make_unique<MyCustomSystem>());
```

Each system implements `Nyon::ECS::System` with:

```cpp
class MyCustomSystem : public Nyon::ECS::System
{
public:
    void Initialize(Nyon::ECS::EntityManager& em,
                    Nyon::ECS::ComponentStore& cs) override;
    void Update(float deltaTime) override;
};
```

Registered systems run every fixed step, before `OnECSUpdate` is called.

### 2.7. Summary

- Use `GetEntityManager().CreateEntity()` to create entities.
- Use `GetComponentStore().AddComponent<T>(entity, ...)` to attach data.
- Use `GetComponentStore().ForEachComponent<T>(...)` or `GetComponent<T>(entity)` to read data.
- Keep world creation in `OnECSStart`, deterministic rules in `OnECSFixedUpdate`, and higher-level gameplay in `OnECSUpdate`.
- Register custom systems for reusable, modular logic.

Next, go to **`03-physics-and-collisions.md`** to give your entities bodies, colliders, and a physics world so they can move and collide.
