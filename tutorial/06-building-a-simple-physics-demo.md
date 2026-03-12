## 6. Building a Simple Physics Demo

This final guide walks through building a complete, minimal physics playground using the Nyon engine. It assumes you have followed the previous tutorials.

The demo will:

- Open a window.
- Create a physics world with gravity.
- Add a static ground and several dynamic boxes.
- Enable physics debug rendering.
- Allow spawning new boxes with the mouse.

### 6.1. Project skeleton recap

Directory layout under `game/simple-demo`:

- `CMakeLists.txt` – links the executable against `nyon_engine`.
- `include/SimpleDemoGame.h`
- `src/SimpleDemoGame.cpp`
- `src/main.cpp`

Make sure the root `CMakeLists.txt` has:

```cmake
add_subdirectory(engine)
add_subdirectory(game/simple-demo)
```

### 6.2. Game class header

`include/SimpleDemoGame.h`:

```cpp
#pragma once

#include "nyon/core/ECSApplication.h"

class SimpleDemoGame : public Nyon::ECSApplication
{
public:
    SimpleDemoGame()
        : Nyon::ECSApplication("Nyon Physics Demo", 1280, 720) {}

protected:
    void OnECSStart() override;
    void OnECSFixedUpdate(float deltaTime) override;
    void OnECSUpdate(float deltaTime) override;

private:
    bool m_Paused = false;
};
```

### 6.3. Game implementation – world setup

`src/SimpleDemoGame.cpp`:

```cpp
#include "SimpleDemoGame.h"

#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/systems/PhysicsPipelineSystem.h"
#include "nyon/utils/InputManager.h"

using namespace Nyon;

void SimpleDemoGame::OnECSStart()
{
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();
    auto& systems    = GetSystemManager();

    // 1) Create physics world
    ECS::EntityID worldEntity = entities.CreateEntity();

    ECS::PhysicsWorldComponent world;
    world.gravity = { 0.0f, 980.0f };
    world.timeStep = 1.0f / 60.0f;
    world.enableSleep = true;

    // Enable debug shapes + contact points
    world.SetDebugDraw(
        /*shapes*/   true,
        /*joints*/   false,
        /*aabbs*/    false,
        /*contacts*/ true,
        /*islands*/  false
    );

    components.AddComponent(worldEntity, std::move(world));

    // 2) Register physics pipeline system
    systems.AddSystem(std::make_unique<ECS::PhysicsPipelineSystem>());

    // 3) Create static ground
    {
        ECS::EntityID ground = entities.CreateEntity();

        ECS::TransformComponent t;
        t.position = { 640.0f, 680.0f }; // near bottom of 720px tall window

        ECS::PhysicsBodyComponent body;
        body.isStatic = true;
        body.UpdateMassProperties();

        using Math::Vector2;
        ECS::ColliderComponent::PolygonShape boxShape({
            { -500.0f, -30.0f },
            {  500.0f, -30.0f },
            {  500.0f,  30.0f },
            { -500.0f,  30.0f }
        });

        ECS::ColliderComponent collider(boxShape);
        collider.material.friction    = 0.8f;
        collider.material.restitution = 0.0f;

        components.AddComponent(ground, std::move(t));
        components.AddComponent(ground, std::move(body));
        components.AddComponent(ground, std::move(collider));
    }

    // 4) Create a stack of dynamic boxes
    for (int i = 0; i < 6; ++i)
    {
        ECS::EntityID box = entities.CreateEntity();

        ECS::TransformComponent t;
        t.position = { 640.0f, 200.0f - i * 60.0f };

        ECS::PhysicsBodyComponent body;
        body.mass = 1.0f;
        body.UpdateMassProperties();

        using Math::Vector2;
        ECS::ColliderComponent::PolygonShape boxShape({
            { -25.0f, -25.0f },
            {  25.0f, -25.0f },
            {  25.0f,  25.0f },
            { -25.0f,  25.0f }
        });

        ECS::ColliderComponent collider(boxShape);
        collider.material.friction    = 0.4f;
        collider.material.restitution = 0.1f;

        components.AddComponent(box, std::move(t));
        components.AddComponent(box, std::move(body));
        components.AddComponent(box, std::move(collider));
    }
}
```

This sets up the world, physics pipeline, ground, and a tower of boxes.

### 6.4. Game implementation – input and spawning

Add gameplay logic to the same file:

```cpp
void SimpleDemoGame::OnECSFixedUpdate(float deltaTime)
{
    (void)deltaTime;

    // Toggle pause with P
    if (Utils::InputManager::IsKeyPressed(GLFW_KEY_P))
    {
        m_Paused = !m_Paused;
    }

    if (m_Paused)
        return;

    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    // Left mouse button: spawn a new box at cursor position
    if (Utils::InputManager::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
    {
        double mx = 0.0, my = 0.0;
        Utils::InputManager::GetMousePosition(mx, my);

        ECS::EntityID box = entities.CreateEntity();

        ECS::TransformComponent t;
        t.position = { static_cast<float>(mx), static_cast<float>(my) };

        ECS::PhysicsBodyComponent body;
        body.mass = 1.0f;
        body.UpdateMassProperties();

        using Math::Vector2;
        ECS::ColliderComponent::PolygonShape boxShape({
            { -20.0f, -20.0f },
            {  20.0f, -20.0f },
            {  20.0f,  20.0f },
            { -20.0f,  20.0f }
        });

        ECS::ColliderComponent collider(boxShape);
        collider.material.friction    = 0.4f;
        collider.material.restitution = 0.2f;

        components.AddComponent(box, std::move(t));
        components.AddComponent(box, std::move(body));
        components.AddComponent(box, std::move(collider));
    }
}

void SimpleDemoGame::OnECSUpdate(float deltaTime)
{
    (void)deltaTime;
    // Optional: add UI, score, or non–physics–critical logic here.
}
```

This function:

- Toggles pause with the `P` key.
- Spawns small dynamic boxes whenever the left mouse button is clicked.

### 6.5. main.cpp

`src/main.cpp` is very small:

```cpp
#include "SimpleDemoGame.h"

int main()
{
    SimpleDemoGame game;
    game.Run();
    return 0;
}
```

### 6.6. Building and running

From the repository root:

```bash
mkdir -p build
cd build
cmake ..
cmake --build . --config Debug
./simple_demo   # or the platform’s default run command
```

You should see:

- A window with physics debug shapes visible (boxes and ground).
- The initial stack of boxes falling onto the ground and settling.
- New boxes spawning at the mouse position on left–click.
- Physics simulation pausing/resuming when you press `P`.

### 6.7. Where to go next

- Replace debug shapes with your own rendering components and systems.
- Add joints (e.g., pendulums or ragdolls) using the engine’s joint components.
- Use collision callbacks on `PhysicsWorldComponent` for event–driven gameplay (damage zones, triggers).
- Split your game code into multiple systems (movement, AI, rendering overlays) instead of keeping everything in `SimpleDemoGame`.

At this point you have a minimal but complete “Nyon game” structure you can adapt into more advanced games or specialized physics demos.

