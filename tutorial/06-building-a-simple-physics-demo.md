## 6. Building a Simple Physics Demo

This final guide walks through building a complete, minimal physics playground using the Nyon engine. It assumes you have followed the previous tutorials.

The demo will:

- Open a window.
- Create a physics world with gravity.
- Add a static ground and several dynamic boxes.
- Enable physics debug rendering.
- Render entities with color.
- Allow spawning new boxes with the mouse.

### 6.1. Project skeleton recap

Directory layout under `game/simple-demo`:

```
game/simple-demo/
├── CMakeLists.txt
├── include/
│   └── SimpleDemoGame.h
└── src/
    ├── SimpleDemoGame.cpp
    └── main.cpp
```

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

### 6.3. Game CMakeLists.txt

`CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.10)
project(SimpleDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(simple_demo
    src/main.cpp
    src/SimpleDemoGame.cpp
)

target_include_directories(simple_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/engine/include
)

target_link_libraries(simple_demo PRIVATE
    nyon_engine
    glfw
    OpenGL::GL
)

set_target_properties(simple_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/game/simple-demo"
)
```

### 6.4. Game implementation – world and entity setup

`src/SimpleDemoGame.cpp`:

```cpp
#include "SimpleDemoGame.h"

#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsWorldComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/systems/PhysicsPipelineSystem.h"
#include "nyon/ecs/systems/DebugRenderSystem.h"
#include "nyon/utils/InputManager.h"

using namespace Nyon;

void SimpleDemoGame::OnECSStart()
{
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();
    auto& systems    = GetSystemManager();

    // ── 1) Create physics world ──────────────────────────────────────────────
    ECS::EntityID worldEntity = entities.CreateEntity();

    ECS::PhysicsWorldComponent world;
    world.gravity               = { 0.0f, -980.0f };  // Y-up, pixels/s²
    world.timeStep              = 1.0f / 60.0f;
    world.velocityIterations    = 8;
    world.positionIterations    = 3;
    world.linearSlop            = 0.05f;
    world.maxLinearCorrection   = 2.0f;
    world.enableSleep           = true;
    world.enableWarmStarting    = true;

    // Enable debug visualization
    world.SetDebugDraw(
        /*shapes*/   true,
        /*joints*/   false,
        /*aabbs*/    false,
        /*contacts*/ true,
        /*islands*/  false
    );

    components.AddComponent(worldEntity, std::move(world));

    // ── 2) Register physics and debug systems ─────────────────────────────────
    systems.AddSystem(std::make_unique<ECS::PhysicsPipelineSystem>());
    systems.AddSystem(std::make_unique<ECS::DebugRenderSystem>());

    // ── 3) Static ground ──────────────────────────────────────────────────────
    {
        ECS::EntityID ground = entities.CreateEntity();

        // Transform near the bottom of the 720px screen
        ECS::TransformComponent t;
        t.position         = { 640.0f, 100.0f };
        t.previousPosition = t.position;

        // Static body
        ECS::PhysicsBodyComponent body;
        body.isStatic = true;
        body.UpdateMassProperties();

        // Polygon collider — 800×50 box
        using Math::Vector2;
        ECS::ColliderComponent::PolygonShape groundShape({
            { -400.0f, -25.0f },
            {  400.0f, -25.0f },
            {  400.0f,  25.0f },
            { -400.0f,  25.0f }
        });

        ECS::ColliderComponent collider(groundShape);
        collider.material.friction    = 0.6f;
        collider.material.restitution = 0.3f;

        // Render — grey rectangle
        ECS::RenderComponent render({800.0f, 50.0f}, {0.35f, 0.35f, 0.35f});
        render.origin = {400.0f, 25.0f};  // Align with physics centroid

        components.AddComponent(ground, std::move(t));
        components.AddComponent(ground, std::move(body));
        components.AddComponent(ground, std::move(collider));
        components.AddComponent(ground, std::move(render));
    }

    // ── 4) Initial stack of dynamic boxes ─────────────────────────────────────
    for (int i = 0; i < 6; ++i)
    {
        ECS::EntityID box = entities.CreateEntity();

        ECS::TransformComponent t;
        t.position         = { 640.0f, 200.0f - i * 60.0f };
        t.previousPosition = t.position;

        // Dynamic body
        ECS::PhysicsBodyComponent body;
        body.SetMass(1.0f);
        body.SetInertia(1.0f);
        body.UpdateMassProperties();
        body.SetAwake(true);

        // 50×50 box collider
        ECS::ColliderComponent::PolygonShape boxShape({
            { -25.0f, -25.0f },
            {  25.0f, -25.0f },
            {  25.0f,  25.0f },
            { -25.0f,  25.0f }
        });

        ECS::ColliderComponent collider(boxShape);
        collider.material.friction    = 0.4f;
        collider.material.restitution = 0.1f;

        // Render — colored rectangle (vary hue by index)
        float hue = 0.5f + i * 0.08f;
        ECS::RenderComponent render({50.0f, 50.0f}, {hue, 0.6f, 0.2f});
        render.origin = {25.0f, 25.0f};

        components.AddComponent(box, std::move(t));
        components.AddComponent(box, std::move(body));
        components.AddComponent(box, std::move(collider));
        components.AddComponent(box, std::move(render));
    }
}
```

This sets up:

1. A physics world with gravity pointing downward `{0, -980}`.
2. The `PhysicsPipelineSystem` and `DebugRenderSystem`.
3. A static ground platform near the bottom of the screen.
4. A tower of six colored dynamic boxes.

### 6.5. Game implementation – input and spawning

Add gameplay logic to the same file:

```cpp
void SimpleDemoGame::OnECSFixedUpdate(float deltaTime)
{
    // ── Toggle pause with P ────────────────────────────────────────────────
    if (Utils::InputManager::IsKeyPressed(GLFW_KEY_P))
    {
        m_Paused = !m_Paused;
    }

    if (m_Paused)
        return;

    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    // ── Left mouse button: spawn a new box at cursor position ───────────────
    if (Utils::InputManager::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
    {
        // Get mouse position and convert to world coordinates
        double mx, my;
        Utils::InputManager::GetMousePosition(mx, my);

        int width, height;
        glfwGetWindowSize(GetWindow(), &width, &height);
        float worldX = static_cast<float>(mx);
        float worldY = static_cast<float>(height - static_cast<int>(my));  // Flip Y

        // Create the spawned box entity
        ECS::EntityID box = entities.CreateEntity();

        ECS::TransformComponent t;
        t.position         = { worldX, worldY };
        t.previousPosition = t.position;

        // Dynamic body
        ECS::PhysicsBodyComponent body;
        body.SetMass(1.0f);
        body.SetInertia(1.0f);
        body.UpdateMassProperties();
        body.SetAwake(true);

        // 40×40 box collider
        ECS::ColliderComponent::PolygonShape boxShape({
            { -20.0f, -20.0f },
            {  20.0f, -20.0f },
            {  20.0f,  20.0f },
            { -20.0f,  20.0f }
        });

        ECS::ColliderComponent collider(boxShape);
        collider.material.friction    = 0.4f;
        collider.material.restitution = 0.2f;

        // Render — cyan box
        ECS::RenderComponent render({40.0f, 40.0f}, {0.0f, 1.0f, 1.0f});
        render.origin = {20.0f, 20.0f};

        components.AddComponent(box, std::move(t));
        components.AddComponent(box, std::move(body));
        components.AddComponent(box, std::move(collider));
        components.AddComponent(box, std::move(render));
    }
}

void SimpleDemoGame::OnECSUpdate(float deltaTime)
{
    (void)deltaTime;
    // Optional: add UI, score, or non-physics-critical logic here.
}
```

This function:

- Toggles pause with the `P` key.
- Spawns 40×40 cyan boxes at the clicked position (in world coordinates) when the left mouse button is clicked.

### 6.6. main.cpp

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

### 6.7. Building and running

From the repository root:

```bash
mkdir -p build
cd build
cmake ..
cmake --build . --config Debug
./game/simple-demo/simple_demo   # or the platform's default run command
```

You should see:

- A window with colored boxes rendered and physics debug shapes visible.
- The initial stack of boxes falling onto the ground and settling.
- New boxes spawning at the mouse position on left-click.
- Physics simulation pausing/resuming when you press `P`.

### 6.8. Where to go next

- **Add a player-controlled character**: Use `InputManager` to apply forces/impulses to a specific entity (WASD movement, jump).
- **Use circle colliders**: Replace `PolygonShape` with `CircleShape` for round objects.
- **Add collision callbacks**: Register `beginContact`/`endContact` on `PhysicsWorldComponent` for event-driven gameplay (damage zones, triggers).
- **Add a camera**: Create a `CameraComponent` that follows the player.
- **Use behavior components**: Attach `BehaviorComponent` to entities to give them custom update functions.
- **Create custom systems**: Move gameplay logic into dedicated systems instead of keeping everything in `SimpleDemoGame`.
- **Add joints**: Use the engine's joint components for constraints like pendulums or springs.

At this point you have a minimal but complete "Nyon game" structure you can adapt into more advanced games or specialized physics demos.
