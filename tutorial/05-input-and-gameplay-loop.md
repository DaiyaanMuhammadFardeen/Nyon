## 5. Input and Gameplay Loop

This guide explains how to handle keyboard and mouse input, and where to put gameplay logic in the Nyon engine’s update flow.

### 5.1. Input system overview

Two main components handle input:

- **`Nyon::Utils::InputManager`**
  - Static helper with functions like:
    - `Init(GLFWwindow* window)` – called for you by `ECSApplication::OnStart`.
    - `Update()` – updates input states each frame (managed internally).
    - `IsKeyDown(int key)`, `IsKeyPressed(int key)`, `IsKeyUp(int key)`.
    - `IsMouseDown(int button)`, `IsMousePressed(int button)`, `IsMouseUp(int button)`.
    - `GetMousePosition(double& x, double& y)`.
  - Relies on GLFW callbacks to track key/button changes.
- **`Nyon::ECS::InputSystem`**
  - Registered by `ECSApplication` at startup.
  - Integrates `InputManager` with the ECS world (if needed).

You usually interact directly with `InputManager` from your game logic or custom systems.

### 5.2. Reading keyboard input

Example: move a dynamic body left/right with A/D keys and jump with Space.

```cpp
#include "nyon/utils/InputManager.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"

void SimpleDemoGame::OnECSFixedUpdate(float deltaTime)
{
    auto& components = GetComponentStore();

    // Query input once per fixed step
    bool left  = Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_A);
    bool right = Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_D);
    bool jumpPressed = Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_SPACE);

    // Apply input to all player bodies (you might filter with a PlayerTag)
    components.ForEachComponent<Nyon::ECS::PhysicsBodyComponent>(
        [&](Nyon::ECS::EntityID entity, Nyon::ECS::PhysicsBodyComponent& body)
        {
            if (!body.IsDynamic())
                return;

            const float moveForce = 5000.0f;

            if (left)
            {
                body.ApplyForce({ -moveForce, 0.0f });
            }
            if (right)
            {
                body.ApplyForce({ moveForce, 0.0f });
            }

            if (jumpPressed && body.isGrounded)
            {
                body.ApplyLinearImpulse({ 0.0f, -400.0f });
            }
        }
    );
}
```

Notes:

- `IsKeyDown` is true as long as the key is held.
- `IsKeyPressed` is true only on the frame the key transitions from up to down.
- `PhysicsBodyComponent` has some legacy grounded helpers; in a production game you would compute grounded status based on contacts.

### 5.3. Reading mouse input

For simple interactions (e.g., spawning objects on click), use mouse position and button states:

```cpp
void SimpleDemoGame::OnECSFixedUpdate(float deltaTime)
{
    using namespace Nyon;

    double mx = 0.0, my = 0.0;
    Utils::InputManager::GetMousePosition(mx, my);

    bool leftClick = Utils::InputManager::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT);

    if (leftClick)
    {
        auto& entities   = GetEntityManager();
        auto& components = GetComponentStore();

        auto box = entities.CreateEntity();

        ECS::TransformComponent transform;
        transform.position = { static_cast<float>(mx), static_cast<float>(my) };

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

        components.AddComponent(box, std::move(transform));
        components.AddComponent(box, std::move(body));
        components.AddComponent(box, std::move(collider));
    }
}
```

This spawns a falling box at the cursor position each time the left mouse button is clicked.

### 5.4. Where to put gameplay logic

You have three main hooks in your `ECSApplication`–derived class:

- **`OnECSStart()`**
  - Scene setup: entities, physics world, systems, initial state.
- **`OnECSFixedUpdate(float deltaTime)`**
  - Deterministic logic tied to physics:
    - Read input.
    - Apply forces/impulses.
    - Update state machines that depend on exact physics stepping.
- **`OnECSUpdate(float deltaTime)`**
  - Higher–level gameplay logic:
    - Timers, score management, AI that can be less strict about timing.
    - Logic using the latest physics results (positions, velocities).

Because the engine uses a fixed–timestep loop internally, treating `OnECSFixedUpdate` as “the main gameplay tick” is usually the simplest mental model.

### 5.5. Pausing and stepping

For debugging physics or gameplay, a common pattern is:

- Keep a boolean `m_Paused` flag in your game class.
- In `OnECSFixedUpdate`, early–out if paused:

```cpp
void SimpleDemoGame::OnECSFixedUpdate(float deltaTime)
{
    if (m_Paused)
        return;

    // normal gameplay + physics–driven logic here
}
```

- Toggle `m_Paused` when a key is pressed (`P`, for example).
- Optionally add a “step one frame” key that runs exactly one tick while paused.

### 5.6. Summary

- Use `InputManager`’s `IsKeyDown`/`IsKeyPressed` and mouse helpers to drive gameplay.
- Put deterministic physics–driven gameplay in `OnECSFixedUpdate`.
- Use `OnECSUpdate` for higher–level logic that sits on top of the physics step.

Next, follow **`06-building-a-simple-physics-demo.md`** to bring all of this together into a complete small demo.

