## 5. Input and Gameplay Loop

This guide explains how to handle keyboard and mouse input, and where to put gameplay logic in the Nyon engine's update flow.

### 5.1. Input system overview

Input is handled by `Nyon::Utils::InputManager` — a static helper class:

```cpp
#include "nyon/utils/InputManager.h"
```

The API:

```cpp
// Initialization (called automatically by ECSApplication::OnStart)
static void Init(GLFWwindow* window);

// Called once per frame by Application::Run()
static void Update();

// Keyboard — GLFW key codes (GLFW_KEY_A, GLFW_KEY_SPACE, etc.)
static bool IsKeyPressed(int key);   // True only on the frame the key goes down
static bool IsKeyDown(int key);      // True while the key is held
static bool IsKeyUp(int key);        // True only on the frame the key goes up

// Mouse — GLFW button codes (GLFW_MOUSE_BUTTON_LEFT, etc.)
static bool IsMousePressed(int button);
static bool IsMouseDown(int button);
static bool IsMouseUp(int button);

// Position
static void GetMousePosition(double& x, double& y);
```

**Important**: `Init()` is called automatically by `ECSApplication::OnStart()`. `Update()` is called once per frame by `Application::Run()`. You do **not** need to call these yourself.

GLFW gives mouse coordinates with origin at **top-left**. The Nyon world uses **Y-up** (origin at bottom-left). Convert mouse Y to world coordinates:

```cpp
double mx, my;
Utils::InputManager::GetMousePosition(mx, my);

int width, height;
glfwGetWindowSize(GetWindow(), &width, &height);

float worldX = static_cast<float>(mx);
float worldY = static_cast<float>(height - static_cast<int>(my));  // Flip Y
```

### 5.2. Reading keyboard input

Example: move a dynamic body left/right with A/D keys and jump with W.

```cpp
#include "nyon/utils/InputManager.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"

void SimpleDemoGame::OnECSFixedUpdate(float deltaTime)
{
    auto& components = GetComponentStore();

    // Query input once per fixed step
    bool left       = Utils::InputManager::IsKeyDown(GLFW_KEY_A);
    bool right      = Utils::InputManager::IsKeyDown(GLFW_KEY_D);
    bool jumpPressed = Utils::InputManager::IsKeyPressed(GLFW_KEY_W);

    // Apply input to all dynamic bodies (filter with a PlayerTag or specific entity)
    components.ForEachComponent<Nyon::ECS::PhysicsBodyComponent>(
        [&](Nyon::ECS::EntityID entity, Nyon::ECS::PhysicsBodyComponent& body)
        {
            if (!body.IsDynamic())
                return;

            const float moveForce = 5000.0f;

            if (left)
                body.ApplyForce({ -moveForce, 0.0f });
            if (right)
                body.ApplyForce({  moveForce, 0.0f });

            if (jumpPressed)
                body.ApplyLinearImpulse({ 0.0f, 400.0f });  // Upward (Y-up)
        }
    );
}
```

Notes:

- `IsKeyDown` is true as long as the key is held (useful for continuous movement).
- `IsKeyPressed` is true only on the frame the key transitions from up to down (useful for single actions like jumping).
- In Y-up coordinates, a positive Y impulse moves the body upward.

### 5.3. Spawning objects with mouse input

For simple interactions (e.g., spawning objects on click):

```cpp
void SimpleDemoGame::OnECSFixedUpdate(float deltaTime)
{
    using namespace Nyon;

    if (!Utils::InputManager::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
        return;

    // Get mouse position and convert to world coordinates
    double mx, my;
    Utils::InputManager::GetMousePosition(mx, my);

    int width, height;
    glfwGetWindowSize(GetWindow(), &width, &height);
    float worldX = static_cast<float>(mx);
    float worldY = static_cast<float>(height - static_cast<int>(my));  // Flip Y

    // Spawn a new dynamic box
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    auto box = entities.CreateEntity();

    // Transform at mouse position
    ECS::TransformComponent t;
    t.position         = { worldX, worldY };
    t.previousPosition = t.position;

    // Physics body
    ECS::PhysicsBodyComponent body;
    body.SetMass(1.0f);
    body.SetInertia(1.0f);
    body.UpdateMassProperties();
    body.SetAwake(true);

    // Box collider (50×50)
    ECS::ColliderComponent::PolygonShape boxShape({
        { -25.0f, -25.0f },
        {  25.0f, -25.0f },
        {  25.0f,  25.0f },
        { -25.0f,  25.0f }
    });
    ECS::ColliderComponent collider(boxShape);
    collider.material.friction    = 0.4f;
    collider.material.restitution = 0.2f;

    // Render component (green box)
    ECS::RenderComponent render({50.0f, 50.0f}, {0.3f, 1.0f, 0.3f});
    render.origin = {25.0f, 25.0f};

    components.AddComponent(box, std::move(t));
    components.AddComponent(box, std::move(body));
    components.AddComponent(box, std::move(collider));
    components.AddComponent(box, std::move(render));
}
```

### 5.4. Where to put gameplay logic

You have three main hooks in your `ECSApplication`-derived class:

| Hook | Called | Purpose |
|------|--------|---------|
| `OnECSStart()` | Once | Scene setup: create entities, set up world, register systems. |
| `OnECSFixedUpdate(float dt)` | Fixed rate (60 Hz) | Deterministic physics-driven logic: read input, apply forces, handle collisions. |
| `OnECSUpdate(float dt)` | Fixed rate, after systems | Higher-level gameplay: timers, score, AI, logic that depends on physics results. |

The engine's update order each fixed tick:

```
Application::OnFixedUpdate(dt) (final)
  ├── ECSApplication::OnFixedUpdate(dt)
  │   ├── SystemManager::Update(dt)        ← all systems run
  │   │   ├── InputSystem
  │   │   ├── PhysicsPipelineSystem
  │   │   ├── CameraSystem
  │   │   ├── DebugRenderSystem
  │   │   └── [your custom systems]
  │   ├── OnECSUpdate(dt)                  ← your after-physics logic
  │   └── OnECSFixedUpdate(dt)             ← your gameplay/physics logic
  └── (back to Application loop)
```

Because the engine uses a fixed-timestep loop internally, treating `OnECSFixedUpdate` as "the main gameplay tick" is usually the simplest mental model.

### 5.5. Ground detection using contact manifolds

You can detect whether a player is touching the ground by inspecting contact manifolds:

```cpp
bool IsPlayerGrounded(Nyon::ECS::EntityID playerEntity)
{
    auto& components = GetComponentStore();

    auto worldEntities = components.GetEntitiesWithComponent<
        Nyon::ECS::PhysicsWorldComponent>();
    if (worldEntities.empty())
        return false;

    const auto& world = components.GetComponent<
        Nyon::ECS::PhysicsWorldComponent>(worldEntities[0]);

    const Nyon::Math::Vector2 up{0.0f, 1.0f};

    for (const auto& manifold : world.contactManifolds)
    {
        if (manifold.points.empty())
            continue;

        bool isPlayerA = (manifold.entityIdA == playerEntity);
        bool isPlayerB = (manifold.entityIdB == playerEntity);
        if (!isPlayerA && !isPlayerB)
            continue;

        // manifold.normal points from A toward B
        // If player is A, negate to get the surface normal toward the player
        Nyon::Math::Vector2 contactNormal =
            isPlayerA ? -manifold.normal : manifold.normal;

        float dotUp = Nyon::Math::Vector2::Dot(contactNormal, up);
        if (dotUp > 0.7f)
        {
            // Check separation at contact points
            for (const auto& pt : manifold.points)
            {
                if (pt.separation < 1.0f)
                    return true;
            }
        }
    }
    return false;
}
```

### 5.6. Pausing and stepping

For debugging physics or gameplay, a common pattern:

```cpp
// In your game class header
bool m_Paused = false;
bool m_StepOneFrame = false;

void SimpleDemoGame::OnECSFixedUpdate(float deltaTime)
{
    // Toggle pause with P
    if (Utils::InputManager::IsKeyPressed(GLFW_KEY_P))
    {
        m_Paused = !m_Paused;
    }

    // Step one frame with O while paused
    if (Utils::InputManager::IsKeyPressed(GLFW_KEY_O))
    {
        m_StepOneFrame = true;
    }

    if (m_Paused && !m_StepOneFrame)
        return;

    m_StepOneFrame = false;

    // Normal gameplay + physics-driven logic here
    // ...
}
```

### 5.7. Accelerating time or slowing down

For a speed-up or slow-motion effect, multiply `deltaTime` by a factor:

```cpp
float m_TimeScale = 1.0f;

void SimpleDemoGame::OnECSFixedUpdate(float deltaTime)
{
    float effectiveDt = deltaTime * m_TimeScale;

    // Use effectiveDt for force calculations, timers, etc.
    // (The physics pipeline uses its own fixed step internally.)
}
```

### 5.8. Summary

- Use `InputManager::IsKeyDown`/`IsKeyPressed`/`IsKeyUp` for keyboard input.
- Use `InputManager::IsMousePressed` and `GetMousePosition` for mouse input.
- Convert mouse Y from GLFW (top-left origin) to world (bottom-left origin) by `worldY = height - mouseY`.
- Put deterministic physics-driven gameplay in `OnECSFixedUpdate`.
- Use `OnECSUpdate` for higher-level logic that sits on top of the physics step.
- Inspect `world.contactManifolds` for ground detection and collision events.

Next, follow **`06-building-a-simple-physics-demo.md`** to bring all of this together into a complete small demo.
