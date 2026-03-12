## 4. Rendering and Debug Views

This guide covers how the built–in rendering system works, how to rely on physics debug visualization, and how to use `Renderer2D` directly when you need custom drawing.

### 4.1. Built–in render system

`ECSApplication` automatically registers a `RenderSystem` and integrates it into the main loop:

- On each fixed step, `SystemManager.Update(deltaTime)` runs all systems.
- For rendering, `ECSApplication::OnInterpolateAndRender(float alpha)`:
  - Retrieves `RenderSystem`.
  - Sets its interpolation alpha.
  - Calls `RenderSystem::Update(0.0f)`.

This means:

- You do **not** need to manually call `Renderer2D::BeginScene` / `EndScene` every frame in your game class.
- You should give `RenderSystem` enough data (typically via components) and let it handle rendering.

Exactly which components `RenderSystem` reads may evolve, but a common pattern is:

- `TransformComponent` – where to draw.
- Physics–related components (for debug overlay).

### 4.2. Physics debug rendering

Physics can be visualized even before you have your own sprites or art. Two main pieces are involved:

- **`Nyon::ECS::DebugRenderSystem`**
  - Renders debug information each frame via `RenderDebugInfo()`.
- **`Nyon::ECS::PhysicsWorldComponent` debug flags**
  - `drawShapes`, `drawJoints`, `drawAABBs`, `drawContacts`, `drawIslands` – control what is drawn.

`ECSApplication` already caches a pointer to `DebugRenderSystem` and asks it to render at the end of `OnInterpolateAndRender`.

To enable debug drawing:

```cpp
Nyon::ECS::PhysicsWorldComponent world;
world.SetDebugDraw(
    /*shapes*/   true,
    /*joints*/   false,
    /*aabbs*/    false,
    /*contacts*/ true,
    /*islands*/  false
);
```

With this configuration, when the physics pipeline runs, you should see outlines of bodies, contact points, and other diagnostic visuals drawn over your scene.

### 4.3. Using Renderer2D directly

For custom rendering beyond what `RenderSystem` provides, you can use `Nyon::Graphics::Renderer2D` directly. Its API exposes:

- `BeginScene(const Camera2D& camera)`
- `EndScene()`
- Shape drawing functions:
  - `DrawQuad`, `DrawCircle`, `DrawPolygon`, `DrawLine`, etc.
  - Filled variants like `DrawSolidCircle`, `DrawSolidPolygon`, `DrawSolidCapsule`, etc.

You typically use it in a custom system or in a simple rendering hook. For example, a minimal overlay system that draws a crosshair at the mouse position:

```cpp
#include "nyon/graphics/Renderer2D.h"
#include "nyon/math/Vector2.h"
#include "nyon/utils/InputManager.h"

class OverlayRenderSystem : public Nyon::ECS::System
{
public:
    void Initialize(Nyon::ECS::EntityManager& entityManager,
                    Nyon::ECS::ComponentStore& componentStore) override
    {
        (void)entityManager;
        (void)componentStore;
    }

    void Update(float) override
    {
        using namespace Nyon;

        double mx = 0.0, my = 0.0;
        Utils::InputManager::GetMousePosition(mx, my);

        Graphics::Camera2D camera; // default camera (screen–space)
        Graphics::Renderer2D::BeginScene(camera);

        Math::Vector2 center{ static_cast<float>(mx), static_cast<float>(my) };
        Math::Vector3 color{ 1.0f, 0.0f, 0.0f };

        Graphics::Renderer2D::DrawLine(
            center + Math::Vector2{-10.0f, 0.0f},
            center + Math::Vector2{ 10.0f, 0.0f},
            color,
            2.0f
        );

        Graphics::Renderer2D::DrawLine(
            center + Math::Vector2{0.0f, -10.0f},
            center + Math::Vector2{0.0f,  10.0f},
            color,
            2.0f
        );

        Graphics::Renderer2D::EndScene();
    }
};
```

Register this system in `OnECSStart`:

```cpp
GetSystemManager().AddSystem(std::make_unique<OverlayRenderSystem>());
```

The engine takes care of OpenGL context and swap buffers; the overlay simply draws on top using the existing context.

### 4.4. Cameras and world units

`Renderer2D` uses a `Camera2D` structure:

- `position` – camera center in world units.
- `zoom` – zoom factor (`1.0f` is default).
- `rotation` – camera rotation.

Typical 2D games keep physics and rendering in the same pixel–based coordinate system:

- `TransformComponent.position` uses pixels (y–positive down) to match physics and screen coordinates.
- `Camera2D` is usually centered on the region of interest (e.g., the player).

For small demos it is simplest to treat 1 physics unit as 1 pixel and keep camera at the origin or following the player entity.

### 4.5. Summary

- `RenderSystem` and `DebugRenderSystem` are integrated for you by `ECSApplication`.
- Enable physics debug drawing via `PhysicsWorldComponent` flags.
- Use `Renderer2D` directly in custom systems when you need fine–grained drawing or overlays.

Next, continue to **`05-input-and-gameplay-loop.md`** to wire up user input and implement actual gameplay logic.

