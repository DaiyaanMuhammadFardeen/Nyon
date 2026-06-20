## 4. Rendering and Debug Views

This guide covers how the built-in rendering system works, how to use the physics debug visualization, and how to use `Renderer2D` directly for custom drawing.

### 4.1. RenderComponent

To make an entity visible, attach a `RenderComponent` alongside its `TransformComponent`:

```cpp
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/components/TransformComponent.h"
```

The `RenderComponent` struct:

```cpp
struct RenderComponent {
    Math::Vector2 size       = {32.0f, 32.0f};     // Visual size in pixels
    Math::Vector3 color      = {1.0f, 1.0f, 1.0f}; // RGB color
    Math::Vector2 origin     = {16.0f, 16.0f};      // Pivot point (default center for 32×32)
    ShapeType shapeType      = ShapeType::Rectangle; // Rectangle=0, Circle=1, Polygon=2
    std::string texturePath  = "";                   // Texture file (empty = solid color)
    bool visible             = true;
    int layer                = 0;                    // Higher = drawn on top
};
```

Example — a visible red rectangle entity:

```cpp
void SimpleDemoGame::OnECSStart()
{
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    auto entity = entities.CreateEntity();

    // Transform at screen center
    components.AddComponent(entity,
        Nyon::ECS::TransformComponent{{640.0f, 360.0f}});

    // Render as a 64×48 red rectangle
    Nyon::ECS::RenderComponent render({64.0f, 48.0f}, {1.0f, 0.0f, 0.0f});
    render.origin = {32.0f, 24.0f};  // Center pivot (half of width/height)

    components.AddComponent(entity, std::move(render));
}
```

> **Origin alignment**: Always set `origin` to `{width/2, height/2}` so that the rendered shape is centered on `transform.position`, matching the physics centroid. If `origin` is `{0,0}`, the top-left corner of the quad will align with the transform position instead.

Shape types available:

```cpp
enum class ShapeType {
    Rectangle = 0,  // Uses DrawQuad
    Circle    = 1,  // Uses DrawSolidCircle
    Polygon   = 2   // Drawn as polygon via Renderer2D (experimental)
};
```

Convenient constructors:

```cpp
RenderComponent();                                                 // Default 32×32 white rectangle
RenderComponent(const Math::Vector2& sz);                          // Custom size, white
RenderComponent(const Math::Vector2& sz, const Math::Vector3& col); // Custom size and color
RenderComponent(const Math::Vector2& sz, const Math::Vector3& col,
                const std::string& tex);                           // With texture path
RenderComponent(const Math::Vector2& sz, const Math::Vector3& col,
                ShapeType shape);                                  // Custom shape type
```

### 4.2. Built-in render system

`ECSApplication` automatically registers a `RenderSystem` and integrates it into the main loop. You do **not** need to add it yourself.

How rendering works each frame:

1. `ECSApplication::OnInterpolateAndRender(float alpha)` is called.
2. It sets the interpolation alpha on `RenderSystem` and `DebugRenderSystem`.
3. `RenderSystem::Update(0)` iterates over all entities with both `TransformComponent` and `RenderComponent`.
4. For each visible entity, it uses `TransformComponent::GetInterpolatedPosition(alpha)` and `GetInterpolatedRotation(alpha)` for smooth rendering.
5. It draws rectangles via `Graphics::Renderer2D::DrawQuad()` and circles via `DrawSolidCircle()`.

The `RenderSystem` also handles camera management:

- If any entity has a `CameraComponent` with `isActive = true`, that camera is used.
- Otherwise, a default camera at `{0, 0}` with zoom `1.0` is used.

> **Important**: You do **not** need to manually call `Renderer2D::BeginScene` / `EndScene` in your game class. The `RenderSystem` handles this for you.

### 4.3. Using CameraComponent

To control the view, create a camera entity:

```cpp
#include "nyon/ecs/components/CameraComponent.h"

void SimpleDemoGame::OnECSStart()
{
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    // Create camera entity
    Nyon::ECS::EntityID camEntity = entities.CreateEntity();
    Nyon::ECS::CameraComponent camera(1280.0f, 720.0f);
    camera.isActive                = true;
    camera.priority                = 1.0f;
    camera.SetPosition({640.0f, 360.0f});  // Center of the screen
    camera.SetZoom(1.0f);
    camera.SetRotation(0.0f);

    components.AddComponent(camEntity, std::move(camera));
}
```

`CameraComponent` wraps `Graphics::Camera2D` and provides:

```cpp
struct CameraComponent {
    Graphics::Camera2D camera;        // The underlying camera
    bool   isActive       = true;     // Active camera for rendering
    float  priority       = 0.0f;     // Higher priority wins
    int    layer          = 0;        // Camera rendering layer
    
    // Viewport (normalized 0-1)
    float  viewportX, viewportY, viewportWidth, viewportHeight;

    // Follow behavior
    bool   followTarget   = false;
    EntityID targetEntity = INVALID_ENTITY;
    Math::Vector2 followOffset    = {0, 0};
    float  followSmoothness = 0.0f;

    void SetPosition(const Math::Vector2& pos);
    void Move(const Math::Vector2& delta);
    void SetZoom(float zoom);
    void Zoom(float delta);
    void SetRotation(float angle);
    void Rotate(float delta);
    Math::Vector2 ScreenToWorld(const Math::Vector2& screenPos) const;
    Math::Vector2 WorldToScreen(const Math::Vector2& worldPos) const;
};
```

The `CameraSystem` selects the active camera by priority. Multiple cameras can coexist.

### 4.4. Physics debug rendering

The `DebugRenderSystem` visualizes physics shapes, contacts, and other diagnostic information.

To enable it, add it to your system manager:

```cpp
#include "nyon/ecs/systems/DebugRenderSystem.h"

void SimpleDemoGame::OnECSStart()
{
    // ... create world entity, register physics pipeline ...

    // Add debug rendering
    GetSystemManager().AddSystem(std::make_unique<Nyon::ECS::DebugRenderSystem>());
}
```

Control what is drawn via the `PhysicsWorldComponent` debug flags:

```cpp
auto& world = components.GetComponent<Nyon::ECS::PhysicsWorldComponent>(worldEntity);
world.SetDebugDraw(
    /*shapes*/   true,   // Outline all physics shapes
    /*joints*/   false,
    /*aabbs*/    true,   // Show bounding boxes
    /*contacts*/ true,   // Show contact points
    /*islands*/  false
);
```

Or via `DebugRenderSystem::SetFlags()`:

```cpp
auto* debugSystem = GetSystemManager().GetSystem<Nyon::ECS::DebugRenderSystem>();
if (debugSystem)
{
    debugSystem->SetFlags(true, true, true, false);
    // Parameters: drawShapes, drawAABBs, drawContacts, drawCOM
}
```

`ECSApplication` also has an internal `m_DebugOverlayEnabled` flag toggled by **F1** via `m_DebugRenderSystem`.

### 4.5. Using Renderer2D directly

For custom rendering beyond what `RenderSystem` provides, use `Nyon::Graphics::Renderer2D` directly. This is useful for UI overlays, custom debug visualizations, or special effects.

The API:

```cpp
// Scene management
static void BeginScene(const Camera2D& camera = Camera2D());
static void EndScene();

// Basic shapes
static void DrawQuad(const Math::Vector2& position, const Math::Vector2& size,
                     const Math::Vector2& origin, const Math::Vector3& color,
                     float rotation = 0.0f);
static void DrawCircle(const Math::Vector2& center, float radius,
                       const Math::Vector3& color, int segments = 32);
static void DrawPolygon(const std::vector<Math::Vector2>& vertices,
                        const Math::Vector3& color);
static void DrawLine(const Math::Vector2& start, const Math::Vector2& end,
                     const Math::Vector3& color, float thickness = 1.0f);

// Filled variants
static void DrawSolidCircle(const Math::Vector2& center, float radius,
                            const Math::Vector3& color, int segments = 32);
static void DrawSolidPolygon(const std::vector<Math::Vector2>& vertices,
                             const Math::Vector3& color);

// Advanced shapes
static void DrawCapsule(const Math::Vector2& center1, const Math::Vector2& center2,
                        float radius, const Math::Vector3& color, int segments = 16);
static void DrawEllipse(...), DrawArc(...), DrawSector(...), DrawChain(...);

// Debug
static void DrawManifold(const Math::Vector2& contactPoint,
                         const Math::Vector2& normal, float separation, bool isTouching);
static void DrawAABB(const Math::Vector2& min, const Math::Vector2& max,
                     const Math::Vector3& color);
```

Example — a custom system that draws a crosshair at the mouse position:

```cpp
#include "nyon/graphics/Renderer2D.h"
#include "nyon/math/Vector2.h"
#include "nyon/utils/InputManager.h"

class CrosshairSystem : public Nyon::ECS::System
{
public:
    void Initialize(Nyon::ECS::EntityManager&, Nyon::ECS::ComponentStore&) override {}

    void Update(float) override
    {
        using namespace Nyon;

        double mx = 0.0, my = 0.0;
        Utils::InputManager::GetMousePosition(mx, my);

        // Convert from GLFW screen coords (Y-down) to world coords (Y-up)
        int width, height;
        GLFWwindow* window = Application::Get().GetWindow();
        glfwGetWindowSize(window, &width, &height);
        float worldY = static_cast<float>(height - static_cast<int>(my));

        Math::Vector2 center{ static_cast<float>(mx), worldY };
        Math::Vector3 red{ 1.0f, 0.0f, 0.0f };

        Graphics::Renderer2D::BeginScene();
        Graphics::Renderer2D::DrawLine(
            center + Math::Vector2{-10.0f, 0.0f},
            center + Math::Vector2{ 10.0f, 0.0f},
            red, 2.0f);
        Graphics::Renderer2D::DrawLine(
            center + Math::Vector2{0.0f, -10.0f},
            center + Math::Vector2{0.0f,  10.0f},
            red, 2.0f);
        Graphics::Renderer2D::EndScene();
    }
};
```

Register custom render systems in `OnECSStart`:

```cpp
GetSystemManager().AddSystem(std::make_unique<CrosshairSystem>());
```

> **Note**: Custom systems that use `Renderer2D` must call `BeginScene`/`EndScene` themselves. They run after the built-in `RenderSystem` clears the screen.

### 4.6. Summary

- Attach `RenderComponent` + `TransformComponent` to make entities visible.
- `RenderSystem` is auto-registered — no manual setup needed.
- Use `CameraComponent` for view control (pan, zoom, follow).
- Add `DebugRenderSystem` to visualize physics shapes and contacts.
- Use `Renderer2D` directly in custom systems for overlays and custom drawing.
- Always set `origin = {width/2, height/2}` for correct centroid alignment.

Next, continue to **`05-input-and-gameplay-loop.md`** to wire up user input and implement actual gameplay logic.
