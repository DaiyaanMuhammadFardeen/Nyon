# 2D Renderer Quick Reference Guide

## Quick Start

```cpp
#include "nyon/graphics/Renderer2D.h"
#include "nyon/graphics/PhysicsDebugRenderer.h"

// Initialize
Renderer2D::Init();
Camera2D camera{{640, 360}, 1.0f};  // position, zoom

// Render loop
Renderer2D::BeginScene(camera);

// Draw stuff here
Renderer2D::DrawSolidCircle({100, 100}, 50, {1, 0, 0});

Renderer2D::EndScene();  // Auto-flushes

// Cleanup
Renderer2D::Shutdown();
```

## Shape Drawing Functions

### Circles
```cpp
// Outline
DrawCircle(center, radius, color, segments = 32)

// Filled
DrawSolidCircle(center, radius, color, segments = 32)
```

### Polygons
```cpp
std::vector<Math::Vector2> verts = {/* ... */};

// Outline
DrawPolygon(verts, color)

// Filled
DrawSolidPolygon(verts, color)
```

### Rectangles/Quads
```cpp
DrawQuad(position, size, origin, color)
```

### Lines
```cpp
// Thin line (1 pixel)
DrawLine(start, end, color)

// Thick line (as rectangle)
DrawLine(start, end, color, thickness = 5.0f)
```

### Capsules
```cpp
Math::Vector2 c1{100, 100}, c2{200, 200};
float radius = 15.0f;

// Outline
DrawCapsule(c1, c2, radius, color, segments = 16)

// Filled
DrawSolidCapsule(c1, c2, radius, color, segments = 16)
```

### Segments (with thickness)
```cpp
// Thin segment
DrawSegment(p1, p2, 0, color)

// Thick segment with rounded ends
DrawSegment(p1, p2, thickness, color)
```

### Chains
```cpp
std::vector<Math::Vector2> chain = {/* ... */};

// Open chain
DrawChain(chain, color, thickness, closed = false)

// Closed loop
DrawChain(chain, color, thickness, closed = true)
```

### Ellipses
```cpp
// Outline
DrawEllipse(center, radiusX, radiusY, color, segments = 32)

// Filled
DrawSolidEllipse(center, radiusX, radiusY, color, segments = 32)
```

### Arcs (curved lines)
```cpp
// Angles in radians
DrawArc(center, radius, angleStart, angleEnd, color, thickness, segments)
```

### Sectors (pie slices)
```cpp
// Filled wedge
DrawSector(center, radius, angleStart, angleEnd, color, segments)
DrawSolidSector(center, radius, angleStart, angleEnd, color, segments)
```

## Physics Debug Rendering

```cpp
PhysicsDebugRenderer debug;
debug.SetActiveFlags(DebugRenderFlag::All);

Renderer2D::BeginScene(camera);

// Draw collider
debug.DrawCollider(collider, transform, color);

// Draw contact manifold
debug.DrawManifold(manifold);

// Draw AABB
debug.DrawAABB(min, max, color);

// Draw velocity
debug.DrawVelocityVector(pos, vel, color, scale);

// Draw center of mass
debug.DrawCenterOfMass(position, color);

Renderer2D::EndScene();
```

## Debug Flags

```cpp
// Single flag
debug.SetActiveFlags(DebugRenderFlag::Shapes);

// Multiple flags
debug.SetActiveFlags(
    DebugRenderFlag::Shapes | 
    DebugRenderFlag::Contacts |
    DebugRenderFlag::AABBs
);

// All flags
debug.SetActiveFlags(DebugRenderFlag::All);
```

Available flags:
- `Shapes` - Collider outlines
- `AABBs` - Bounding boxes
- `Contacts` - Contact points
- `Manifolds` - Full manifold visualization
- `CentersOfMass` - Center markers
- `VelocityVectors` - Velocity arrows
- `CollisionNormals` - Face normals
- `Sensors` - Sensor volumes
- `ChainLinks` - Chain vertices

## Camera

```cpp
Camera2D camera;
camera.position = {640, 360};  // World position at screen center
camera.zoom = 2.0f;             // Zoom level
camera.rotation = 0.5f;         // Radians

// Use in rendering
Renderer2D::BeginScene(camera);

// Coordinate conversion
Math::Vector2 world = camera.ScreenToWorld(screen, width, height);
Math::Vector2 screen = camera.WorldToScreen(world, width, height);
```

## Render States

```cpp
// Blending
Renderer2D::EnableBlending(true);

// Depth test
Renderer2D::EnableDepthTest(false);

// Face culling
Renderer2D::EnableCulling(false);

// Line width
Renderer2D::SetLineWidth(2.0f);
float w = Renderer2D::GetLineWidth();
```

## Generic Shape Interface

```cpp
ShapeDescriptor shape;
shape.type = ShapeType::Circle;
shape.position = {400, 300};
shape.color = {1, 0, 0};
shape.params.circle.radius = 50;
shape.filled = true;
shape.segments = 32;

Renderer2D::DrawShape(shape);
```

Shape types:
- `Circle`, `Polygon`, `Rectangle`
- `Capsule`, `Segment`, `Chain`
- `Ellipse`, `Arc`, `Sector`

## Color Helpers

```cpp
// HSV to RGB
Math::Vector3 color = PhysicsDebugRenderer::ColorFromHSV(h, s, v);

// Linear interpolation
Math::Vector3 result = PhysicsDebugRenderer::LerpColor(a, b, t);
```

## Common Patterns

### Rainbow of colors
```cpp
for (int i = 0; i < 10; i++) {
    float hue = i / 10.0f;
    auto color = PhysicsDebugRenderer::ColorFromHSV(hue, 1.0f, 1.0f);
    DrawCircle({100 + i * 50, 100}, 20, color);
}
```

### Semi-transparent shapes
```cpp
// Note: Alpha not yet supported in current implementation
// Use lighter colors for "transparency" effect
Math::Vector3 lightRed = {1.0f, 0.5f, 0.5f};
```

### Rotated polygon
```cpp
std::vector<Math::Vector2> rotatedVerts;
float cosR = std::cos(angle), sinR = std::sin(angle);
for (auto& v : originalVerts) {
    rotatedVerts.push_back({
        v.x * cosR - v.y * sinR,
        v.x * sinR + v.y * cosR
    });
}
DrawSolidPolygon(rotatedVerts, color);
```

### Thick outline
```cpp
// Draw filled then outline
DrawSolidPolygon(verts, fillColor);
DrawPolygon(verts, outlineColor);
```

## Performance Tips

1. **Batch similar shapes** - Group by color/type
2. **Minimize state changes** - Set render states once per frame
3. **Use appropriate segments** - Don't over-tessellate small shapes
4. **Call BeginScene once** - Then draw everything, then EndScene
5. **Let auto-flush handle it** - Don't manually flush unless needed

## Troubleshooting

**Nothing renders:**
- Check `Init()` was called
- Verify OpenGL context exists
- Call `BeginScene()` before drawing

**Wrong colors:**
- Colors are RGB floats (0.0-1.0)
- No alpha channel currently

**Performance issues:**
- Reduce segment count
- Batch draw calls
- Avoid redundant state changes

**Crash on shutdown:**
- Don't call render functions after `Shutdown()`
- Proper initialization order matters

## Complete Example

```cpp
#include "nyon/graphics/Renderer2D.h"

void Game::Render()
{
    // Setup
    Camera2D camera;
    camera.position = {640, 360};
    camera.zoom = 1.0f;
    
    Renderer2D::BeginScene(camera);
    
    // Background
    Renderer2D::DrawQuad({640, 360}, {1280, 720}, {640, 360}, {0.1f, 0.1f, 0.2f});
    
    // Player (circle)
    Renderer2D::DrawSolidCircle(playerPos, 30, {0, 1, 0});
    Renderer2D::DrawCircle(playerPos, 30, {1, 1, 1}, 2.0f);
    
    // Enemies (polygons)
    for (auto& enemy : enemies) {
        Renderer2D::DrawSolidPolygon(enemy.vertices, {1, 0, 0});
    }
    
    // Bullets (lines)
    for (auto& bullet : bullets) {
        Renderer2D::DrawLine(bullet.start, bullet.end, {1, 1, 0}, 3.0f);
    }
    
    // UI text position marker
    Renderer2D::DrawTransform(uiPos, 0, 1.0f);
    
    Renderer2D::EndScene();
}
```

---

**For full documentation see:** `RENDERING_SYSTEM.md`  
**For implementation details see:** `IMPLEMENTATION_SUMMARY.md`
