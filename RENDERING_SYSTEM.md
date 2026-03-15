# Advanced 2D Rendering System

## Overview

The Nyon Engine now features a comprehensive 2D rendering system built on OpenGL with advanced features for physics visualization and game development. The renderer supports all major 2D shapes with both outlined and filled variants, instanced rendering capabilities, and extensive debug visualization tools.

## Architecture

### Core Components

1. **Renderer2D** - Main rendering interface
2. **PhysicsDebugRenderer** - Physics-specific visualization
3. **Camera2D** - 2D camera with zoom and pan
4. **ShapeDescriptor** - Generic shape description

## Features

### Basic Shapes
- **Quads/Rectangles** - Positioned, sized, and origin-controlled
- **Circles** - Outline and filled variants with configurable segments
- **Polygons** - Arbitrary convex and concave polygons
- **Lines** - Variable thickness with smooth rendering

### Advanced Shapes
- **Capsules** - Perfect for character controllers
- **Segments** - Line segments with thickness
- **Chains** - Connected line strips (open or closed)
- **Ellipses** - Stretched circles with independent radii
- **Arcs** - Partial circle outlines
- **Sectors** - Pie-slice shaped wedges

### Rendering Modes
- **Outlined** - Wireframe representation
- **Filled** - Solid triangle-filled shapes
- **Thick Lines** - Anti-aliased wide lines
- **Transformed** - Rotation, scale, and position support

### OpenGL Features
- Vertex Buffer Objects (VBO)
- Vertex Array Objects (VAO)
- Dynamic buffer updates
- Efficient batching
- Blending with alpha
- Depth testing (optional)
- Face culling (optional)
- Custom shader pipeline

## Usage Examples

### Basic Setup

```cpp
#include "nyon/graphics/Renderer2D.h"

// Initialize renderer
Renderer2D::Init();

// Setup camera
Camera2D camera;
camera.position = {640.0f, 360.0f};  // Screen center
camera.zoom = 1.0f;

// Begin scene
Renderer2D::BeginScene(camera);

// Draw shapes
Renderer2D::DrawQuad({100.0f, 100.0f}, {50.0f, 50.0f}, {25.0f, 25.0f}, {1.0f, 0.0f, 0.0f});
Renderer2D::DrawSolidCircle({200.0f, 200.0f}, 30.0f, {0.0f, 1.0f, 0.0f});

// End scene (automatically flushes)
Renderer2D::EndScene();

// Shutdown
Renderer2D::Shutdown();
```

### Drawing Circles

```cpp
// Outline circle
Renderer2D::DrawCircle(center, radius, color, segments = 32);

// Filled circle
Renderer2D::DrawSolidCircle(center, radius, color, segments = 32);
```

### Drawing Polygons

```cpp
std::vector<Math::Vector2> vertices = {
    {0.0f, 0.0f},
    {50.0f, 0.0f},
    {50.0f, 50.0f},
    {0.0f, 50.0f}
};

// Outline
Renderer2D::DrawPolygon(vertices, color);

// Filled
Renderer2D::DrawSolidPolygon(vertices, color);
```

### Drawing Capsules

```cpp
Math::Vector2 center1{100.0f, 100.0f};
Math::Vector2 center2{200.0f, 200.0f};
float radius = 15.0f;

// Outline
Renderer2D::DrawCapsule(center1, center2, radius, color, segments = 16);

// Filled
Renderer2D::DrawSolidCapsule(center1, center2, radius, color, segments = 16);
```

### Drawing with Thickness

```cpp
// Thick line (rendered as rectangle)
Renderer2D::DrawLine(start, end, color, thickness = 5.0f);

// Segment with rounded ends
Renderer2D::DrawSegment(point1, point2, thickness, color);

// Chain of connected segments
std::vector<Math::Vector2> chainPoints = {/* ... */};
Renderer2D::DrawChain(chainPoints, color, thickness, closed = false);
```

### Advanced Shapes

```cpp
// Ellipse
Renderer2D::DrawEllipse(center, radiusX, radiusY, color, segments = 32);
Renderer2D::DrawSolidEllipse(center, radiusX, radiusY, color, segments = 32);

// Arc (partial circle outline)
Renderer2D::DrawArc(center, radius, angleStart, angleEnd, color, thickness, segments);

// Sector (filled pie slice)
Renderer2D::DrawSector(center, radius, angleStart, angleEnd, color, segments);
Renderer2D::DrawSolidSector(center, radius, angleStart, angleEnd, color, segments);
```

## Physics Debug Visualization

The `PhysicsDebugRenderer` provides comprehensive visualization for physics simulations:

```cpp
#include "nyon/graphics/PhysicsDebugRenderer.h"

PhysicsDebugRenderer debugRenderer;
debugRenderer.SetActiveFlags(DebugRenderFlag::All);

// In your render loop:
Renderer2D::BeginScene(camera);

// Draw collider
debugRenderer.DrawCollider(collider, transform, color);

// Draw contact manifolds
debugRenderer.DrawManifold(manifold);

// Draw AABB
debugRenderer.DrawAABB(min, max, color);

// Draw velocity vectors
debugRenderer.DrawVelocityVector(position, velocity, color, scale);

// Draw center of mass
debugRenderer.DrawCenterOfMass(position, color);

Renderer2D::EndScene();
```

### Debug Render Flags

Control what gets rendered:

```cpp
enum class DebugRenderFlag
{
    None                = 0,
    Shapes              = (1 << 0),
    AABBs               = (1 << 1),
    Contacts            = (1 << 2),
    Manifolds           = (1 << 3),
    Joints              = (1 << 4),
    CentersOfMass       = (1 << 5),
    VelocityVectors     = (1 << 6),
    CollisionNormals    = (1 << 7),
    Sensors             = (1 << 8),
    ChainLinks          = (1 << 9),
    All                 = 0xFFFF
};

// Combine flags
debugRenderer.SetActiveFlags(
    DebugRenderFlag::Shapes | 
    DebugRenderFlag::Contacts |
    DebugRenderFlag::AABBs
);
```

## Camera System

The `Camera2D` class provides view/projection transformations:

```cpp
Camera2D camera;
camera.position = {640.0f, 360.0f};  // World position at screen center
camera.zoom = 2.0f;                   // Zoom level
camera.rotation = 0.5f;               // Camera rotation in radians
camera.nearPlane = -1.0f;             // Near clipping plane
camera.farPlane = 1.0f;               // Far clipping plane

// Convert screen coordinates to world
Math::Vector2 worldPos = camera.ScreenToWorld(screenPos, screenWidth, screenHeight);

// Convert world coordinates to screen
Math::Vector2 screenPos = camera.WorldToScreen(worldPos, screenWidth, screenHeight);

// Get transformation matrices
glm::mat4 view = camera.GetViewMatrix();
glm::mat4 proj = camera.GetProjectionMatrix(screenWidth, screenHeight);
glm::mat4 viewProj = camera.GetViewProjectionMatrix(screenWidth, screenHeight);
```

## Shape Descriptor (Generic Interface)

For a unified shape drawing interface:

```cpp
ShapeDescriptor shape;
shape.type = ShapeType::Circle;
shape.position = {400.0f, 300.0f};
shape.color = {1.0f, 0.0f, 0.0f};
shape.params.circle.radius = 50.0f;
shape.segments = 32;
shape.filled = true;

Renderer2D::DrawShape(shape);
```

### Available Shape Types

```cpp
enum class ShapeType
{
    Circle,
    Polygon,
    Capsule,
    Segment,
    Chain,
    Rectangle,
    Ellipse,
    Arc,
    Sector
};
```

## Performance Considerations

### Batching
- Shapes are automatically batched for efficient GPU submission
- Maximum batch size: 20,000 quads (120,000 vertices)
- Automatic flush when buffer is full

### State Management
- Minimize state changes between draw calls
- Group similar shapes together
- Use appropriate segment counts (don't over-tessellate)

### Best Practices
1. Call `BeginScene()` once per frame
2. Batch draw calls by material/color
3. Use `EndScene()` which calls `Flush()`
4. Reuse vertex buffers when possible
5. Adjust segment count based on shape size/distance

## Advanced Features

### Vertex Format

Each vertex contains:
- Position (x, y) - 2 floats
- Color (r, g, b) - 3 floats
- Texture coordinates (u, v) - 2 floats
- Normal (nx, ny) - 2 floats

Total: 9 floats = 36 bytes per vertex

### Shader Pipeline

The renderer uses a custom GLSL shader program:

**Vertex Shader:**
- Transforms vertices by view/projection matrices
- Passes color, texture coords, and normals to fragment shader
- Outputs world position for effects

**Fragment Shader:**
- Applies vertex colors
- Supports basic lighting from normals
- Extensible for textures and effects

### Render States

```cpp
// Enable/disable blending
Renderer2D::EnableBlending(true);

// Enable/disable depth testing
Renderer2D::EnableDepthTest(false);

// Enable/disable face culling
Renderer2D::EnableCulling(false);

// Set line width
Renderer2D::SetLineWidth(2.0f);
float currentWidth = Renderer2D::GetLineWidth();
```

## Integration with Physics Engine

The rendering system is designed to work seamlessly with the physics engine:

```cpp
// In your physics simulation loop:
void RenderPhysicsWorld()
{
    Camera2D camera;
    Renderer2D::BeginScene(camera);
    
    PhysicsDebugRenderer debugRenderer;
    debugRenderer.SetActiveFlags(DebugRenderFlag::All);
    
    // For each physics body:
    for (auto& body : physicsWorld.bodies)
    {
        // Draw collider
        debugRenderer.DrawCollider(body.collider, body.transform, body.color);
        
        // Draw velocity
        debugRenderer.DrawVelocityVector(body.transform.position, 
                                        body.velocity, 
                                        {0.0f, 1.0f, 0.0f});
        
        // Draw contacts
        for (auto& contact : body.contacts)
        {
            debugRenderer.DrawManifold(contact.manifold);
        }
    }
    
    Renderer2D::EndScene();
}
```

## Troubleshooting

### Common Issues

1. **Nothing renders:**
   - Check if `Init()` was called
   - Verify OpenGL context exists
   - Ensure `BeginScene()` is called before drawing

2. **Shapes appear distorted:**
   - Check coordinate system (y-axis points down in screen space)
   - Verify camera zoom and position

3. **Performance issues:**
   - Reduce segment count for small/distant shapes
   - Batch draw calls
   - Avoid excessive state changes

4. **Crash on shutdown:**
   - Ensure proper initialization order
   - Don't call rendering functions after `Shutdown()`

## Future Enhancements

Planned features:
- Texture mapping support
- Gradient fills
- Dashed/dotted lines
- Rounded rectangle corners
- Instanced rendering for particles
- Sprite batch rendering
- Font/text rendering
- Particle system integration
- Post-processing effects

## API Reference

For complete API documentation, see the header files:
- `engine/include/nyon/graphics/Renderer2D.h`
- `engine/include/nyon/graphics/PhysicsDebugRenderer.h`

## Examples

See `RenderingDemo.cpp` for comprehensive usage examples demonstrating all features.
