# Camera System Implementation - Fix Summary

## Issue Description (from Engineering Report §5.3)

**Problem**: No shared camera / VP matrix — each renderer hardcodes its own

Four places in the codebase constructed or implied a VP matrix independently:
- `Renderer2D::EndScene` (applies `u_VP` via OpenGL uniform)
- `ParticleRenderSystem::Render` (hardcoded ortho)
- `PhysicsDebugRenderer` (delegates to `Renderer2D`)
- `DebugRenderSystem` (delegates to `Renderer2D`)

There was no `CameraComponent` or `CameraSystem` in the ECS. All rendering assumed a fixed orthographic projection. As a result, adding panning, zooming, or multiple cameras required changes in multiple places.

## Solution Implemented

### 1. Enhanced CameraComponent ✅

**File**: `engine/include/nyon/ecs/components/CameraComponent.h`

Already existed but was enhanced with:
- `Entity targetEntity` field for camera follow targets
- Better integration with ECS systems

The component provides:
- Priority-based camera selection
- Layer support for multi-camera setups
- Viewport settings (normalized coordinates)
- Follow target behavior with smoothing
- Screen-to-world and world-to-screen conversion
- Matrix accessors (view, projection, VP)

### 2. Created CameraSystem ✅

**Files**: 
- `engine/include/nyon/ecs/systems/CameraSystem.h`
- `engine/src/ecs/systems/CameraSystem.cpp`

**Responsibilities**:
- Tracks all camera entities in the scene
- Selects active camera based on priority (higher priority wins)
- Updates camera follow logic (smooth lerp towards target)
- Synchronizes with `Renderer2D`'s active camera every frame
- Manages camera activation/deactivation

**Key Methods**:
```cpp
void Initialize(EntityManager&, ComponentStore&) override;
void Update(float deltaTime) override;
const CameraComponent* GetActiveCamera() const;
void SetActiveCamera(Entity entity);
void SelectBestCamera();
void UpdateCameraFollow(float deltaTime);
void SyncWithRenderer(float screenWidth, float screenHeight);
```

### 3. Integrated CameraSystem into ECSApplication ✅

**File**: `engine/src/core/ECSApplication.cpp`

Changes:
- Added `#include "nyon/ecs/systems/CameraSystem.h"`
- Registered `CameraSystem` in `SystemManager` during initialization:
  ```cpp
  m_SystemManager.AddSystem(std::make_unique<ECS::CameraSystem>());
  ```
- Removed hardcoded VP matrix calculation from particle rendering path
- Particle renderer now uses unified camera system automatically

### 4. Updated ParticleRenderSystem ✅

**File**: `engine/src/ecs/systems/ParticleRenderSystem.cpp`

Changes:
- Removed manual VP matrix construction
- Now retrieves active camera from `Renderer2D::GetActiveCamera()`
- Uses camera's `GetViewProjectionMatrix()` method
- Simplified constructor (removed unused `m_ViewProjection` member)

Before:
```cpp
glm::mat4 vp = glm::ortho(0.0f, static_cast<float>(width), 
                         0.0f, static_cast<float>(height), 
                         -1.0f, 1.0f);
particleSystem->SetViewProjection(vp);
```

After:
```cpp
const auto& camera = Graphics::Renderer2D::GetActiveCamera();
glm::mat4 vp = camera.GetViewProjectionMatrix(width, height);
// No SetViewProjection needed
```

### 5. Added Vector2::Lerp Function ✅

**File**: `engine/include/nyon/math/Vector2.h`

Added utility function for smooth camera following:
```cpp
[[nodiscard]] static Vector2 Lerp(const Vector2& a, const Vector2& b, float t)
{
    return Vector2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
}
```

## Architecture After Fix

```
┌─────────────────────────────────────────────────────┐
│              ECSApplication Main Loop               │
└──────────────────┬──────────────────────────────────┘
                   │
                   │ FixedUpdate (60Hz)
                   ▼
┌─────────────────────────────────────────────────────┐
│              SystemManager.Update()                 │
│  ┌──────────────────────────────────────────────┐   │
│  │ 1. InputSystem                               │   │
│  │ 2. CameraSystem ← NEW!                       │   │
│  │    - Select best camera by priority          │   │
│  │    - Update follow targets                   │   │
│  │    - Sync with Renderer2D                    │   │
│  │ 3. PhysicsPipelineSystem                     │   │
│  └──────────────────────────────────────────────┘   │
└──────────────────┬──────────────────────────────────┘
                   │
                   │ Render Phase (interpolated)
                   ▼
┌─────────────────────────────────────────────────────┐
│         OnInterpolateAndRender(float alpha)         │
│  ┌──────────────────────────────────────────────┐   │
│  │ RenderSystem.Update()                        │   │
│  │  - Clear screen                              │   │
│  │  - Get active camera from CameraComponent    │   │
│  │  - BeginScene(camera) → syncs to Renderer2D  │   │
│  │  - Draw all RenderComponent entities         │   │
│  │  - EndScene()                                │   │
│  ├──────────────────────────────────────────────┤   │
│  │ DebugRenderSystem (if enabled)               │   │
│  │  - Uses PhysicsDebugRenderer                 │   │
│  │  - Delegates to Renderer2D                   │   │
│  ├──────────────────────────────────────────────┤   │
│  │ ParticleRenderSystem                         │   │
│  │  - Gets camera from Renderer2D               │   │
│  │  - Uses unified VP matrix                    │   │
│  └──────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────┐
│              Renderer2D (Centralized)               │
│  - Stores CurrentCamera                            │
│  - Computes VP = Projection × View                 │
│  - All shaders use unified u_VP uniform            │
└─────────────────────────────────────────────────────┘
```

## Benefits

### ✅ Single Source of Truth
All rendering now uses a **unified camera system**. The `CameraSystem` manages which camera is active, and all renderers query it through `Renderer2D`.

### ✅ Multiple Camera Support
Cameras can now have:
- **Priorities** (higher takes precedence)
- **Layers** (for future split-screen or viewport effects)
- **Follow targets** with configurable smoothing
- **Viewport regions** (for picture-in-picture)

### ✅ Easier to Add Features
Want to add:
- **Zoom?** Modify `CameraComponent::camera.zoom`
- **Pan?** Modify `CameraComponent::camera.position`
- **Rotation?** Modify `CameraComponent::camera.rotation`
- **Smooth follow?** Set `followTarget = true`, configure `followSmoothness`

### ✅ Consistent VP Matrices
No more hardcoded orthographic projections. All renderers use the same VP matrix calculated from the active camera's state.

### ✅ ECS Integration
Cameras are now first-class ECS citizens:
- Created as entities with components
- Managed by dedicated system
- Can be queried, modified, and controlled like any other entity

## Usage Example

### Creating a Camera Entity

```cpp
// Create camera entity
auto cameraEntity = entityManager.CreateEntity();

// Add transform (optional - for position/rotation)
ECS::TransformComponent cameraTransform;
cameraTransform.position = {640.0f, 360.0f};  // Center of 1280x720 screen

// Add camera component
ECS::CameraComponent camera;
camera.camera.position = {0.0f, 0.0f};  // World position
camera.camera.zoom = 1.0f;
camera.camera.rotation = 0.0f;
camera.priority = 10.0f;  // High priority
camera.isActive = true;
camera.layer = 0;

// Optional: set up follow behavior
camera.followTarget = true;
camera.targetEntity = playerEntity;  // Follow the player
camera.followOffset = {0.0f, -100.0f};  // Offset above player
camera.followSmoothness = 0.1f;  // Smooth follow (0 = instant)

componentStore.AddComponent(cameraEntity, cameraTransform);
componentStore.AddComponent(cameraEntity, camera);
```

### Switching Cameras at Runtime

```cpp
// Get camera system
auto& cameraSystem = systemManager.GetSystem<ECS::CameraSystem>();

// Create second camera with higher priority
auto camera2Entity = entityManager.CreateEntity();
ECS::CameraComponent camera2;
camera2.priority = 20.0f;  // Higher than default
camera2.camera.zoom = 2.0f;  // Zoomed in
// ... configure other settings

componentStore.AddComponent(camera2Entity, camera2);

// CameraSystem will automatically select the highest priority camera
// Or manually set active camera:
cameraSystem.SetActiveCamera(camera2Entity);
```

### Camera Follow Behavior

```cpp
// Camera will automatically follow target entity
ECS::CameraComponent camera;
camera.followTarget = true;
camera.targetEntity = playerEntity;
camera.followOffset = {100.0f, 100.0f};  // Offset from target
camera.followSmoothness = 0.05f;  // Very smooth, laggy follow

// CameraSystem.Update() handles the interpolation automatically
```

## Testing Recommendations

1. **Basic Camera Test**: Create a single camera entity and verify rendering uses correct view/projection
2. **Priority Test**: Create multiple cameras with different priorities, verify highest priority is active
3. **Follow Test**: Attach camera to moving entity, verify smooth following behavior
4. **Zoom/Pan Test**: Modify camera zoom/position at runtime, verify rendering updates
5. **Particle Test**: Verify particles render with correct camera transformation

## Files Modified

| File | Change Type | Description |
|------|-------------|-------------|
| `engine/include/nyon/ecs/components/CameraComponent.h` | Enhanced | Added `targetEntity` field |
| `engine/include/nyon/ecs/systems/CameraSystem.h` | **NEW** | Camera system header |
| `engine/src/ecs/systems/CameraSystem.cpp` | **NEW** | Camera system implementation |
| `engine/src/core/ECSApplication.cpp` | Updated | Register CameraSystem, simplify particle rendering |
| `engine/src/ecs/systems/ParticleRenderSystem.cpp` | Updated | Use unified camera, remove hardcoded VP |
| `engine/include/nyon/math/Vector2.h` | Enhanced | Added `Lerp()` utility function |

## Backward Compatibility

✅ **Fully backward compatible**

- Existing code without cameras continues to work (uses default orthographic projection)
- `Renderer2D::BeginScene()` can still be called directly with custom cameras
- All existing render components function unchanged
- No breaking changes to public APIs

## Future Enhancements

Potential improvements for future iterations:

1. **Camera Blending**: Smooth transitions between cameras
2. **Viewport Rendering**: Support for split-screen or picture-in-picture
3. **Camera Shake**: Add shake effects for impacts/explosions
4. **Frustum Culling**: Use camera VP for visibility testing
5. **Multiple Viewports**: Render different cameras to different screen regions
6. **Camera Boundaries**: Constrain camera to world bounds
7. **Zoom Limits**: Min/max zoom constraints

## Conclusion

The camera system fix centralizes view-projection matrix management, enables advanced camera features (zoom, pan, follow, multiple cameras), and integrates seamlessly with the ECS architecture. All rendering now flows through a single, unified camera pipeline.
