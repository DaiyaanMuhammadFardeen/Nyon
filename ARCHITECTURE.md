# Nyon Engine Architecture

**Version**: 2D Game Engine  
**Language**: C++17  
**Rendering**: OpenGL 4.6 (core profile), GLFW, GLAD  
**Math**: GLM + custom 2D math library  
**Build**: CMake  

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Directory Layout](#2-directory-layout)
3. [Application Layer](#3-application-layer)
4. [ECS Framework](#4-ecs-framework)
5. [System Architecture (Tree)](#5-system-architecture-tree)
6. [Physics Pipeline](#6-physics-pipeline)
7. [Rendering Pipeline](#7-rendering-pipeline)
8. [Camera System](#8-camera-system)
9. [Particle System](#9-particle-system)
10. [Input System](#10-input-system)
11. [Component Reference](#11-component-reference)
12. [Multi-Threading](#12-multi-threading)
13. [Math Library](#13-math-library)
14. [Build System](#14-build-system)
15. [Known Limitations](#15-known-limitations)

---

## 1. Project Overview

Nyon is a **2D game engine** built around an **Entity-Component-System (ECS)** architecture. It provides:

- **OpenGL 4.6 GPU-instanced rendering** with 6 dedicated pipelines (quad, circle, line, capsule, polygon fill, polygon line)
- **Custom 2D physics engine** with DynamicTree broad-phase, SAT narrow-phase, and a sequential impulse solver
- **ECS-based particle system** with parallel physics updates and spatial-hash collision detection
- **Fully ECS-integrated camera system** with priority-based selection, smooth follow, and viewport support

External dependencies are minimal: **GLFW** (window/input), **GLAD** (OpenGL loader), and **GLM** (mathematics). The build system is **CMake**.

---

## 2. Directory Layout

```
Nyon/
├── CMakeLists.txt                  # Root build file
├── engine/
│   ├── CMakeLists.txt              # Engine static library build
│   ├── assets/
│   │   └── shaders/                # 14 GLSL shader files (6 pipelines)
│   │       ├── quad.vert / quad.frag
│   │       ├── quad_instanced.vert / quad_instanced.frag
│   │       ├── circle.vert / circle.frag
│   │       ├── circle_instanced.vert / circle_instanced.frag
│   │       ├── line.vert / line.frag
│   │       ├── capsule.vert / capsule.frag
│   │       └── polygon.vert / polygon.frag
│   ├── include/
│   │   └── nyon/
│   │       ├── core/
│   │       │   ├── Application.h
│   │       │   └── ECSApplication.h
│   │       ├── ecs/
│   │       │   ├── EntityManager.h
│   │       │   ├── ComponentStore.h
│   │       │   ├── System.h
│   │       │   ├── SystemManager.h
│   │       │   ├── components/
│   │       │   │   ├── BehaviorComponent.h
│   │       │   │   ├── CameraComponent.h
│   │       │   │   ├── ColliderComponent.h
│   │       │   │   ├── JointComponent.h
│   │       │   │   ├── ParticleComponent.h
│   │       │   │   ├── ParticleEmitterComponent.h
│   │       │   │   ├── PhysicsBodyComponent.h
│   │       │   │   ├── PhysicsWorldComponent.h
│   │       │   │   ├── RenderComponent.h
│   │       │   │   └── TransformComponent.h
│   │       │   └── systems/
│   │       │       ├── CameraSystem.h
│   │       │       ├── DebugRenderSystem.h
│   │       │       ├── InputSystem.h
│   │       │       ├── ParticlePipelineSystem.h
│   │       │       ├── ParticleRenderSystem.h
│   │       │       ├── PhysicsPipelineSystem.h
│   │       │       └── RenderSystem.h
│   │       ├── graphics/
│   │       │   ├── ParticleRenderer.h
│   │       │   ├── PhysicsDebugRenderer.h
│   │       │   └── Renderer2D.h
│   │       ├── math/
│   │       │   ├── Vector2.h
│   │       │   └── Vector3.h
│   │       ├── physics/
│   │       │   ├── ContactTypes.h
│   │       │   ├── DynamicTree.h
│   │       │   ├── Island.h
│   │       │   └── ManifoldGenerator.h
│   │       ├── utils/
│   │       │   ├── InputManager.h
│   │       │   └── ThreadPool.h
│   │       └── EngineConstants.h
│   └── src/
│       ├── core/
│       │   ├── Application.cpp
│       │   └── ECSApplication.cpp
│       ├── ecs/
│       │   ├── EntityManager.cpp
│       │   ├── ComponentStore.cpp
│       │   ├── SystemManager.cpp
│       │   └── systems/
│       │       ├── CameraSystem.cpp
│       │       ├── DebugRenderSystem.cpp
│       │       ├── ParticlePipelineSystem.cpp
│       │       ├── ParticleRenderSystem.cpp
│       │       ├── PhysicsPipelineSystem.cpp
│       │       └── RenderSystem.cpp
│       ├── graphics/
│       │   ├── ParticleRenderer.cpp
│       │   ├── PhysicsDebugRenderer.cpp
│       │   └── Renderer2D.cpp
│       ├── physics/
│       │   ├── DynamicTree.cpp
│       │   ├── Island.cpp
│       │   └── ManifoldGenerator.cpp
│       ├── utils/
│       │   ├── InputManager.cpp
│       │   └── ThreadPool.cpp
│       └── glad.c
├── game/
│   ├── simple-physics-demo/
│   ├── breakout-demo/
│   ├── flappy-demo/
│   └── tower-stack-demo/
└── test/
```

---

## 3. Application Layer

### 3.1 `Nyon::Application`

A **singleton** base class (`s_Instance` static pointer) that owns the GLFW window and runs the fixed-timestep game loop.

```
Application::Application(title, width, height)
  └─ s_Instance = this
  └─ Init()
       ├─ glfwInit()
       ├─ glfwCreateWindow(width, height, title)
       ├─ gladLoadGLLoader()
       ├─ glViewport / glEnable(GL_BLEND)
       └─ Renderer2D::Init()
```

**Virtual hooks for subclasses:**
| Method | When called | Purpose |
|---|---|---|
| `OnStart()` | Once, before main loop | Game initialization |
| `OnUpdate(dt)` | Each frame, after fixed-step loop | Per-frame game logic |
| `OnFixedUpdate(dt)` | Each fixed step (60 Hz) | Physics/game simulation |
| `OnInterpolateAndRender(alpha)` | Each frame, after physics | Rendering with interpolation |

### 3.2 Fixed-Timestep Game Loop

The loop in `Application::Run()` implements a **fixed-timestep accumulator pattern** with interpolation:

```cpp
// Pseudocode of the game loop
OnStart();

while (!glfwWindowShouldClose(window))
{
    double frameTime = currentTime - lastTime;
    currentTime = glfwGetTime();

    // Spiral-of-death protection: cap frame time
    if (frameTime > MAX_FRAME_TIME_D)   // 0.25 seconds
        frameTime = MAX_FRAME_TIME_D;

    accumulator += frameTime;

    // Input: save previous state, then poll
    InputManager::Update();
    glfwPollEvents();

    // Consume fixed timesteps
    while (accumulator >= FIXED_TIMESTEP_D)  // 1/60 second
    {
        OnFixedUpdate(FIXED_TIMESTEP_D);
        accumulator -= FIXED_TIMESTEP_D;
    }

    // Per-frame logic (once per frame)
    OnUpdate(frameTime);

    // Render with interpolation alpha
    alpha = accumulator / FIXED_TIMESTEP_D;
    OnInterpolateAndRender(alpha);

    glfwSwapBuffers(window);
}
```

**Key constants** (from `EngineConstants.h`):
| Constant | Value | Purpose |
|---|---|---|
| `FIXED_TIMESTEP_D` | `1.0 / 60.0` | Physics tick rate (double) |
| `FIXED_TIMESTEP` | `1.0f / 60.0f` | Physics tick rate (float) |
| `MAX_FRAME_TIME_D` | `0.25` | Spiral-of-death cap (double) |
| `MAX_FRAME_TIME` | `0.25f` | Spiral-of-death cap (float) |

### 3.3 `Nyon::ECSApplication`

Extends `Application` with full ECS orchestration. It **overrides** `OnStart()`, `OnFixedUpdate()`, and `OnInterpolateAndRender()` as `final`, providing virtual hooks for game code instead.

**Owns:**
- `ECS::EntityManager m_EntityManager`
- `ECS::ComponentStore m_ComponentStore`
- `ECS::SystemManager m_SystemManager`
- `std::unique_ptr<ECS::RenderSystem> m_RenderSystem`
- `std::unique_ptr<ECS::DebugRenderSystem> m_DebugRenderSystem`

**System registration order** (from `ECSApplication::OnStart()`):

```
1. InputSystem              // Input processing before everything
2. CameraSystem             // Camera updates before rendering
3. PhysicsPipelineSystem    // Physics after camera, before render
```

Note: `RenderSystem` is **not** added to `SystemManager` — it is called separately during the interpolation/render phase.

**ECSApplication override chain:**

```
OnStart() ──final──>
  ├─ InputManager::Init(window)
  ├─ OnECSStart()              ← game hook (create entities/components)
  ├─ Register ECS systems (ordered)
  ├─ Init RenderSystem
  └─ Init DebugRenderSystem

OnFixedUpdate(dt) ──final──>
  ├─ F1 toggle for debug overlay
  ├─ m_SystemManager.Update(dt)   ← runs InputSystem → CameraSystem → PhysicsPipelineSystem
  ├─ OnECSFixedUpdate(dt)         ← game hook
  └─ OnECSUpdate(dt)             ← game hook

OnInterpolateAndRender(alpha) ──final──>
  ├─ RenderSystem.SetInterpolationAlpha(alpha)
  ├─ RenderSystem.Update(0)     ← BeginScene + draw + EndScene
  ├─ DebugRenderSystem (if F1)  ← separate render pass
  └─ ParticleRenderSystem::Render(alpha)
```

---

## 4. ECS Framework

### 4.1 Entity ID

```cpp
using EntityID = uint32_t;
static constexpr EntityID INVALID_ENTITY = 0xFFFFFFFF;
```

### 4.2 EntityManager

Manages entity creation, destruction, and lifecycle.

```
EntityManager
├─ m_NextID: EntityID          ← next sequential ID
├─ m_EntityStates: vector<bool>  ← active/destroyed tracking
├─ m_ActiveEntities: unordered_set<EntityID>
└─ m_FreeIDs: vector<EntityID>   ← recycled destroyed IDs
```

**Key operations:**
- `CreateEntity()` — reuses from `m_FreeIDs` if available, otherwise uses `m_NextID`
- `DestroyEntity(entity)` — marks inactive, pushes ID to free list
- `DestroyEntity(entity, componentStore)` — also calls `RemoveAllComponents()`
- `IsEntityValid(entity)` — checks active state
- `GetActiveEntities()` — returns reference to the active set

### 4.3 ComponentStore

True **Structure-of-Arrays (SoA)** storage pattern using type-erased containers.

Internal structure per component type `T`:

```cpp
template<typename T>
struct ComponentContainer {
    std::vector<T> components;                    // Dense component data
    std::vector<EntityID> entityIds;               // Parallel entity ID array
    std::vector<bool> activeFlags;                 // Active flags
    std::unordered_map<EntityID, size_t> indexMap; // O(1) entity → index lookup
};
```

**Key operations:**

| Operation | Complexity | Notes |
|---|---|---|
| `AddComponent<T>(entity, component)` | Amortized O(1) | Updates existing if entity already has component |
| `RemoveComponent<T>(entity)` | O(1) | Swap-and-pop with last element |
| `GetComponent<T>(entity)` | O(1) | Throws `std::runtime_error` if missing |
| `HasComponent<T>(entity)` | O(1) | Checks both existence and active flag |
| `GetEntitiesWithComponent<T>()` | O(1) | Returns reference to `entityIds` vector |
| `ForEachComponent<T>(func)` | O(N) | Cache-friendly contiguous iteration |
| `RemoveAllComponents(entity)` | O(C) | Iterates all container types |
| `GetComponentCount<T>()` | O(1) | Returns `indexMap.size()` |

**Storage internals:**
- Top-level: `unordered_map<type_index, unique_ptr<IComponentContainer>>`
- Lazy container creation on first `AddComponent<T>()` call (via `GetOrCreateContainer<T>()`)
- Swap-and-pop removal keeps arrays dense with no fragmentation
- `GetComponent` throws on failure (strict contract), `HasComponent` is the safe check

### 4.4 System

Abstract base class:

```cpp
class System {
    virtual void Initialize(EntityManager&, ComponentStore&);
    virtual void Update(float deltaTime) = 0;
    virtual void Shutdown();
protected:
    EntityManager* m_EntityManager;
    ComponentStore* m_ComponentStore;
};
```

### 4.5 SystemManager

Ordered system execution with type-indexed O(1) lookup:

```
SystemManager
├─ m_Systems: vector<unique_ptr<System>>        ← ordered execution
└─ m_SystemLookup: unordered_map<type_index, System*>  ← O(1) lookup
```

- `AddSystem<T>(unique_ptr<T>)` — calls `Initialize()`, appends to vector, caches in lookup
- `Update(dt)` — iterates `m_Systems` in insertion order
- `GetSystem<T>()` — O(1) lookup via `m_SystemLookup`

---

## 5. System Architecture (Tree)

```
Application (singleton)
└── ECSApplication
      ├── EntityManager
      │     └── Active entity tracking, ID recycling
      ├── ComponentStore
      │     └── SoA storage, type-erased containers
      ├── SystemManager
      │     ├── InputSystem
      │     │     └── Processes input → BehaviorComponent callbacks
      │     ├── CameraSystem
      │     │     ├── Priority-based active camera selection
      │     │     ├── Follow-target smooth lerp
      │     │     └── Viewport / coordinate conversion
      │     └── PhysicsPipelineSystem
      │           ├── Broad-phase (DynamicTree)
      │           ├── Narrow-phase (SAT ManifoldGenerator)
      │           ├── Island detection (BFS)
      │           ├── Constraint solving (sequential impulse)
      │           └── Sleep management
      ├── RenderSystem (owned separately, not in SystemManager)
      │     ├── Active camera discovery
      │     ├── Interpolated entity rendering
      │     └── BeginScene / EndScene lifecycle
      ├── DebugRenderSystem (owned separately)
      │     └── Physics visualization (shapes, AABBs, contacts)
      └── ParticleRenderSystem (accessed via SystemManager)
            └── Instanced GPU rendering of particles

ParticlePipelineSystem (in SystemManager but called implicitly via physics tick)
      ├── Phase 1: Process emitters (spawn)
      ├── Phase 2: Parallel physics (gravity, drag, integration)
      ├── Phase 3: Particle-particle broadphase (spatial hash)
      ├── Phase 4: Particle-body collisions (TODO)
      ├── Phase 5: Lifecycle management (death callbacks)
      └── Phase 6: Cleanup dead particles
```

---

## 6. Physics Pipeline

The `PhysicsPipelineSystem` implements a complete 2D rigid-body physics engine inspired by Box2D. The pipeline runs in 11 ordered phases:

### 6.1 Pipeline Flow

```
Update(dt)
  ├─ Lazy init (find PhysicsWorldComponent)
  ├─ Detect sub-stepping (if any body > 400 px/s)
  ├─ Save pre-substep transforms (for interpolation)
  └─ For each sub-step:
       ├─ 1. PrepareBodiesForUpdate()
       │     ├─ Build SolverBody array from PhysicsBodyComponents
       │     ├─ Auto-compute mass & inertia from collider shape geometry
       │     └─ Categorize static/dynamic/awake bodies
       ├─ 2. BroadPhaseDetection() [or ParallelBroadPhase]
       │     ├─ Update shape AABBs in DynamicTree
       │     ├─ Fat AABB margins (40px), AABB_MULTIPLIER = 2.0
       │     └─ Query tree for overlapping proxy pairs
       ├─ 3. NarrowPhaseDetection() [or ParallelNarrowPhase]
       │     ├─ ManifoldGenerator dispatcher
       │     └─ Generate ContactManifolds from overlapping pairs
       ├─ 4. IslandDetection()
       │     ├─ BFS flood-fill contact graph
       │     └─ Build awake/sleeping island sets
       ├─ 5. ConstraintInitialization()
       │     ├─ Build VelocityConstraints from ContactManifolds
       │     ├─ Cache impulses for warm starting
       │     └─ Compute constraint masses (normalMass, tangentMass)
       ├─ 6. VelocitySolving(dt) [or ParallelVelocitySolving]
       │     ├─ Warm start constraints
       │     ├─ Sequential impulse iteration (configurable count)
       │     ├─ Normal impulse + friction (tangent) impulse
       │     └─ Baumgarte stabilization bias
       ├─ 7. PositionSolving(dt) [or ParallelPositionSolving]
       │     ├─ Position-based correction with slop
       │     └─ Max correction limits
       ├─ 8. Integration()
       │     ├─ Semi-implicit Euler (velocity first, then position)
       │     ├─ Apply gravity, damping, speed limits
       │     └─ Respect motion locks
       ├─ 9. StoreImpulses()
       │     └─ Cache normalImpulse + tangentImpulse per contact for warm starting
       ├─ 10. UpdateSleeping()
       │     ├─ Timer-based (SLEEP_THRESHOLD = 2.0 px/s)
       │     ├─ ANGULAR_SLEEP_THRESHOLD = 0.2 rad/s
       │     ├─ TIME_TO_SLEEP = 0.5 seconds
       │     └─ Cross-frame sleep state preservation
       └─ 11. UpdateTransformsFromSolver()
             └─ Copy solver positions back to TransformComponents
  └─ Restore pre-substep previousPosition for render interpolation
```

### 6.2 Broad-Phase: DynamicTree

An **AABB tree** (axis-aligned bounding box hierarchy) for efficient spatial queries. Inspired by Box2D's `b2DynamicTree`.

```
DynamicTree
├─ TreeNode pool (vector)
├─ AABB_EXTENSION = 10.0 px  (fat AABB margin)
├─ AABB_MULTIPLIER = 2.0     (movement margin multiplier)
├─ Node pool growth: NODE_CAPACITY_INCREMENT = 16
├─ Tree balancing via Balance() (SAH-inspired rotation)
└─ Queries: Query(AABB), RayCast(origin, direction)
```

**Fat AABB strategy:** Each proxy's AABB is extended by `AABB_EXTENSION` pixels on each side, plus `AABB_MULTIPLIER × displacement`. This reduces tree update frequency for fast-moving objects.

### 6.3 Narrow-Phase: ManifoldGenerator

Dispatches collision detection based on shape type pairs:

| Pair | Algorithm |
|---|---|
| Circle × Circle | Distance-based with radius sum |
| Circle × Polygon | Closest point on polygon edges to circle center |
| Polygon × Polygon | **SAT** (Separating Axis Theorem) with face normals |
| Circle × Capsule | Distance to capsule segment + circle radius |
| Polygon × Capsule | SAT with capsule as swept circle |
| Capsule × Capsule | Segment-segment distance + combined radii |
| Segment × Segment | Segment distance test |

**Manifold output** (`ContactManifold`):
- Up to 2 contact points
- Contact normal (pointing from A to B)
- Separation distance (negative = penetration)
- Combined friction/restitution
- Feature IDs for contact persistence

### 6.4 Solver Architecture

**Velocity constraint structure:**

```cpp
struct VelocityConstraint {
    Vector2 normal, tangent;
    vector<ContactPointConstraint> points;
    uint32_t indexA, indexB;  // indices into SolverBody array
    float friction, restitution;
    float invMassA, invMassB, invIA, invIB;
};
```

**Solving steps:**
1. **Warm start** — apply cached impulses from previous frame
2. **Velocity iteration** (× `velocityIterations`):
   - Compute relative velocity at contact point
   - Compute normal impulse (with restitution bias)
   - Compute tangent impulse (friction, clamped to μ × normal impulse)
   - Accumulate impulses
3. **Position iteration** (× `positionIterations`):
   - Compute position correction (Baumgarte: `beta / dt × penetration`)
   - Apply with slop threshold (`linearSlop = 0.5`)
   - Clamp to `maxLinearCorrection`

### 6.5 Island System

BFS flood-fill on the contact graph to identify connected components (islands):

```
IslandManager
├─ BuildContactGraph()  ← adjacency from contact pairs
├─ FindIslands()        ← BFS with FloodFill
├─ UpdateSleepTimers()  ← accumulate stationary time
├─ PutIslandsToSleep()  ← if below thresholds for TIME_TO_SLEEP
└─ WakeSleepingIslands() ← if body moves above wake threshold
```

**Constants:**
| Parameter | Value | Description |
|---|---|---|
| `SLEEP_THRESHOLD` | 2.0 px/s | Linear speed below which body can sleep |
| `ANGULAR_SLEEP_THRESHOLD` | 0.2 rad/s | Angular speed below which body can sleep |
| `TIME_TO_SLEEP` | 0.5 s | Duration below threshold before sleep |
| `LINEAR_WAKE_THRESHOLD` | 2.0 px/s | Speed required to wake |
| `ANGULAR_WAKE_THRESHOLD` | 0.5 rad/s | Angular speed to wake |

### 6.6 Sub-Stepping

If any body exceeds `SUBSTEP_SPEED_THRESHOLD` (400 px/s), the physics pipeline runs in 2 sub-steps. This prevents tunneling and improves stability for high-speed objects.

---

## 7. Rendering Pipeline

### 7.1 `Renderer2D` — Static Singleton with PIMPL

```cpp
class Renderer2D {
    static void Init();
    static void Shutdown();
    static void BeginScene(const Camera2D&);
    static void EndScene();
    // ... draw calls ...
private:
    struct Impl;
    static unique_ptr<Impl> s_Instance;
};
```

All public API is static. The implementation (`Impl` struct) is defined in the `.cpp` file.

### 7.2 Six GPU Pipelines

| Pipeline | Draw Mode | Instance Data | Max Instances |
|---|---|---|---|
| **Quad** | `glDrawArraysInstanced` (6-vertex unit quad) | 40 bytes (pos, size, origin, angle, color) | 131,072 |
| **Circle** | `glDrawArraysInstanced` (SDF fragment shader, 16-seg unit circle) | 28 bytes (center, radius, color, outline mode) | 524,288 |
| **Line** | `glDrawArraysInstanced` (unit rect mesh) | 32 bytes (p0, p1, color, thickness) | 262,144 |
| **Capsule** | `glDrawArraysInstanced` (pre-built capsule mesh) | 36 bytes (c1, c2, radius, color, outline) | 32,768 |
| **Poly Fill** | CPU fan tessellation → persistent-mapped VBO, `GL_TRIANGLES` | Vertex data (position, color, UV, normal) | 65,536 verts |
| **Poly Line** | CPU outline tessellation → persistent-mapped VBO, `GL_LINES` | Vertex data | 65,536 verts |

### 7.3 Triple-Buffering

All instance VBOs use `GL_ARB_buffer_storage` (GL 4.4+) with **persistent coherent mapping** and `NUM_FRAMES = 3` triple-buffering, guarded by `GLsync` fences:

```cpp
static constexpr int NUM_FRAMES = 3;
int CurrentFrame = 0;
GLsync Fences[NUM_FRAMES] = {};
QuadInstance* QuadInstBase = nullptr;  // persistently mapped GPU pointer
```

Each frame writes to the current frame's region, fences are checked before reuse to prevent GPU/CPU desync.

### 7.4 CPU-Side Tessellation

For filled polygons and lines, the CPU tessellates world-space geometry:
- **Circle fan**: 16-segment triangle fan for filled circles
- **Polygon fan**: Ear-clipping-style fan from centroid
- **Capsule**: Rectangle body + two semicircles of 8 segments each
- **Ellipse**: Segmented triangle fan with elliptical radius
- **Arc**: Line strip along arc
- **Sector**: Triangle fan from center to arc endpoints

### 7.5 Camera2D

```cpp
struct Camera2D {
    Vector2 position;
    float zoom = 1.0f;
    float rotation = 0.0f;
    float nearPlane = -1.0f;
    float farPlane = 1.0f;

    glm::mat4 GetViewMatrix() const;             // rotate(-rot) × translate(-pos)
    glm::mat4 GetProjectionMatrix(w, h) const;   // glm::ortho(0, w/zoom, 0, h/zoom)
    glm::mat4 GetViewProjectionMatrix(w, h) const;

    Vector2 ScreenToWorld(screenPos, w, h) const;
    Vector2 WorldToScreen(worldPos, w, h) const;
};
```

OpenGL coordinate convention: **Y-up** with origin at bottom-left.

### 7.6 Shader Files (14 total)

| File Pair | Pipeline |
|---|---|
| `quad.vert` / `quad.frag` | Quad (instanced) |
| `quad_instanced.vert` / `quad_instanced.frag` | Quad (instanced variant) |
| `circle.vert` / `circle.frag` | Circle (instanced, SDF) |
| `circle_instanced.vert` / `circle_instanced.frag` | Circle instanced variant |
| `line.vert` / `line.frag` | Line (instanced) |
| `capsule.vert` / `capsule.frag` | Capsule (instanced) |
| `polygon.vert` / `polygon.frag` | Polygon fill/line |

### 7.7 RenderSystem

Not registered in `SystemManager` — owned directly by `ECSApplication` and called during `OnInterpolateAndRender()`:

```cpp
void RenderSystem::Update(dt) {
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    // Find active camera (highest priority, isActive = true)
    // Fall back to default ortho if no camera entities exist

    Renderer2D::BeginScene(activeCamera);

    // For each entity with RenderComponent + TransformComponent:
    //   Use interpolated position/rotation
    //   Dispatch DrawQuad or DrawSolidCircle based on shapeType

    Renderer2D::EndScene();
}
```

---

## 8. Camera System

### 8.1 CameraComponent

Wraps `Graphics::Camera2D` with ECS metadata:

```cpp
struct CameraComponent {
    Graphics::Camera2D camera;
    bool isActive = true;
    float priority = 0.0f;          // Higher = selected first
    int layer = 0;
    float viewportX, viewportY;     // Normalized viewport (0-1)
    float viewportWidth, viewportHeight;
    bool followTarget = false;
    EntityID targetEntity = INVALID_ENTITY;
    Vector2 followOffset;
    float followSmoothness = 0.0f;  // 0 = instant, >0 = lerp factor
};
```

### 8.2 CameraSystem

Registered in `SystemManager`, updated during `OnFixedUpdate()`:

```
CameraSystem::Update(dt)
  ├─ UpdateCameraFollow(dt)
  │     └─ If followTarget enabled:
  │           target = targetTransform.position + followOffset
  │           smoothness = clamp(followSmoothness, 0, 1)
  │           t = 1 - pow(1 - smoothness, dt × 60)
  │           camera.position = lerp(current, target, t)
  └─ SyncWithRenderer(screenWidth, screenHeight)
        └─ Set Renderer2D's active camera from ECS camera
```

**Priority-based selection** (`SelectBestCamera`): iterates all camera entities, picks the one with the highest `priority` where `isActive == true`.

### 8.3 Coordinate Conversion

```cpp
// Screen → World: unproject through inverse VP matrix
Vector2 Camera2D::ScreenToWorld(screenPos, screenWidth, screenHeight);

// World → Screen: project through VP matrix
Vector2 Camera2D::WorldToScreen(worldPos, screenWidth, screenHeight);
```

---

## 9. Particle System

### 9.1 Architecture

Particles are **full ECS entities** with `TransformComponent`, `PhysicsBodyComponent`, `ColliderComponent`, and `ParticleComponent`. The `ParticlePipelineSystem` handles simulation, the `ParticleRenderSystem` handles GPU-instanced rendering.

```
Particle Entity Components:
├─ TransformComponent    → position, previousPosition (interpolation)
├─ PhysicsBodyComponent  → velocity, mass, drag, gravity
├─ ColliderComponent     → circle/polygon shape, radius, filter
└─ ParticleComponent     → lifetime, age, alpha, color, sizeScale

Emitter Entity Components:
├─ TransformComponent
└─ ParticleEmitterComponent → spawn rate, burst, shape, params
```

### 9.2 ParticlePipelineSystem — 6-Phase Architecture

```
ProcessEmitters(dt) — Main Thread
  ├─ For each emitter entity:
  │     ├─ Accumulate spawnTimer += dt × spawnRate
  │     ├─ While spawnTimer ≥ 1.0 and count < max:
  │     │     ├─ Sample spawn position (Point/Circle/Rectangle/Annulus)
  │     │     ├─ Create particle entity with all 4 components
  │     │     ├─ Sample random spawn params (speed, angle, lifetime, etc.)
  │     │     └─ Call onSpawn callback
  │     ├─ Handle burstCount (spawn N immediately)
  │     └─ Update currentCount

UpdateParticlePhysicsParallel(start, end, dt) — ThreadPool
  ├─ For each particle in range:
  │     ├─ Apply gravity (from PhysicsWorldComponent × gravityScale)
  │     ├─ Apply drag (body.drag exponential decay)
  │     ├─ Clamp speed to maxLinearSpeed
  │     ├─ Euler integration: position += velocity × dt
  │     ├─ Update age, interpolate alpha/color/sizeScale
  │     └─ Store previous* values for interpolation

DetectParticleCollisions — Spatial Hash / Brute Force
  ├─ Build spatial grid (cellSize = 50px)
  ├─ For each particle, check neighbors in same cell
  │     └─ Resolve overlap, apply restitution

DetectParticleBodyCollisions — (TODO)

ProcessParticleLifecycle(dt) — Main Thread
  └─ Mark particles as dead if age ≥ lifetime
       └─ Call onDeath callback

CleanupDeadParticles() — Main Thread
  ├─ Destroy dead particle entities
  └─ Update emitter currentCount
```

### 9.3 ParticleEmitterComponent

```cpp
struct ParticleEmitterComponent {
    // Spawn control
    float spawnRate = 10.0f;       // particles/sec
    uint32_t burstCount = 0;       // instant burst
    uint32_t maxParticles = 1000;
    bool loop = true;
    bool active = true;

    // Emission shape
    enum EmissionShape { Point, Circle, Rectangle, Annulus };
    EmissionShape emissionShape = Point;
    float emissionRadius = 0.0f;
    Vector2 emissionSize{0, 0};

    // Spawn parameter ranges (min/max, sampled uniformly)
    ParticleSpawnParams spawnParams;

    // Physics
    float gravityScale = 1.0f;
    bool collidesWithBodies = false;
    bool collidesWithParticles = true;

    // Callbacks
    function<void(EntityID)> onSpawn;
    function<void(EntityID, float)> onUpdate;
    function<void(EntityID)> onDeath;
    function<void(EntityID, EntityID)> onCollision;
};
```

### 9.4 ParticleRenderer — GPU Instanced

Designed for high particle counts (up to 4M):

```cpp
class ParticleRenderer {
    static constexpr uint32_t MAX_PARTICLES = 4'000'000;
    static constexpr int CIRCLE_SEGMENTS = 16;

    // Per-instance data: 32 bytes
    struct alignas(16) ParticleInstance {
        float x, y;          // world position (8 bytes)
        float angle;         // rotation (4 bytes)
        float radius;        // size (4 bytes)
        float r, g, b;       // color (12 bytes)
        float aspectRatio;   // width/height for quads (4 bytes)
    };  // Total: 32 bytes

    // Circle VAO + Quad VAO, each with mesh VBO + instance VBO
    void SubmitCircle(x, y, radius, r, g, b);
    void SubmitQuad(x, y, halfExtent, angle, r, g, b, aspectRatio);
    void Flush(viewProjection);  // glDrawArraysInstanced
};
```

---

## 10. Input System

### 10.1 InputManager

Double-buffered key/mouse state with GLFW callbacks:

```
InputManager (static)
├─ s_CurrentKeys[GLFW_KEY_LAST]
├─ s_PreviousKeys[GLFW_KEY_LAST]
├─ s_CurrentMouseButtons[GLFW_MOUSE_BUTTON_LAST]
├─ s_PreviousMouseButtons[GLFW_MOUSE_BUTTON_LAST]
├─ GLFW key/mouse callbacks
└─ Update() → copies current → previous, then GLFW poll updates current
```

**Query methods:**
| Method | Returns `true` when |
|---|---|
| `IsKeyPressed(key)` | Key just went down this frame |
| `IsKeyDown(key)` | Key is currently held |
| `IsKeyUp(key)` | Key was released this frame |
| `IsMousePressed(btn)` | Mouse button just went down |
| `IsMouseDown(btn)` | Mouse button is currently held |
| `IsMouseUp(btn)` | Mouse button was released |

### 10.2 InputSystem

ECS system that dispatches input to `BehaviorComponent` callbacks:

```cpp
InputSystem::Update(dt) {
    for each entity with BehaviorComponent:
        behavior.Update(entity, dt);
}
```

`InputManager::Update()` is called at the top of `Application::Run()` before `glfwPollEvents()` to ensure correct transition detection.

---

## 11. Component Reference

| Component | Header | Key Fields | Purpose |
|---|---|---|---|
| **TransformComponent** | `TransformComponent.h` | `position`, `previousPosition`, `scale`, `rotation`, `previousRotation` | Spatial state with interpolation support. `PrepareForUpdate()` copies current→previous. `GetInterpolatedPosition(alpha)` returns smooth render position. |
| **RenderComponent** | `RenderComponent.h` | `size` (Vector2), `color` (Vector3), `origin`, `shapeType` (Rectangle/Circle/Polygon), `texturePath`, `visible`, `layer` | Visual representation. Layer controls draw order. |
| **PhysicsBodyComponent** | `PhysicsBodyComponent.h` | `velocity`, `force`, `mass`, `inverseMass`, `inertia`, `inverseInertia`, `friction`, `restitution`, `angularVelocity`, `torque`, `isStatic`, `isKinematic`, `isBullet`, `isAwake`, `motionLocks`, `drag`, `angularDamping`, `maxLinearSpeed`, `maxAngularSpeed`, `centerOfMass` | Rigid body dynamics. Auto-computes mass/inertia from collider shape. Body type flags: static (immovable), kinematic (user-controlled, affects dynamics), dynamic (full simulation). |
| **ColliderComponent** | `ColliderComponent.h` | `variant<Circle,Polygon,Capsule,Segment,Chain,Composite>`, `Filter {categoryBits, maskBits, groupIndex}`, `isSensor`, `material {friction, restitution, density}`, `density`, `color` | Collision shape with filtering, sensing, and material properties. `CalculateAABB()` handles rotation. `CalculateArea()` uses shoelace. `CalculateInertiaPerUnitMass()` computes shape-correct inertia. |
| **PhysicsWorldComponent** | `PhysicsWorldComponent.h` | `gravity` (default: {0, -980} px/s²), `timeStep`, `velocityIterations` (8), `positionIterations` (3), `subStepCount` (4), `baumgarteBeta` (0.2), `linearSlop` (0.5), `enableSleep`, `enableWarmStarting`, `enableContinuous`, `contactManifolds`, `callbacks {beginContact, endContact, preSolve, postSolve, jointBreak, sensorBegin, sensorEnd}`, `profile`, `counters` | Singleton physics world config. Stores contact manifolds after narrow-phase. Event callbacks for contact/sensor lifecycle. |
| **CameraComponent** | `CameraComponent.h` | `Camera2D camera`, `isActive`, `priority`, `layer`, `viewport {x,y,width,height}`, `followTarget`, `targetEntity`, `followOffset`, `followSmoothness` | ECS camera with priority, viewport, and follow-target features. |
| **ParticleComponent** | `ParticleComponent.h` | `lifetime`, `age`, `alive`, `alpha`, `alphaStart`, `alphaEnd`, `colorStart`, `colorEnd`, `sizeScale`, `emitterEntityId`, `userData`, `prev*` interpolation fields | Particle lifecycle and visual interpolation. |
| **ParticleEmitterComponent** | `ParticleEmitterComponent.h` | `spawnRate`, `burstCount`, `maxParticles`, `loop`, `active`, `emissionShape` (Point/Circle/Rectangle/Annulus), `spawnParams` (min/max ranges for speed, angle, radius, mass, lifetime, drag, restitution, friction, color), `gravityScale`, `collidesWithBodies`, `collidesWithParticles`, `onSpawn/onUpdate/onDeath/onCollision` callbacks | Configurable particle emitter with emission shapes and range-based spawn parameters. |
| **BehaviorComponent** | `BehaviorComponent.h` | `UpdateFunction(entity, dt)`, `CollisionFunction(entity, other)` | Attach custom logic to entities via std::function callbacks. |
| **JointComponent** | `JointComponent.h` | `type` (Distance/Revolute/Prismatic/Weld/Wheel/Motor), `entityIdA`, `entityIdB`, `localAnchorA/B`, `distanceJoint`, `revoluteJoint`, `prismaticJoint`, `weldJoint`, `wheelJoint`, `motorJoint`, `breakForce`, `breakTorque`, `collideConnected` | **⚠️ Joint solver NOT implemented.** Component is fully defined but has no physical effect. |

---

## 12. Multi-Threading

### 12.1 ThreadPool

Singleton thread pool (`Utils::ThreadPool`) with work-stealing queue pattern:

```
ThreadPool (static singleton)
├─ m_Workers: vector<thread>
├─ m_Tasks: queue<function<void()>>
├─ mutex + condition_variable
├─ m_ActiveTasks: atomic<size_t>  (for WaitAll)
├─ m_Stop: atomic<bool>
├─ m_AllDoneCondition
└─ tls_IsWorkerThread (thread_local, deadlock prevention)

Initialize(numThreads)   → creates workers
Submit(f, args...)       → returns future<T>
WaitAll()                → blocks until all tasks complete
```

**Deadlock prevention:** `tls_IsWorkerThread` is set to `true` in worker threads. If `WaitAll()` is called from a worker thread, it detects this and the caller must handle it (assertion/documentation).

### 12.2 Parallelization Targets

| System | Parallel Work | Granularity |
|---|---|---|
| **PhysicsPipelineSystem** | `ParallelBroadPhase()` — query tree per body | Per-body tree query |
| | `ParallelNarrowPhase()` — manifold generation | Per-pair manifold generation |
| | `ParallelVelocitySolving()` — constraint solving | Per-constraint warm start + solve |
| | `ParallelPositionSolving()` — position correction | Per-constraint position solve |
| **ParticlePipelineSystem** | `UpdateParticlePhysicsParallel()` — gravity, drag, integration | Per-particle batch (disjoint ranges) |
| | `DetectParticleCollisionsParallel()` — spatial hash + collision | Per-cell collision pairs |

All parallel work uses `ThreadPool::Submit()` with `std::future` synchronization.

---

## 13. Math Library

### 13.1 Vector2

```cpp
struct Vector2 {
    float x, y;
    // Arithmetic: +, -, *, /, +=, -=, *=, /=, unary negation
    float Length();
    float LengthSquared();
    Vector2 Normalize();
    static float Dot(a, b);
    static float Cross(a, b);         // scalar = a.x*b.y - a.y*b.x
    static Vector2 Cross(s, v);      // s × v = (-s*v.y, s*v.x)
    static Vector2 Cross(v, s);      // v × s = (s*v.y, -s*v.x)
    static float DistanceSquared(a, b);
    static Vector2 Lerp(a, b, t);
};
```

### 13.2 Rotation2D

Efficient 2D rotation stored as cos/sin pair:

```cpp
struct Rotation2D {
    float c, s;   // cos(angle), sin(angle)
    Rotation2D() : c(1), s(0) {}
    explicit Rotation2D(float angle);
    Rotation2D operator*(Vector2 v) const;   // rotate v
    Rotation2D Inverse() const;               // transpose = inverse
};
```

### 13.3 Vector3

```cpp
struct Vector3 {
    float x, y, z;
    // Arithmetic: +, -, *, /, +=, -=, *=, /=, unary negation
    float Length();
    float LengthSquared();
    Vector3 Normalize();
    Vector3 Cross(other);
    float Dot(other);
};
```

Note: `Vector3.h` includes `union { struct { float x, y, z; }; struct { float r, g, b; }; }` style member aliases in some versions. There is an untested `v.xy()` swizzle function defined but not validated.

---

## 14. Build System

### 14.1 CMake Structure

```
Root CMakeLists.txt
├── engine/CMakeLists.txt           → nyon_engine static library
├── game/simple-physics-demo/       → demo executable
├── game/breakout-demo/             → demo executable
├── game/flappy-demo/               → demo executable
└── game/tower-stack-demo/          → demo executable
```

**Engine library** (`engine/CMakeLists.txt`):
- C++17 required
- Dependencies: OpenGL, GLFW, GLM
- Source files: `GLOB_RECURSE src/*.cpp` + `src/glad.c`
- Excluded sources:
  - `.*/RenderingDemo\.cpp$` (sample code)
  - `.*/PhysicsPipeline\.cpp$` (compilation issues, legacy)
  - `.*/StabilizationSystem\.cpp$` (compilation issues, legacy)

### 14.2 Shader Path Resolution

Shaders are loaded relative to the executable at runtime:

```
nyonDir = exe.parent_path().parent_path().parent_path().parent_path()
          e.g., .../build/game/simple-physics-demo/ → .../Nyon/
shaderDir = nyonDir / "engine" / "assets" / "shaders/"
```

Fallback: `../../../engine/assets/shaders/` (for running from `build/game/simple-physics-demo/`).

---

## 15. Known Limitations

### 15.1 Joint Solver NOT Implemented

`JointComponent` defines 6 joint types (Distance, Revolute, Prismatic, Weld, Wheel, Motor) with full data structures, but **no joint solver exists**. Creating joints has no physical effect. Joint-related systems and code paths are disabled.

### 15.2 Excluded Source Files

The following files are **excluded from the build** via `CMakeLists.txt` `FILTER` rules:

| File | Reason |
|---|---|
| `PhysicsPipeline.cpp` | Compilation issues — legacy pipeline replaced by `PhysicsPipelineSystem` |
| `StabilizationSystem.cpp` | Compilation issues — stabilization merged into pipeline |
| `RenderingDemo.cpp` | Sample/demo code, not engine core |

### 15.3 Particle-Body Collisions (TODO)

Phase 4 of the `ParticlePipelineSystem` (`DetectParticleBodyCollisions()`) is a **stub/empty function**. Particles can collide with each other (Phase 3) but not with physics bodies. This requires integrating particle colliders with the main physics DynamicTree.

### 15.4 Vector3 Has No Swizzle Accessor

`Vector3.h` does not define an `xy()` member or any swizzle accessor. To obtain xy components, construct a `Vector2{x, y}` explicitly.

### 15.5 No Texture Support

`RenderComponent.texturePath` is defined but the rendering pipeline does not implement texture-based rendering. All shapes are drawn with solid colors. Texture rendering would require a new GPU pipeline with UV coordinate handling and texture binding.

### 15.6 No Hierarchical Transforms

`TransformComponent` has no parent-child hierarchy. All transforms are in world space. There is no transform tree or local-to-world matrix computation.

### 15.7 No Audio System

The engine has no audio subsystem. No sound loading, playback, or mixing capabilities exist.

### 15.8 No Asset Pipeline

Assets (shaders excepted) are not managed by the engine. There is no asset registry, loading system, or hot-reloading. Shaders are loaded from filesystem paths computed relative to the executable.

---

*This document reflects the Nyon engine codebase as of June 2026. Engine architecture is subject to change.*
