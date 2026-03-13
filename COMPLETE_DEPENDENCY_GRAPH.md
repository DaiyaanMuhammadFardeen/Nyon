# Complete Dependency Graph - Visual Map

## 📊 File Dependency Tree

```
┌─────────────────────────────────────────────────────────────────────┐
│                        ENTRY POINT                                   │
│              game/physics-demo/src/PhysicsDemoGame.cpp               │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     PhysicsDemoGame Class                            │
│  Inherits: ECSApplication → Application                              │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                ┌───────────────────┴───────────────────┐
                │                                       │
                ▼                                       ▼
┌───────────────────────────┐           ┌───────────────────────────┐
│    Core Framework          │           │     ECS Components        │
│                           │           │                           │
│  Application.h/cpp        │           │  TransformComponent.h     │
│  ECSApplication.h/cpp     │           │  PhysicsBodyComponent.h   │
│                           │           │  ColliderComponent.h      │
│  Dependencies:            │           │  PhysicsWorldComponent.h  │
│  - GLFW, OpenGL           │           │  RenderComponent.h        │
│  - glad.h                 │           │  JointComponent.h         │
│                           │           │  BehaviorComponent.h      │
└───────────────────────────┘           └───────────────────────────┘
                │                                       │
                │                                       │
                └───────────────────┬───────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      ECS Managers                                    │
│                                                                      │
│  EntityManager.h/cpp       ComponentStore.h/cpp     SystemManager   │
│  - CreateEntity()          - AddComponent<>         .h/cpp          │
│  - DestroyEntity()         - GetComponent<>         - AddSystem()   │
│  - GetActiveEntities()     - ForEachComponent<>     - Update()      │
│                            - HasComponent<>         - GetSystem()   │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    ECS Systems (4 Active)                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │ InputSystem.h/cpp                                            │   │
│  │ - Processes keyboard/mouse input                             │   │
│  │ - Depends on: InputManager.h                                 │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │ PhysicsPipelineSystem.h/cpp ⭐ THE MAIN PHYSICS SYSTEM       │   │
│  │                                                              │   │
│  │  Dependencies:                                               │   │
│  │  ├─ DynamicTree.h/cpp (broad phase)                         │   │
│  │  ├─ Island.h/cpp (sleeping)                                 │   │
│  │  ├─ ManifoldGenerator.h/cpp (contacts)                      │   │
│  │  └─ All Component headers                                    │   │
│  │                                                              │   │
│  │  Internal Structure:                                         │   │
│  │  ├─ ContactPoint, ContactManifold                           │   │
│  │  ├─ VelocityConstraint, PositionConstraint                  │   │
│  │  ├─ SolverBody                                               │   │
│  │  └─ BroadPhaseCallback                                      │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │ RenderSystem.h/cpp                                           │   │
│  │ - Depends on: Renderer2D.h/cpp                               │   │
│  │ - Draws all RenderComponents                                 │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │ DebugRenderSystem.h/cpp                                      │   │
│  │ - Renders debug overlays                                     │   │
│  │ - Depends on: Renderer2D.h/cpp                               │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                   Graphics Layer                                     │
│                                                                      │
│  Renderer2D.h/cpp                                                   │
│  - OpenGL-based 2D rendering                                        │
│  - Shape drawing: Quad, Circle, Polygon, etc.                       │
│  - Depends on: GLM (math library), glad.h                           │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                   Platform Layer                                     │
│                                                                      │
│  GLFW (window management)    GLAD (OpenGL loader)                   │
│  OpenGL (graphics API)       GLM (math library)                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 🔴 DEAD CODE - NOT REFERENCED ANYWHERE

```
┌─────────────────────────────────────────────────────────────────────┐
│ ❌ THESE FILES HAVE NO ACTIVE DEPENDENCIES                          │
│                                                                      │
│  CollisionPipelineSystem.h/cpp                                      │
│    → Replaced by PhysicsPipelineSystem                              │
│    → Never instantiated in ECSApplication                           │
│                                                                      │
│  BoundarySystem.h/cpp                                               │
│    → Boundaries created manually as entities                        │
│    → PhysicsDemoGame::CreateBounds() commented out                  │
│                                                                      │
│  ContinuousCollisionDetection.h/cpp                                 │
│    → Not integrated into main pipeline                              │
│    → PhysicsBodyComponent::isBullet flag exists but unused          │
│                                                                      │
│  ConstraintSolver.h/cpp                                             │
│    → Replaced by integrated solver in PhysicsPipelineSystem         │
│    → Duplicate functionality                                        │
│                                                                      │
│  SATCollisionDetector.h/cpp                                         │
│    → Standalone collision detector                                  │
│    → ManifoldGenerator used instead                                 │
│                                                                      │
│  StabilizationSystem.h/cpp                                          │
│    → Excluded from CMakeLists.txt build                             │
│    → Functionality absorbed into PositionSolving()                  │
│                                                                      │
│  PhysicsPipeline.h/cpp (old)                                        │
│    → Different from PhysicsPipelineSystem!                          │
│    → Legacy system, excluded from build                             │
│                                                                      │
│  PhysicsIntegrationSystem                                           │
│    → Only referenced in comments as "legacy"                        │
│    → No actual file exists                                          │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 🟢 ACTIVE CODE PATH - Runtime Call Graph

```
main()
  │
  ├─► PhysicsDemoGame constructor
  │     ├─► ECSApplication constructor
  │     │     ├─► Application constructor
  │     │     │     ├─► s_Instance = this
  │     │     │     └─► Init()
  │     │     │           ├─► glfwInit()
  │     │     │           ├─► glfwCreateWindow()
  │     │     │           ├─► gladLoadGLLoader()
  │     │     │           ├─► glViewport(), glEnable()
  │     │     │           └─► Renderer2D::Init()
  │     │     │
  │     │     ├─► EntityManager constructor
  │     │     ├─► ComponentStore constructor
  │     │     └─► SystemManager constructor
  │     │
  │     └─► [Game-specific initialization]
  │
  ├─► Application::Run()
  │     │
  │     ├─► OnStart() [VIRTUAL CALL]
  │     │     ├─► ECSApplication::OnStart()
  │     │     │     ├─► InputManager::Init()
  │     │     │     ├─► OnECSStart() [GAME HOOK]
  │     │     │     │     └─► PhysicsDemoGame::OnECSStart()
  │     │     │     │           ├─► CreatePhysicsWorld()
  │     │     │     │           │     └─► Entity with PhysicsWorldComponent
  │     │     │     │           ├─► CreateBounds() [CURRENTLY DISABLED]
  │     │     │     │           ├─► CreateStackTest()
  │     │     │     │           │     └─► 48 entities with components
  │     │     │     │           ├─► CreateRampFrictionTest()
  │     │     │     │           │     └─► 6 entities (ramp + boxes)
  │     │     │     │           └─► CreateBounceTest()
  │     │     │     │                 └─► 6 entities (balls)
  │     │     │     │
  │     │     │     ├─► SystemManager::AddSystem<InputSystem>()
  │     │     │     ├─► SystemManager::AddSystem<PhysicsPipelineSystem>()
  │     │     │     ├─► SystemManager::AddSystem<RenderSystem>()
  │     │     │     └─► SystemManager::AddSystem<DebugRenderSystem>()
  │     │     │
  │     │     └─► [Game continues...]
  │     │
  │     ├─► Main Loop (WHILE !glfwWindowShouldClose)
  │     │     │
  │     │     ├─► Calculate frameTime, m_Accumulator
  │     │     │
  │     │     ├─► ProcessInput()
  │     │     │     └─► Check ESC key
  │     │     │
  │     │     ├─► WHILE accumulator >= FIXED_TIMESTEP (1/60s)
  │     │     │     │
  │     │     │     └─► OnFixedUpdate(FIXED_TIMESTEP) [VIRTUAL]
  │     │     │           ├─► ECSApplication::OnFixedUpdate()
  │     │     │           │     ├─► SystemManager::Update()
  │     │     │           │     │     ├─► InputSystem::Update()
  │     │     │           │     │     ├─► PhysicsPipelineSystem::Update() ⭐
  │     │     │           │     │     │     │
  │     │     │           │     │     │     ├─► PrepareBodiesForUpdate()
  │     │     │           │     │     │     │     └─► Copy components to solver bodies
  │     │     │           │     │     │     │
  │     │     │           │     │     │     ├─► BroadPhaseDetection()
  │     │     │           │     │     │     │     ├─► Update DynamicTree proxies
  │     │     │           │     │     │     │     └─► Find overlapping AABB pairs
  │     │     │           │     │     │     │
  │     │     │           │     │     │     ├─► NarrowPhaseDetection()
  │     │     │           │     │     │     │     ├─► TestCollision() per pair
  │     │     │           │     │     │     │     └─► GenerateManifold() via ManifoldGenerator
  │     │     │           │     │     │     │
  │     │     │           │     │     │     ├─► IslandDetection()
  │     │     │           │     │     │     │     └─► IslandManager::UpdateIslands()
  │     │     │           │     │     │     │
  │     │     │           │     │     │     ├─► ConstraintInitialization()
  │     │     │           │     │     │     │     └─► Build velocity constraints
  │     │     │           │     │     │     │
  │     │     │           │     │     │     ├─► VelocitySolving()
  │     │     │           │     │     │     │     ├─► WarmStartConstraints()
  │     │     │           │     │     │     │     ├─► Apply gravity
  │     │     │           │     │     │     │     ├─► IntegrateVelocities()
  │     │     │           │     │     │     │     └─► SolveVelocityConstraints() × 8 iterations
  │     │     │           │     │     │     │
  │     │     │           │     │     │     ├─► PositionSolving()
  │     │     │           │     │     │     │     ├─► IntegratePositions()
  │     │     │           │     │     │     │     └─► SolvePositionConstraints() × 3 iterations
  │     │     │           │     │     │     │
  │     │     │           │     │     │     ├─► Integration()
  │     │     │           │     │     │     │     └─► Store velocities back to components
  │     │     │           │     │     │     │
  │     │     │           │     │     │     ├─► StoreImpulses()
  │     │     │           │     │     │     │     └─► Preserve for warm start
  │     │     │           │     │     │     │
  │     │     │           │     │     │     ├─► UpdateSleeping()
  │     │     │           │     │     │     │     └─► Update awake states from islands
  │     │     │           │     │     │     │
  │     │     │           │     │     │     └─► UpdateTransformsFromSolver()
  │     │     │           │     │     │           └─► Write positions to TransformComponent
  │     │     │           │     │     │
  │     │     │           │     │     └─► RenderSystem::Update()
  │     │     │           │     │             └─► [SKIPPED IN FixedUpdate]
  │     │     │           │     │
  │     │     │           │     ├─► OnECSFixedUpdate() [GAME HOOK]
  │     │     │           │     │     └─► PhysicsDemoGame::OnECSFixedUpdate()
  │     │     │           │     │           └─► HandleGlobalInput()
  │     │     │           │     │
  │     │     │           │     └─► OnECSUpdate() [GAME HOOK]
  │     │     │           │           └─► PhysicsDemoGame::OnECSUpdate()
  │     │     │           │
  │     │     │           └─► accumulator -= FIXED_TIMESTEP
  │     │     │
  │     │     ├─► Calculate alpha = accumulator / FIXED_TIMESTEP
  │     │     │
  │     │     └─► OnInterpolateAndRender(alpha) [VIRTUAL]
  │     │           ├─► ECSApplication::OnInterpolateAndRender()
  │     │           │     ├─► RenderSystem::SetInterpolationAlpha(alpha)
  │     │           │     ├─► RenderSystem::Update()
  │     │           │     │     ├─► glClear()
  │     │           │     │     ├─► Renderer2D::BeginScene()
  │     │           │     │     ├─► For each RenderComponent:
  │     │           │     │     │     ├─► Get interpolated position
  │     │           │     │     │     └─► Draw based on shape type
  │     │           │     │     └─► Renderer2D::EndScene()
  │     │           │     │
  │     │           │     └─► DebugRenderSystem::RenderDebugInfo()
  │     │           │           └─► Render contacts, AABBs, stats
  │     │           │
  │     │           ├─► glfwSwapBuffers()
  │     │           └─► glfwPollEvents()
  │     │
  │     └─► [LOOP BACK TO START IF RUNNING]
  │
  └─► Application destructor
        ├─► Renderer2D::Shutdown()
        ├─► glfwDestroyWindow()
        └─► glfwTerminate()
```

---

## 📦 Component Data Flow

```
┌──────────────────────────────────────────────────────────────────┐
│                    ENTITY COMPONENT STRUCTURE                     │
│                                                                   │
│  Entity ID: XXXX                                                  │
│  │                                                                │
│  ├─► TransformComponent                                          │
│  │   ├─ position: Vector2                                        │
│  │   ├─ rotation: float                                          │
│  │   ├─ previousPosition: Vector2  ← for interpolation           │
│  │   └─ previousRotation: float                                  │
│  │                                                                │
│  ├─► PhysicsBodyComponent                                        │
│  │   ├─ isStatic: bool                                           │
│  │   ├─ mass: float                                              │
│  │   ├─ inverseMass: float                                       │
│  │   ├─ velocity: Vector2                                        │
│  │   ├─ angularVelocity: float                                   │
│  │   ├─ inertia: float                                           │
│  │   ├─ inverseInertia: float                                    │
│  │   ├─ centerOfMass: Vector2                                    │
│  │   ├─ isAwake: bool                                            │
│  │   └─ material: {friction, restitution}                        │
│  │                                                                │
│  ├─► ColliderComponent                                           │
│  │   ├─ type: enum (Circle/Polygon)                              │
│  │   ├─ polygon/circle data                                      │
│  │   ├─ material: {friction, restitution}                        │
│  │   └─ filter: collision layers                                 │
│  │                                                                │
│  └─► RenderComponent                                             │
│      ├─ size: Vector2                                            │
│      ├─ color: Vector3                                           │
│      ├─ shapeType: enum                                          │
│      └─ visible: bool                                            │
└──────────────────────────────────────────────────────────────────┘
```

### Data Flow During Physics Update

```
Frame N
│
├─► READ (components → solver)
│   │
│   ├─ TransformComponent.position ──────┐
│   ├─ PhysicsBodyComponent.velocity ────┤
│   ├─ PhysicsBodyComponent.inverseMass ─┤
│   └─ PhysicsBodyComponent.isAwake ─────┤
│                                         ▼
│                              ┌─────────────────────┐
│                              │   SolverBody[]      │
│                              │  (temporary array)  │
│                              └─────────────────────┘
│
├─► SOLVE (physics calculations)
│   │
│   ├─ Detect collisions → ContactManifolds
│   ├─ Build constraints → VelocityConstraints
│   ├─ Solve impulses → Updated velocities
│   └─ Position correction → Updated positions
│
└─► WRITE (solver → components)
    │
    ├─ SolverBody.velocity ────► PhysicsBodyComponent.velocity
    ├─ SolverBody.angularVel ──► PhysicsBodyComponent.angularVelocity
    └─ SolverBody.position ────► TransformComponent.position
```

---

## 🎯 Single Point of Truth

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│  ⭐ PhysicsPipelineSystem IS THE SINGLE SOURCE OF TRUTH ⭐     │
│                                                                 │
│  ALL physics simulation happens here:                          │
│  ✓ Broad phase collision detection                             │
│  ✓ Narrow phase contact generation                             │
│  ✓ Constraint solving (velocity & position)                    │
│  ✓ Sleeping optimization                                       │
│  ✓ Integration                                                 │
│                                                                 │
│  NO OTHER system modifies physics state!                       │
│  (Legacy systems like CollisionPipelineSystem are dead)        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🗂️ Include Dependency Graph

```
PhysicsDemoGame.cpp
│
├─► "PhysicsDemoGame.h"
│
├─► "nyon/ecs/EntityManager.h"
├─► "nyon/ecs/ComponentStore.h"
├─► "nyon/ecs/SystemManager.h"
│
├─► "nyon/ecs/components/TransformComponent.h"
├─► "nyon/ecs/components/PhysicsWorldComponent.h"
├─► "nyon/ecs/components/PhysicsBodyComponent.h"
├─► "nyon/ecs/components/ColliderComponent.h"
├─► "nyon/ecs/components/RenderComponent.h"
│
├─► "nyon/utils/InputManager.h"
│
└─► <GLFW/glfw3.h>


PhysicsPipelineSystem.h
│
├─► "nyon/ecs/System.h"
│   ├─► "EntityManager.h"
│   └─► "ComponentStore.h"
│
├─► "nyon/ecs/components/PhysicsWorldComponent.h"
├─► "nyon/ecs/components/PhysicsBodyComponent.h"
├─► "nyon/ecs/components/ColliderComponent.h"
├─► "nyon/ecs/components/TransformComponent.h"
│
├─► "nyon/physics/Island.h"
│   └─► "nyon/ecs/ComponentStore.h"
│
├─► "nyon/physics/DynamicTree.h"
│   └─► "nyon/math/Vector2.h"
│
└─► <vector>, <unordered_map>


RenderSystem.h
│
├─► "nyon/ecs/System.h"
├─► "nyon/ecs/components/TransformComponent.h"
├─► "nyon/ecs/components/RenderComponent.h"
│
├─► "nyon/graphics/Renderer2D.h"
│   ├─► "nyon/math/Vector2.h"
│   ├─► "nyon/math/Vector3.h"
│   ├─► <glm/glm.hpp>
│   └─► <vector>, <string>, <memory>
│
└─► <glad/glad.h>
```

---

## 🏗️ Build System Dependencies

```
CMakeLists.txt (root)
│
├─► engine/CMakeLists.txt
│   │
│   ├─► Collects all .cpp files (GLOB_RECURSE)
│   ├─► Excludes:
│   │   ├─ RenderingDemo.cpp
│   │   ├─ PhysicsPipeline.cpp (legacy)
│   │   └─ StabilizationSystem.cpp
│   │
│   ├─► Creates: nyon_engine STATIC LIBRARY
│   └─► Links: OpenGL, glfw, glm
│
├─► game/physics-demo/CMakeLists.txt
│   │
│   ├─► Executable: nyon_physics_demo
│   ├─► Sources: PhysicsDemoGame.cpp
│   └─► Links: nyon_engine, glfw, OpenGL
│
└─► game/simple-test/CMakeLists.txt
    │
    ├─► Executable: nyon_simple_test
    ├─► Sources: main.cpp
    └─► Links: nyon_engine, glfw, OpenGL
```

---

## 📊 Summary Statistics

### File Counts

| Category | Active | Dead | Total |
|----------|--------|------|-------|
| Core Framework | 4 | 0 | 4 |
| ECS Managers | 6 | 0 | 6 |
| ECS Systems | 4 | 2 | 6 |
| Components | 7 | 0 | 7 |
| Graphics | 2 | 0 | 2 |
| Physics Modules | 3 | 11 | 14 |
| Utils & Math | 4 | 0 | 4 |
| **TOTAL** | **30** | **13** | **43** |

Note: Some counts differ slightly from earlier analysis due to header-only files.

### Lines of Code (Approximate)

| Component | LOC |
|-----------|-----|
| PhysicsPipelineSystem | ~850 |
| Renderer2D | ~600 |
| ECSApplication | ~110 |
| Application | ~160 |
| Component headers | ~800 |
| Manager code | ~300 |
| Dead code | ~2000+ |
| **Total active** | **~2820** |
| **Total with dead** | **~4820** |

**After cleanup: Remove ~2000 lines of dead code (42% reduction!)**

---

## 🎯 The One True Path

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│  If you remember ONLY this diagram:                            │
│                                                                 │
│                                                                 │
│  PhysicsDemoGame                                                │
│       ↓                                                         │
│  ECSApplication                                                 │
│       ↓                                                         │
│  SystemManager                                                  │
│       ↓                                                         │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ PhysicsPipelineSystem (THE ONE TRUE PHYSICS SYSTEM) ⭐   │   │
│  │                                                          │   │
│  │  Prep → Broad → Narrow → Islands → Constraints → Solve  │   │
│  │                                                          │   │
│  └──────────────────────────────────────────────────────────┘   │
│       ↓                                                         │
│  Update Transforms                                              │
│       ↓                                                         │
│  Render                                                         │
│                                                                 │
│                                                                 │
│  EVERYTHING ELSE IS DEAD WEIGHT - DELETE IT!                   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```
