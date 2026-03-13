# Runtime Execution Flow - Visual Graph

## Main Game Loop (60 FPS Physics, Variable Render)

```
┌─────────────────────────────────────────────────────────────────┐
│                    PhysicsDemoGame::main()                      │
│                         (Entry Point)                            │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│              PhysicsDemoGame Constructor (line 19-22)           │
│   → Calls ECSApplication constructor                            │
│   → Calls Application constructor                               │
│   → Creates GLFW window & OpenGL context                        │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                  Application::Run() (line 87-141)               │
│                   MAIN GAME LOOP STARTS                         │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │       Application::OnStart()          │
        │         (Called once at start)        │
        └───────────────────────────────────────┘
                        │
                        ▼
        ┌───────────────────────────────────────┐
        │      ECSApplication::OnStart()        │
        │         (line 32-56)                  │
        │                                       │
        │  1. InputManager::Init()              │
        │  2. OnECSStart() ← GAME HOOK          │
        │     └─ PhysicsDemoGame::OnECSStart()  │
        │        ├─ CreatePhysicsWorld()        │
        │        │  └─ Entity with              │
        │        │     PhysicsWorldComponent    │
        │        ├─ CreateBounds() [DISABLED]   │
        │        ├─ CreateStackTest()           │
        │        │  └─ 48 boxes (6×8 stack)     │
        │        ├─ CreateRampFrictionTest()    │
        │        │  └─ Ramp + 5 boxes           │
        │        └─ CreateBounceTest()          │
        │           └─ 6 balls with varying     │
        │              restitution              │
        │                                       │
        │  3. Add ECS Systems:                  │
        │     ├─ InputSystem                    │
        │     ├─ PhysicsPipelineSystem ⭐       │
        │     ├─ RenderSystem                   │
        │     └─ DebugRenderSystem              │
        │                                       │
        │  4. Cache DebugRenderSystem pointer   │
        └───────────────────────────────────────┘
                        │
                        ▼
        ┌═══════════════════════════════════════════════┐
        ║           MAIN LOOP WHILE RUNNING             ║
        ║      (Application::Run - line 102-137)        ║
        └═══════════════════════════════════════════════┘
                        │
        ┌───────────────▼───────────────┐
        │  Calculate frameTime & delta  │
        │  m_Accumulator += frameTime   │
        └───────────────┬───────────────┘
                        │
        ┌───────────────▼───────────────┐
        │    ProcessInput()             │
        │  └─ Check ESC key             │
        └───────────────┬───────────────┘
                        │
        ┌───────────────▼───────────────┐
        │  WHILE accumulator >= 1/60s   │
        │    (Fixed Timestep Loop)      │
        └───────────────┬───────────────┘
                        │
                        ▼
        ┌───────────────────────────────────────────────────────────┐
        │         ECSApplication::OnFixedUpdate(deltaTime)          │
        │                  (line 58-76)                             │
        │                                                           │
        │  1. SystemManager.Update()                                │
        │     Updates ALL systems in order:                         │
        │                                                           │
        │     ┌─────────────────────────────────────────────────┐   │
        │     │ A. InputSystem::Update()                        │   │
        │     │    - Processes keyboard/mouse input             │   │
        │     │    - Applies forces to bodies                   │   │
        │     └─────────────────────────────────────────────────┘   │
        │                          │                                │
        │                          ▼                                │
        │     ┌─────────────────────────────────────────────────┐   │
        │     │ B. PhysicsPipelineSystem::Update() ⭐           │   │
        │     │    (line 46-127, detailed below)                │   │
        │     └─────────────────────────────────────────────────┘   │
        │                          │                                │
        │                          ▼                                │
        │     ┌─────────────────────────────────────────────────┐   │
        │     │ C. RenderSystem::Update()                       │   │
        │     │    - SKIPPED in OnFixedUpdate                   │   │
        │     │    - Only updated in OnInterpolateAndRender     │   │
        │     └─────────────────────────────────────────────────┘   │
        │                          │                                │
        │                          ▼                                │
        │     ┌─────────────────────────────────────────────────┐   │
        │     │ D. DebugRenderSystem::Update()                  │   │
        │     │    - Renders debug overlays                     │   │
        │     └─────────────────────────────────────────────────┘   │
        │                                                           │
        │  2. OnECSFixedUpdate() ← GAME HOOK                        │
        │     └─ PhysicsDemoGame::OnECSFixedUpdate() (line 33-45)   │
        │        ├─ HandleGlobalInput()                             │
        │        │  ├─ P key: Pause/Resume                         │
        │        │  ├─ 1/2/3 keys: Time scale (0.25x/1x/2x)        │
        │        │  ├─ Left click: Spawn box                       │
        │        │  └─ Right click: Spawn ball                     │
        │        └─ (No additional game logic in demo)              │
        │                                                           │
        │  3. OnECSUpdate() ← GAME HOOK                             │
        │     └─ PhysicsDemoGame::OnECSUpdate() (line 47-50)        │
        │        └─ (Empty - could display HUD stats)               │
        └───────────────────────────────────────────────────────────┘
                        │
                        ▼
        ┌───────────────────────────────────────────────────────────┐
        │     accumulator -= 1/60s                                  │
        │     (Consume fixed timestep)                              │
        └───────────────────────────────────────────────────────────┘
                        │
                        ▼
        ┌───────────────────────────────────────────────────────────┐
        │  Calculate interpolation alpha = accumulator / (1/60s)    │
        │  (How far between physics frames we are)                  │
        └───────────────────────────────────────────────────────────┘
                        │
                        ▼
        ┌───────────────────────────────────────────────────────────┐
        │      ECSApplication::OnInterpolateAndRender(alpha)        │
        │                   (line 78-101)                           │
        │                                                           │
        │  1. Get RenderSystem                                      │
        │  2. SetInterpolationAlpha(alpha)                          │
        │  3. RenderSystem::Update()                                │
        │     ├─ Clear screen (dark blue)                           │
        │     ├─ Begin Renderer2D scene                             │
        │     ├─ For each RenderComponent:                          │
        │     │  ├─ Get interpolated position:                      │
        │     │  │  lerp(prevPos, currPos, alpha)                   │
        │     │  ├─ Draw based on shape type:                       │
        │     │  │  ├─ Circle → DrawSolidCircle()                   │
        │     │  │  └─ Rectangle → DrawQuad()                       │
        │     │  └─ Apply color, rotation, scale                    │
        │     └─ End scene                                          │
        │                                                           │
        │  4. DebugRenderSystem::RenderDebugInfo()                  │
        │     ├─ Collision contacts (red/green markers)             │
        │     ├─ AABBs (yellow boxes)                               │
        │     ├─ Island visualization                               │
        │     └─ Performance statistics (FPS, physics time)         │
        │                                                           │
        │  5. Swap buffers (glfwSwapBuffers)                        │
        │  6. Poll events (glfwPollEvents)                          │
        └───────────────────────────────────────────────────────────┘
                        │
                        └──────────────┐
                                       │
                        ┌──────────────▼──────────────┐
                        │  glfwWindowShouldClose?     │
                        │  If NO: loop back to start  │
                        │  If YES: exit loop          │
                        └──────────────┬──────────────┘
                                       │
                                       ▼
                    ┌──────────────────────────────────┐
                    │    Application::~Application()   │
                    │     (Destructor cleanup)         │
                    │                                  │
                    │  1. Renderer2D::Shutdown()       │
                    │  2. glfwDestroyWindow()          │
                    │  3. glfwTerminate()              │
                    └──────────────────────────────────┘
```

---

## PhysicsPipelineSystem::Update() Deep Dive

```
┌─────────────────────────────────────────────────────────────────┐
│          PhysicsPipelineSystem::Update(deltaTime)               │
│                   (engine/src line 46-127)                      │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ Lazy Init: Find PhysicsWorldComponent │
        │  └─ Search all entities for world     │
        └───────────────────────────────────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ Check validity:                       │
        │  if (!m_PhysicsWorld || !m_ComponentStore) return; │
        └───────────────────────────────────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ Accumulate time:                      │
        │  m_Accumulator += deltaTime           │
        │  m_Accumulator = min(m_Accumulator, 0.25s) │
        └───────────────────────────────────────┘
                                │
                                ▼
        ┌═══════════════════════════════════════┐
        ║  WHILE accumulator >= 16.67ms (60Hz)  ║
        ║         (Sub-stepping loop)           ║
        └═══════════════════════════════════════┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ 1. Collect Active Entities            │
        │    ForEachComponent<PhysicsBodyComponent>: │
        │     └─ if (!isStatic) add to list     │
        └───────────────────────────────────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ 2. PrepareBodiesForUpdate()           │
        │    (line 129-182)                     │
        │                                       │
        │    For each active entity:            │
        │     ├─ Create SolverBody struct       │
        │     ├─ Copy from PhysicsBodyComponent:│
        │     │  ├─ velocity, angularVelocity   │
        │     │  ├─ inverseMass, inverseInertia │
        │     │  └─ centerOfMass                │
        │     ├─ Copy from TransformComponent:  │
        │     │  ├─ position, rotation          │
        │     │  └─ previousPosition, prevRotation│
        │     └─ Initialize force/torque to 0   │
        │                                       │
        │    Result: m_SolverBodies array       │
        └───────────────────────────────────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ 3. BroadPhaseDetection()              │
        │    (line 184-266)                     │
        │                                       │
        │    For each ColliderComponent:        │
        │     ├─ Calculate AABB (min/max)       │
        │     ├─ Update DynamicTree proxy       │
        │     │  ├─ CreateProxy() if new        │
        │     │  └─ MoveProxy() if existing     │
        │     └─ Store in m_ShapeProxyMap       │
        │                                       │
        │    Brute-force overlap test:          │
        │     For all proxy pairs:              │
        │      └─ if AABBs overlap:             │
        │         └─ Add to m_BroadPhasePairs   │
        │                                       │
        │    Result: Potential collision pairs  │
        └───────────────────────────────────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ 4. NarrowPhaseDetection()             │
        │    (line 268-303)                     │
        │                                       │
        │    For each pair in m_BroadPhasePairs:│
        │     ├─ TestCollision():               │
        │     │  └─ AABB overlap check          │
        │     │                                 │
        │     └─ GenerateManifold():            │
        │        ├─ Call ManifoldGenerator      │
        │        │  └─ SAT or circle-circle     │
        │        ├─ Create ContactManifold       │
        │        │  ├─ Contact points           │
        │        │  ├─ Contact normal           │
        │        │  └─ Separation distance      │
        │        └─ Store in m_ContactManifolds │
        │                                       │
        │    Result: Contact manifolds with     │
        │           penetration data            │
        └───────────────────────────────────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ 5. IslandDetection()                  │
        │    (line 305-312)                     │
        │                                       │
        │    IslandManager::UpdateIslands():    │
        │     ├─ Build connectivity graph       │
        │     │  (bodies connected by contacts) │
        │     ├─ Identify independent islands   │
        │     └─ Mark sleeping candidates       │
        │                                       │
        │    Result: Grouped bodies for         │
        │           sleeping optimization       │
        └───────────────────────────────────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ 6. ConstraintInitialization()         │
        │    (line 314-352)                     │
        │                                       │
        │    For each ContactManifold:          │
        │     ├─ Create VelocityConstraint      │
        │     ├─ Copy contact points            │
        │     ├─ Calculate constraint mass      │
        │     │  (based on body masses)         │
        │     └─ Store friction/restitution     │
        │                                       │
        │    Result: m_VelocityConstraints      │
        └───────────────────────────────────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ 7. VelocitySolving()                  │
        │    (line 354-379)                     │
        │                                       │
        │    A. WarmStartConstraints()          │
        │       └─ Apply last frame's impulses  │
        │                                       │
        │    B. Apply forces:                   │
        │       For each solver body:           │
        │        └─ Apply gravity: F = m * g    │
        │                                       │
        │    C. IntegrateVelocities(dt)         │
        │       └─ v += (F * invMass) * dt      │
        │                                       │
        │    D. SolveVelocityConstraints()      │
        │       For velocityIterations (8x):    │
        │        For each constraint:           │
        │         ├─ Calculate relative vel     │
        │         ├─ Calculate impulse          │
        │         ├─ Clamp impulse (≥0)         │
        │         └─ Apply to both bodies       │
        │                                       │
        │    Result: Updated velocities with    │
        │           collision response          │
        └───────────────────────────────────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ 8. PositionSolving()                  │
        │    (line 381-402)                     │
        │                                       │
        │    A. IntegratePositions(dt)          │
        │       └─ pos += vel * dt              │
        │                                       │
        │    B. SolvePositionConstraints()      │
        │       For positionIterations (3x):    │
        │        For each constraint:           │
        │         ├─ if penetration > slop:     │
        │         │  ├─ Calculate correction    │
        │         │  │  C = baumgarte * penetration │
        │         │  ├─ Apply to positions      │
        │         │  │  (based on inverse mass) │
        │         │  └─ NEVER move static body  │
        │         └─ Log corrections             │
        │                                       │
        │    Result: Corrected positions to     │
        │           resolve penetrations        │
        └───────────────────────────────────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ 9. Integration()                      │
        │    (line 404-417)                     │
        │                                       │
        │    For each solver body:              │
        │     └─ Copy velocity back to          │
        │        PhysicsBodyComponent           │
        └───────────────────────────────────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ 10. StoreImpulses()                   │
        │     (line 419-430)                    │
        │                                       │
        │     Preserve accumulated impulses     │
        │     for warm starting next frame      │
        └───────────────────────────────────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ 11. UpdateSleeping()                  │
        │     (line 432-447)                    │
        │                                       │
        │     Update PhysicsBodyComponent.isAwake │
        │     based on IslandManager state      │
        └───────────────────────────────────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ 12. UpdateTransformsFromSolver()      │
        │     (line 449-476)                    │
        │                                       │
        │     For each solver body:             │
        │      ├─ Skip if isStatic              │
        │      └─ Update TransformComponent:    │
        │         ├─ position = solver.position │
        │         └─ rotation = solver.angle    │
        │                                       │
        │     ⚠️ BUG RISK: Static bodies skipped │
        └───────────────────────────────────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ accumulator -= 16.67ms                │
        │ (Consume fixed timestep)              │
        └───────────────────────────────────────┘
                                │
                                ▼
                    ┌───────────────────────┐
                    │ accumulator < 16.67ms?│
                    │  NO: Loop again       │
                    │  YES: Exit loop       │
                    └───────────┬───────────┘
                                │
                                ▼
        ┌───────────────────────────────────────┐
        │ Update statistics:                    │
        │  m_Stats.updateTime = elapsed ms      │
        └───────────────────────────────────────┘
```

---

## Component Data Flow

```
┌──────────────────────────────────────────────────────────────────┐
│                         ENTITY STRUCTURE                         │
│  (Example: A falling box in PhysicsDemoGame)                     │
└──────────────────────────────────────────────────────────────────┘

Entity ID: 42
│
├─► TransformComponent
│   ├─ position: Vector2(500, 800)
│   ├─ rotation: 0.0f
│   ├─ previousPosition: Vector2(500, 805)
│   └─ previousRotation: 0.0f
│
├─► PhysicsBodyComponent
│   ├─ isStatic: false
│   ├─ isBullet: false
│   ├─ isAwake: true
│   ├─ mass: 1.0f
│   ├─ inverseMass: 1.0f
│   ├─ inertia: 0.0833f
│   ├─ inverseInertia: 12.0f
│   ├─ velocity: Vector2(0, -50)  // Falling up (Y+ is down)
│   ├─ angularVelocity: 0.0f
│   ├─ centerOfMass: Vector2(0, 0)
│   └─ material: {friction: 0.4f, restitution: 0.0f}
│
├─► ColliderComponent
│   ├─ type: Polygon
│   ├─ polygon: {vertices: [(-25,-25), (25,-25), (25,25), (-25,25)]}
│   ├─ material: {friction: 0.4f, restitution: 0.0f}
│   └─ filter: {categoryBits: 0x0001, maskBits: 0xFFFF}
│
└─► RenderComponent
    ├─ size: Vector2(50, 50)
    ├─ origin: Vector2(0, 0)
    ├─ color: Vector3(0.2, 0.4, 1.0)  // Blue
    ├─ shapeType: Rectangle
    └─ visible: true


┌──────────────────────────────────────────────────────────────────┐
│                    DATA FLOW DURING PHYSICS UPDATE               │
└──────────────────────────────────────────────────────────────────┘

Frame N:
│
├─► READ PHASE (PhysicsPipelineSystem::PrepareBodiesForUpdate)
│   │
│   ├─ Read TransformComponent.position → SolverBody.position
│   ├─ Read PhysicsBodyComponent.velocity → SolverBody.velocity
│   └─ Read PhysicsBodyComponent.inverseMass → SolverBody.invMass
│
├─► SOLVE PHASE (PhysicsPipelineSystem::Update)
│   │
│   ├─ Detect collisions → ContactManifolds
│   ├─ Build constraints → VelocityConstraints
│   ├─ Solve velocities → Updated SolverBody.velocity
│   └─ Solve positions → Updated SolverBody.position
│
└─► WRITE PHASE (Integration + UpdateTransformsFromSolver)
    │
    ├─ Write SolverBody.velocity → PhysicsBodyComponent.velocity
    └─ Write SolverBody.position → TransformComponent.position


┌──────────────────────────────────────────────────────────────────┐
│                    RENDERING DATA FLOW                           │
└──────────────────────────────────────────────────────────────────┘

Frame N (rendering with interpolation alpha=0.5):
│
├─► Read TransformComponent:
│   ├─ current position: (500, 800)
│   └─ previous position: (500, 805)
│
├─► Calculate interpolated position:
│   └─ lerp((500, 805), (500, 800), 0.5) = (500, 802.5)
│
└─► Render at (500, 802.5) using RenderComponent properties
     ├─ Draw quad with size (50, 50)
     ├─ Color (0.2, 0.4, 1.0) - blue
     └─ Rotation from interpolation
```

---

## Key Architecture Patterns

### 1. **Layered Architecture**
```
┌─────────────────────────────────────┐
│  Game Layer (PhysicsDemoGame)       │  ← User code
├─────────────────────────────────────┤
│  ECS Framework (ECSApplication)     │  ← Engine core
├─────────────────────────────────────┤
│  Systems (Physics, Render, Input)   │  ← Engine modules
├─────────────────────────────────────┤
│  Components (Data-only structs)     │  ← Data storage
├─────────────────────────────────────┤
│  Low-level (OpenGL, GLFW, Math)     │  ← Platform layer
└─────────────────────────────────────┘
```

### 2. **Data-Oriented Design**
```
Components are plain data (no logic):
  ✓ TransformComponent - just position/rotation
  ✓ PhysicsBodyComponent - just physics properties
  ✓ ColliderComponent - just shape data

Systems contain all logic:
  ✓ PhysicsPipelineSystem - all physics calculations
  ✓ RenderSystem - all rendering logic
  ✓ InputSystem - all input processing
```

### 3. **Fixed Timestep with Interpolation**
```
Physics: Fixed 60 Hz (16.67ms per step)
Rendering: Variable (as fast as possible)
Interpolation: Smooths between physics frames

Timeline:
  Physics: |----16.67ms----|----16.67ms----|----16.67ms----|
  Render:  |--10ms--|--12ms--|--8ms--|--15ms--|--5ms--|--20ms--|
  
  Alpha interpolates between physics states for smooth visuals
```

### 4. **Unified Physics Pipeline**
```
PhysicsPipelineSystem handles EVERYTHING:
  ✓ Broad phase (DynamicTree)
  ✓ Narrow phase (SAT/circle collision)
  ✓ Constraint solving (iterative impulse method)
  ✓ Sleeping (island-based optimization)
  ✓ Integration (velocity/position update)

NO external dependencies on other physics systems
```

---

## Single Points of Failure

If physics isn't working, check these in order:

1. **PhysicsWorldComponent existence** (line 52-81 in PhysicsDemoGame)
2. **Broad phase tree updates** (DynamicTree::CreateProxy/MoveProxy)
3. **Collision pair generation** (m_BroadPhasePairs not empty?)
4. **Contact manifold generation** (ManifoldGenerator returning contacts?)
5. **Position correction** (SolvePositionConstraints actually moving bodies?)
6. **Transform updates** (UpdateTransformsFromSolver writing to components?)
7. **Render component sync** (Are render positions matching physics?)
