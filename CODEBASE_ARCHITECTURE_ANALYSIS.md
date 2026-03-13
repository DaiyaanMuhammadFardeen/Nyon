# Nyon Engine Codebase Architecture Analysis

## Executive Summary

The codebase is **NOT spaghetti**, but it does have **legacy/unused components** that should be removed. The architecture follows a clear **ECS (Entity-Component-System) pattern** with proper separation of concerns. However, there's significant **redundancy and dead code**.

## Key Findings

### ✅ What's Actually Being Used (Active Code Path)

```
PhysicsDemoGame (main executable)
└── ECSApplication (core application framework)
    ├── Application (base class - window management, game loop)
    ├── EntityManager (entity ID management)
    ├── ComponentStore (component data storage)
    └── SystemManager (system orchestration)
        ├── InputSystem (processes input)
        ├── PhysicsPipelineSystem ⭐ (MAIN PHYSICS SYSTEM - active)
        ├── RenderSystem (renders entities)
        └── DebugRenderSystem (debug visualization)
```

### ❌ What's DEAD/UNUSED Code

1. **CollisionPipelineSystem** - Replaced by PhysicsPipelineSystem
2. **PhysicsIntegrationSystem** - Legacy system (mentioned in comments as obsolete)
3. **StabilizationSystem.cpp** - Excluded from build (compilation issues)
4. **PhysicsPipeline.cpp** - Excluded from build (compilation issues)
5. **BoundarySystem** - Not used in current demo
6. **ContinuousCollisionDetection** - Not integrated into active pipeline
7. **ConstraintSolver (physics folder)** - Replaced by integrated solver in PhysicsPipelineSystem
8. **ManifoldGenerator (physics folder)** - Used but creates confusion (duplicate logic exists)
9. **SATCollisionDetector** - Standalone collision detector not used in main pipeline
10. **DynamicTree** - Used internally by PhysicsPipelineSystem but could be simplified
11. **Island** - Used by PhysicsPipelineSystem for sleeping optimization

---

## Complete Dependency Tree

### Level 0: Entry Point
```
game/physics-demo/src/PhysicsDemoGame.cpp
└── main() → PhysicsDemoGame::Run()
```

### Level 1: Application Layer
```
PhysicsDemoGame : ECSApplication : Application
│
├── Manages GLFW window, OpenGL context
├── Implements game loop with fixed timestep
├── Handles input polling
└── Calls virtual methods:
    ├── OnStart() → line 32
    ├── OnFixedUpdate() → line 33 (physics tick)
    └── OnInterpolateAndRender() → line 78 (rendering)
```

### Level 2: ECS Framework
```
ECSApplication (line 18-108)
│
├── Constructor (line 18-25)
│   └── Initializes:
│       ├── EntityManager
│       ├── ComponentStore
│       └── SystemManager
│
├── OnStart() (line 32-56)
│   ├── Calls InputManager::Init()
│   ├── Calls OnECSStart() (game-specific init)
│   │   └── PhysicsDemoGame::OnECSStart() → line 24
│   │       ├── CreatePhysicsWorld() → line 52
│   │       ├── CreateBounds() → line 86
│   │       ├── CreateStackTest() → line 151
│   │       ├── CreateRampFrictionTest() → line 206
│   │       └── CreateBounceTest() → line 296
│   └── Adds ECS Systems:
│       ├── InputSystem
│       ├── PhysicsPipelineSystem ⭐
│       ├── RenderSystem
│       └── DebugRenderSystem
│
├── OnFixedUpdate() (line 58-76)
│   ├── Calls SystemManager.Update()
│   │   └── Updates all systems in order
│   ├── Calls OnECSFixedUpdate() (game logic)
│   └── Calls OnECSUpdate() (post-physics logic)
│
└── OnInterpolateAndRender() (line 78-101)
    ├── Sets interpolation alpha on RenderSystem
    ├── Calls RenderSystem.Update()
    └── Calls DebugRenderSystem.RenderDebugInfo()
```

### Level 3: Component Managers
```
EntityManager (line 1-61)
├── CreateEntity() → generates unique IDs
├── DestroyEntity() → recycles IDs
└── Tracks active entities

ComponentStore (not shown in detail)
├── Stores components by type
├── Provides component queries
└── ForEachComponent<> iteration

SystemManager (line 1-67)
├── AddSystem<> → registers systems
├── Update() → calls all systems
└── GetSystem<> → retrieves specific systems
```

### Level 4: Active Systems (The Core)

#### PhysicsPipelineSystem ⭐ (line 1-209, implementation 1-831)
```
PhysicsPipelineSystem : System
│
├── Initialize() (line 24-44)
│   └── Finds PhysicsWorldComponent
│
├── Update() (line 46-127) - MAIN LOOP
│   ├── Lazy initialization
│   ├── Fixed timestep accumulation
│   └── Pipeline phases:
│       ├── PrepareBodiesForUpdate() → line 129
│       │   └── Builds solver body array from components
│       │
│       ├── BroadPhaseDetection() → line 184
│       │   └── Uses DynamicTree to find overlapping pairs
│       │
│       ├── NarrowPhaseDetection() → line 268
│       │   └── Generates contact manifolds
│       │
│       ├── IslandDetection() → line 305
│       │   └── Groups connected bodies for sleeping
│       │
│       ├── ConstraintInitialization() → line 314
│       │   └── Creates velocity constraints
│       │
│       ├── VelocitySolving() → line 354
│       │   ├── WarmStartConstraints()
│       │   ├── Apply gravity
│       │   ├── IntegrateVelocities()
│       │   └── SolveVelocityConstraints() (iterative)
│       │
│       ├── PositionSolving() → line 381
│       │   ├── IntegratePositions()
│       │   └── SolvePositionConstraints() (iterative)
│       │
│       ├── Integration() → line 404
│       │   └── Stores velocities back to components
│       │
│       ├── StoreImpulses() → line 419
│       │   └── Preserves impulses for warm start
│       │
│       ├── UpdateSleeping() → line 432
│       │   └── Updates body awake states
│       │
│       └── UpdateTransformsFromSolver() → line 449
│           └── Writes positions to TransformComponent
│
└── Helper classes:
    ├── ContactPoint, ContactManifold
    ├── VelocityConstraint, PositionConstraint
    ├── SolverBody
    └── BroadPhaseCallback
```

#### RenderSystem (line 1-116)
```
RenderSystem : System
│
├── Initialize() (line 19-25)
│   └── Sets interpolation alpha
│
├── Update() (line 27-103)
│   ├── Clears screen
│   ├── Begins Renderer2D scene
│   ├── Iterates RenderComponent entities
│   │   └── Draws:
│   │       ├── Circles (DrawSolidCircle)
│   │       └── Rectangles (DrawQuad)
│   └── Ends scene
│
└── Shutdown() (line 108-111)
    └── Calls Renderer2D::Shutdown()
```

#### InputSystem (not shown in detail)
```
InputSystem : System
└── Processes keyboard/mouse input
    └── Updates PhysicsBodyComponent forces/velocities
```

#### DebugRenderSystem (not shown in detail)
```
DebugRenderSystem : System
└── Renders debug overlays:
    ├── Collision contacts
    ├── AABBs
    ├── Island visualization
    └── Performance statistics
```

### Level 5: Supporting Modules

#### Graphics Layer
```
Renderer2D (line 1-225)
├── Static singleton pattern
├── OpenGL-based 2D rendering
├── Shape drawing:
│   ├── DrawQuad(), DrawCircle(), DrawPolygon()
│   ├── DrawSolidCircle(), DrawSolidPolygon()
│   └── Advanced shapes (capsule, ellipse, arc, etc.)
├── Camera2D support
└── Batch rendering with vertex buffers
```

#### Physics Components (Data Only)
```
PhysicsWorldComponent
└── Global physics settings:
    ├── gravity, timeStep
    ├── velocityIterations, positionIterations
    ├── subStepCount
    └── enableSleep, enableWarmStarting, enableContinuous

PhysicsBodyComponent
└── Body properties:
    ├── isStatic, isBullet, isAwake
    ├── mass, inverseMass
    ├── inertia, inverseInertia
    ├── velocity, angularVelocity
    ├── centerOfMass
    └── material (friction, restitution)

ColliderComponent
└── Collision shapes:
    ├── PolygonShape, CircleShape
    ├── AABB calculation
    └── Material properties

TransformComponent
└── Spatial data:
    ├── position, rotation
    ├── previousPosition, previousRotation (for interpolation)
    └── GetInterpolatedPosition/Rotation()
```

---

## Redundant/Dead Code Analysis

### 1. ❌ CollisionPipelineSystem
**Location:** `engine/src/ecs/systems/CollisionPipelineSystem.cpp`
**Status:** DEAD CODE
**Why:** Completely replaced by PhysicsPipelineSystem
**Evidence:** 
- Not added in ECSApplication::OnStart()
- PhysicsPipelineSystem handles entire collision detection pipeline
- CMakeLists.txt doesn't exclude it, but it's never instantiated

### 2. ❌ PhysicsIntegrationSystem
**Location:** Referenced in comments only
**Status:** LEGACY
**Why:** Functionality absorbed into PhysicsPipelineSystem
**Evidence:** PhysicsDemoGame.cpp line 83: "No need to add PhysicsIntegrationSystem - that's a legacy system"

### 3. ❌ StabilizationSystem.cpp
**Location:** `engine/src/physics/StabilizationSystem.cpp`
**Status:** EXCLUDED FROM BUILD
**Why:** Compilation issues (CMakeLists.txt line 26)
**Evidence:** Explicitly excluded in CMakeLists.txt

### 4. ❌ PhysicsPipeline.cpp (physics folder)
**Location:** `engine/src/physics/PhysicsPipeline.cpp`
**Status:** EXCLUDED FROM BUILD
**Why:** Compilation issues (CMakeLists.txt line 23)
**Evidence:** Explicitly excluded in CMakeLists.txt
**Note:** This is DIFFERENT from PhysicsPipelineSystem.cpp which IS active

### 5. ❌ BoundarySystem
**Location:** `engine/include/nyon/physics/BoundarySystem.h` + `.cpp`
**Status:** UNUSED
**Why:** PhysicsDemoGame creates boundaries manually using entities
**Evidence:** PhysicsDemoGame.cpp line 86-149 shows CreateBounds() is commented out entirely

### 6. ❌ ContinuousCollisionDetection
**Location:** `engine/include/nyon/physics/ContinuousCollisionDetection.h` + `.cpp`
**Status:** NOT INTEGRATED
**Why:** PhysicsBodyComponent has `isBullet` flag but CCD is not called anywhere
**Evidence:** No references to ContinuousCollisionDetection in active code paths

### 7. ⚠️ Duplicate Physics Logic

**Problem:** Two separate constraint solvers exist:
- `PhysicsPipelineSystem` (active, integrated solver)
- `physics/ConstraintSolver.h` (standalone, unused)

**Problem:** Two manifold generators:
- Internal ContactManifold in PhysicsPipelineSystem
- `physics/ManifoldGenerator` (used but creates confusion)

---

## Why Physics Might Not Be Working

Based on the code analysis, here are potential issues:

### Issue 1: Missing Boundaries
**Location:** PhysicsDemoGame.cpp line 86-149
**Problem:** CreateBounds() is completely commented out
**Impact:** Objects fall forever without floor/walls
**Fix:** Uncomment boundary creation

### Issue 2: Gravity Direction
**Location:** PhysicsDemoGame.cpp line 62
```cpp
world.gravity = { 0.0f, 980.0f };  // y-positive DOWN
```
**Problem:** Gravity points in +Y direction (screen coordinates)
**Impact:** Objects fall DOWN (correct for screen coords), but might conflict with expectations
**Check:** Ensure coordinate system matches expectations

### Issue 3: Static Body Transform Updates
**Location:** PhysicsPipelineSystem.cpp line 449-476
**Code:** Skips static bodies in UpdateTransformsFromSolver()
**Potential Bug:** If static bodies are moved after creation, their transforms won't update

### Issue 4: Contact Generation Complexity
**Location:** PhysicsPipelineSystem.cpp line 600-647
**Problem:** GenerateManifold() converts between two manifold types
**Risk:** Potential data loss or incorrect conversion

### Issue 5: Position Correction Logic
**Location:** PhysicsPipelineSystem.cpp line 726-783
**Code:** SolvePositionConstraints()
**Potential Issue:** Complex branching logic for static vs dynamic bodies
**Risk:** May not correctly resolve penetrations in all cases

---

## Recommended Cleanup Actions

### Priority 1: Remove Dead Files
Delete these files entirely:
1. `engine/src/ecs/systems/CollisionPipelineSystem.cpp`
2. `engine/src/ecs/systems/CollisionPipelineSystem.h`
3. `engine/include/nyon/physics/BoundarySystem.h`
4. `engine/src/physics/BoundarySystem.cpp`
5. `engine/include/nyon/physics/ContinuousCollisionDetection.h`
6. `engine/src/physics/ContinuousCollisionDetection.cpp`
7. `engine/include/nyon/physics/ConstraintSolver.h`
8. `engine/src/physics/ConstraintSolver.cpp`
9. `engine/include/nyon/physics/SATCollisionDetector.h`
10. `engine/src/physics/SATCollisionDetector.cpp`
11. `engine/include/nyon/physics/StabilizationSystem.h`
12. `engine/src/physics/StabilizationSystem.cpp`
13. `engine/include/nyon/physics/PhysicsPipeline.h` (the old one, not PhysicsPipelineSystem)
14. `engine/src/physics/PhysicsPipeline.cpp`

### Priority 2: Simplify Architecture
1. **Keep ECSApplication** - It's the core framework
2. **Keep PhysicsPipelineSystem** - Main physics system
3. **Merge ManifoldGenerator** - Move logic directly into PhysicsPipelineSystem
4. **Simplify DynamicTree** - Keep but document better

### Priority 3: Fix Physics Issues
1. **Uncomment boundaries** in PhysicsDemoGame
2. **Add debug logging** for collision pairs
3. **Verify contact generation** with simple test cases
4. **Test position correction** with stacking scenarios

---

## Minimal Viable Product (MVP) Architecture

If you want the SIMPLEST working version, keep ONLY:

```
Core:
├── Application.h/cpp (window, game loop)
├── ECSApplication.h/cpp (ECS framework)
├── EntityManager.h/cpp
├── ComponentStore.h/cpp
└── SystemManager.h/cpp

Components:
├── TransformComponent.h
├── PhysicsBodyComponent.h
├── ColliderComponent.h
├── PhysicsWorldComponent.h
└── RenderComponent.h

Systems:
├── PhysicsPipelineSystem.h/cpp (UNIFIED physics)
├── RenderSystem.h/cpp
├── InputSystem.h/cpp
└── DebugRenderSystem.h/cpp

Graphics:
└── Renderer2D.h/cpp

Physics Internals:
├── DynamicTree.h/cpp (broad phase)
├── Island.h/cpp (sleeping)
└── ManifoldGenerator.h/cpp (contact generation)

Utils:
└── InputManager.h/cpp

Math:
├── Vector2.h
└── Vector3.h
```

**Total:** ~15-20 files instead of 40+

---

## Conclusion

The codebase is **over-engineered with legacy code**, but the architecture itself is sound. The main issue is having **multiple implementations of the same functionality** (old systems, duplicate solvers). 

**Action Plan:**
1. Delete all dead files listed above
2. Enable boundaries in the demo
3. Add comprehensive debug logging
4. Test with simple scenarios first (single box falling)
5. Gradually increase complexity

The PhysicsPipelineSystem is well-designed and follows modern physics engine patterns. Once cleaned up, it should work correctly.
