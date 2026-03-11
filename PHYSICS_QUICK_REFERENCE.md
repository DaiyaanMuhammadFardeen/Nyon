# Physics System Quick Reference Guide

## 🚀 Getting Started

```cpp
#include <nyon/physics/PhysicsPipeline.h>

// 1. Create and initialize
Nyon::PhysicsPipeline pipeline;
pipeline.Initialize(1000, 5000); // maxEntities, maxContacts

// 2. Define world
Nyon::Boundary bounds{{0.0f, 0.0f}, {800.0f, 600.0f}};
Math::Vector2 gravity{0.0f, -9.81f};

// 3. Update each frame
float dt = 1.0f / 60.0f;
pipeline.Update(componentStore, dt, gravity, bounds);
```

---

## 📦 Core Components

| Component | Purpose | Key Features |
|-----------|---------|--------------|
| **DynamicTree** | Broad-phase | AABB hierarchy, O(log n) queries |
| **SATCollisionDetector** | Narrow-phase | SAT for all shapes, manifold generation |
| **ContinuousCollisionDetection** | CCD | TOI computation, prevents tunneling |
| **BoundarySystem** | World limits | 4 planes, solid boundaries |
| **ConstraintSolver** | Resolution | Sequential impulses, warm starting |
| **StabilizationSystem** | Anti-flicker | Contact persistence, sleep management |
| **PhysicsPipeline** | Orchestrator | Complete 6-step pipeline |

---

## 🔧 Configuration

### Boundary System
```cpp
Nyon::Boundary bounds{
    .minBounds = {0.0f, 0.0f},   // Bottom-left
    .maxBounds = {800.0f, 600.0f}, // Top-right
    .thickness = 1.0f,
    .enabled = true
};
```

### Constraint Solver
```cpp
m_ConstraintSolver.SetBaumgarteParameters(0.2f, 0.005f);
m_ConstraintSolver.SetWarmStarting(true);
m_ConstraintSolver.SetSplitImpulses(true);
```

### Sleep Management
```cpp
auto& sleepManager = m_StabilizationSystem.GetSleepManager();
sleepManager.sleepThreshold = 0.01f;
sleepManager.wakeThreshold = 0.1f;
sleepManager.timeToSleep = 0.5f;
```

---

## 🎯 Key Algorithms

### SAT (Separating Axis Theorem)
- Test all edge normals from both polygons
- Find separating axis → no collision
- No separating axis → collision detected
- Generate contact manifold via clipping

### CCD (Continuous Collision Detection)
- Circle-Circle: Quadratic solving `a*t² + b*t + c = 0`
- Polygons: Conservative advancement
- Activate when `velocity > size / dt * 0.5`

### Constraint Solving
1. Warm start (use cached impulses)
2. Solve velocity constraints (sequential impulses)
3. Integrate velocities
4. Solve position constraints (Baumgarte)
5. Apply split impulses

### Stabilization
1. Match contacts using feature IDs
2. Cache impulses for warm starting
3. Update sleep timers
4. Put bodies to sleep if below threshold

---

## 📊 Performance Tips

| Scenario | Recommendation |
|----------|----------------|
| **Many fast objects** | Enable CCD selectively |
| **Stacked boxes** | Increase position iterations (5-10) |
| **Flickering** | Check sleep thresholds, enable warm starting |
| **Tunneling** | Reduce time step or increase CCD usage |
| **Performance** | Use fewer velocity iterations (4-6) |

**Default Iterations**:
- Velocity: 8
- Position: 3
- Good balance of stability and performance

---

## 🐛 Common Issues & Solutions

### Issue: Box flickers on ground
**Solution**: ✅ Already fixed with stabilization system
- Contact persistence enabled by default
- Warm starting active
- Split impulses applied

### Issue: Fast object phases through wall
**Solution**: ✅ Already fixed with CCD
- Automatic CCD activation for fast objects
- Conservative advancement prevents tunneling

### Issue: Objects don't settle
**Solution**: ✅ Already fixed with sleep management
- Bodies sleep after 0.5s below velocity threshold
- Wake on contact automatically

---

## 📈 File Locations

```
engine/include/nyon/physics/
├── PhysicsPipeline.h          ← Main entry point
├── BoundarySystem.h           ← World boundaries
├── SATCollisionDetector.h     ← Collision detection
├── ContinuousCollisionDetection.h ← CCD
├── ConstraintSolver.h         ← Resolution
└── StabilizationSystem.h      ← Anti-flickering

engine/src/physics/
├── PhysicsPipeline.cpp        ← Implementation
├── BoundarySystem.cpp
├── SATCollisionDetector.cpp
├── ContinuousCollisionDetection.cpp
├── ConstraintSolver.cpp
└── StabilizationSystem.cpp
```

---

## 🎮 Example: Creating Physics Entities

```cpp
// Box
auto boxEntity = componentStore.CreateEntity();
auto* transform = &componentStore.AddComponent<TransformComponent>();
transform->position = {400.0f, 300.0f};
transform->scale = {50.0f, 50.0f};

auto* body = &componentStore.AddComponent<PhysicsBodyComponent>();
body->inverseMass = 1.0f; // Dynamic body
body->inverseInertia = 0.0833f; // For box

auto* collider = &componentStore.AddComponent<ColliderComponent>();
ColliderComponent::PolygonShape box;
box.vertices = {{-25,-25}, {25,-25}, {25,25}, {-25,25}};
collider->shape = box;

// Circle
auto ballEntity = componentStore.CreateEntity();
// ... similar setup with CircleShape
```

---

## 📋 Statistics & Debugging

```cpp
auto stats = pipeline.GetStatistics();
std::cout << "Broad-phase pairs: " << stats.broadPhasePairs << "\n";
std::cout << "Narrow-phase contacts: " << stats.narrowPhaseContacts << "\n";
std::cout << "CCD queries: " << stats.CCDQueries << "\n";
std::cout << "Sleeping bodies: " << stats.sleepingBodies << "\n";
```

---

## ✅ All Requirements Met

✅ Dynamic tree for broad-phase/narrow-phase decision  
✅ CCD for very fast objects  
✅ SAT-based collision detection  
✅ Screen boundaries (solid, no phasing)  
✅ Complete momentum conservation  
✅ Advanced resolution system  
✅ Zero tolerance for collisions unanswered  
✅ Anti-flickering stabilization  

---

**For detailed documentation**: See `PHYSICS_COMPLETE_SUMMARY.md`
