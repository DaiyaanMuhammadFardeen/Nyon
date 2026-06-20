# Nyon — 2D Game Engine

**Nyon** is a C++17 ECS-based 2D game engine with OpenGL 4.6 GPU-instanced rendering, a custom rigid-body physics pipeline, and a particle system. It ships with four playable demos.

---

## Build

### Prerequisites

| Dependency | Version | Notes |
|---|---|---|
| OpenGL | 4.6 Core profile | GPU driver |
| GLFW | 3.x | Windowing |
| GLM | Any recent | Math library |
| GLAD | Bundled (`engine/include/glad/glad.h`) | OpenGL loader |
| CMake | ≥ 3.10 | Build system |
| C++ compiler | C++17 capable | GCC, Clang, MSVC |

### Commands

```bash
# Configure
cmake -B build

# Build engine library + 4 demos
cmake --build build

# Run a demo
./build/game/simple-physics-demo/simple-physics-demo
./build/game/breakout-demo/breakout-demo
./build/game/flappy-demo/flappy-demo
./build/game/tower-stack-demo/tower-stack-demo

# Optional: enable tests
cmake -B build -DENABLE_TESTING=ON
cmake --build build
```

---

## Repository Structure

```
├── CMakeLists.txt               # Root — builds engine + 4 demos
├── engine/
│   ├── CMakeLists.txt           # Builds nyon_engine static library
│   ├── include/
│   │   ├── glad/glad.h          # OpenGL 4.6 Core profile loader
│   │   └── nyon/
│   │       ├── EngineConstants.h         # Fixed timestep, max frame time
│   │       ├── core/
│   │       │   ├── Application.h         # GLFW window + fixed-timestep loop
│   │       │   └── ECSApplication.h      # ECS orchestration layer
│   │       ├── ecs/
│   │       │   ├── EntityManager.h       # EntityID (uint32_t), free-list recycling
│   │       │   ├── ComponentStore.h      # Structure-of-Arrays storage, O(1) lookup
│   │       │   ├── System.h / SystemManager.h
│   │       │   ├── components/           # 10 component types
│   │       │   └── systems/              # 7 systems
│   │       ├── math/
│   │       │   ├── Vector2.h             # + Rotation2D (cos/sin)
│   │       │   └── Vector3.h
│   │       ├── physics/
│   │       │   ├── ContactTypes.h        # ContactPoint, ContactManifold
│   │       │   ├── ManifoldGenerator.h   # SAT narrow-phase
│   │       │   ├── Island.h              # BFS island building + sleep
│   │       │   └── DynamicTree.h         # AABB tree broad-phase
│   │       ├── graphics/
│   │       │   ├── Renderer2D.h          # GPU-instanced, triple-buffered
│   │       │   ├── PhysicsDebugRenderer.h
│   │       │   └── ParticleRenderer.h    # 4M max particles, instanced
│   │       └── utils/
│   │           ├── ThreadPool.h          # Work-stealing thread pool
│   │           └── InputManager.h        # GLFW keyboard/mouse
│   ├── src/                              # Implementations
│   └── assets/shaders/                   # 14 GLSL shader files
├── game/
│   ├── simple-physics-demo/
│   ├── breakout-demo/
│   ├── flappy-demo/
│   └── tower-stack-demo/
├── tutorial/                             # 6-step tutorial series
├── docs/                                 # Engineering reports, bug reports
└── test/                                 # Tests (disabled by default)
```

---

## Demo Controls

| Demo | Controls |
|---|---|
| **simple-physics-demo** | WASD move, W jump, Left Click spawn box, auto-spawns circles |
| **breakout-demo** | A/D or Left/Right move paddle, Space launch ball, R reset |
| **flappy-demo** | Space flap / start, R restart on game over |
| **tower-stack-demo** | Space drop block, R restart |

---

## Key Architecture

### Fixed Timestep Loop

The game loop in `Application::Run()` uses an accumulator pattern with a spiral-of-death cap:

```cpp
inline constexpr double FIXED_TIMESTEP  = 1.0 / 60.0;  // 60 Hz physics
inline constexpr double MAX_FRAME_TIME  = 0.25;          // safety cap
```

Each frame: accumulate elapsed time, consume fixed steps up to the cap, then render with interpolation alpha for visual smoothness.

### GPU-Instanced Rendering

- Uses `GL_ARB_buffer_storage` (GL 4.4+) for persistent-mapped buffer memory
- Triple-buffered instance VBOs with `GLsync` fence synchronization
- No `glBufferData` calls at runtime — writes go directly into mapped GPU pointers
- Separate pipelines for filled geometry, wireframe outlines, and particles

### Physics Pipeline

The custom 2D physics system executes in this order each fixed step:

1. **Prepare** — Update previous transforms for interpolation, reset forces
2. **Broad phase** — `DynamicTree` AABB tree (parallel via `ThreadPool`)
3. **Narrow phase** — SAT-based `ManifoldGenerator` (parallel)
4. **Island detection** — BFS over contact graph, sleep management
5. **Warm starting** — Apply cached impulses from previous frame
6. **Sequential impulse solver** — Velocity + position constraints (parallel sub-steps)
7. **Integration** — Update positions and velocities (parallel)
8. **Sleeping** — Bodies below thresholds fall asleep after 0.5s

### ECS Architecture

- **EntityID**: `uint32_t` with free-list recycling
- **ComponentStore**: Structure-of-Arrays (SoA) storage with dense component arrays and an `EntityID → index` map for O(1) lookup
- **SystemManager**: Ordered system execution; each system receives `(EntityManager&, ComponentStore&)` on its `Update(float dt)` call
- Particles are full ECS entities (Transform + PhysicsBody + Collider + ParticleComponent)

### Particle System

- 4M max particles via GPU-instanced rendering (32 bytes/instance)
- Full ECS integration — particles have physics bodies and colliders
- Spatial hash for particle-particle collision (optional)
- Emitter component supports rate-based and burst spawning with configurable emission shapes

### Multi-Threading

- `ThreadPool` with work-stealing queues
- Parallelized: broad-phase AABB queries, narrow-phase SAT detection, velocity/position solving, particle physics updates
- All parallel tasks are isolated — no locks needed during worker execution

---

## Component Quick Reference

| Component | Header | Purpose |
|---|---|---|
| `TransformComponent` | `components/TransformComponent.h` | Position, rotation, scale + interpolation |
| `RenderComponent` | `components/RenderComponent.h` | Shape (Rectangle/Circle/Polygon), size, color, layer |
| `PhysicsBodyComponent` | `components/PhysicsBodyComponent.h` | Velocity, force, mass, inertia, damping, sleep, motion locks |
| `ColliderComponent` | `components/ColliderComponent.h` | Circle/Polygon/Capsule/Segment/Chain/Composite, sensors, filters |
| `CameraComponent` | `components/CameraComponent.h` | Camera2D with follow-target, priority, viewport |
| `ParticleComponent` | `components/ParticleComponent.h` | Lifetime, age, alpha/color interpolation |
| `ParticleEmitterComponent` | `components/ParticleEmitterComponent.h` | Rate/burst spawning, emission shapes, spawn ranges |
| `PhysicsWorldComponent` | `components/PhysicsWorldComponent.h` | Gravity, solver config, callbacks, contact manifolds |
| `BehaviorComponent` | `components/BehaviorComponent.h` | `std::function` update/collision callbacks |
| `JointComponent` | `components/JointComponent.h` | 6 joint types defined — **solver NOT implemented** |

---

## System Quick Reference

| System | Header | Responsibility |
|---|---|---|
| `RenderSystem` | `systems/RenderSystem.h` | Renders entities via `Renderer2D` with interpolation |
| `CameraSystem` | `systems/CameraSystem.h` | Updates camera transforms, follow-target logic |
| `InputSystem` | `systems/InputSystem.h` | Polls `InputManager`, invokes `BehaviorComponent` callbacks |
| `PhysicsPipelineSystem` | `systems/PhysicsPipelineSystem.h` | Full physics pipeline (broad → narrow → solve → integrate → sleep) |
| `ParticlePipelineSystem` | `systems/ParticlePipelineSystem.h` | Emitter ticking, parallel physics, spatial hash collisions, lifecycle |
| `ParticleRenderSystem` | `systems/ParticleRenderSystem.h` | GPU-instanced particle rendering (circles/quads) |
| `DebugRenderSystem` | `systems/DebugRenderSystem.h` | Physics debug overlays (AABBs, contact points, collider shapes) |

---

## Writing a Demo

All demos extend `Nyon::ECSApplication` and override two hooks:

```cpp
#include <nyon/core/ECSApplication.h>

class MyDemo : public Nyon::ECSApplication {
    void OnECSStart() override {
        // Create entities, attach components
        auto entity = GetEntityManager().CreateEntity();
        GetComponentStore().AddComponent(entity, Nyon::ECS::TransformComponent{{400.0f, 300.0f}});
        GetComponentStore().AddComponent(entity, Nyon::ECS::RenderComponent{{64.0f, 64.0f}});
    }

    void OnECSFixedUpdate(float dt) override {
        // Game logic runs here at 60 Hz
    }
};

int main() {
    MyDemo demo("My Game", 800, 600);
    demo.Run();
    return 0;
}
```

See the `game/` directory for complete working examples.

---

## Physics World Contact Manifolds

After each fixed update, collision results are available on the `PhysicsWorldComponent` singleton:

```cpp
auto& world = componentStore.GetComponent<PhysicsWorldComponent>(worldEntity);
for (const auto& manifold : world.contactManifolds) {
    // manifold.entityIdA, manifold.entityIdB
    // manifold.points — vector of ContactPoint
    //   each point has: position, normal, separation, normalImpulse, tangentImpulse
    // manifold.friction, manifold.restitution
    // manifold.touching — whether shapes are in contact
}
```

Contact manifolds persist across frames with warm starting via feature IDs.

---

## Known Limitations

- **JointComponent** — All 6 joint types (Distance, Revolute, Prismatic, Weld, Wheel, Motor) are fully defined but the joint solver is **not implemented**. Joints will have no physical effect.
- **Legacy files excluded from build** — `PhysicsPipeline.cpp` and `StabilizationSystem.cpp` are filtered out by CMake due to pre-existing compilation issues; the active physics pipeline is `PhysicsPipelineSystem.cpp`.
- **Particle-body collisions** — Marked as "future implementation" in `ParticlePipelineSystem` (Phase 4). Particle-particle collisions are implemented.
- **Tests** — The `test/` directory exists but is disabled by default (`ENABLE_TESTING=OFF`).

---

## Documentation Index

| Document | Location |
|---|---|
| README (this file) | `README.md` |
| Architecture reference | `ARCHITECTURE.md` |
| Demo games technical reference | `DEMO_GAMES.md` |
| Tutorial series index | `tutorial/README.md` |
| Tutorial 1: Project setup | `tutorial/01-project-setup.md` |
| Tutorial 2: ECS basics | `tutorial/02-ecs-basics-and-entities.md` |
| Tutorial 3: Physics & collisions | `tutorial/03-physics-and-collisions.md` |
| Tutorial 4: Rendering & debug views | `tutorial/04-rendering-and-debug-views.md` |
| Tutorial 5: Input & gameplay loop | `tutorial/05-input-and-gameplay-loop.md` |
| Tutorial 6: Complete physics demo | `tutorial/06-building-a-simple-physics-demo.md` |
| Engineering reports | Root `.md` files: `NYON_ENGINE_ENGINEERING_REPORT.md`, `NYON_ENGINE_REPORT_2_DEEP_SWEEP.md` |
| Bug reports | Root `.md` files: `nyon_bug_report.md` |
| Camera system fix summary | `CAMERA_SYSTEM_FIX_SUMMARY.md` |
