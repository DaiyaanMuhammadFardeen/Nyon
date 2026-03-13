# Quick Cleanup & Fix Guide

## 🗑️ DELETE THESE FILES IMMEDIATELY

### Dead ECS Systems (3 files)
```bash
rm engine/src/ecs/systems/CollisionPipelineSystem.cpp
rm engine/src/ecs/systems/CollisionPipelineSystem.h
# PhysicsIntegrationSystem is only mentioned in comments, no file to delete
```

### Dead Physics Modules (14 files)
```bash
# Boundary system - not used
rm engine/include/nyon/physics/BoundarySystem.h
rm engine/src/physics/BoundarySystem.cpp

# Continuous collision detection - not integrated
rm engine/include/nyon/physics/ContinuousCollisionDetection.h
rm engine/src/physics/ContinuousCollisionDetection.cpp

# Standalone constraint solver - replaced by integrated solver
rm engine/include/nyon/physics/ConstraintSolver.h
rm engine/src/physics/ConstraintSolver.cpp

# SAT collision detector - not used in main pipeline
rm engine/include/nyon/physics/SATCollisionDetector.h
rm engine/src/physics/SATCollisionDetector.cpp

# Stabilization system - excluded from build anyway
rm engine/include/nyon/physics/StabilizationSystem.h
rm engine/src/physics/StabilizationSystem.cpp

# Old physics pipeline - different from PhysicsPipelineSystem
rm engine/include/nyon/physics/PhysicsPipeline.h
rm engine/src/physics/PhysicsPipeline.cpp
```

**Total: 17 files to delete**

---

## ✅ KEEP THESE (Core Architecture)

### Essential Core (5 files)
```
engine/include/nyon/core/Application.h
engine/src/core/Application.cpp
engine/include/nyon/core/ECSApplication.h
engine/src/core/ECSApplication.cpp
```

### ECS Framework (6 files)
```
engine/include/nyon/ecs/EntityManager.h
engine/src/ecs/EntityManager.cpp
engine/include/nyon/ecs/ComponentStore.h
engine/src/ecs/ComponentStore.cpp
engine/include/nyon/ecs/SystemManager.h
engine/src/ecs/SystemManager.cpp
engine/include/nyon/ecs/System.h
```

### Active Systems (8 files)
```
engine/include/nyon/ecs/systems/PhysicsPipelineSystem.h
engine/src/ecs/systems/PhysicsPipelineSystem.cpp  ⭐ MAIN PHYSICS
engine/include/nyon/ecs/systems/RenderSystem.h
engine/src/ecs/systems/RenderSystem.cpp
engine/include/nyon/ecs/systems/InputSystem.h
engine/src/ecs/systems/InputSystem.cpp
engine/include/nyon/ecs/systems/DebugRenderSystem.h
engine/src/ecs/systems/DebugRenderSystem.cpp
```

### Components (7 files)
```
engine/include/nyon/ecs/components/TransformComponent.h
engine/include/nyon/ecs/components/PhysicsBodyComponent.h
engine/include/nyon/ecs/components/ColliderComponent.h
engine/include/nyon/ecs/components/PhysicsWorldComponent.h
engine/include/nyon/ecs/components/RenderComponent.h
engine/include/nyon/ecs/components/JointComponent.h
engine/include/nyon/ecs/components/BehaviorComponent.h
```

### Graphics (2 files)
```
engine/include/nyon/graphics/Renderer2D.h
engine/src/graphics/Renderer2D.cpp
```

### Physics Internals (3 files - keep minimal)
```
engine/include/nyon/physics/DynamicTree.h
engine/src/physics/DynamicTree.cpp  (broad phase support)
engine/include/nyon/physics/Island.h
engine/src/physics/Island.cpp  (sleeping optimization)
engine/include/nyon/physics/ManifoldGenerator.h
engine/src/physics/ManifoldGenerator.cpp  (contact generation)
```

### Utils & Math (4 files)
```
engine/include/nyon/utils/InputManager.h
engine/src/utils/InputManager.cpp
engine/include/nyon/math/Vector2.h
engine/include/nyon/math/Vector3.h
```

**Total to keep: ~35 files (down from 52+)**

---

## 🔧 FIX THE PHYSICS DEMO

### Step 1: Enable Boundaries (CRITICAL)

Edit `game/physics-demo/src/PhysicsDemoGame.cpp` line 86-149:

**CHANGE:**
```cpp
void PhysicsDemoGame::CreateBounds()
{
    // TEMPORARILY DISABLED - Testing collision system without boundaries
    // Uncomment to enable boundary walls
    
    /*
    ... all the code is commented out ...
    */
}
```

**TO:**
```cpp
void PhysicsDemoGame::CreateBounds()
{
    auto& entities   = GetEntityManager();
    auto& components = GetComponentStore();

    const float width  = 1600.0f;
    const float height = 900.0f;
    const float thickness = 50.0f;

    using Nyon::Math::Vector2;

    auto createWall = [&](const Vector2& center, const Vector2& halfExtents)
    {
        ECS::EntityID wall = entities.CreateEntity();

        ECS::TransformComponent t;
        t.position = center;

        ECS::PhysicsBodyComponent body;
        body.isStatic = true;
        body.UpdateMassProperties();

        ECS::ColliderComponent::PolygonShape shape({
            {-halfExtents.x, -halfExtents.y},
            { halfExtents.x, -halfExtents.y},
            { halfExtents.x,  halfExtents.y},
            {-halfExtents.x,  halfExtents.y}
        });

        ECS::ColliderComponent collider(shape);
        collider.material.friction    = 0.8f;
        collider.material.restitution = 0.0f;

        ECS::RenderComponent render(
            {halfExtents.x * 2.0f, halfExtents.y * 2.0f},
            {0.2f, 0.2f, 0.2f}
        );

        components.AddComponent(wall, std::move(t));
        components.AddComponent(wall, std::move(body));
        components.AddComponent(wall, std::move(collider));
        components.AddComponent(wall, std::move(render));
    };

    // Floor
    createWall({width * 0.5f, height + thickness * 0.5f - 1.0f}, {width * 0.5f, thickness * 0.5f});
    
    // Ceiling
    createWall({width * 0.5f, -thickness * 0.5f + 1.0f}, {width * 0.5f, thickness * 0.5f});
    
    // Left wall
    createWall({-thickness * 0.5f + 1.0f, height * 0.5f}, {thickness * 0.5f, height * 0.5f});
    
    // Right wall
    createWall({width + thickness * 0.5f - 1.0f, height * 0.5f}, {thickness * 0.5f, height * 0.5f});
}
```

### Step 2: Add Debug Logging

Edit `engine/src/ecs/systems/PhysicsPipelineSystem.cpp` line 268-303:

**ADD after NarrowPhaseDetection() opening brace:**
```cpp
void PhysicsPipelineSystem::NarrowPhaseDetection()
{
    m_ContactManifolds.clear();
    m_ContactMap.clear();
    
    std::cerr << "[PHYSICS] NarrowPhase: Testing " << m_BroadPhasePairs.size() 
              << " broad phase pairs" << std::endl;
    
    // Test each broad phase pair for actual collision
    for (const auto& [entityIdA, entityIdB] : m_BroadPhasePairs)
    {
        std::cerr << "[PHYSICS]   Testing pair: " << entityIdA << " vs " << entityIdB << std::endl;
        
        if (TestCollision(entityIdA, entityIdB))
        {
            std::cerr << "[PHYSICS]     ✓ Collision detected!" << std::endl;
            
            ContactManifold manifold = GenerateManifold(entityIdA, entityIdB);
            if (!manifold.points.empty())
            {
                std::cerr << "[PHYSICS]     ✓ Generated " << manifold.points.size() 
                          << " contact points" << std::endl;
                
                // ... rest of existing code ...
```

### Step 3: Verify Gravity Direction

In `game/physics-demo/src/PhysicsDemoGame.cpp` line 62:

**Current (Y-down coordinate system):**
```cpp
world.gravity = { 0.0f, 980.0f };  // Objects fall DOWN (positive Y)
```

**If you want Y-up (more intuitive):**
```cpp
world.gravity = { 0.0f, -980.0f };  // Objects fall DOWN (negative Y)
```

### Step 4: Simplify Test Scene

Temporarily reduce complexity. In `PhysicsDemoGame::OnECSStart()` line 24-31:

**CHANGE:**
```cpp
void PhysicsDemoGame::OnECSStart()
{
    CreatePhysicsWorld();
    CreateBounds();  // Enable boundaries first
    CreateStackTest();  // Comment out for initial testing
    CreateRampFrictionTest();  // Comment out
    CreateBounceTest();  // Comment out
}
```

**TO (simplest test):**
```cpp
void PhysicsDemoGame::OnECSStart()
{
    CreatePhysicsWorld();
    CreateBounds();  // ← MUST enable this
    
    // Disable complex tests initially
    // CreateStackTest();
    // CreateRampFrictionTest();
    // CreateBounceTest();
    
    // Simple test: single falling box
    auto& entities = GetEntityManager();
    auto& components = GetComponentStore();
    
    ECS::EntityID box = entities.CreateEntity();
    
    ECS::TransformComponent t;
    t.position = {800.0f, 400.0f};  // Start in middle of screen
    
    ECS::PhysicsBodyComponent body;
    body.mass = 1.0f;
    body.UpdateMassProperties();
    
    ECS::ColliderComponent::PolygonShape shape({
        {-25.0f, -25.0f},
        { 25.0f, -25.0f},
        { 25.0f,  25.0f},
        {-25.0f,  25.0f}
    });
    
    ECS::ColliderComponent collider(shape);
    collider.material.friction = 0.4f;
    collider.material.restitution = 0.2f;
    
    ECS::RenderComponent render(
        {50.0f, 50.0f},
        {1.0f, 0.0f, 0.0f}  // Red box
    );
    
    components.AddComponent(box, std::move(t));
    components.AddComponent(box, std::move(body));
    components.AddComponent(box, std::move(collider));
    components.AddComponent(box, std::move(render));
}
```

---

## 🧪 TESTING CHECKLIST

After making changes:

### Build Test
```bash
cd build
cmake ..
make -j4
# Should compile without errors
```

### Runtime Test - Single Box
1. Run the physics demo
2. Expect: Single red box falls and lands on floor
3. If box doesn't fall:
   - Check gravity is set
   - Check body.mass > 0
   - Check body.isStatic = false
4. If box falls through floor:
   - Check boundaries are enabled
   - Check floor position is correct
   - Look for collision debug output

### Runtime Test - Stack
1. Enable `CreateStackTest()`
2. Expect: Boxes stack and settle
3. If boxes explode:
   - Reduce friction (try 0.2f)
   - Increase position iterations (try 5)
   - Check for initial overlaps

### Debug Output
Look for these console messages:
```
[PHYSICS] Found PhysicsWorldComponent at entity ID: X
[PHYSICS] BroadPhase: Found X potential pairs
[PHYSICS] NarrowPhase: Generated X contact manifolds
[PHYSICS] Updated X bodies
```

If you don't see these, physics isn't running!

---

## 🐛 COMMON ISSUES & FIXES

### Issue 1: No collision sounds happening
**Symptom:** Objects pass through each other
**Check:**
- Are both objects non-static?
- Do both have ColliderComponent?
- Is friction/restitution > 0?
- Check debug output for "Collision detected"

### Issue 2: Objects teleport or jitter
**Symptom:** Unstable simulation
**Fix:**
- Reduce time step (try 1/120s instead of 1/60s)
- Increase position iterations (try 5-10)
- Check for large mass ratios (< 10:1 recommended)

### Issue 3: Performance is terrible
**Symptom:** < 30 FPS with few objects
**Fix:**
- Enable sleeping (should already be on)
- Check DynamicTree is being used (not brute force)
- Reduce debug logging frequency

### Issue 4: Static bodies move
**Symptom:** "Static" walls get pushed
**Check:**
- `PhysicsBodyComponent::isStatic = true`
- Position solving shouldn't move static bodies (line 759-779)
- Make sure infinite mass (inverseMass = 0)

---

## 📊 EXPECTED RESULTS

### Minimal Working Demo
After cleanup and fixes, you should have:

**Files:** ~35 source files (was 52+)
**Systems:** 4 active systems (Input, Physics, Render, Debug)
**Physics:** Single unified pipeline

**Performance targets:**
- 60 FPS physics (fixed timestep)
- 60+ FPS rendering with 50-100 objects
- < 5ms physics update time with 50 objects

**Features working:**
- ✓ Gravity
- ✓ Collisions (box-box, circle-circle)
- ✓ Stacking (up to 8-10 objects)
- ✓ Bouncing (restitution)
- ✓ Friction
- ✓ Sleeping (objects at rest)
- ✓ Boundary collisions

---

## 🎯 NEXT STEPS AFTER CLEANUP

Once basics work:

1. **Add more features:**
   - Joints (revolute, prismatic)
   - Continuous collision detection (for bullets)
   - Triggers/sensors

2. **Improve stability:**
   - Sub-stepping (already partially implemented)
   - Better position correction
   - Constraint bias

3. **Add content:**
   - Dominoes
   - Ragdolls
   - Vehicles
   - Particle systems

4. **Optimize:**
   - SIMD for math operations
   - Multi-threaded island solving
   - Better memory layout

But first: **GET THE BASICS WORKING** ✅
