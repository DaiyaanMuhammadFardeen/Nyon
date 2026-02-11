# Nyon Game Engine

A modern 2D game engine built with OpenGL, GLFW, and GLAD featuring a full Entity-Component-System (ECS) architecture for scalable game development.

## Table of Contents
1. [Overview](#overview)
2. [Project Structure](#project-structure)
3. [Prerequisites](#prerequisites)
4. [Building the Project](#building-the-project)
5. [Running the Game](#running-the-game)
6. [ECS Architecture](#ecs-architecture)
7. [Core Systems](#core-systems)
8. [Creating Your Own Game](#creating-your-own-game)
9. [Engine Systems Deep Dive](#engine-systems-deep-dive)
10. [Advanced Topics](#advanced-topics)

## Overview

The Nyon Game Engine is a modern 2D game engine designed with scalability and performance in mind. It features:

- **Full Entity-Component-System (ECS) Architecture**: Modular, data-oriented design for flexible game object composition
- **Advanced Physics System**: SAT collision detection, continuous collision detection, and realistic physics simulation
- **Multi-Shape Collision Support**: Polygons, circles, capsules, and composite shapes
- **Real-time Rendering**: Efficient 2D rendering with batching and interpolation
- **Input Management**: Comprehensive keyboard and mouse input handling
- **Mathematical Foundation**: Vector operations and transformations optimized for game development
- **Modern C++17**: Clean, type-safe codebase with modern C++ features

## Project Structure

```
Nyon/
├── engine/                 # Engine source code
│   ├── src/               # Engine source files
│   │   ├── core/          # Application framework
│   │   ├── ecs/           # Entity-Component-System implementation
│   │   ├── graphics/      # Rendering system
│   │   ├── math/          # Mathematical utilities
│   │   └── utils/         # Utility systems (physics, input)
│   ├── include/nyon/      # Engine public headers
│   │   ├── core/          # Application base classes
│   │   ├── ecs/           # ECS core and components
│   │   │   ├── components/ # Individual component types
│   │   │   └── systems/    # ECS system implementations
│   │   ├── graphics/      # Rendering interfaces
│   │   ├── math/          # Mathematical structures
│   │   └── utils/         # Physics and utility systems
│   └── CMakeLists.txt     # Engine build configuration
├── game/                  # Game projects directory
│   └── smash-bros-clone/  # Sample platformer game demonstrating ECS usage
│       ├── src/           # Game source files
│       ├── include/       # Game headers
│       └── CMakeLists.txt # Game build configuration
├── build/                 # Build artifacts
└── CMakeLists.txt         # Root build configuration
```

## Prerequisites

To build and use the Nyon Engine, you'll need:

- C++17 compatible compiler (GCC 7+, Clang 5+, or MSVC 2017+)
- CMake 3.10 or higher
- OpenGL development libraries
- GLFW3 development libraries
- GLM math library

On Arch Linux:
```bash
sudo pacman -S cmake gcc glfw-x11 glm
```

On Ubuntu/Debian:
```bash
sudo apt-get install build-essential cmake libglfw3-dev libglm-dev
```

On macOS with Homebrew:
```bash
brew install cmake glfw glm
```

## Building the Project

### Step 1: Clone the Repository
```bash
git clone https://github.com/yourusername/Nyon.git
cd Nyon
```

### Step 2: Create Build Directory
```bash
mkdir build
cd build
```

### Step 3: Configure with CMake
```bash
cmake ..
```

### Step 4: Build the Project
```bash
make -j$(nproc)
```

### Step 5: Verify Build Success
You should see targets built:
- `nyon_engine`: Static library containing the engine
- `smash_bros_clone`: Example game demonstrating ECS usage

## Running the Game

After building, run the example game:
```bash
./game/smash-bros-clone/smash_bros_clone
```

### Controls for Example Game
- A/Left Arrow: Move left
- D/Right Arrow: Move right
- Space/Up Arrow: Jump
- ESC: Quit

## ECS Architecture

### Core ECS Components

The Nyon Engine implements a modern Entity-Component-System architecture:

#### EntityManager
Manages entity lifecycle, ID generation, and entity validity tracking.

```cpp
// Create entities
Nyon::ECS::EntityID player = entityManager.CreateEntity();
Nyon::ECS::EntityID enemy = entityManager.CreateEntity();

// Destroy entities
entityManager.DestroyEntity(enemy);
```

#### ComponentStore
Stores components in Structure of Arrays (SoA) layout for optimal cache performance.

```cpp
// Add components to entities
componentStore.AddComponent(player, TransformComponent({100.0f, 100.0f}));
componentStore.AddComponent(player, PhysicsBodyComponent(1.0f));
componentStore.AddComponent(player, RenderComponent({32.0f, 32.0f}, {1.0f, 0.0f, 0.0f}));

// Access components
auto& transform = componentStore.GetComponent<TransformComponent>(player);
auto& physics = componentStore.GetComponent<PhysicsBodyComponent>(player);
```

#### Available Components

**TransformComponent**: Position, rotation, and scale
```cpp
TransformComponent transform({x, y}, {scaleX, scaleY}, rotation);
```

**PhysicsBodyComponent**: Velocity, acceleration, mass, and physical properties
```cpp
PhysicsBodyComponent physics(mass, isStatic);
physics.velocity = {vx, vy};
physics.friction = 0.1f;
```

**ColliderComponent**: Multi-shape collision support with physics materials
```cpp
// Polygon collider
ColliderComponent polygonCollider(polygonShape);
polygonCollider.material.friction = 0.8f;

// Circle collider
ColliderComponent circleCollider(radius);
circleCollider.material.restitution = 0.5f;
```

**RenderComponent**: Visual properties for rendering
```cpp
RenderComponent render({width, height}, {r, g, b}, "texture.png");
```

**BehaviorComponent**: Attachable game logic with lambda functions
```cpp
BehaviorComponent behavior;
behavior.SetUpdateFunction([](EntityID entity, float deltaTime) {
    // Custom update logic
});
```

### ECS Systems

Systems process entities with specific component combinations:

#### PhysicsSystem
Handles physics integration, gravity, friction, and movement.

#### CollisionSystem
Manages collision detection (AABB broad-phase + SAT narrow-phase) and resolution.

#### RenderSystem
Renders entities with interpolation for smooth motion between physics frames.

#### InputSystem
Processes user input and triggers behavior updates.

#### SystemManager
Orchestrates system execution order and manages the update cycle.

### ECS Application

Games inherit from `ECSApplication` to get full ECS functionality:

```cpp
class MyGame : public Nyon::ECSApplication
{
protected:
    void OnECSStart() override {
        // Initialize entities and components
    }
    
    void OnECSUpdate(float deltaTime) override {
        // Game-specific logic
    }
};
```

## Core Systems

### Physics System

Advanced physics simulation with multiple features:

#### Basic Physics Body
```cpp
Nyon::Utils::Physics::Body body;
body.position = {100.0f, 100.0f};
body.velocity = {0.0f, 0.0f};
body.mass = 1.0f;
body.isStatic = false;
```

#### Gravity Physics
```cpp
// Apply gravity with sub-stepping for stability
Nyon::Utils::GravityPhysics::UpdateBody(body, deltaTime, isGrounded);
```

#### Movement Physics
```cpp
// Apply forces and impulses
Nyon::Utils::MovementPhysics::ApplyForce(body, forceVector);
Nyon::Utils::MovementPhysics::ApplyImpulse(body, impulseVector);
```

### Collision Detection

Multiple collision detection methods for different use cases:

#### AABB Collision (Fast Broad-Phase)
```cpp
bool collision = Nyon::Utils::CollisionPhysics::CheckAABBCollision(
    pos1, size1, pos2, size2
);
```

#### SAT Collision (Precise Narrow-Phase)
```cpp
auto result = Nyon::Utils::CollisionPhysics::CheckPolygonCollision(
    polygon1, position1,
    polygon2, position2
);
// result.collided, result.overlapAxis, result.overlapAmount
```

#### Continuous Collision Detection (CCD)
```cpp
auto ccdResult = Nyon::Utils::CollisionPhysics::ContinuousCollisionCheck(
    polygon1, startPos1, endPos1,
    polygon2, startPos2, endPos2
);
// Prevents tunneling for fast-moving objects
```

#### Raycasting
```cpp
auto rayResult = Nyon::Utils::CollisionPhysics::RaycastPolygon(
    rayStart, rayEnd, polygon, polygonPosition
);
```

### Input Management

Comprehensive input handling system:

```cpp
// Check current key state
if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_SPACE)) {
    // Key is currently held
}

// Check single press events
if (Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_ENTER)) {
    // Key was just pressed this frame
}

// Mouse input
if (Nyon::Utils::InputManager::IsMouseDown(GLFW_MOUSE_BUTTON_LEFT)) {
    double mouseX, mouseY;
    Nyon::Utils::InputManager::GetMousePosition(mouseX, mouseY);
}
```

### Graphics Rendering

High-performance 2D rendering system:

```cpp
// Initialize renderer
Nyon::Graphics::Renderer2D::Init();

// Render loop
Nyon::Graphics::Renderer2D::BeginScene();

Nyon::Graphics::Renderer2D::DrawQuad(
    position,    // Vector2 world position
    size,        // Vector2 dimensions
    origin,      // Vector2 pivot point
    color        // Vector3 RGB color (0.0-1.0)
);

Nyon::Graphics::Renderer2D::EndScene();
```

## Creating Your Own Game

### Step 1: Set Up Your Game Directory

Create a new game directory under the `game/` folder:

```bash
mkdir game/my-awesome-game
cd game/my-awesome-game
mkdir src include
```

### Step 2: Create Your Game Application Header

Create `game/my-awesome-game/include/MyGameApp.h`:

```cpp
#pragma once

#include "nyon/core/ECSApplication.h"
#include "nyon/ecs/components/TransformComponent.h"
#include "nyon/ecs/components/PhysicsBodyComponent.h"
#include "nyon/ecs/components/ColliderComponent.h"
#include "nyon/ecs/components/RenderComponent.h"
#include "nyon/ecs/components/BehaviorComponent.h"
#include <vector>

class MyGameApp : public Nyon::ECSApplication
{
public:
    MyGameApp();

protected:
    void OnECSStart() override;
    void OnECSUpdate(float deltaTime) override;

private:
    void CreatePlayer();
    void SetupPlayerControls();
    
    Nyon::ECS::EntityID m_PlayerEntity;
    std::vector<Nyon::ECS::EntityID> m_EnemyEntities;
};
```

### Step 3: Implement Your Game Application

Create `game/my-awesome-game/src/MyGameApp.cpp`:

```cpp
#include "MyGameApp.h"
#include <iostream>

MyGameApp::MyGameApp()
    : ECSApplication("My Awesome Game", 1280, 720)
{
    std::cout << "My game application initialized with ECS" << std::endl;
}

void MyGameApp::OnECSStart()
{
    CreatePlayer();
    SetupPlayerControls();
    
    // Create enemies, platforms, etc.
}

void MyGameApp::CreatePlayer()
{
    auto& entityManager = GetEntityManager();
    auto& componentStore = GetComponentStore();
    
    // Create player entity
    m_PlayerEntity = entityManager.CreateEntity();
    
    // Add components
    componentStore.AddComponent(m_PlayerEntity, 
        Nyon::ECS::TransformComponent({100.0f, 100.0f}));
    
    componentStore.AddComponent(m_PlayerEntity,
        Nyon::ECS::PhysicsBodyComponent(1.0f, false));
    
    componentStore.AddComponent(m_PlayerEntity,
        Nyon::ECS::ColliderComponent({32.0f, 32.0f}));
    
    componentStore.AddComponent(m_PlayerEntity,
        Nyon::ECS::RenderComponent({32.0f, 32.0f}, {0.0f, 0.8f, 1.0f}));
}

void MyGameApp::SetupPlayerControls()
{
    auto& componentStore = GetComponentStore();
    
    Nyon::ECS::BehaviorComponent behavior;
    
    behavior.SetUpdateFunction([this](Nyon::ECS::EntityID entity, float deltaTime) {
        auto& physics = componentStore.GetComponent<Nyon::ECS::PhysicsBodyComponent>(entity);
        
        // Handle input
        if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_A)) {
            physics.velocity.x = -200.0f;
        } else if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_D)) {
            physics.velocity.x = 200.0f;
        } else {
            physics.velocity.x = 0.0f;
        }
        
        if (Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_SPACE) && physics.isGrounded) {
            physics.velocity.y = -400.0f;
            physics.isGrounded = false;
        }
    });
    
    componentStore.AddComponent(m_PlayerEntity, std::move(behavior));
}

void MyGameApp::OnECSUpdate(float deltaTime)
{
    // Additional game logic can go here
    // ECS systems handle most processing automatically
}
```

### Step 4: Create Your Game's CMakeLists.txt

Create `game/my-awesome-game/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.10)
project(my_awesome_game)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

# Add executable
add_executable(my_awesome_game
    src/main.cpp
    src/MyGameApp.cpp
)

# Include directories
target_include_directories(my_awesome_game PRIVATE include)
target_include_directories(my_awesome_game PRIVATE ../engine/include)

# Link libraries
target_link_libraries(my_awesome_game PRIVATE nyon_engine OpenGL::GL glfw glm)

# Set output directory
set_target_properties(my_awesome_game PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/game/my-awesome-game
)
```

### Step 5: Create Your Main Function

Create `game/my-awesome-game/src/main.cpp`:

```cpp
#include "MyGameApp.h"

int main()
{
    MyGameApp app;
    app.Run();
    return 0;
}
```

### Step 6: Register Your Game in Root CMakeLists.txt

Add your game to the root `CMakeLists.txt` file:

```cmake
# Add subdirectories
add_subdirectory(engine)
add_subdirectory(game/smash-bros-clone)
add_subdirectory(game/my-awesome-game)  # Add this line
```

### Step 7: Build and Run Your Game

From your build directory:

```bash
cd build
make my_awesome_game
./game/my-awesome-game/my_awesome_game
```

## Engine Systems Deep Dive

### Advanced ECS Patterns

#### Component Queries
```cpp
// Get all entities with specific components
const auto& physicsEntities = componentStore.GetEntitiesWithComponent<PhysicsBodyComponent>();
const auto& renderEntities = componentStore.GetEntitiesWithComponent<RenderComponent>();

// Process entities with multiple components
for (auto entity : physicsEntities) {
    if (componentStore.HasComponent<ColliderComponent>(entity)) {
        // Entity has both physics and collision components
        auto& physics = componentStore.GetComponent<PhysicsBodyComponent>(entity);
        auto& collider = componentStore.GetComponent<ColliderComponent>(entity);
        // Process combined behavior
    }
}
```

#### System Dependencies
Systems are executed in order to ensure proper data flow:
1. InputSystem - Processes user input first
2. PhysicsSystem - Updates physics bodies
3. CollisionSystem - Detects and resolves collisions
4. RenderSystem - Draws the final frame

#### Custom Systems
Create custom systems by inheriting from the System base class:

```cpp
class AISystem : public Nyon::ECS::System
{
public:
    void Update(float deltaTime) override {
        const auto& aiEntities = m_ComponentStore->GetEntitiesWithComponent<AIBehaviorComponent>();
        for (auto entity : aiEntities) {
            auto& ai = m_ComponentStore->GetComponent<AIBehaviorComponent>(entity);
            // Process AI logic
        }
    }
};
```

### Physics Materials and Surfaces

Configure realistic surface interactions:

```cpp
ColliderComponent collider;
collider.material.friction = 0.1f;      // Ice-like surface
collider.material.restitution = 0.8f;   // Bouncy surface
collider.material.density = 2.0f;       // Heavy material
collider.material.surfaceType = "metal"; // For audio/effects
```

### Multi-Shape Colliders

Support for different collision geometries:

```cpp
// Circle collider for balls/projectiles
ColliderComponent circle(16.0f); // 16-pixel radius

// Polygon collider for complex shapes
std::vector<Vector2> complexShape = {
    {0, 0}, {32, 0}, {48, 16}, {32, 32}, {0, 32}
};
ColliderComponent customPolygon(complexShape);

// Composite collider for complex objects
ColliderComponent::CompositeShape composite;
composite.subShapes.push_back(baseShape);
composite.subShapes.push_back(attachmentShape);
```

### Behavior System

Attach custom logic to entities without modifying engine code:

```cpp
BehaviorComponent behavior;

// Update behavior (runs every frame)
behavior.SetUpdateFunction([](EntityID entity, float deltaTime) {
    // Custom per-frame logic
});

// Collision behavior (runs on collision events)
behavior.SetCollisionFunction([](EntityID entity, EntityID other) {
    // Handle collision with other entity
});

componentStore.AddComponent(entity, std::move(behavior));
```

## Advanced Topics

### Performance Optimization

1. **Entity Pooling**: Reuse entity IDs to reduce allocation overhead
2. **System Ordering**: Arrange systems to minimize data dependencies
3. **Batch Rendering**: Group similar render calls for better GPU performance
4. **Spatial Partitioning**: Implement quadtrees/octrees for collision optimization
5. **Component Archetypes**: Group entities with similar component layouts

### Memory Management

The ECS uses Structure of Arrays for optimal cache performance:
- Components of the same type are stored contiguously
- Iteration over components is cache-friendly
- Memory layout can be optimized for specific access patterns

### Extending the Engine

#### Adding New Components
1. Create header in `engine/include/nyon/ecs/components/`
2. Components should be pure data structures
3. Register with ComponentStore template system

#### Adding New Systems
1. Inherit from `Nyon::ECS::System`
2. Implement `Update()` method
3. Register with SystemManager in ECSApplication

#### Custom Physics Shapes
Extend the ColliderComponent to support new shapes:
1. Add new ShapeType enum value
2. Extend the variant with new shape structure
3. Implement collision detection in CollisionPhysics

### Debugging and Profiling

Enable debug output:
```cpp
#define NYON_DEBUG_LOGGING 1
```

Monitor system performance:
```cpp
// Track entity counts
size_t entityCount = entityManager.GetActiveEntityCount();

// Monitor component usage
const auto& physicsEntities = componentStore.GetEntitiesWithComponent<PhysicsBodyComponent>();
std::cout << "Physics entities: " << physicsEntities.size() << std::endl;
```

## Features

### Core Engine Features
- **Entity-Component-System Architecture**: Modern, scalable game object model
- **Advanced Physics**: SAT collision detection, CCD, and realistic physics simulation
- **Multi-Shape Collisions**: Polygons, circles, capsules, and composite shapes
- **Real-time Rendering**: Efficient 2D rendering with batching and interpolation
- **Input Management**: Comprehensive keyboard and mouse input handling
- **Mathematical Foundation**: Optimized vector operations for game development
- **Modular Design**: Clean separation between engine systems and game code

### ECS Features
- **Flexible Entity Composition**: Mix components freely without inheritance hierarchies
- **High Performance**: Structure of Arrays storage for cache-friendly access
- **Extensible Systems**: Easy to add custom processing systems
- **Behavior Attachment**: Lambda-based behaviors for custom entity logic
- **Automatic Cleanup**: Proper resource management and entity lifecycle handling

### Physics Features
- **Continuous Collision Detection**: Prevents tunneling for fast-moving objects
- **Physics Materials**: Per-collider friction, restitution, and surface properties
- **Raycasting**: Line-of-sight and projectile collision detection
- **Collision Response**: Realistic collision resolution with momentum transfer
- **Grounded State Detection**: Automatic platform and surface detection

## Architecture

The Nyon Engine follows modern game engine architecture principles:

### Data-Oriented Design
- Components stored in arrays for cache efficiency
- Systems process data in batches
- Minimized indirection and virtual function calls

### Separation of Concerns
- Engine provides core systems and infrastructure
- Games implement specific logic through ECS components
- Clear API boundaries between engine and game code

### Scalability
- ECS architecture scales from simple prototypes to complex games
- Component-based design allows easy feature addition
- Systems can be enabled/disabled based on game requirements

The engine is production-ready for 2D game development with enterprise-grade architecture suitable for both indie developers and professional studios.