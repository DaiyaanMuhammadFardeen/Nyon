README.md
# Nyon Game Engine

A 2D game engine built with OpenGL, GLFW, and GLAD for creating 2D games.

## Table of Contents
1. [Overview](#overview)
2. [Project Structure](#project-structure)
3. [Prerequisites](#prerequisites)
4. [Building the Project](#building-the-project)
5. [Running the Game](#running-the-game)
6. [Understanding the Engine Architecture](#understanding-the-engine-architecture)
7. [Creating Your Own Game](#creating-your-own-game)
8. [Engine Systems Deep Dive](#engine-systems-deep-dive)
9. [Advanced Topics](#advanced-topics)

## Overview

The Nyon Game Engine is a lightweight 2D game engine designed to provide developers with essential systems needed to create 2D games. It features:

- **Core Application Framework**: Base classes for creating game applications
- **2D Graphics Rendering**: Quad rendering with color support
- **Physics System**: Both AABB and SAT (Separating Axis Theorem) collision detection
- **Input Management**: Keyboard input handling system
- **Math Utilities**: Vector operations and transformations
- **Modular Architecture**: Clear separation between engine and game code

## Project Structure

```
Nyon/
├── engine/                 # Engine source code
│   ├── src/               # Engine source files
│   ├── include/nyon/      # Engine public headers
│   │   ├── core/          # Application framework
│   │   ├── graphics/      # Rendering system
│   │   ├── math/          # Mathematical utilities
│   │   └── utils/         # Utility systems (physics, input)
│   └── CMakeLists.txt     # Engine build configuration
├── game/                  # Game projects directory
│   └── smash-bros-clone/  # Sample 2D fighting game
│       ├── src/           # Game source files
│       ├── include/       # Game headers
│       └── CMakeLists.txt # Game build configuration
├── build/                 # Build artifacts
└── CMakeLists.txt         # Root build configuration
```

## Prerequisites

To build and use the Nyon Engine, you'll need:

- C++17 compatible compiler (GCC, Clang, or MSVC)
- CMake 3.10 or higher
- OpenGL development libraries
- GLFW3 development libraries
- GLM math library

On Arch Linux, install the required packages:
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
make
```

### Step 5: Verify Build Success
You should see targets built:
- `nyon_engine`: Static library containing the engine
- `smash_bros_clone`: Example game demonstrating engine usage

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

## Understanding the Engine Architecture

### Core Components

#### 1. Application Framework (`engine/include/nyon/core/`)
The Application class serves as the base for all games. It manages the main game loop, window creation, and event handling.

Key responsibilities:
- Initializes OpenGL context and creates a window
- Manages the main game loop (Update/Render cycle)
- Handles window events and cleanup

#### 2. Graphics System (`engine/include/nyon/graphics/`)
Provides 2D rendering capabilities through the Renderer2D class.

Features:
- Quad rendering with position, size, and color
- Batch rendering for performance
- Scene management with BeginScene/EndScene

#### 3. Math Utilities (`engine/include/nyon/math/`)
Contains mathematical structures needed for 2D game development.

Available types:
- Vector2: 2D vector operations
- Vector3: 3D vector operations (for colors and positions)

#### 4. Utility Systems (`engine/include/nyon/utils/`)
Additional systems that games commonly need.

Available systems:
- Physics: Collision detection and gravity simulation
- InputManager: Keyboard input handling

### Engine-Game Separation Strategy

The engine follows a clear separation strategy:
- Engine code lives in `engine/` directory
- Games live in `game/<game-name>/` directories
- Games link against the engine using CMake's `target_link_libraries`
- Engine headers are exposed through `engine/include/nyon/`

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

#include "nyon/core/Application.h"
#include "nyon/math/Vector2.h"
#include "nyon/math/Vector3.h"
#include "nyon/utils/InputManager.h"
#include "nyon/utils/Physics.h"
#include <vector>

class MyGameApp : public Nyon::Application
{
public:
    MyGameApp();

protected:
    virtual void OnStart() override;
    virtual void OnUpdate(float deltaTime) override;
    virtual void OnRender() override;

private:
    void HandleInput(float deltaTime);
    void UpdateGameLogic(float deltaTime);

    // Example game objects
    Nyon::Utils::Physics::Body m_Player;
    Nyon::Utils::Physics::Polygon m_PlayerShape;
    Nyon::Math::Vector2 m_PlayerSize;
    Nyon::Math::Vector3 m_PlayerColor;
    
    // More game-specific members...
};
```

### Step 3: Implement Your Game Application

Create `game/my-awesome-game/src/MyGameApp.cpp`:

```cpp
#include "MyGameApp.h"
#include "nyon/graphics/Renderer2D.h"
#include <iostream>

MyGameApp::MyGameApp()
    : Application("My Awesome Game", 1280, 720)
{
    std::cout << "My game application initialized" << std::endl;
}

void MyGameApp::OnStart()
{
    // Initialize renderer
    Nyon::Graphics::Renderer2D::Init();
    
    // Initialize input manager
    Nyon::Utils::InputManager::Init(GetWindow());
    
    // Initialize your game objects
    m_Player.position = Nyon::Math::Vector2(100.0f, 100.0f);
    m_Player.velocity = Nyon::Math::Vector2(0.0f, 0.0f);
    m_Player.acceleration = Nyon::Math::Vector2(0.0f, 0.0f);
    m_Player.mass = 1.0f;
    m_Player.isStatic = false;
    
    m_PlayerSize = Nyon::Math::Vector2(32.0f, 32.0f);
    m_PlayerColor = Nyon::Math::Vector3(0.0f, 0.8f, 1.0f); // Cyan
    
    // Define player as a polygon for SAT collision
    m_PlayerShape = {
        Nyon::Math::Vector2(0.0f, 0.0f),                    // bottom-left
        Nyon::Math::Vector2(m_PlayerSize.x, 0.0f),          // bottom-right
        Nyon::Math::Vector2(m_PlayerSize.x, m_PlayerSize.y), // top-right
        Nyon::Math::Vector2(0.0f, m_PlayerSize.y)           // top-left
    };
}

void MyGameApp::OnUpdate(float deltaTime)
{
    // Update input
    Nyon::Utils::InputManager::Update();
    
    // Handle player input
    HandleInput(deltaTime);
    
    // Update game logic
    UpdateGameLogic(deltaTime);
}

void MyGameApp::HandleInput(float deltaTime)
{
    // Handle keyboard input
    if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_A) || 
        Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_LEFT))
    {
        m_Player.position.x -= 200.0f * deltaTime; // Move left
    }
    else if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_D) || 
             Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_RIGHT))
    {
        m_Player.position.x += 200.0f * deltaTime; // Move right
    }
    
    // Add more input handling as needed
}

void MyGameApp::UpdateGameLogic(float deltaTime)
{
    // Apply physics updates, game logic, etc.
    // This is where your game-specific code goes
}

void MyGameApp::OnRender()
{
    // Clear screen
    glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    
    // Begin rendering
    Nyon::Graphics::Renderer2D::BeginScene();
    
    // Render your game objects
    Nyon::Graphics::Renderer2D::DrawQuad(
        m_Player.position,
        m_PlayerSize,
        Nyon::Math::Vector2(0.0f, 0.0f),
        m_PlayerColor
    );
    
    // End rendering
    Nyon::Graphics::Renderer2D::EndScene();
}
```

### Step 4: Create Your Game's CMakeLists.txt

Create `game/my-awesome-game/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.10)
project(my_awesome_game)

set(CMAKE_CXX_STANDARD 17)

# Find required packages
find_package(OpenGL REQUIRED)

# Add executable
add_executable(my_awesome_game
    src/main.cpp
    src/MyGameApp.cpp
    include/MyGameApp.h
)

# Include directories
target_include_directories(my_awesome_game PRIVATE include)
target_include_directories(my_awesome_game PRIVATE ../engine/include)

# Link libraries
target_link_libraries(my_awesome_game PRIVATE nyon_engine OpenGL::GL)

# Set the executable output directory
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
# ... existing content ...

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

### Physics System

The physics system provides both traditional AABB and advanced SAT collision detection:

#### AABB Collision (Simple Rectangles)
```cpp
bool collision = Nyon::Utils::Physics::CheckCollision(
    body1, size1,
    body2, size2
);
```

#### SAT Collision (Arbitrary Convex Polygons)
```cpp
using Polygon = std::vector<Nyon::Math::Vector2>;

Nyon::Utils::Physics::Polygon playerShape = {
    {0.0f, 0.0f},
    {32.0f, 0.0f},
    {32.0f, 32.0f},
    {0.0f, 32.0f}
};

bool collision = Nyon::Utils::Physics::CheckPolygonCollision(
    playerShape, playerPosition,
    enemyShape, enemyPosition
);
```

#### Physics Utilities
- Gravity constant: `Nyon::Utils::Physics::Gravity`
- Apply gravity: `Nyon::Utils::Physics::ApplyGravity(body)`
- Update body position: `Nyon::Utils::Physics::UpdateBody(body, deltaTime)`

### Input Management

The InputManager provides keyboard input handling:

```cpp
// Check if key is currently held down
if (Nyon::Utils::InputManager::IsKeyDown(GLFW_KEY_SPACE)) {
    // Action continues while key is held
}

// Check if key was just pressed (one frame only)
if (Nyon::Utils::InputManager::IsKeyPressed(GLFW_KEY_W)) {
    // Action happens once when key is pressed
}
```

### Graphics Rendering

The Renderer2D system provides simple quad rendering:

```cpp
// Initialize renderer (typically in OnStart)
Nyon::Graphics::Renderer2D::Init();

// In render loop
Nyon::Graphics::Renderer2D::BeginScene();

// Draw a quad
Nyon::Graphics::Renderer2D::DrawQuad(
    position,    // Nyon::Math::Vector2
    size,        // Nyon::Math::Vector2
    origin,      // Nyon::Math::Vector2 (usually {0, 0})
    color        // Nyon::Math::Vector3 (RGB values 0.0-1.0)
);

Nyon::Graphics::Renderer2D::EndScene();
```

## Advanced Topics

### Adding Custom Engine Features

To add new engine features:

1. Add headers to the appropriate directory in `engine/include/nyon/`
2. Implement functionality in `engine/src/`
3. Update the engine's CMakeLists.txt if adding new source files
4. Ensure the feature is accessible through public headers

### Performance Tips

1. **Batch Rendering**: Use the renderer's batch system by calling multiple DrawQuad between BeginScene/EndScene
2. **Minimize State Changes**: Group similar objects when rendering
3. **Efficient Collision**: Use AABB for simple checks, SAT only when polygon precision is needed
4. **Object Pooling**: For games with many temporary objects

### Debugging Tips

1. Use the provided debug prints in the sample game as a template
2. Check that your window is properly initialized before using InputManager
3. Validate that physics bodies are properly configured (mass, isStatic, etc.)
4. Monitor frame rate and optimize if needed

## Features

- Basic 2D rendering with sprite drawing
- Physics system with gravity and both AABB and SAT collision detection
- Input management system
- Entity-component-system ready architecture
- Separate engine and game code structure

## Architecture

The engine is designed with a clear separation between engine code and game code. The engine provides:
- Core application framework
- Graphics rendering system
- Input management
- Basic physics simulation with SAT collision detection
- Math utilities
- Modular design allowing easy extension

Games can extend the base Application class and use the engine's systems to build their specific game logic.