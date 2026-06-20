## 1. Project Setup – Creating Your First Nyon Game

This guide shows how to create a new game or physics demo target that links against the `nyon_engine` library.

### 1.1. Repository layout (relevant pieces)

The important parts of the existing tree are:

- **Engine library** (already configured)
  - `engine/CMakeLists.txt` – builds the `nyon_engine` static library.
  - `engine/include/nyon/...` – public headers for the engine.
- **Existing demos** (reference examples)
  - `game/simple-physics-demo/` – basic falling-boxes demo.
  - `game/breakout-demo/` – Breakout game.
  - `game/flappy-demo/` – Flappy Bird clone.
  - `game/tower-stack-demo/` – Tower stacking game.
- **Root CMake project**
  - `CMakeLists.txt` – adds `engine` and each `game/*` subdirectory.

You will add your own game under `game/your-game-name/`.

### 1.2. Create the game directory

Create a new folder (for example `simple-demo`):

```
game/simple-demo/
├── CMakeLists.txt
├── include/
│   └── SimpleDemoGame.h
└── src/
    ├── SimpleDemoGame.cpp
    └── main.cpp
```

The exact name is up to you; this tutorial uses `simple-demo` in examples.

### 1.3. Root CMake: add your game

Open the root `CMakeLists.txt` in the repository and ensure it adds your new subdirectory:

```cmake
add_subdirectory(engine)
add_subdirectory(game/simple-demo)
```

If you later add multiple games, simply add more `add_subdirectory(...)` lines.

### 1.4. Game CMakeLists.txt

Inside `game/simple-demo/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.10)
project(SimpleDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(simple_demo
    src/main.cpp
    src/SimpleDemoGame.cpp
)

target_include_directories(simple_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/engine/include
)

target_link_libraries(simple_demo PRIVATE
    nyon_engine
    glfw
    OpenGL::GL
)

# Output to build/game/simple-demo/
set_target_properties(simple_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/game/simple-demo"
)
```

Key points:

- **`nyon_engine`** is the static library exported by `engine/CMakeLists.txt`.
- The executable includes both its own headers and the engine's public headers.
- GLFW and OpenGL are linked transitively through `nyon_engine`, but it is good practice to also find them explicitly in your own CMakeLists.

### 1.5. Minimal ECSApplication-based game class

Create `include/SimpleDemoGame.h`:

```cpp
#pragma once

#include "nyon/core/ECSApplication.h"

class SimpleDemoGame : public Nyon::ECSApplication
{
public:
    SimpleDemoGame()
        : Nyon::ECSApplication("Simple Demo", 1280, 720) {}

protected:
    // Called once after ECS is initialized — create entities here.
    void OnECSStart() override;

    // Called on a fixed timestep (60 Hz) — physics and deterministic logic.
    void OnECSFixedUpdate(float deltaTime) override;

    // Called after systems update — higher-level gameplay logic.
    void OnECSUpdate(float deltaTime) override;
};
```

Create `src/SimpleDemoGame.cpp`:

```cpp
#include "SimpleDemoGame.h"

using namespace Nyon;

void SimpleDemoGame::OnECSStart()
{
    // Here you will set up entities, components, and initial scene state.
}

void SimpleDemoGame::OnECSFixedUpdate(float deltaTime)
{
    // Fixed-step physics/game logic (e.g., apply forces, movement).
}

void SimpleDemoGame::OnECSUpdate(float deltaTime)
{
    // Per-step gameplay logic that runs after physics each fixed tick.
}
```

### 1.6. main.cpp – bootstrapping the engine

Create `src/main.cpp`:

```cpp
#include "SimpleDemoGame.h"

int main()
{
    SimpleDemoGame game;
    game.Run();  // Provided by Nyon::Application
    return 0;
}
```

The base class `Nyon::Application` (and thus `ECSApplication`) handles:

- Creating a GLFW window with an OpenGL context.
- Running a fixed-timestep game loop with interpolation:
  - `OnFixedUpdate` at 60 Hz for physics and ECS systems.
  - `OnInterpolateAndRender` for smooth rendering between fixed steps.
- `ECSApplication` overrides `OnStart()`, `OnFixedUpdate()`, and `OnInterpolateAndRender()` as `final`, routing to the ECS hooks (`OnECSStart`, `OnECSUpdate`, `OnECSFixedUpdate`).

### 1.7. Configuring and building

From the repository root:

```bash
mkdir -p build
cd build
cmake ..
cmake --build . --config Debug
```

If everything is configured correctly, you should get a `simple_demo` executable in `build/game/simple-demo/`. Running it should open an empty window with the main loop running.

### 1.8. Coordinate system note

Nyon uses a **Y-up** coordinate system consistent with OpenGL orthographic projection. The default camera maps world coordinates directly to screen pixels:

- `(0, 0)` is the bottom-left of the window.
- `(1280, 720)` is the top-right (for a 1280×720 window).
- Gravity points downward: `{0, -980}` pixels/s².

Next: move on to **`02-ecs-basics-and-entities.md`** to actually create entities and components inside your new game.
