# Nyon Game Engine

A 2D game engine built with OpenGL, GLFW, and GLAD for creating 2D games.

## Project Structure

```
Nyon/
├── engine/                 # Engine source code
│   ├── src/               # Engine source files
│   ├── include/nyon/      # Engine public headers
│   └── CMakeLists.txt     # Engine build configuration
├── game/                  # Game projects directory
│   └── smash-bros-clone/  # Sample 2D fighting game
│       ├── src/           # Game source files
│       ├── include/       # Game headers
│       └── CMakeLists.txt # Game build configuration
├── build/                 # Build artifacts
└── CMakeLists.txt         # Root build configuration
```

## Build Instructions

### Prerequisites

- C++17 compatible compiler (GCC, Clang, or MSVC)
- CMake 3.10 or higher
- OpenGL development libraries
- GLFW3 development libraries
- GLM math library

On Arch Linux, install the required packages:
```bash
sudo pacman -S cmake gcc glfw-x11 glm
```

### Building the Project

1. Create a build directory:
```bash
mkdir build
cd build
```

2. Configure the project with CMake:
```bash
cmake ..
```

3. Build the project:
```bash
make
```

4. Run the game:
```bash
./game/smash-bros-clone/smash_bros_clone
```

### Controls

- A/Left Arrow: Move left
- D/Right Arrow: Move right
- Space/Up Arrow: Jump
- ESC: Quit

## Features

- Basic 2D rendering with sprite drawing
- Physics system with gravity and collision detection
- Input management system
- Entity-component-system ready architecture
- Separate engine and game code structure

## Architecture

The engine is designed with a clear separation between engine code and game code. The engine provides:
- Core application framework
- Graphics rendering system
- Input management
- Basic physics simulation
- Math utilities

Games can extend the base Application class and use the engine's systems to build their specific game logic.