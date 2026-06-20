cmake_minimum_required(VERSION 3.10)
project(nyon_flappy_demo VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

# Include directories
include_directories(
    include
    ../../engine/include
)

# Executable
add_executable(nyon_flappy_demo
    src/main.cpp
    src/FlappyDemo.cpp
)

# Link libraries
target_link_libraries(nyon_flappy_demo
    glfw
    OpenGL::GL
    nyon_engine
)

# Output to build/game/flappy-demo/
set_target_properties(nyon_flappy_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/game/flappy-demo"
)
