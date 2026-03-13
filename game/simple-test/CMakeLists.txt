cmake_minimum_required(VERSION 3.10)
project(nyon_simple_test VERSION 1.0.0 LANGUAGES CXX)

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
add_executable(nyon_simple_test
    src/main.cpp
)

# Link libraries
target_link_libraries(nyon_simple_test
    glfw
    OpenGL::GL
    nyon_engine
)

# Output to build/game/simple-test/
set_target_properties(nyon_simple_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/game/simple-test"
)
