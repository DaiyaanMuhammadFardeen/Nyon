cmake_minimum_required(VERSION 3.10)
project(nyon_camera_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/game/camera-demo/include
    ${CMAKE_SOURCE_DIR}/engine/include
    ${CMAKE_SOURCE_DIR}/vendor/glfw/include
    ${CMAKE_SOURCE_DIR}/vendor/glm
)

# Source files
set(SOURCES
    src/CameraDemo.cpp
    src/main.cpp
)

# Create executable
add_executable(nyon_camera_demo ${SOURCES})

# Link with engine and dependencies
target_link_libraries(nyon_camera_demo
    nyon_engine
    glfw
    ${CMAKE_DL_LIBS}
)

# Set output directory
set_target_properties(nyon_camera_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/game/camera-demo
)
