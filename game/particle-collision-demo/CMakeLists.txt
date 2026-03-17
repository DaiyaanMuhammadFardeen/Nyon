cmake_minimum_required(VERSION 3.10)
project(ParticleCollisionDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

# Collect demo source files
file(GLOB DEMO_SOURCES "src/*.cpp")

# Create the executable
add_executable(particle_collision_demo ${DEMO_SOURCES})

# Link the engine library
target_link_libraries(particle_collision_demo PRIVATE nyon_engine)

# Set up include directories
target_include_directories(particle_collision_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Copy assets (shaders) to build directory
# (Assuming shaders are in engine/assets/shaders/)
# You may need to adjust this based on your build setup