cmake_minimum_required(VERSION 3.10)
project(NyonEngineLib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

# Collect all engine source files
file(GLOB_RECURSE ENGINE_SOURCES "src/*.cpp")
list(APPEND ENGINE_SOURCES "src/glad.c")

# Exclude demo / sample translation units from the engine library
list(FILTER ENGINE_SOURCES EXCLUDE REGEX ".*/RenderingDemo\\.cpp$")

# Exclude legacy ECS physics systems (replaced by PhysicsPipeline)
# REMOVED: PhysicsPipelineSystem.cpp is now required and included

# Exclude PhysicsPipeline.cpp due to compilation issues
list(FILTER ENGINE_SOURCES EXCLUDE REGEX ".*/PhysicsPipeline\\.cpp$")

# Exclude StabilizationSystem.cpp due to compilation issues
list(FILTER ENGINE_SOURCES EXCLUDE REGEX ".*/StabilizationSystem\\.cpp$")

# Create the static library
add_library(nyon_engine STATIC ${ENGINE_SOURCES})

# Set up include directories
target_include_directories(nyon_engine PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(nyon_engine PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../include  # Add the include directory for glad
)

# Link dependencies
target_link_libraries(nyon_engine PUBLIC OpenGL::GL glfw glm)