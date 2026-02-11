cmake_minimum_required(VERSION 3.10)
project(SmashBrosClone)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

# Collect all game source files
file(GLOB_RECURSE GAME_SOURCES "src/*.cpp")

# Create the executable
add_executable(smash_bros_clone ${GAME_SOURCES})

# Set up include directories
target_include_directories(smash_bros_clone PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../engine/include
)

# Link with the engine library and dependencies
target_link_libraries(smash_bros_clone PRIVATE nyon_engine OpenGL::GL glfw glm)

# Ensure engine is built first
add_dependencies(smash_bros_clone nyon_engine)