cmake_minimum_required(VERSION 3.10)
project(SmashBrosClone)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

# Collect game source files
file(GLOB_RECURSE GAME_SOURCES "src/*.cpp")

# Create the game executable
add_executable(smash_bros_clone ${GAME_SOURCES})

# Include directories
target_include_directories(smash_bros_clone PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../engine/include
)

# Link the engine library
target_link_libraries(smash_bros_clone PRIVATE nyon_engine)