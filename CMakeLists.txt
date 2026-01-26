cmake_minimum_required(VERSION 3.10)
project(NyonRoot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

# Add the engine and game subdirectories
add_subdirectory(engine)
add_subdirectory(game/smash-bros-clone)
