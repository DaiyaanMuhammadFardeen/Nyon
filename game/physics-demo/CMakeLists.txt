cmake_minimum_required(VERSION 3.10)
project(PhysicsDemoGame)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

# Create executable
add_executable(physics_demo_game 
    src/main.cpp
    src/PhysicsDemoGame.cpp
)

# Include directories
target_include_directories(physics_demo_game PRIVATE 
    include
    ../../engine/include
)

# Link with engine library and dependencies
target_link_libraries(physics_demo_game 
    nyon_engine
    OpenGL::GL 
    glfw 
    glm
)

# Copy shaders if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../engine/src/graphics/shaders")
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../../engine/src/graphics/shaders" 
         DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
endif()