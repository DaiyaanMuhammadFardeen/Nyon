cmake_minimum_required(VERSION 3.10)
project(NyonPhysicsDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(nyon_physics_demo
    src/main.cpp
    src/PhysicsDemoGame.cpp
)

target_include_directories(nyon_physics_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/engine/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(nyon_physics_demo PRIVATE nyon_engine)

