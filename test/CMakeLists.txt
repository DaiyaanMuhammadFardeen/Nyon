cmake_minimum_required(VERSION 3.14)
project(NyonTests)

# Enable testing
enable_testing()

# Download and setup GoogleTest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Add test executable
file(GLOB_RECURSE TEST_SOURCES "*.cpp")
add_executable(nyon_tests ${TEST_SOURCES})

# Set up include directories
target_include_directories(nyon_tests PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../engine/include
)

# Link against the engine library and GoogleTest
target_link_libraries(nyon_tests 
    nyon_engine
    gtest_main
    gmock
)

# Add test discovery
include(GoogleTest)
gtest_discover_tests(nyon_tests)

# Custom target to run tests
add_custom_target(run_tests
    COMMAND nyon_tests
    DEPENDS nyon_tests
    WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)