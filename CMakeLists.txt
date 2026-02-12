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

# Optional: Add testing subdirectory
option(ENABLE_TESTING "Enable testing" ON)
if(ENABLE_TESTING)
    enable_testing()
    add_subdirectory(test)
endif()

# Optional: Add documentation target
if(BUILD_DOCUMENTATION)
    find_package(Doxygen QUIET)
    if(DOXYGEN_FOUND)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
        add_custom_target(doc
            ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM
        )
    endif()
endif()

# Install documentation files
install(FILES 
    README.md 
    TUTORIAL.md 
    CHANGELOG.md
    DESTINATION share/doc/nyon
)