cmake_minimum_required(VERSION 3.10)
project(Nyon)

# Add subdirectories
add_subdirectory(engine)
# add_subdirectory(test)  # Temporarily disabled due to CMake conflicts
add_subdirectory(game/physics-demo)  # Add our new physics demo game

# Main project executable (optional - can be removed if not needed)
# add_executable(nyon_main src/main.cpp)
# target_link_libraries(nyon_main nyon_engine)

# Optional: Add testing subdirectory
option(ENABLE_TESTING "Enable testing" OFF)
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