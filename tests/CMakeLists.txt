# Tests CMakeLists.txt
include(FetchContent)

# Workaround for MSVC: GoogleTest enables C language which fails with /SUBSYSTEM:WINDOWS
# We need to set the linker flags for console applications
if (MSVC)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:CONSOLE")
endif()

# Fetch GoogleTest
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
# Disable GoogleTest's C language requirement
set(CMAKE_C_COMPILER_WORKS TRUE CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Enable testing
enable_testing()

# Collect all test source files
set(TEST_SOURCES
    test_math.cpp
    test_core.cpp
    test_shapes.cpp
    test_physics_system.cpp
    test_collision.cpp
    test_constraints.cpp
    test_character.cpp
    test_vehicle.cpp
    test_skeleton.cpp
)

# Create test executable
add_executable(joltc_tests ${TEST_SOURCES})

target_link_libraries(joltc_tests
    PRIVATE
        ${TARGET_NAME}
        GTest::gtest
        GTest::gtest_main
)

target_include_directories(joltc_tests
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)

# Add tests to CTest
include(GoogleTest)
gtest_discover_tests(joltc_tests)
