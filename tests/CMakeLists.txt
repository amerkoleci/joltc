# Tests CMakeLists.txt

include(FetchContent)

# Fetch GoogleTest
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
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

# Ensure console subsystem on Windows (required for gtest_main)
if(MSVC)
    target_link_options(joltc_tests PRIVATE /SUBSYSTEM:CONSOLE)
endif()

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

# Add tests to CTest (skip discovery for cross-compilation or when target arch differs from host)
include(GoogleTest)

# Detect cross-compilation scenarios where test discovery won't work
set(SKIP_TEST_DISCOVERY FALSE)
if(CMAKE_CROSSCOMPILING)
    set(SKIP_TEST_DISCOVERY TRUE)
elseif(MSVC AND CMAKE_GENERATOR_PLATFORM)
    # Visual Studio: check if target platform differs from host
    if(CMAKE_GENERATOR_PLATFORM STREQUAL "ARM64" OR CMAKE_GENERATOR_PLATFORM STREQUAL "ARM")
        set(SKIP_TEST_DISCOVERY TRUE)
    endif()
endif()

if(SKIP_TEST_DISCOVERY)
    # For cross-compilation, just add the test executable without discovery
    add_test(NAME joltc_tests COMMAND joltc_tests)
else()
    gtest_discover_tests(joltc_tests)
endif()
