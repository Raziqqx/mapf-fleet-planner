cmake_minimum_required(VERSION 3.24)
project(mapf_fleet_planner VERSION 0.1.0 LANGUAGES CXX)

# Entry points (see CMakePresets.json):
#   cmake --preset release && cmake --build --preset release
#   cmake --preset asan && cmake --build --preset asan && ctest --preset asan

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(MAPF_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(MAPF_BUILD_TESTS "Build the GoogleTest suite (downloads GoogleTest)" ON)

add_library(mapf_build_flags INTERFACE)
target_compile_options(mapf_build_flags INTERFACE -Wall -Wextra -Wpedantic -Werror)
if(MAPF_SANITIZE)
    # -fno-sanitize-recover: a UB report fails the test instead of only being logged.
    set(MAPF_SANITIZE_FLAGS -fsanitize=address,undefined -fno-sanitize-recover=undefined
                            -fno-omit-frame-pointer)
    target_compile_options(mapf_build_flags INTERFACE ${MAPF_SANITIZE_FLAGS})
    target_link_options(mapf_build_flags INTERFACE ${MAPF_SANITIZE_FLAGS})
endif()

# Planner library: grids, MovingAI loaders, prioritized planning, CBS, ECBS.
add_library(mapf STATIC
    src/cbs.cpp
    src/conflicts.cpp
    src/grid.cpp
    src/instance.cpp
    src/low_level.cpp
    src/prioritized.cpp
    src/space_time.cpp
)
target_include_directories(mapf PUBLIC include PRIVATE src)
target_link_libraries(mapf PUBLIC mapf_build_flags)

# Independent solution validator: standard library only, no planner code.
add_library(mapf_validator STATIC validator/src/validator.cpp)
target_include_directories(mapf_validator PUBLIC validator/include)
target_link_libraries(mapf_validator PUBLIC mapf_build_flags)

# Benchmark runner: map x agents x planner x scenario -> CSV.
add_executable(mapf_bench bench/bench_main.cpp bench/adapter.cpp)
target_include_directories(mapf_bench PRIVATE bench)
target_link_libraries(mapf_bench PRIVATE mapf mapf_validator)

if(MAPF_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    add_executable(mapf_tests
        tests/test_io.cpp
        tests/test_low_level.cpp
        tests/test_planners.cpp
        tests/test_brute_force.cpp
        tests/test_property.cpp
        tests/test_validator.cpp
        bench/adapter.cpp
    )
    # Tests reach into src/ for the low-level search and conflict detection.
    target_include_directories(mapf_tests PRIVATE src bench tests)
    target_link_libraries(mapf_tests PRIVATE mapf mapf_validator GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(mapf_tests DISCOVERY_TIMEOUT 60)
endif()
