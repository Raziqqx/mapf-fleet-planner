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
# Tests and tools default to on only when this is the top-level project, so a
# consumer using add_subdirectory() gets just the libraries.
option(MAPF_BUILD_TESTS "Build the GoogleTest suite (downloads GoogleTest)" ${PROJECT_IS_TOP_LEVEL})
option(MAPF_BUILD_TOOLS "Build the benchmark and profiling executables" ${PROJECT_IS_TOP_LEVEL})
option(MAPF_BUILD_PYTHON "Build the pybind11 module mapf_py" OFF)

# Warnings-as-errors and sanitizers apply to this project's own targets only
# (linked PRIVATE), never to a consumer that links the libraries.

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
target_include_directories(mapf PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                       $<INSTALL_INTERFACE:include> PRIVATE src)
target_link_libraries(mapf PRIVATE $<BUILD_INTERFACE:mapf_build_flags>)
set_target_properties(mapf PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(mapf::mapf ALIAS mapf)

# Independent solution validator: standard library only, no planner code.
add_library(mapf_validator STATIC validator/src/validator.cpp)
target_include_directories(mapf_validator
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/validator/include>
           $<INSTALL_INTERFACE:include>)
target_link_libraries(mapf_validator PRIVATE $<BUILD_INTERFACE:mapf_build_flags>)
set_target_properties(mapf_validator PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(mapf::validator ALIAS mapf_validator)

# `cmake --install` puts both libraries, their headers and a CMake package
# (find_package(mapf_fleet_planner) -> mapf::mapf, mapf::validator) under the prefix.
include(GNUInstallDirs)
install(TARGETS mapf mapf_validator EXPORT mapf_targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/mapf validator/include/mapf_validator
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT mapf_targets NAMESPACE mapf:: FILE mapf_fleet_plannerConfig.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mapf_fleet_planner)
set_target_properties(mapf_validator PROPERTIES EXPORT_NAME validator)

if(MAPF_BUILD_TOOLS)
    # Benchmark runner: map x agents x planner x scenario -> CSV.
    add_executable(mapf_bench bench/bench_main.cpp bench/adapter.cpp)
    target_include_directories(mapf_bench PRIVATE bench)
    target_link_libraries(mapf_bench PRIVATE mapf mapf_validator mapf_build_flags)

    # Profiling driver: one instance group under one SearchTuning -> one CSV row.
    add_executable(mapf_profile bench/profile_main.cpp bench/adapter.cpp)
    target_include_directories(mapf_profile PRIVATE bench)
    target_link_libraries(mapf_profile PRIVATE mapf mapf_validator mapf_build_flags)
endif()

if(MAPF_BUILD_PYTHON)
    # Prefer an installed pybind11 (pip install pybind11), else fetch it.
    find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)
    find_package(pybind11 2.11 CONFIG QUIET)
    if(NOT pybind11_FOUND)
        include(FetchContent)
        FetchContent_Declare(pybind11
            URL https://github.com/pybind/pybind11/archive/refs/tags/v2.13.6.tar.gz
            DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
        FetchContent_MakeAvailable(pybind11)
    endif()
    pybind11_add_module(mapf_py python/bindings.cpp bench/adapter.cpp)
    target_include_directories(mapf_py PRIVATE bench)
    target_link_libraries(mapf_py PRIVATE mapf mapf_validator)
endif()

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
        tests/test_tuning.cpp
        bench/adapter.cpp
    )
    # Tests reach into src/ for the low-level search and conflict detection.
    target_include_directories(mapf_tests PRIVATE src bench tests)
    target_link_libraries(mapf_tests PRIVATE mapf mapf_validator mapf_build_flags GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(mapf_tests DISCOVERY_TIMEOUT 60)
endif()
