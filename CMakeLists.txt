cmake_minimum_required(VERSION 3.16)
project(GpsFreeSeekingSim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(Git QUIET)
set(GFS_GIT_HEAD "unspecified")
set(GFS_CONFIGURE_DIRTY "unknown")
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GFS_GIT_HEAD
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} status --porcelain --untracked-files=no
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GFS_GIT_STATUS
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    if(GFS_GIT_STATUS STREQUAL "")
        set(GFS_CONFIGURE_DIRTY "false")
    else()
        set(GFS_CONFIGURE_DIRTY "true")
    endif()
endif()

if(CMAKE_BUILD_TYPE)
    set(GFS_BUILD_MODE_VALUE ${CMAKE_BUILD_TYPE})
else()
    set(GFS_BUILD_MODE_VALUE "$<CONFIG>")
endif()

# Legacy pre-SE(2)-gauge model (Model.hpp/Scenarios.hpp): the vehicle here
# carries a compass (world-aligned heading), which conflicts with the
# paper's premise of body-frame-only odometry and an unknown relay-to-
# odometry yaw. Not used by any paper result; excluded from the default
# build and kept only for reference. Build explicitly with
# `cmake --build . --target gps_free_seeking_sim` if needed.
add_executable(gps_free_seeking_sim EXCLUDE_FROM_ALL src/main.cpp)
add_executable(gps_free_seeking_tests EXCLUDE_FROM_ALL tests/CoreTests.cpp)

# Campaign 2027 SE(2) stack (registration + supervisor + fixed-lag smoother).
add_executable(gps_free_seeking_se2_tests tests/Se2CoreTests.cpp)
add_executable(gps_free_seeking_campaign2027_smoke src/smoke_main.cpp)
add_executable(gps_free_seeking_campaign src/campaign_main.cpp)
target_compile_definitions(gps_free_seeking_campaign PRIVATE
    GFS_GIT_HEAD="${GFS_GIT_HEAD}"
    GFS_CONFIGURE_DIRTY="${GFS_CONFIGURE_DIRTY}"
    GFS_BUILD_MODE="${GFS_BUILD_MODE_VALUE}")

enable_testing()
add_test(NAME se2_core_tests COMMAND gps_free_seeking_se2_tests)
