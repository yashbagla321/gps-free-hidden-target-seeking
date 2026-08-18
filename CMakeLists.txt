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

add_executable(gps_free_seeking_sim src/main.cpp)
add_executable(gps_free_seeking_tests tests/CoreTests.cpp)

# Campaign 2027 SE(2) stack (registration + supervisor + fixed-lag smoother).
add_executable(gps_free_seeking_se2_tests tests/Se2CoreTests.cpp)
add_executable(gps_free_seeking_campaign2027_smoke src/smoke_main.cpp)
add_executable(gps_free_seeking_campaign src/campaign_main.cpp)
target_compile_definitions(gps_free_seeking_campaign PRIVATE
    GFS_GIT_HEAD="${GFS_GIT_HEAD}"
    GFS_CONFIGURE_DIRTY="${GFS_CONFIGURE_DIRTY}"
    GFS_BUILD_MODE="${GFS_BUILD_MODE_VALUE}")

enable_testing()
add_test(NAME core_tests COMMAND gps_free_seeking_tests)
add_test(NAME se2_core_tests COMMAND gps_free_seeking_se2_tests)
