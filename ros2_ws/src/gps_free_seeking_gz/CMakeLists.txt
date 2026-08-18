cmake_minimum_required(VERSION 3.16)
project(gps_free_seeking_gz LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gps_free_seeking_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Header-only estimator core shared with the offline campaign.
set(GFS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
execute_process(
  COMMAND git -C "${GFS_ROOT}" rev-parse HEAD
  OUTPUT_VARIABLE GFS_BUILD_COMMIT
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE GFS_GIT_RESULT)
if(NOT GFS_GIT_RESULT EQUAL 0)
  set(GFS_BUILD_COMMIT "0000000000000000000000000000000000000000")
endif()
execute_process(
  COMMAND git -C "${GFS_ROOT}" diff --quiet HEAD --
          CMakeLists.txt include src config scripts ros2_ws/src
  RESULT_VARIABLE GFS_DIFF_RESULT)
execute_process(
  COMMAND git -C "${GFS_ROOT}" ls-files --others --exclude-standard --
          CMakeLists.txt include src config scripts ros2_ws/src
  OUTPUT_VARIABLE GFS_UNTRACKED_RELEVANT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT GFS_DIFF_RESULT EQUAL 0 OR NOT GFS_UNTRACKED_RELEVANT STREQUAL "")
  set(GFS_BUILD_DIRTY 1)
  set(GFS_BUILD_DIRTY_BOOL true)
else()
  set(GFS_BUILD_DIRTY 0)
  set(GFS_BUILD_DIRTY_BOOL false)
endif()

set(GFS_GENERATED_INCLUDE "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${GFS_GENERATED_INCLUDE}/gps_free_seeking_gz")
configure_file(
  include/gps_free_seeking_gz/build_info.hpp.in
  "${GFS_GENERATED_INCLUDE}/gps_free_seeking_gz/build_info.hpp" @ONLY)
configure_file(
  config/build_provenance.env.in
  "${CMAKE_CURRENT_BINARY_DIR}/build_provenance.env" @ONLY)
include_directories("${GFS_ROOT}/include" "${GFS_GENERATED_INCLUDE}")

add_executable(relay_emulator_node src/relay_emulator_node.cpp)
ament_target_dependencies(relay_emulator_node
  rclcpp nav_msgs gps_free_seeking_msgs)

add_executable(gfs_seeker_node src/gfs_seeker_node.cpp)
ament_target_dependencies(gfs_seeker_node
  rclcpp nav_msgs geometry_msgs std_msgs gps_free_seeking_msgs)

add_executable(gz_evaluator_node src/gz_evaluator_node.cpp)
ament_target_dependencies(gz_evaluator_node
  rclcpp nav_msgs std_msgs gps_free_seeking_msgs)

install(TARGETS relay_emulator_node gfs_seeker_node gz_evaluator_node
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch worlds config
        DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE
        PATTERN "*.pyc" EXCLUDE
        PATTERN "*.in" EXCLUDE)
install(PROGRAMS scripts/run_gz_batch.sh scripts/validate_gz_run.py
        DESTINATION share/${PROJECT_NAME}/scripts)
install(FILES scripts/gz_result_contract.py
        DESTINATION share/${PROJECT_NAME}/scripts)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/build_provenance.env"
        DESTINATION share/${PROJECT_NAME}/config)
install(FILES README.md VALIDATION_CHECKLIST.md
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(gz_result_contract_test
                        test/test_gz_result_contract.py TIMEOUT 30)
  ament_add_pytest_test(gz_launch_smoke_test
                        test/test_gz_launch_smoke.py TIMEOUT 45)
  ament_add_pytest_test(gz_aggregation_test
                        test/test_gz_aggregation.py TIMEOUT 30)
endif()
ament_package()
