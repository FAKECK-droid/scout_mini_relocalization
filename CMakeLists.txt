cmake_minimum_required(VERSION 3.16)
project(scout_mini_relocalization LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(small_gicp 1.0.1 CONFIG REQUIRED)
find_package(Threads REQUIRED)

add_library(relocalization_geometry INTERFACE)
target_compile_features(relocalization_geometry INTERFACE cxx_std_17)
target_include_directories(relocalization_geometry INTERFACE include)
target_link_libraries(relocalization_geometry INTERFACE Eigen3::Eigen small_gicp::small_gicp)

add_executable(small_gicp_relocalizer src/relocalizer.cpp)
target_compile_options(small_gicp_relocalizer PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(small_gicp_relocalizer PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(small_gicp_relocalizer relocalization_geometry ${PCL_LIBRARIES} Threads::Threads)
ament_target_dependencies(small_gicp_relocalizer
  rclcpp sensor_msgs geometry_msgs diagnostic_msgs tf2_ros pcl_conversions)
install(TARGETS small_gicp_relocalizer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_relocalization_geometry test/test_relocalization_geometry.cpp)
  target_link_libraries(test_relocalization_geometry PRIVATE relocalization_geometry)
  add_test(NAME relocalization_geometry COMMAND test_relocalization_geometry)
  set_tests_properties(relocalization_geometry PROPERTIES TIMEOUT 30)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME relocalization_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_relocalization_integration.py
      --binary $<TARGET_FILE:small_gicp_relocalizer>)
  set_tests_properties(relocalization_integration PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE)
  add_test(NAME relocalization_ekf_isolation
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_odometry_isolation.py
      --binary $<TARGET_FILE:small_gicp_relocalizer>)
  set_tests_properties(relocalization_ekf_isolation PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE)
endif()
ament_package()
