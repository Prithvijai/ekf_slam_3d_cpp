cmake_minimum_required(VERSION 3.8)
project(ekf_slam_3d_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)


add_executable(ekf_slam_node
   src/main.cpp
   src/ekf_slam_node.cpp   
)
target_link_libraries(ekf_slam_node Eigen3::Eigen)
ament_target_dependencies(ekf_slam_node rclcpp std_msgs sensor_msgs nav_msgs Eigen3)


install(TARGETS 
    ekf_slam_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
