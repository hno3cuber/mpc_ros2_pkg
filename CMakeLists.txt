cmake_minimum_required(VERSION 3.8)
project(mpc_ros2_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(include)

# test_node
add_executable (mpc_test_node
  src/GetRefPath.cpp
  src/mpc_test_node.cpp
  src/MpcCal.cpp
)
ament_target_dependencies (mpc_test_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  Eigen3
  px4_msgs
)
target_link_libraries (mpc_test_node
  Eigen3::Eigen
  osqp::osqp
  OsqpEigen::OsqpEigen
)
install (TARGETS
  mpc_test_node
  DESTINATION lib/${PROJECT_NAME}
)

# leader_node
add_executable (leader_node
  src/GetRefPath.cpp
  src/leader_node.cpp
  src/MpcCal.cpp
)
ament_target_dependencies (leader_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  Eigen3
  px4_msgs
)
target_link_libraries (leader_node
  Eigen3::Eigen
  osqp::osqp
  OsqpEigen::OsqpEigen
)
install (TARGETS
  leader_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES src/ref.csv
  DESTINATION share/${PROJECT_NAME}/config
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()