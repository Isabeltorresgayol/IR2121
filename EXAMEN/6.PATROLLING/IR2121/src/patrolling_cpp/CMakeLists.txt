cmake_minimum_required(VERSION 3.8)
project(patrolling_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================= DEPENDENCIAS =================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)        # NUEVO (actions)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)            # NUEVO (NavigateToPose)

# TF2 (usado en V5)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ================= EJECUTABLE =================
add_executable(patrolling_node src/patrolling_node.cpp)

# ================= VINCULAR DEPENDENCIAS =================
ament_target_dependencies(patrolling_node
  rclcpp
  rclcpp_action        # NUEVO
  geometry_msgs
  nav_msgs
  nav2_msgs            # NUEVO
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ================= INSTALACIÓN =================
install(TARGETS
  patrolling_node
  DESTINATION lib/${PROJECT_NAME}
)

# ================= TEST =================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

