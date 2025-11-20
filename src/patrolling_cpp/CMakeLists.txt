cmake_minimum_required(VERSION 3.8)
project(patrolling_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Buscar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)   

# Crear ejecutable
add_executable(patrolling_node src/patrolling_node.cpp)

# Vincular dependencias
ament_target_dependencies(patrolling_node rclcpp geometry_msgs nav_msgs)

# Instalar ejecutable
install(TARGETS
  patrolling_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

