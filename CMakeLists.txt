cmake_minimum_required(VERSION 3.8)
project(mppi_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)


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

add_executable(offboard_node
  src/offboard_node.cpp
  src/anytime_scheduler.cpp
  src/mppi_controller.cpp
  src/drone_model.cpp
)
target_include_directories(offboard_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(offboard_node
  rclcpp
  geometry_msgs
  mavros_msgs
)

add_executable(test_drone_model
  src/test_drone_model.cpp
  src/drone_model.cpp
)
target_include_directories(test_drone_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(test_mpc_controller
  src/test_mpc_controller.cpp
  src/mpc_controller.cpp
  src/drone_model.cpp
)
target_include_directories(test_mpc_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(test_mppi_controller
  src/test_mppi_controller.cpp
  src/mppi_controller.cpp
  src/drone_model.cpp
)
target_include_directories(test_mppi_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(test_anytime_mppi
  src/test_anytime_mppi.cpp
  src/mppi_controller.cpp
  src/anytime_scheduler.cpp
  src/drone_model.cpp
)
target_include_directories(test_anytime_mppi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(cpu_load_generator
  src/cpu_load_generator.cpp
)
target_link_libraries(cpu_load_generator Threads::Threads)

add_executable(test_n_quality_sweep
  src/test_n_quality_sweep.cpp
  src/mppi_controller.cpp
  src/drone_model.cpp
)
target_include_directories(test_n_quality_sweep PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(analyze_equivalent_budget
  src/analyze_equivalent_budget.cpp
)
install(TARGETS
  offboard_node
  test_drone_model
  test_mpc_controller
  test_mppi_controller
  test_anytime_mppi
  cpu_load_generator
  test_n_quality_sweep
  analyze_equivalent_budget
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
