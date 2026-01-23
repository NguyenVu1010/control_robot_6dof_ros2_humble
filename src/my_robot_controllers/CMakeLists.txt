cmake_minimum_required(VERSION 3.8)
project(my_robot_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
# CHỈ DÙNG VENDOR
find_package(orocos_kdl_vendor REQUIRED) 
find_package(Eigen3 REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(cartesian_velocity_controller SHARED
  src/cartesian_velocity_controller.cpp
  src/algo/fk_solver.cpp
  src/algo/jacobian_solver.cpp
  src/algo/math_utils.cpp
  src/algo/robot_kinematics.cpp
  src/algo/path_generator.cpp
  src/algo/trajectory_solver.cpp
  src/shm/shm_manager.cpp
)

target_include_directories(cartesian_velocity_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# QUAN TRỌNG: Không dùng target_link_libraries thủ công
# XÓA BỎ DÒNG: target_link_libraries(... orocos-kdl) NẾU CÓ

ament_target_dependencies(cartesian_velocity_controller
  rclcpp
  pluginlib
  controller_interface
  hardware_interface
  geometry_msgs
  kdl_parser
  orocos_kdl_vendor  # Vendor sẽ tự động link đúng thư viện
  realtime_tools
  Eigen3
)

pluginlib_export_plugin_description_file(controller_interface my_robot_controllers_plugins.xml)

install(TARGETS cartesian_velocity_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES my_robot_controllers_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(cartesian_velocity_controller)
ament_export_dependencies(
  rclcpp
  controller_interface
  hardware_interface
  kdl_parser
  orocos_kdl_vendor
  realtime_tools
  Eigen3
)

ament_package()