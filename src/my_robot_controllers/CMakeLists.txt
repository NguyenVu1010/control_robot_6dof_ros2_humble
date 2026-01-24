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
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)

# --- 1. TÌM THƯ VIỆN MỚI ---
find_package(simple_kinematics_lib REQUIRED)
find_package(simple_trajectory_lib REQUIRED)

# --- 2. TẠO THƯ VIỆN CONTROLLER ---
add_library(cartesian_velocity_controller SHARED
  src/cartesian_velocity_controller.cpp
  src/shm/shm_manager.cpp  # Chỉ giữ lại source SHM nội bộ
)

target_include_directories(cartesian_velocity_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# Link thư viện Realtime (cho shm_open)
target_link_libraries(cartesian_velocity_controller rt)

# --- 3. LIÊN KẾT DEPENDENCY ---
ament_target_dependencies(cartesian_velocity_controller
  rclcpp
  pluginlib
  controller_interface
  hardware_interface
  geometry_msgs
  realtime_tools
  Eigen3
  simple_kinematics_lib  # <--- Link Lib Kinematics
  simple_trajectory_lib  # <--- Link Lib Trajectory
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

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(cartesian_velocity_controller)
ament_export_dependencies(
  rclcpp controller_interface hardware_interface realtime_tools 
  simple_kinematics_lib simple_trajectory_lib
)

ament_package()