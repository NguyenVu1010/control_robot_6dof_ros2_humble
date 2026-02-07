cmake_minimum_required(VERSION 3.8)
project(simple_trajectory_lib)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
add_library(${PROJECT_NAME} SHARED
  src/trajectory_generator.cpp
  src/theta_star_3d.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  /opt/ros/humble/include/ompl-1.7
)
# 5. LIÊN KẾT THƯ VIỆN OMPL THỦ CÔNG
target_link_libraries(${PROJECT_NAME}
  ${OMPL_LIBRARIES}
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(Eigen3)

ament_export_dependencies(ompl)
ament_export_include_directories(include /opt/ros/humble/include/ompl-1.7)
ament_export_libraries(simple_trajectory_lib)

ament_package()