cmake_minimum_required(VERSION 3.8)
project(robot_gui)

find_package(ament_cmake REQUIRED)

# Cài đặt các file Python chính vào thư mục share của package
install(FILES
  main.py
  shm_manager.py
  sequence_manager.py
  config.py
  DESTINATION share/${PROJECT_NAME}
)

# Cài đặt nguyên thư mục tabs
install(DIRECTORY tabs/
  DESTINATION share/${PROJECT_NAME}/tabs
)

ament_package()