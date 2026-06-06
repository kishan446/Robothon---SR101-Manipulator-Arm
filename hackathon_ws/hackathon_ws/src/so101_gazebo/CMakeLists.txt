cmake_minimum_required(VERSION 3.8)
project(so101_gazebo)

find_package(ament_cmake REQUIRED)

# 1. Install Python scripts as executables (for ros2 run)
install(PROGRAMS
  scripts/object_detector.py
  scripts/palletizing_fetch.py
  scripts/pick_place_test.py
  scripts/so101_gazebo_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# 2. Install the scripts folder to the share directory (for ros2 launch)
# We map the 'scripts' folder to 'launch' in the install space so ROS finds it easily
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 3. Install other resource directories
install(DIRECTORY
  launch
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()