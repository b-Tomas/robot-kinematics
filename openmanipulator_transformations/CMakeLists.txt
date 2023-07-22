cmake_minimum_required(VERSION 3.0.2)
project(openmanipulator_transformations)

## Find catkin macros and libraries, and other packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  open_manipulator_msgs
)

## Run python setup.py
catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Transform.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Mark executable scripts for installation
install(PROGRAMS
  nodes/inverse_kinematics_srv.py
  nodes/forward_kinematics_srv.py
  scripts/client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
