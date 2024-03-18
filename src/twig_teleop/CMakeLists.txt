cmake_minimum_required(VERSION 3.22)
project(twig_teleop)

# Component Nodes (Shared libraries) ############################
set(JOYSTICK_TELEOP_NODE_NAME twig_teleop)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  geometry_msgs
  pluginlib
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#####################
## Component Nodes ##
#####################

# Add executable for using a controller
add_library(${JOYSTICK_TELEOP_NODE_NAME} SHARED
  src/joystick_teleop.cpp
  src/joystick_direct_teleop.cpp
)

ament_target_dependencies(${JOYSTICK_TELEOP_NODE_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

rclcpp_components_register_nodes(${JOYSTICK_TELEOP_NODE_NAME} "twig_teleop::JoystickTeleop")
rclcpp_components_register_nodes(${JOYSTICK_TELEOP_NODE_NAME} "twig_teleop::JoystickDirectTeleop")

#############
## Install ##
#############

# Install Libraries
install(
  TARGETS
    ${JOYSTICK_TELEOP_NODE_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include, launch, config directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()