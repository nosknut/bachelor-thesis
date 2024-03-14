# https://github.com/ros-controls/ros2_control_demos/blob/humble/example_1/CMakeLists.txt

cmake_minimum_required(VERSION 3.22)
project(twig_hardware)

# Component Nodes (Shared libraries) ############################
set(TWIG_HARDWARE_NODE_NAME twig_hardware)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  geometry_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  rclcpp_components
  std_msgs
  std_srvs
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  ${TWIG_HARDWARE_NODE_NAME}
  SHARED
  src/twig_lib.cpp
  src/twig_hardware.cpp
  src/twig_hardware_node.cpp
)

target_compile_features(${TWIG_HARDWARE_NODE_NAME} PUBLIC cxx_std_17)
target_include_directories(${TWIG_HARDWARE_NODE_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${TWIG_HARDWARE_NODE_NAME}>
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "TWIG_HARDWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface twig_hardware.xml)

ament_target_dependencies(${TWIG_HARDWARE_NODE_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

#####################
## Component Nodes ##
#####################

rclcpp_components_register_nodes(${TWIG_HARDWARE_NODE_NAME} "twig_hardware::TwigHardwareNode")

#############
## Install ##
#############

# Install Libraries
install(
  TARGETS
    ${TWIG_HARDWARE_NODE_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include, launch, config directories
install(DIRECTORY include DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()