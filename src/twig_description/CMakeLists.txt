cmake_minimum_required(VERSION 3.8)
project(twig_description)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})