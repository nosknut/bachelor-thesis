cmake_minimum_required(VERSION 3.8)
project(twig)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})