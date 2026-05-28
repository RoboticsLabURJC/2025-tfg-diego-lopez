cmake_minimum_required(VERSION 3.5)
project(g1_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  models
  worlds
  bridges
  launch
  g1_sim
  DESTINATION share/${PROJECT_NAME}
)

ament_package()