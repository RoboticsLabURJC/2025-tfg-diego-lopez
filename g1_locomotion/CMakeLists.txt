cmake_minimum_required(VERSION 3.8)
project(g1_locomotion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  tf2
  ament_index_cpp
)
include_directories(include)

add_library(${PROJECT_NAME} SHARED src/g1_locomotion/Inference.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

include_directories(/usr/local/include)

target_link_libraries(${PROJECT_NAME}
  /usr/local/lib/libonnxruntime.so
)

add_executable(g1_locomotion_main src/g1_locomotion_main.cpp)
ament_target_dependencies(g1_locomotion_main ${dependencies})
target_link_libraries(g1_locomotion_main ${PROJECT_NAME})

install(TARGETS
    ${PROJECT_NAME}
    g1_locomotion_main
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY 
  policies
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
