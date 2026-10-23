cmake_minimum_required(VERSION 3.8)
project(g1_locomotion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
  set(policy_yaml_target yaml-cpp::yaml-cpp)
else()
  set(policy_yaml_target yaml-cpp)
endif()

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

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  "msg/PolicyList.msg"
  LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(policy_typesupport_target
  ${PROJECT_NAME}_interfaces
  "rosidl_typesupport_cpp"
)

add_library(${PROJECT_NAME} SHARED 
  src/g1_locomotion/Inference.cpp
  src/g1_locomotion/Policy_manager.cpp
  src/g1_locomotion/Random_movement.cpp
  src/g1_locomotion/Navigation.cpp
  src/g1_locomotion/Move_box.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})



include_directories(/usr/local/include)

target_link_libraries(${PROJECT_NAME}
  /usr/local/lib/libonnxruntime.so
  "${policy_typesupport_target}"
  ${policy_yaml_target}
)

add_executable(g1_locomotion_main src/g1_locomotion_main.cpp)
ament_target_dependencies(g1_locomotion_main ${dependencies})
target_link_libraries(g1_locomotion_main ${PROJECT_NAME})

add_executable(random_movement_main src/random_movement_main.cpp)
ament_target_dependencies(random_movement_main ${dependencies})
target_link_libraries(random_movement_main ${PROJECT_NAME})

add_executable(navigation_main src/navigation_main.cpp)
ament_target_dependencies(navigation_main ${dependencies})
target_link_libraries(navigation_main ${PROJECT_NAME})

add_executable(move_box_main src/move_box_main.cpp)
ament_target_dependencies(move_box_main ${dependencies})
target_link_libraries(move_box_main ${PROJECT_NAME})

add_executable(policy_manager_main src/policy_manager_main.cpp)
ament_target_dependencies(policy_manager_main ${dependencies})
target_link_libraries(policy_manager_main ${PROJECT_NAME})

install(TARGETS
    ${PROJECT_NAME}
    g1_locomotion_main
    random_movement_main
    navigation_main
    move_box_main
    policy_manager_main
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
  config
  DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_export_dependencies(rosidl_default_runtime)

ament_package()
