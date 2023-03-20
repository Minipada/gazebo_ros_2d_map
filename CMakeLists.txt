cmake_minimum_required(VERSION 3.8)
project(gazebo_ros_2d_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  gazebo_msgs
  gazebo_dev
  gazebo_ros
  gazebo_plugins
  pluginlib
  nav_msgs
  rclcpp
  std_srvs
  nav2_costmap_2d
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include ${GAZEBO_MSG_INCLUDE_DIRS}
)
link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(TARGETS ${PROJECT_NAME}
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(gazebo_ros_2d_map gazebo_ros_2d_map.xml)

ament_package()
