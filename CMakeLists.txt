cmake_minimum_required(VERSION 3.16)
project(project LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  PCL
  pcl_conversions
  pcl_ros
  tf2_ros
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(pointcloud_accumulator src/pointcloud_accumulator.cpp)

include_directories(pointcloud_accumulator ${PCL_INCLUDE_DIRS})
link_directories(pointcloud_accumulator ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ament_target_dependencies(pointcloud_accumulator pcl_ros pcl_conversions rclcpp sensor_msgs tf2_ros)

target_link_libraries(pointcloud_accumulator ${PCL_LIBRARIES}) 

INSTALL (TARGETS pointcloud_accumulator
  DESTINATION lib/${PROJECT_NAME}
)
INSTALL (DIRECTORY launch rviz_config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()