cmake_minimum_required(VERSION 3.16)
project(project LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wno-unused-variable)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_srvs
  PCL
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2
  tf2_geometry_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
# eigen
find_package(Eigen3 REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(pointcloud_accumulator src/pointcloud_accumulator.cpp)

include_directories(pointcloud_accumulator ${PCL_INCLUDE_DIRS})
link_directories(pointcloud_accumulator ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ament_target_dependencies(pointcloud_accumulator pcl_ros pcl_conversions rclcpp sensor_msgs std_srvs tf2_ros)

target_link_libraries(pointcloud_accumulator ${PCL_LIBRARIES}) 


################ Voronoi calculator ##############################

add_executable(voronoi_calculator src/voronoi_calculator.cpp)

find_file(libvoro_include NAMES "voro++/")
# message(STATUS ${libvoro_include})
include_directories(${libvoro_include})

find_library(libvoro++_loc NAMES libvoro++.a)
# message(STATUS ${libvoro++_loc})

add_library(libvoro++ STATIC IMPORTED)
set_target_properties(libvoro++ PROPERTIES IMPORTED_LOCATION ${libvoro++_loc})

target_link_libraries(voronoi_calculator libvoro++)
target_include_directories(voronoi_calculator PUBLIC include/)
ament_target_dependencies(voronoi_calculator rclcpp geometry_msgs tf2_ros)

################ Function integrator ##############################

add_executable(integrator src/integration.cpp)
ament_target_dependencies(integrator rclcpp geometry_msgs tf2_ros tf2 tf2_geometry_msgs)

################ Test Integrator ###########################################
add_executable(integrator_test src/integrate_test.cpp)
target_include_directories(integrator_test PUBLIC include/)
target_link_libraries(integrator_test Eigen3::Eigen libvoro++)


INSTALL (TARGETS pointcloud_accumulator voronoi_calculator integrator integrator_test
  DESTINATION lib/${PROJECT_NAME}
)
# python executable
install(PROGRAMS
  scripts/vis.py
  DESTINATION lib/${PROJECT_NAME}
)

INSTALL (DIRECTORY launch rviz_config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()