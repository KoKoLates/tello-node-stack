cmake_minimum_required(VERSION 2.8.3)

project(image_undistort)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  pcl_ros
  sensor_msgs
  cv_bridge
  image_transport
  tf_conversions
  nodelet
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_nodelet
)

find_package(OpenCV REQUIRED COMPONENTS core calib3d)

find_package(NLOPT)
IF(${NLOPT_FOUND})
  include_directories(
    ${NLOPT_INCLUDE_DIRS}
  )

  add_library(${PROJECT_NAME}_nodelet 
  src/nodelets/image_undistort_nodelet.cpp 
  src/camera_parameters.cpp 
  src/image_undistort.cpp 
  src/undistorter.cpp)

ELSE()

  add_library(${PROJECT_NAME}_nodelet 
  src/nodelets/image_undistort_nodelet.cpp 
  src/camera_parameters.cpp 
  src/image_undistort.cpp 
  src/undistorter.cpp)

  MESSAGE(WARNING "nlopt not found, building without point to bearing nodelet")
ENDIF()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_nodelet ${catkin_EXPORTED_TARGETS})

add_executable(image_undistort_node src/nodes/image_undistort_node.cpp)
target_link_libraries(image_undistort_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

# Install nodelet library
install(TARGETS ${PROJECT_NAME}_nodelet
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install xml files
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

