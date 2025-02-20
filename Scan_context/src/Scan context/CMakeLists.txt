cmake_minimum_required(VERSION 2.8.3)
project(scan_context)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(catkin REQUIRED COMPONENTS  
  pcl_ros
  roscpp
  roslib
  rospy  
  std_msgs
  cv_bridge  
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}  
  ${OpenCV_INCLUDE_DIRS}
  {Boost_INCLUDE_DIR}
)
    
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS pcl_ros roscpp roslib rospy std_msgs 
    DEPENDS EIGEN3 PCL
)

add_executable(scan_context
      src/Scancontext.cpp       
      demo.cpp
      )

target_link_libraries(scan_context ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} Eigen3::Eigen)


