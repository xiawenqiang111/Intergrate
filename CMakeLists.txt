cmake_minimum_required(VERSION 2.8.3)
project(Fusion)


set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
set( CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
find_package(GTSAM REQUIRED)
include_directories(
	include
  ${catkin_INCLUDE_DIRS} 
	${Boost_INCLUDE_DIR}
	${GTSAM_INCLUDE_DIR})


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
)

include_directories( "/usr/include/eigen3" )
include_directories("/usr/local/include/SolveAXXB")

add_executable(Fusion Fusion.cpp)
target_link_libraries(Fusion  ${catkin_LIBRARIES} )

add_executable(Fusion_gtasm Fusion_gtasm.cpp)
target_link_libraries(Fusion_gtasm ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${Boost_LIBRARIES} -lgtsam -ltbb)

add_executable(Test Test.cpp)
target_link_libraries(Test  ${catkin_LIBRARIES} /usr/local/lib/libSolveAXXB.so -lglog)
