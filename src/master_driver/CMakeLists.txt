cmake_minimum_required(VERSION 2.8.3)
project(master_driver)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR};/usr/include/eigen3/Eigen/Eigen")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robot_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES davinci_planing
  CATKIN_DEPENDS roscpp rospy std_msgs 
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(omega_single src/omega_driver_single.cpp)
target_link_libraries(omega_single  ${catkin_LIBRARIES} ${Boost_LIBRARIES} "/usr/lib/libdhd.so.3" "/usr/lib/libdrd.so.3")

add_executable(omega_dual src/omega_driver_dual.cpp)
target_link_libraries(omega_dual  ${catkin_LIBRARIES} ${Boost_LIBRARIES} "/usr/lib/libdhd.so.3" "/usr/lib/libdrd.so.3")


