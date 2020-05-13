cmake_minimum_required(VERSION 2.8.3)
project(franka_teleopreation)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR};/usr/include/eigen3/Eigen/Eigen")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robot_msgs
  trac_ik_lib
)

find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    trac_ik_lib
  DEPENDS
    Boost
    orocos_kdl
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)


# davinci_kdl 
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(robot_inverse_single src/robot_inverse_single.cpp)
target_link_libraries(robot_inverse_single ${catkin_LIBRARIES} 
					   ${Boost_LIBRARIES}
				           ${orocos_kdl_LIBRARIES})

#add_executable(robot_inverse_dual src/robot_inverse_dual.cpp)
#target_link_libraries(robot_inverse_dual ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(TARGETS robot_inverse_single
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})




