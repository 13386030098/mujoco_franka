cmake_minimum_required(VERSION 2.8.3)
project(franka_track_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED
  COMPONENTS
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
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(trac_ik src/trac_ik.cpp)
target_link_libraries(trac_ik
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

install(TARGETS trac_ik
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
