cmake_minimum_required(VERSION 3.0.2)
project(omnirobot)

# set CMAKE_PROJECT_VERSION
file(READ "${PROJECT_SOURCE_DIR}/package.xml" package_xml)
string(REGEX MATCH "<version>([0-9]+\\.[0-9]+\\.[0-9]+)</version>" _ ${package_xml})
set(CMAKE_PROJECT_VERSION ${CMAKE_MATCH_1})
set(CMAKE_PROJECT_VERSION_MAJOR ${CMAKE_MATCH_1})
set(CMAKE_PROJECT_VERSION_MINOR ${CMAKE_MATCH_2})
set(CMAKE_PROJECT_VERSION_PATCH ${CMAKE_MATCH_3})

# set gazebo_DIR to the installation prefix of gazebo
set(gazebo_DIR "/usr/lib/x86_64-linux-gnu/cmake/gazebo")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo
  rospy
  rviz
  urdf
)


catkin_package(
)

catkin_install_python(
  PROGRAMS
    scripts/teleop_omni.py
    scripts/teleop_twist_keyboard.py
    scripts/vel_Publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

