# Copyright (C) 2017-2018 LAAS-CNRS
#
# Author: Olivier Stasse
#
# See LICENSE
cmake_minimum_required(VERSION 3.1)

# Project properties
SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME talos_integration_tests)
SET(PROJECT_DESCRIPTION "Talos integration tests")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

set(CXX_DISABLE_WERROR True)

# Project configuration
SET(CATKIN_ENABLE_TESTING ON)

set(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} -Werror=format-security")

include(cmake/base.cmake)
include(cmake/ros.cmake)
include(cmake/python.cmake)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

find_package(PkgConfig REQUIRED)

# Project dependencies
FINDPYTHON()

SET(CATKIN_REQUIRED_COMPONENTS
  rospy
  talos_gazebo
  roscontrol_sot_talos
  dynamic_graph_bridge_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_REQUIRED_COMPONENTS}
)

include_directories(include ${bullet_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${bullet_LIBRARY_DIRS})


catkin_package(CATKIN_DEPENDS
  rospy
)

add_project_dependency(dynamic_graph_bridge REQUIRED)


# Installing python based scripts
catkin_install_python(PROGRAMS
  scripts/start_talos_gazebo_kine.py
  scripts/start_sot_talos_balance.py
  scripts/start_sot_online_walking.py
  scripts/start_sot_ouster_walking.py
  scripts/start_torque_control.py
  scripts/test_kine.py
  scripts/test_sot_talos_balance.py
  scripts/test_online_walking.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir launch tests)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()

ADD_SUBDIRECTORY(src)

# Testing part
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/test_kine.test)
  add_rostest(tests/test_sot_talos_balance.test)
  add_rostest(tests/test_online_walking.test)
  add_rostest(tests/test_ouster_walking.test)
  add_rostest(tests/test_torque_control_on_spot.test)  #add_rostest(tests/test_torque_control.test ARGS arg1:=true arg2:=false)
  add_rostest(tests/test_torque_control_walk_20.test)
  add_rostest(tests/test_torque_control_on_spot_pg.test)
  add_rostest(tests/test_torque_control_walk_20_pg.test)


endif()
