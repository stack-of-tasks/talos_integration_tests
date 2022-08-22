# Copyright (C) 2017-2018 LAAS-CNRS
#
# Author: Olivier Stasse
#
# See LICENSE
cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME talos_integration_tests)
set(PROJECT_DESCRIPTION "Talos integration tests")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

set(CXX_DISABLE_WERROR True)

# Project configuration
set(CATKIN_ENABLE_TESTING ON)

set(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} -Werror=format-security")

include(cmake/base.cmake)
include(cmake/ros.cmake)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

find_package(PkgConfig REQUIRED)
find_package(eigenpy 2.7.10 REQUIRED)

# Project dependencies
set(CATKIN_REQUIRED_COMPONENTS rospy talos_gazebo roscontrol_sot_talos)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_REQUIRED_COMPONENTS})

include_directories(include ${bullet_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${bullet_LIBRARY_DIRS})

catkin_package(CATKIN_DEPENDS rospy)

add_project_dependency(dynamic_graph_bridge REQUIRED)

# Installing python based scripts
catkin_install_python(
  PROGRAMS
  scripts/start_talos_gazebo_kine.py
  scripts/start_sot_talos_balance.py
  scripts/start_sot_online_walking.py
  scripts/test_kine.py
  scripts/test_sot_talos_balance.py
  scripts/test_online_walking.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir launch tests)
  install(DIRECTORY ${dir} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()

add_subdirectory(src)

# Testing part
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/test_kine.test)
  add_rostest(tests/test_sot_talos_balance.test)
  add_rostest(tests/test_online_walking.test)
endif()
