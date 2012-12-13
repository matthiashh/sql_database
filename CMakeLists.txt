cmake_minimum_required(VERSION 2.8.3)
project(sql_database)

find_package(catkin REQUIRED roscpp rosconsole)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
  roscpp
)

include_directories(database_interface/include)

add_subdirectory(database_interface)
add_subdirectory(student_database)