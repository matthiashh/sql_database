cmake_minimum_required(VERSION 2.8.3)
project(sql_database)

find_package(catkin REQUIRED roscpp)

catkin_package(
  LIBRARIES postgresql_database
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS database_interface/include
)

include_directories(database_interface/include)

add_subdirectory(database_interface)
add_subdirectory(student_database)
