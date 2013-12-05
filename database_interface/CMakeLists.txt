# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(database_interface)

set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp #common_rosdeps 
#libpq-dev 
#yaml-cpp
)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#add_message_files(
#  FILES
  # TODO: List your msg files here
#)
#uncomment if you have defined services
#add_service_files(
#  FILES
  # TODO: List your msg files here
#)

#check for libpq's configure tool
find_program(PG_CONFIG NAMES pg_config DOC "libpq config tool")
if (NOT PG_CONFIG)
  message(FATAL_ERROR "Couldn't find pg_config. Is libpq installed?")
endif(NOT PG_CONFIG)

find_package(PkgConfig)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)
include_directories(${SQLITE3_INCLUDE_DIRS})
link_directories(${SQLITE3_LIBRARY_DIRS})

#get the libpq include and lib directories
execute_process( 
  COMMAND pg_config --includedir
  ERROR_VARIABLE IGNORE_VAR 
  OUTPUT_VARIABLE PQ_INCLUDE_DIR 
  OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process( 
  COMMAND pg_config --libdir
  ERROR_VARIABLE IGNORE_VAR 
  OUTPUT_VARIABLE PQ_LIB_DIR 
  OUTPUT_STRIP_TRAILING_WHITESPACE)

include(CheckIncludeFiles)
set(CMAKE_REQUIRED_INCLUDES ${PQ_INCLUDE_DIR})
check_include_files(libpq-fe.h HAVE_LIBPQ)
if (NOT HAVE_LIBPQ)
  message(FATAL_ERROR "Error: PostgreSQL implementation cannot find libpq-fe.h")
endif(NOT HAVE_LIBPQ)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

link_directories(${PQ_LIB_DIR})
include_directories(${PQ_INCLUDE_DIR})
add_library(postgresql_database src/postgresql_database.cpp)
target_link_libraries(postgresql_database pq)
target_link_libraries(postgresql_database yaml-cpp)

#commented - test later
#add_executable(postgresql_interface_test src/postgresql_interface_test.cpp)
#commented for later purposes - test later
#target_link_libraries(postgresql_interface_test postgresql_database)
## Generate added messages and services with any dependencies listed here
#generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
#    DEPENDS roscpp common_rosdeps libpq-dev yaml-cpp
#    CATKIN_DEPENDS # TODO
#    INCLUDE_DIRS include
#    LIBRARIES database_interface
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

