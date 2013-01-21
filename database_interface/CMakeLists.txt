#check for libpq's configure tool
find_program(PG_CONFIG NAMES pg_config DOC "libpq config tool")
if (NOT PG_CONFIG)
  message(FATAL_ERROR "Couldn't find pg_config. Is libpq installed?")
endif(NOT PG_CONFIG)

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

link_directories(${PQ_LIB_DIR})
include_directories(${PQ_INCLUDE_DIR})
add_library(postgresql_database src/postgresql_database.cpp)
target_link_libraries(postgresql_database pq)
target_link_libraries(postgresql_database yaml-cpp)
target_link_libraries(postgresql_database ${catkin_LIBRARIES})

add_executable(postgresql_interface_test src/postgresql_interface_test.cpp)
target_link_libraries(postgresql_interface_test postgresql_database)
target_link_libraries(postgresql_interface_test ${catkin_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS postgresql_database LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS postgresql_interface_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

