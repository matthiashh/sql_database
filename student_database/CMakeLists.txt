add_executable(students src/students.cpp)
target_link_libraries(students postgresql_database)
target_link_libraries(students ${catkin_LIBRARIES})

install(TARGETS students RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
