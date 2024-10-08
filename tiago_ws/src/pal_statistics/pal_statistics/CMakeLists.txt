cmake_minimum_required(VERSION 2.8.3)
project(pal_statistics)

find_package(catkin REQUIRED COMPONENTS
  pal_statistics_msgs
  roscpp
  rospy
  rosbag
)
catkin_python_setup()
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} extract_rosbag_signals
  CATKIN_DEPENDS pal_statistics_msgs roscpp
#  DEPENDS Boost
)

include_directories( SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

include_directories(include)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/pal_statistics_utils.cpp
  src/pal_statistics.cpp
  src/pal_statistics_macros.cpp
)

add_library(extract_rosbag_signals
  src/extract_rosbag_signals.cpp
)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )

## Specify libraries to link a library or executable target against
 target_link_libraries(extract_rosbag_signals
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/pal_statistics_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} extract_rosbag_signals
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    catkin_add_gtest(buffer-test test/gtest_buffer.cpp)
    target_link_libraries(buffer-test
                                ${PROJECT_NAME}
                                ${catkin_LIBRARIES})

    add_rostest_gmock(pal_statistics-test test/pal_statistics.test test/gtest_pal_statistics.cpp)
    target_link_libraries(pal_statistics-test
                                ${PROJECT_NAME}
                                ${catkin_LIBRARIES})

    add_rostest(test/pal_statistics_py.test)

    add_rostest_gtest(extract_rosbag_signals-test test/extract_rosbag_signals_test.test test/extract_rosbag_signals_test.cpp)
    target_link_libraries(extract_rosbag_signals-test
                                extract_rosbag_signals
                                ${PROJECT_NAME}
                                ${catkin_LIBRARIES})
endif(CATKIN_ENABLE_TESTING)
## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pal_statistics.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
