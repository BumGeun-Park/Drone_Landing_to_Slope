cmake_minimum_required(VERSION 3.0.2)
project(rplidar_data)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

find_package(cmake_modules REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
 add_message_files(
   FILES
   polar.msg
   xyz.msg
   phi.msg
   alpha.msg
   map.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES rplidar_data
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS Eigen

#  DEPENDS system_lib
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

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rplidar_data.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(Raw_data2Information src/Raw_data2Information.cpp)
 add_executable(assemble src/assemble.cpp)
 add_executable(servo_motor src/servo_motor.cpp)

 add_executable(find_min1 src/find_min1.cpp)
 add_executable(find_min2 src/find_min2.cpp)
 add_executable(find_min3 src/find_min3.cpp)
 add_executable(find_min4 src/find_min4.cpp)

 add_executable(alpha src/alpha.cpp)
 
 add_executable(local_map src/local_map.cpp)

 add_executable(avg_z src/avg_z.cpp)

 add_executable(Leg_servo1 src/Leg_servo1.cpp)
 add_executable(Leg_servo2 src/Leg_servo2.cpp)
 add_executable(Leg_servo3 src/Leg_servo3.cpp)
 add_executable(Leg_servo4 src/Leg_servo4.cpp)

 add_executable(z_dist1 src/z_dist1.cpp)
 add_executable(z_dist2 src/z_dist2.cpp)
 add_executable(z_dist3 src/z_dist3.cpp)
 add_executable(z_dist4 src/z_dist4.cpp)

 add_executable(feedback1 src/feedback1.cpp)
 add_executable(feedback2 src/feedback2.cpp)
 add_executable(feedback3 src/feedback3.cpp)
 add_executable(feedback4 src/feedback4.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(Raw_data2Information ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(assemble ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(servo_motor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 add_dependencies(find_min1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(find_min2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(find_min3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(find_min4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 add_dependencies(alpha ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 add_dependencies(local_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 add_dependencies(avg_z ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 add_dependencies(Leg_servo1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(Leg_servo2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(Leg_servo3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(Leg_servo4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 add_dependencies(z_dist1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(z_dist2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(z_dist3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(z_dist4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 add_dependencies(feedback1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(feedback2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(feedback3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(feedback4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
 target_link_libraries(Raw_data2Information
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )
target_link_libraries(assemble
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )
target_link_libraries(servo_motor
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )
################################################

target_link_libraries(find_min1
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )
target_link_libraries(find_min2
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )
target_link_libraries(find_min3
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )
target_link_libraries(find_min4
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )

################################################
target_link_libraries(alpha
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )

################################################
target_link_libraries(local_map
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )

################################################
target_link_libraries(avg_z
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )

################################################
target_link_libraries(Leg_servo1
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )
target_link_libraries(Leg_servo2
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )
target_link_libraries(Leg_servo3
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )
target_link_libraries(Leg_servo4
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )

#################################################
target_link_libraries(z_dist1
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )
target_link_libraries(z_dist2
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )
target_link_libraries(z_dist3
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )
target_link_libraries(z_dist4
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )

#################################################
target_link_libraries(feedback1
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )
target_link_libraries(feedback2
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )
target_link_libraries(feedback3
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )
target_link_libraries(feedback4
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
 )



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rplidar_data.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
