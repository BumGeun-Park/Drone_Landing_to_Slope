cmake_minimum_required(VERSION 3.0.2)
project(rplidar_data)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

find_package(cmake_modules REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})


 add_message_files(
   FILES
   polar.msg
   xyz.msg
   phi.msg
   alpha.msg
   map.msg
   time.msg
   packet.msg
 )


 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
  LIBRARIES rplidar_data
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS Eigen
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)


 add_executable(Raw_data2Information src/Raw_data2Information.cpp)
 add_dependencies(Raw_data2Information ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(Raw_data2Information ${catkin_LIBRARIES} ${Eigen_LIBRARIES})


 add_executable(assemble src/assemble.cpp)
 add_dependencies(assemble ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(assemble ${catkin_LIBRARIES} ${Eigen_LIBRARIES})


 add_executable(servo_motor src/servo_motor.cpp)
 add_dependencies(servo_motor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(servo_motor ${catkin_LIBRARIES} ${Eigen_LIBRARIES})


 add_executable(find_min src/find_min.cpp)
 add_dependencies(find_min ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(find_min ${catkin_LIBRARIES} ${Eigen_LIBRARIES})


 add_executable(alpha src/alpha.cpp)
 add_dependencies(alpha ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(alpha ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
 

 add_executable(local_map src/local_map.cpp)
 add_dependencies(local_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(local_map ${catkin_LIBRARIES} ${Eigen_LIBRARIES})


 add_executable(Leg_servo src/Leg_servo.cpp)
 add_dependencies(Leg_servo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(Leg_servo ${catkin_LIBRARIES} ${Eigen_LIBRARIES})


 add_executable(z_dist_error src/z_dist_error.cpp)
 add_dependencies(z_dist_error ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(z_dist_error ${catkin_LIBRARIES} ${Eigen_LIBRARIES})


 add_executable(control_theta src/control_theta.cpp)
 add_dependencies(control_theta ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(control_theta ${catkin_LIBRARIES} ${Eigen_LIBRARIES})


 add_executable(PI_controller src/PI_controller.cpp)
 add_dependencies(PI_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(PI_controller ${catkin_LIBRARIES} ${Eigen_LIBRARIES})


