cmake_minimum_required(VERSION 2.8.3)
project(offboard)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tf
  tf2
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  mavros
  mavros_msgs
  roscpp
  rospy
  message_generation
  sensor_msgs
)


set(THREADS_PREFER_PTHREAD_FLAG ON)

## Non ROS libraries
find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED)

include_directories(${CURSES_INCLUDE_DIR})

# find_package(Boost REQUIRED COMPONENTS system)


# catkin_python_setup()



add_message_files(
  FILES
  Health.msg
  ActuatorArray.msg
)

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
#   geometry_msgs#   mavros_msgs
)



# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uav_test
#  CATKIN_DEPENDS geometry_msgs mavros mavros_msgs roscpp rospy
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/uav_test.cpp
# )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_executable(offb_node src/offb_node.cpp)
#target_link_libraries(offb_node ${catkin_LIBRARIES})
#add_executable(att_node src/att_node.cpp)
#target_link_libraries(att_node ${catkin_LIBRARIES})
#add_executable(att src/att.cpp)
#target_link_libraries(att ${catkin_LIBRARIES})

add_library(mavsdk_helper src/mavsdk_helper.h src/mavsdk_helper.cpp)
add_library(uav_monitor src/uav_monitor.h src/uav_monitor.cpp)

target_link_libraries(mavsdk_helper
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_info
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk
)
target_link_libraries(uav_monitor
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_info
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk

	mavsdk_helper

	${CMAKE_THREAD_LIBS_INIT}
	${catkin_LIBRARIES}
	)

add_executable(ros_interface src/ros_interface.cpp)
add_executable(mocap_to_quat src/mocap_to_quat.cpp)
target_link_libraries(mocap_to_quat
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	)
target_link_libraries(ros_interface 
	${catkin_LIBRARIES}
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_info
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk

	uav_monitor
	mavsdk_helper

	${Boost_LIBRARIES}
)
add_executable(test src/test.cpp)
target_link_libraries(test
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	)

add_executable(joy_ctrl src/joystick_ctrl.cpp)
target_link_libraries(joy_ctrl
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	)
#add_executable(${PROJECT_NAME}_node src/pos.cpp)

# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


# catkin_add_gtest(${PROJECT_NAME}-test test/test_uav_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# catkin_add_nosetests(test)
