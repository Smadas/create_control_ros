cmake_minimum_required(VERSION 3.0.2)
project(create_control)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation
        cv_bridge
)

 add_message_files(
         DIRECTORY
         msg
         FILES
         create_sensors.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES create_control
#  CATKIN_DEPENDS create roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(simple_control src/simple_control.cpp)
target_link_libraries(simple_control ${catkin_LIBRARIES})
add_dependencies(simple_control create_control_generate_messages_cpp)

add_executable(sensors src/sensors.cpp)
target_link_libraries(sensors ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})
add_dependencies(sensors create_control_generate_messages_cpp)

catkin_install_python(PROGRAMS scripts/simple_control_py.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )