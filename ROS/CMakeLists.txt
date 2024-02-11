cmake_minimum_required(VERSION 3.0.2)
project(robot1)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  turtlesim
  message_generation
)


generate_messages(
   DEPENDENCIES
   std_msgs  
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot1
#  CATKIN_DEPENDS roscpp rospy turtlesim
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(node1 src/node1.cpp)
target_link_libraries(node1 ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(node2 src/node2.cpp)
target_link_libraries(node2 ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)



