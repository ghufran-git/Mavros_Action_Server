cmake_minimum_required(VERSION 2.8.3)
project(swarm_actions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	moveit_core 
	roscpp 
	actionlib 
	actionlib_msgs 
	geometry_msgs 
	mavros_msgs 
	trajectory_msgs 
	message_generation
	control_msgs
	control_toolbox
	)
	
find_package(Boost REQUIRED COMPONENTS system)
add_action_files(
  DIRECTORY action
  FILES SwarmActions.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  control_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp actionlib_msgs mavros_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_executable(swarm_actions src/swarm_actions.cpp)
target_link_libraries(swarm_actions ${catkin_LIBRARIES})
add_dependencies(swarm_actions ${catkin_EXPORTED_TARGETS})