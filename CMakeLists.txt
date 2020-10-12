cmake_minimum_required(VERSION 2.8.3)

project(doggo_description)

find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(mimic_node src/mimic.cpp include/mimic.hpp)
target_link_libraries(mimic_node ${catkin_LIBRARIES})
add_dependencies(mimic_node  ${catkin_EXPORTED_TARGETS})