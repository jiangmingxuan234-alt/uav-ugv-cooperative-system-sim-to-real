cmake_minimum_required(VERSION 3.0.2)
project(drone_ctrl_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(takeoff_node src/takeoff_node.cpp)
add_executable(land_node src/land_node.cpp)
add_executable(move_node src/move_node.cpp)
add_executable(move_forward_node src/move_forward_node.cpp)
add_executable(set_point_node src/set_point_node.cpp)
add_executable(set_vel_node src/set_vel_node.cpp)
add_executable(keyboard_ctrl_node src/keyboard_ctrl_node.cpp)
add_executable(keyboard_input_node src/keyboard_input_node.cpp)
add_executable(vel_ctrl_node src/vel_ctrl_node.cpp)
add_executable(red_ballon_node src/red_ballon_node.cpp)
add_executable(pose_ctrl_node src/pose_ctrl_node.cpp)
add_executable(test_node src/test_node.cpp)
add_executable(test2 src/test2.cpp)
add_executable(keyboard_pose_pub_node src/keyboard_pose_pub_node.cpp)
add_executable(pose_ctrl_node2 src/pose_ctrl_node2.cpp)

target_link_libraries(takeoff_node ${catkin_LIBRARIES})
target_link_libraries(land_node ${catkin_LIBRARIES})
target_link_libraries(move_node ${catkin_LIBRARIES})
target_link_libraries(move_forward_node ${catkin_LIBRARIES})
target_link_libraries(set_point_node ${catkin_LIBRARIES})
target_link_libraries(set_vel_node ${catkin_LIBRARIES})
target_link_libraries(keyboard_ctrl_node ${catkin_LIBRARIES})
target_link_libraries(keyboard_input_node ${catkin_LIBRARIES})
target_link_libraries(vel_ctrl_node ${catkin_LIBRARIES})
target_link_libraries(red_ballon_node ${catkin_LIBRARIES})
target_link_libraries(pose_ctrl_node ${catkin_LIBRARIES})
target_link_libraries(test_node ${catkin_LIBRARIES})
target_link_libraries(test2 ${catkin_LIBRARIES})
target_link_libraries(keyboard_pose_pub_node ${catkin_LIBRARIES})
target_link_libraries(pose_ctrl_node2 ${catkin_LIBRARIES})

install(TARGETS takeoff_node land_node move_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
add_dependencies(pose_ctrl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
