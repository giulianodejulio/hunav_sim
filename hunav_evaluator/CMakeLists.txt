cmake_minimum_required(VERSION 3.0.2)
project(hunav_evaluator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  hunav_msgs
)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  hunav_evaluator/hunav_evaluator_node.py
  hunav_evaluator/hunav_metrics.py
  hunav_evaluator/sfm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  hunav_evaluator/hunav_evaluator_node.py
  scripts/hunav_robot_state_node.py
  scripts/hunav_human_states_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)