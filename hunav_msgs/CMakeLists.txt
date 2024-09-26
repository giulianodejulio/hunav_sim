cmake_minimum_required(VERSION 3.0.2)
project(hunav_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Agent.msg
  Agents.msg
  AgentBehavior.msg
)

add_service_files(
  FILES
  ComputeAgents.srv
  ComputeAgent.srv
  MoveAgent.srv
  GetAgents.srv
  ResetAgents.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

