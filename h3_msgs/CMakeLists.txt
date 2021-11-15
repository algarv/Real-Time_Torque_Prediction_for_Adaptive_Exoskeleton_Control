cmake_minimum_required(VERSION 3.0.2)
project(h3_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
FILES
State.msg
TaskCommand.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   DataRecording.srv
   TriggerOutput.srv
   ControlType.srv
   Joint.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(DEPENDENCIES std_msgs)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

