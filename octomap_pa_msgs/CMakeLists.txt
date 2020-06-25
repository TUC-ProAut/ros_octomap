cmake_minimum_required(VERSION 2.8.3)
project(octomap_pa_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
    ConfigBase.msg
    ConfigDegrading.msg
    ConfigInsertion.msg

    Config.msg
)
add_service_files(
  FILES
    Reset.srv
    SetConfigDegrading.srv
    SetConfigInsertion.srv
    GetConfig.srv

    AddCloud.srv
    AddCloudTf.srv
    GetCloud.srv
    GetSize.srv

    FileName.srv
)
generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
)

catkin_package(
  DEPENDS

  INCLUDE_DIRS

  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    message_runtime
  LIBRARIES
)
