cmake_minimum_required(VERSION 2.8.3)
project(pr2_amazon_challenge_sim)


find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS gazebo_ros
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_custom_target(
  copy_kiva_pod ALL
  COMMAND mkdir -p $ENV{HOME}/.gazebo/models && cp -a ${CMAKE_CURRENT_SOURCE_DIR}/gazebo_models/kiva_pod $ENV{HOME}/.gazebo/models
)

add_custom_target(
  copy_kiva_pod_medres ALL
  COMMAND mkdir -p $ENV{HOME}/.gazebo/models && cp -a ${CMAKE_CURRENT_SOURCE_DIR}/gazebo_models/kiva_pod_medres $ENV{HOME}/.gazebo/models
)

add_custom_target(
  copy_cheezit_big_original ALL
  COMMAND mkdir -p $ENV{HOME}/.gazebo/models && cp -a ${CMAKE_CURRENT_SOURCE_DIR}/gazebo_models/cheezit_big_original $ENV{HOME}/.gazebo/models
)

add_custom_target(
  copy_crayola_64_ct ALL
  COMMAND mkdir -p $ENV{HOME}/.gazebo/models && cp -a ${CMAKE_CURRENT_SOURCE_DIR}/gazebo_models/crayola_64_ct $ENV{HOME}/.gazebo/models
)