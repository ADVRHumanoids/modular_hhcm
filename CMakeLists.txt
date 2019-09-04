cmake_minimum_required(VERSION 2.8.3)
project(modular)

find_package(catkin REQUIRED)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY launch urdf rviz srdf cartesio scripts web/static/models/modular/meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#configure_file(${CMAKE_CURRENT_SOURCE_DIR}/urdf/ModularBot.urdf ${CMAKE_CURRENT_SOURCE_DIR}/../../configs/ADVR_shared/ModularBot/urdf/ModularBot.urdf COPYONLY)

file(COPY urdf/ModularBot.urdf DESTINATION ../../../configs/ADVR_shared/ModularBot/urdf)
file(COPY srdf/ModularBot.srdf DESTINATION ../../../configs/ADVR_shared/ModularBot/srdf)
file(COPY joint_map/ModularBot_joint_map.yaml DESTINATION ../../../configs/ADVR_shared/ModularBot/joint_map)
