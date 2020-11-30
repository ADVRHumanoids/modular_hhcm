cmake_minimum_required(VERSION 2.8.3)
project(modular)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

# catkin_install_python(PROGRAMS src/modular/RobotDesignStudio
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

# install(DIRECTORY launch rviz scripts src/modular/static/models/modular/meshes
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# if(EXISTS ModularBot)
# 	install(DIRECTORY ModularBot
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# endif()

#### NOW YOU CAN EVENTUALLY DIRECTLY COPY ModularBot FOLDER INTO configs ####
#file(COPY ModularBot/urdf/ModularBot.urdf DESTINATION ../../../configs/ADVR_shared/ModularBot/urdf)
#file(COPY ModularBot/srdf/ModularBot.srdf DESTINATION ../../../configs/ADVR_shared/ModularBot/srdf)
#file(COPY ModularBot/joint_map/ModularBot_joint_map.yaml DESTINATION ../../../configs/ADVR_shared/ModularBot/joint_map)
