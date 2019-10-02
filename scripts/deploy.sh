#!/bin/bash

RED='\033[0;31m'
PURPLE='\033[0;35m'
GREEN='\033[0;32m'
ORANGE='\033[0;33m'
YELLOW='\033[1;33m'
NC='\033[0m'

if [ "$#" -lt 1 ]; then
       printf "${RED}No package name argument passed!${NC}" 
       echo 
       echo "Try something like:"
       echo "./deply.sh modular_ros_pkg"
       exit
fi


package_name="$1"
printf "Deploying package ${GREEN}${package_name}${NC} into ${RED}$ROBOTOLOGY_ROOT/robots${NC}"
echo

# this way the script can be called from any directory
SCRIPT_ROOT=$(dirname $(readlink --canonicalize --no-newline $BASH_SOURCE))

cd $SCRIPT_ROOT/..

cp -R ModularBot $ROBOTOLOGY_ROOT/robots/${package_name}

cd $ROBOTOLOGY_ROOT/robots/${package_name}

cat > package.xml << EOF
<package>

  <name>$package_name</name>
  <version>1.0.0</version>
  <description>
  Modular robots tool
  </description>
  <maintainer email="edoardo.romiti@iit.it">Edoardo Romiti</maintainer>
  <license>BSD</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roslaunch</build_depend>
  <run_depend>joint_state_publisher</run_depend>
  <run_depend>robot_state_publisher</run_depend>
  <run_depend>rviz</run_depend>
  <run_depend>xacro</run_depend>

</package>
EOF

cat > CMakeLists.txt << EOF
cmake_minimum_required(VERSION 2.8.3)
project($package_name)

find_package(catkin REQUIRED)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(cartesio)
endif()

install(DIRECTORY urdf srdf launch
  DESTINATION 
EOF

cat >> CMakeLists.txt << 'EOF' 
${CATKIN_PACKAGE_SHARE_DESTINATION})
EOF

cat > ${package_name}_basic.yaml << EOF
XBotCore:
  config_path: "robots/${package_name}/configs/ModularBot.yaml"

XBotInterface:
  urdf_path: "robots/${package_name}/urdf/ModularBot.urdf"
  srdf_path: "robots/${package_name}/srdf/ModularBot.srdf"
  joint_map_path: "robots/${package_name}/joint_map/ModularBot_joint_map.yaml"

RobotInterface:
  framework_name: "ROS"

ModelInterface:
  model_type: "RBDL"
  is_model_floating_base: "false"

RobotInterfaceROS:
  publish_tf: true
  
MasterCommunicationInterface:
  framework_name: "ROS"

XBotRTPlugins:
  plugins: ["HomingExample", "CartesianImpedancePlugin"]
  io_plugins: ["CartesianImpedanceIO"]
  
NRTPlugins:
  plugins: []
 
#WebServer:
  #enable: "true"
  #address: "10.24.7.100"
  #port: "8081"

SimulationOptions:
  verbose_mode: "false"

EOF

mkdir cartesio
cd cartesio
cat > cartesio.launch << EOF
<launch>
    <arg name="pkg_name"  default="$package_name"/>
EOF

cat >> cartesio.launch << 'EOF'
    <param name="robot_description" textfile="$(eval find(pkg_name) + '/urdf/ModularBot.urdf')"/>
    <param name="robot_description_semantic" textfile="$(eval find(pkg_name) + '/srdf/ModularBot.srdf')"/>
    <param name="cartesian/problem_description" textfile="$(eval find(pkg_name) + '/cartesio/stack.yaml')"/>
    
    <arg name="solver" default="OpenSot"/>
    <arg name="prefix" default=""/>
    <arg name="use_xbot_config" default="false"/>
    <arg name="verbosity" default="2"/>
    <arg name="rate" default="500.0"/>
    <arg name="tf_prefix" default="ci"/>
    <arg name="markers" default="true"/>
    <arg name="namespace" default=""/> <!-- dummy argument avoids pass_all_args error in parent launch file -->
    <arg name="robot" default=""/>
    <arg name="is_model_floating_base" default="true"/>
       
    
    <node pkg="cartesian_interface" type="ros_server_node" 
                                    name="ros_server_node" 
                                    required="true" 
                                    output="screen" 
                                    launch-prefix="$(arg prefix)">
                                    
        <param name="is_model_floating_base" value="$(arg is_model_floating_base)"/>
        <param name="model_type" value="RBDL"/>
        <param name="solver" value="$(arg solver)"/>
        <param name="use_xbot_config" value="$(arg use_xbot_config)"/>
        <param name="verbosity" value="$(arg verbosity)"/>
        <param name="rate" value="$(arg rate)"/>
        <param name="tf_prefix" value="$(arg tf_prefix)"/>
        
    </node>

    <node if="$(arg markers)" pkg="cartesian_interface" type="marker_spawner" name="interactive_markers" output="screen">
        <param name="tf_prefix" value="$(arg tf_prefix)"/>
    </node>



</launch>
EOF
cd ..

mkdir launch
cd launch

cat >> ${package_name}_sliders.launch << EOF
<launch>
    <arg name="gui" default="true" />
    <arg name="pkg_name"  default="$package_name"/>
EOF

cat >> ${package_name}_sliders.launch << 'EOF'
    <param name="robot_description" textfile="$(eval find(pkg_name) + '/urdf/ModularBot.urdf')"/>
    <param name="robot_description_semantic" textfile="$(eval find(pkg_name) + '/srdf/ModularBot.srdf')"/>
    
    <param name="use_gui" value="$(arg gui)"/>
    <param name="rate" value="50.0"/>
     
        
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="publish_default_efforts" value="True"/>
    </node>

    <!-- start robot state publisher -->
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen" >
        <param name="publish_frequency" type="double" value="250.0" />
    </node> 

</launch>
EOF
cd ..