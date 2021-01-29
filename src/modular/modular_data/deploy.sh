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
       echo "Try something like:                         ./deply.sh modular_ros_pkg"
       echo "you can also add a destination folder like: ./deply.sh modular_ros_pkg ~/catkin_ws/src"
       exit
fi
package_name="$1"

if [ $# -lt 2 ]; then
    export DESTINATION_FOLDER=$ROBOTOLOGY_ROOT/robots
else
    export DESTINATION_FOLDER="$2"
fi
printf "Deploying package ${GREEN}${package_name}${NC} into ${RED}$DESTINATION_FOLDER${NC}"
echo

# this way the script can be called from any directory
SCRIPT_ROOT=$(dirname $(readlink --canonicalize --no-newline $BASH_SOURCE))

cp -TRfv /tmp/modular $DESTINATION_FOLDER/${package_name}

cd $DESTINATION_FOLDER/${package_name}

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

#cat > ${package_name}_basic.yaml << EOF

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

#cat > ${package_name}_cartesio.launch << EOF
#<launch>
#    <arg name="pkg_name"  default="${package_name}"/>
#EOF
#
#cat >> ${package_name}_cartesio.launch << 'EOF'
#    <arg name="rate" default="100.0"/>
#    <arg name="prefix" default=""/>
#    <param name="cartesian/problem_description"
#        textfile="$(eval find(pkg_name) + '/cartesio/ModularBot_cartesio_config.yaml')"/>
#
#    <param name="is_model_floating_base" value="false"/>
#</launch>
#EOF

cd ..

mkdir launch
cd launch

printf "${GREEN}Creating ModularBot_ik.launch${NC}"
echo

cat > ModularBot_ik.launch << EOF
<launch>
    <arg name="pkg_name"  default="$package_name"/>
EOF

cat >> ModularBot_ik.launch << 'EOF'
    <param name="robot_description" textfile="$(eval find(pkg_name) + '/urdf/ModularBot.urdf')"/>
    <param name="robot_description_semantic" textfile="$(eval find(pkg_name) + '/srdf/ModularBot.srdf')"/>
    <param name="cartesian/problem_description" textfile="$(eval find(pkg_name) + '/cartesio/ModularBot_cartesio_IK_config.yaml')"/>

    <arg name="solver" default="OpenSot"/>
    <arg name="prefix" default=""/>
    <arg name="use_xbot_config" default="false"/>
    <arg name="verbosity" default="0"/>
    <arg name="rate" default="100.0"/>
    <arg name="tf_prefix" default="ci"/>
    <arg name="markers" default="true"/>
    <arg name="namespace" default=""/>


    <node pkg="cartesian_interface" type="ros_server_node"
                                    name="ros_server_node"
                                    output="screen"
                                    launch-prefix="$(arg prefix)"
                                    respawn="true">
                                    <!-- required="true" -->

        <param name="is_model_floating_base" type="bool" value="false"/>
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

    <!-- <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
        <param name="use_gui" value="true"/>
        <param name="rate" value="10"/>
        <remap from="joint_states" to="cartesian/posture/reference"/>
        <remap from="zeros" to="cartesian/posture/home"/>
    </node> -->

</launch>

EOF

cat > ModularBot_sliders.launch << EOF
<launch>
    <arg name="gui" default="true" />
    <arg name="pkg_name"  default="$package_name"/>
EOF

cat >> ModularBot_sliders.launch << 'EOF'
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

mkdir -p $DESTINATION_FOLDER/${package_name}/database/ModularBot_fixed_base
# cd $SCRIPT_ROOT/../web/static/
cp -TRfv $SCRIPT_ROOT/../web/static/models $DESTINATION_FOLDER/${package_name}/database

#cd $DESTINATION_FOLDER/${package_name}
cd ./database/ModularBot_fixed_base
cat > manifest.xml << EOF
<?xml version="1.0"?>
<model>
  <name>${package_name}</name>
  <version>0.1</version>
  <sdf version='1.6'>${package_name}.sdf</sdf>

  <author>
   <name>Edoardo Romiti</name>
   <email>edoardo.romiti@iit.it</email>
  </author>

  <description>
    Simulation of the Alberobotics's ${package_name}.
  </description>
</model>

EOF

cat > model.config << EOF
<?xml version="1.0"?>
<model>
  <name>${package_name}</name>
  <version>0.1</version>
  <sdf version='1.6'>${package_name}.sdf</sdf>

  <author>
   <name>Edoardo Romiti</name>
   <email>edoardo.romiti@iit.it</email>
  </author>

  <description>
    Simulation of the Alberobotics's ${package_name}.
  </description>
</model>

EOF

cat > modular_world.sdf << EOF
<sdf version='1.6'>
  <world name='default'>
  <physics type="ode">
    <max_step_size>0.0001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>10000</real_time_update_rate>
    <ode>
      <solver>
        <type>quick</type>
        <min_step_size>0.00001</min_step_size>
        <iters>70</iters>
        <sor>1.3</sor>
        <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
      </solver>
      <constraints>
        <cfm>0</cfm>
        <erp>0.2</erp>
        <contact_surface_layer>0</contact_surface_layer>
        <contact_max_correcting_vel>100</contact_max_correcting_vel>
      </constraints>
    </ode>
  </physics>
  <gravity>0.0 0.0 -9.81</gravity>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose frame=''>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>5 -5 2 0 0.275643 2.35619</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>

EOF

gz sdf --print \
    $DESTINATION_FOLDER/${package_name}/urdf/ModularBot.urdf > \
    $DESTINATION_FOLDER/${package_name}/database/ModularBot_fixed_base/${package_name}.sdf

printf "\n\n${GREEN}Package ${package_name} succesfully deployed into $DESTINATION_FOLDER${NC}\n\n"
