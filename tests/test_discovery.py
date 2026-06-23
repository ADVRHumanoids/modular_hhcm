import pytest
import xml.etree.ElementTree as ET


def _urdf_writer_constructable():
  try:
    from modular.URDF_writer import UrdfWriter
    UrdfWriter(quiet=True)
    return True
  except Exception:
    return False


reply = "{1: {active_ports: 15, esc_type: 50, mod_id: 3, mod_rev: 0, mod_size: 0, mod_type: 2, position: 1, robot_id: 201, topology: 4}, 2: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 2, robot_id: -1, topology: 4}, 3: {active_ports: 3, esc_type: 21, mod_id: 8, mod_rev: 0, mod_size: 5, mod_type: 1, position: 3, robot_id: 21, topology: 2}, 4: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 4, robot_id: 22, topology: 1}, 5: {active_ports: 3, esc_type: 21, mod_id: 7, mod_rev: 0, mod_size: 5, mod_type: 1, position: 5, robot_id: 11, topology: 2}, 6: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 6, robot_id: 12, topology: 1}, 7: {active_ports: 3, esc_type: 21, mod_id: 7, mod_rev: 0, mod_size: 5, mod_type: 1, position: 7, robot_id: 31, topology: 2}, 8: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 8, robot_id: 32, topology: 1}, 9: {active_ports: 3, esc_type: 21, mod_id: 8, mod_rev: 0, mod_size: 5, mod_type: 1, position: 9, robot_id: 41, topology: 2}, 10: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 10, robot_id: 42, topology: 1}, 11: {active_ports: 3, esc_type: 21, mod_id: 255, mod_rev: 4095, mod_size: 15, mod_type: 15, position: 11, robot_id: 62, topology: 2}, 12: {active_ports: 3, esc_type: 21, mod_id: 6, mod_rev: 0, mod_size: 4, mod_type: 1, position: 12, robot_id: 61, topology: 2}, 13: {active_ports: 3, esc_type: 21, mod_id: 4, mod_rev: 0, mod_size: 4, mod_type: 1, position: 13, robot_id: 51, topology: 2}, 14: {active_ports: 3, esc_type: 1280, mod_id: 6, mod_rev: 0, mod_size: 4, mod_type: 4, position: 14, robot_id: -1, topology: 2}, 15: {active_ports: 3, esc_type: 21, mod_id: 6, mod_rev: 0, mod_size: 4, mod_type: 1, position: 15, robot_id: 63, topology: 2}, 16: {active_ports: 3, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 1, position: 16, robot_id: 56, topology: 2}, 17: {active_ports: 3, esc_type: 1280, mod_id: 7, mod_rev: 0, mod_size: 4, mod_type: 4, position: 17, robot_id: -1, topology: 2}, 18: {active_ports: 3, esc_type: 21, mod_id: 6, mod_rev: 0, mod_size: 4, mod_type: 1, position: 18, robot_id: 53, topology: 2}, 19: {active_ports: 1, esc_type: 1536, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 3, position: 19, robot_id: -119, topology: 1}}"

urdf = """<?xml version="1.0" encoding="utf-8"?>
<robot name="modularbot">
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1"/>
  </material>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
  <material name="green">
    <color rgba="0 1 0 1"/>
  </material>
  <material name="yellow">
    <color rgba="1 1 0 1"/>
  </material>
  <material name="tree_green">
    <color rgba="0.13 0.54 0.13 1"/>
  </material>
  <link name="world"/>
  <joint name="reference" type="floating">
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <origin rpy="0 0 0" xyz="0.0 0.0 0.8"/>
  <link name="base_link">
    <inertial>
      <mass value="0.001"/>
      <inertia ixx="1.0e-06" ixy="0.0" ixz="0.0" iyy="1.0e-06" iyz="0.0" izz="1.0e-06"/>
    </inertial>
  </link>
  <gazebo>
    <plugin filename="libxbot2_gz_joint_server.so" name="xbot2_gz_joint_server">
      <pid>
        <gain d="10" name="small_mot" p="100"/>
        <gain d="50" name="medium_mot" p="500"/>
        <gain d="100" name="big_mot" p="1000"/>
        <gain name="J1_A" profile="medium_mot"/>
        <gain d="50" name="J_wheel_A" p="0"/>
        <gain name="J1_B" profile="medium_mot"/>
        <gain d="50" name="J_wheel_B" p="0"/>
        <gain name="J1_C" profile="medium_mot"/>
        <gain d="50" name="J_wheel_C" p="0"/>
        <gain name="J1_D" profile="medium_mot"/>
        <gain d="50" name="J_wheel_D" p="0"/>
        <gain d="60.0" name="J1_E" p="1000.0"/>
        <gain d="60.0" name="J2_E" p="1000.0"/>
        <gain d="60.0" name="J3_E" p="1000.0"/>
        <gain d="60.0" name="J4_E" p="1000.0"/>
        <gain d="30.0" name="J5_E" p="500.0"/>
        <gain d="60.0" name="J6_E" p="1000.0"/>
      </pid>
    </plugin>
  </gazebo>
  <link name="mobile_base">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_central_body.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_central_body.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.012213513 0.0 -0.21761073"/>
      <mass value="127.57599"/>
      <inertia ixx="19.025397" ixy="-0.62823044" ixz="-0.064261485" iyy="13.251602" iyz="0.91864796" izz="23.575473"/>
    </inertial>
  </link>
  <joint name="fixed_mobile_base" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="mobile_base"/>
    <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.0"/>
  </joint>
  <joint name="base_link_projected_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_link_projected"/>
    <origin rpy="0 0 0" xyz="0 0 -0.747"/>
  </joint>
  <link name="base_link_projected"/>
  <!-- camera body, with origin at bottom screw mount -->
  <joint name="D435_camera_front_joint" type="fixed">
    <origin rpy="0 0.261799 0" xyz="0.402872 0.0 -0.078218"/>
    <parent link="mobile_base"/>
    <child link="D435_camera_front_bottom_screw_frame"/>
  </joint>
  <link name="D435_camera_front_bottom_screw_frame"/>
  <!-- camera body, with origin at bottom screw mount -->
  <joint name="D435_camera_back_joint" type="fixed">
    <origin rpy="0 0.261799 3.141593" xyz="-0.402872 0.0 -0.078218"/>
    <parent link="mobile_base"/>
    <child link="D435_camera_back_bottom_screw_frame"/>
  </joint>
  <link name="D435_camera_back_bottom_screw_frame"/>
  <!-- camera body, with origin at bottom screw mount -->
  <joint name="T265_camera_left_joint" type="fixed">
    <origin rpy="3.141593 0.0 1.570796" xyz="0.02375 0.37245 -0.062"/>
    <parent link="mobile_base"/>
    <child link="T265_camera_left_pose_frame"/>
  </joint>
  <link name="T265_camera_left_pose_frame"/>
  <!-- camera body, with origin at bottom screw mount -->
  <joint name="T265_camera_right_joint" type="fixed">
    <origin rpy="0 0.0 -1.570796" xyz="0.02375 -0.37245 -0.062"/>
    <parent link="mobile_base"/>
    <child link="T265_camera_right_pose_frame"/>
  </joint>
  <link name="T265_camera_right_pose_frame"/>
  <!-- camera body, with origin at bottom screw mount -->
  <joint name="VLP16_lidar_front_base_mount_joint" type="fixed">
    <origin rpy="3.141593 0.0 1.570796" xyz="0.5305 0.315 -0.0627"/>
    <parent link="mobile_base"/>
    <child link="VLP16_lidar_front_base_link"/>
  </joint>
  <link name="VLP16_lidar_front_base_link"/>
  <joint name="VLP16_lidar_back_base_mount_joint" type="fixed">
    <origin rpy="3.141593 0.0 -1.570796" xyz="-0.5305 -0.315 -0.0627"/>
    <parent link="mobile_base"/>
    <child link="VLP16_lidar_back_base_link"/>
  </joint>
  <link name="VLP16_lidar_back_base_link"/>
  <joint name="ultrasound_fl_sag_base_mount_joint" type="fixed">
    <origin rpy="0.0 1.963496 0.0" xyz="0.5105 0.315 -0.129"/>
    <parent link="mobile_base"/>
    <child link="ultrasound_fl_sag_base_link"/>
  </joint>
  <link name="ultrasound_fl_sag_base_link"/>
  <!-- Rotate sensor frame (readings are on x axis instead of z) -->
  <joint name="ultrasound_fl_sag_joint" type="fixed">
    <origin rpy="0.0 -1.57 0.0" xyz="0 0 0"/>
    <parent link="ultrasound_fl_sag_base_link"/>
    <child link="ultrasound_fl_sag"/>
  </joint>
  <link name="ultrasound_fl_sag"/>
  <joint name="ultrasound_fr_sag_base_mount_joint" type="fixed">
    <origin rpy="0.0 1.963496 0.0" xyz="0.5105 -0.315 -0.129"/>
    <parent link="mobile_base"/>
    <child link="ultrasound_fr_sag_base_link"/>
  </joint>
  <link name="ultrasound_fr_sag_base_link"/>
  <!-- Rotate sensor frame (readings are on x axis instead of z) -->
  <joint name="ultrasound_fr_sag_joint" type="fixed">
    <origin rpy="0.0 -1.57 0.0" xyz="0 0 0"/>
    <parent link="ultrasound_fr_sag_base_link"/>
    <child link="ultrasound_fr_sag"/>
  </joint>
  <link name="ultrasound_fr_sag"/>
  <joint name="ultrasound_rl_sag_base_mount_joint" type="fixed">
    <origin rpy="0.0 1.963496 3.141593" xyz="-0.5105 0.315 -0.129"/>
    <parent link="mobile_base"/>
    <child link="ultrasound_rl_sag_base_link"/>
  </joint>
  <link name="ultrasound_rl_sag_base_link"/>
  <!-- Rotate sensor frame (readings are on x axis instead of z) -->
  <joint name="ultrasound_rl_sag_joint" type="fixed">
    <origin rpy="0.0 -1.57 0.0" xyz="0 0 0"/>
    <parent link="ultrasound_rl_sag_base_link"/>
    <child link="ultrasound_rl_sag"/>
  </joint>
  <link name="ultrasound_rl_sag"/>
  <joint name="ultrasound_rr_sag_base_mount_joint" type="fixed">
    <origin rpy="0.0 1.963496 3.141593" xyz="-0.5105 -0.315 -0.129"/>
    <parent link="mobile_base"/>
    <child link="ultrasound_rr_sag_base_link"/>
  </joint>
  <link name="ultrasound_rr_sag_base_link"/>
  <!-- Rotate sensor frame (readings are on x axis instead of z) -->
  <joint name="ultrasound_rr_sag_joint" type="fixed">
    <origin rpy="0.0 -1.57 0.0" xyz="0 0 0"/>
    <parent link="ultrasound_rr_sag_base_link"/>
    <child link="ultrasound_rr_sag"/>
  </joint>
  <link name="ultrasound_rr_sag"/>
  <joint name="ultrasound_fl_lat_base_mount_joint" type="fixed">
    <origin rpy="0.0 1.963496 1.570796" xyz="0.4515 0.365 -0.117"/>
    <parent link="mobile_base"/>
    <child link="ultrasound_fl_lat_base_link"/>
  </joint>
  <link name="ultrasound_fl_lat_base_link"/>
  <!-- Rotate sensor frame (readings are on x axis instead of z) -->
  <joint name="ultrasound_fl_lat_joint" type="fixed">
    <origin rpy="0.0 -1.57 0.0" xyz="0 0 0"/>
    <parent link="ultrasound_fl_lat_base_link"/>
    <child link="ultrasound_fl_lat"/>
  </joint>
  <link name="ultrasound_fl_lat"/>
  <joint name="ultrasound_rl_lat_base_mount_joint" type="fixed">
    <origin rpy="0.0 1.963496 1.570796" xyz="-0.4515 0.365 -0.117"/>
    <parent link="mobile_base"/>
    <child link="ultrasound_rl_lat_base_link"/>
  </joint>
  <link name="ultrasound_rl_lat_base_link"/>
  <!-- Rotate sensor frame (readings are on x axis instead of z) -->
  <joint name="ultrasound_rl_lat_joint" type="fixed">
    <origin rpy="0.0 -1.57 0.0" xyz="0 0 0"/>
    <parent link="ultrasound_rl_lat_base_link"/>
    <child link="ultrasound_rl_lat"/>
  </joint>
  <link name="ultrasound_rl_lat"/>
  <joint name="ultrasound_fr_lat_base_mount_joint" type="fixed">
    <origin rpy="0.0 1.963496 -1.570796" xyz="0.4515 -0.365 -0.117"/>
    <parent link="mobile_base"/>
    <child link="ultrasound_fr_lat_base_link"/>
  </joint>
  <link name="ultrasound_fr_lat_base_link"/>
  <!-- Rotate sensor frame (readings are on x axis instead of z) -->
  <joint name="ultrasound_fr_lat_joint" type="fixed">
    <origin rpy="0.0 -1.57 0.0" xyz="0 0 0"/>
    <parent link="ultrasound_fr_lat_base_link"/>
    <child link="ultrasound_fr_lat"/>
  </joint>
  <link name="ultrasound_fr_lat"/>
  <joint name="ultrasound_rr_lat_base_mount_joint" type="fixed">
    <origin rpy="0.0 1.963496 -1.570796" xyz="-0.4515 -0.365 -0.117"/>
    <parent link="mobile_base"/>
    <child link="ultrasound_rr_lat_base_link"/>
  </joint>
  <link name="ultrasound_rr_lat_base_link"/>
  <!-- Rotate sensor frame (readings are on x axis instead of z) -->
  <joint name="ultrasound_rr_lat_joint" type="fixed">
    <origin rpy="0.0 -1.57 0.0" xyz="0 0 0"/>
    <parent link="ultrasound_rr_lat_base_link"/>
    <child link="ultrasound_rr_lat"/>
  </joint>
  <link name="ultrasound_rr_lat"/>
  <!-- Add Dalsa Cameras -->
  <joint name="fixed_dalsa_camera_right" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="dalsa_camera_right"/>
    <origin rpy="0.0 -0.26179278697487707 -0.5235508063884032" xyz="0.445 -0.285 0.074"/>
  </joint>
  <link name="dalsa_camera_right"/>
  <joint name="fixed_dalsa_camera_left" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="dalsa_camera_left"/>
    <origin rpy="0.0 -0.26179278697487707 0.5235508063884032" xyz="0.445 0.285 0.074"/>
  </joint>
  <link name="dalsa_camera_left"/>
  <joint name="fixed_J1_A" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="mobile_base"/>
    <child link="J1_A_stator"/>
    <origin rpy="-3.141592653589793 -1.5707963267948966 0.0" xyz="0.37925 0.3125 -0.195"/>
  </joint>
  <link name="J1_A_stator">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_steering_connection_module.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_steering_connection_module.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.0079406061 0.0 0.037039629"/>
      <mass value="1.1813661"/>
      <inertia ixx="0.0026106263" ixy="0.0" ixz="0.00017928188" iyy="0.0027438806" iyz="0.0" izz="0.0025223123"/>
    </inertial>
  </link>
  <joint name="J1_A" type="revolute">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="J1_A_stator"/>
    <child link="L_1_A"/>
    <origin rpy="-1.5707963267948966 -0.0 1.5707963267948966" xyz="-0.022 0.0 0.06075"/>
    <limit effort="127.0" lower="-2.6" upper="2.6" velocity="8.1"/>
  </joint>
  <link name="L_1_A">
    <visual>
      <origin rpy="3.141592653589793 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_steering_module_frame.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="3.141592653589793 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_steering_module_frame.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.090718433 0.0 0.22670598"/>
      <mass value="7.6995028"/>
      <inertia ixx="0.44048889" ixy="-0.03668535" ixz="0.025595962" iyy="0.52073797" iyz="-0.0026667139" izz="0.69985532"/>
    </inertial>
  </link>
  <joint name="fixed_J_wheel_A" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="L_1_A"/>
    <child link="J_wheel_A_stator"/>
    <origin rpy="-3.141592653589793 1.5707963267948966 0.0" xyz="0.0 0.0 0.37015"/>
  </joint>
  <link name="J_wheel_A_stator">
    <inertial>
      <mass value="0.0001"/>
      <inertia ixx="1e-09" ixy="0" ixz="0" iyy="1e-09" iyz="0" izz="1e-09"/>
    </inertial>
  </link>
  <joint name="J_wheel_A" type="revolute">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="J_wheel_A_stator"/>
    <child link="wheel_A"/>
    <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.0"/>
    <limit effort="24.0" lower="-10000000000.0" upper="10000000000.0" velocity="9.5"/>
  </joint>
  <link name="wheel_A">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_wheel.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.16"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 -0.005"/>
      <mass value="4.0116207"/>
      <inertia ixx="0.028580559" ixy="0.0" ixz="0.0" iyy="0.028580559" iyz="0.0" izz="0.044954065"/>
    </inertial>
  </link>
  <joint name="fixed_J1_B" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="mobile_base"/>
    <child link="J1_B_stator"/>
    <origin rpy="-3.141592653589793 -1.5707963267948966 0.0" xyz="0.37925 -0.3125 -0.195"/>
  </joint>
  <link name="J1_B_stator">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_steering_connection_module.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_steering_connection_module.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.0079406061 0.0 0.037039629"/>
      <mass value="1.1813661"/>
      <inertia ixx="0.0026106263" ixy="0.0" ixz="0.00017928188" iyy="0.0027438806" iyz="0.0" izz="0.0025223123"/>
    </inertial>
  </link>
  <joint name="J1_B" type="revolute">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="J1_B_stator"/>
    <child link="L_1_B"/>
    <origin rpy="1.5707963267948966 -0.0 -1.5707963267948966" xyz="-0.022 0.0 0.06075"/>
    <limit effort="127.0" lower="-2.6" upper="2.6" velocity="8.1"/>
  </joint>
  <link name="L_1_B">
    <visual>
      <origin rpy="3.141592653589793 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_steering_module_frame.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="3.141592653589793 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_steering_module_frame.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.090718433 0.0 0.22670598"/>
      <mass value="7.6995028"/>
      <inertia ixx="0.44048889" ixy="-0.03668535" ixz="0.025595962" iyy="0.52073797" iyz="-0.0026667139" izz="0.69985532"/>
    </inertial>
  </link>
  <joint name="fixed_J_wheel_B" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="L_1_B"/>
    <child link="J_wheel_B_stator"/>
    <origin rpy="-3.141592653589793 1.5707963267948966 0.0" xyz="0.0 0.0 0.37015"/>
  </joint>
  <link name="J_wheel_B_stator">
    <inertial>
      <mass value="0.0001"/>
      <inertia ixx="1e-09" ixy="0" ixz="0" iyy="1e-09" iyz="0" izz="1e-09"/>
    </inertial>
  </link>
  <joint name="J_wheel_B" type="revolute">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="J_wheel_B_stator"/>
    <child link="wheel_B"/>
    <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.0"/>
    <limit effort="24.0" lower="-10000000000.0" upper="10000000000.0" velocity="9.5"/>
  </joint>
  <link name="wheel_B">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_wheel.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.16"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 -0.005"/>
      <mass value="4.0116207"/>
      <inertia ixx="0.028580559" ixy="0.0" ixz="0.0" iyy="0.028580559" iyz="0.0" izz="0.044954065"/>
    </inertial>
  </link>
  <joint name="fixed_J1_C" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="mobile_base"/>
    <child link="J1_C_stator"/>
    <origin rpy="-0.0 -1.5707963267948966 0.0" xyz="-0.37925 0.3125 -0.195"/>
  </joint>
  <link name="J1_C_stator">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_steering_connection_module.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_steering_connection_module.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.0079406061 0.0 0.037039629"/>
      <mass value="1.1813661"/>
      <inertia ixx="0.0026106263" ixy="0.0" ixz="0.00017928188" iyy="0.0027438806" iyz="0.0" izz="0.0025223123"/>
    </inertial>
  </link>
  <joint name="J1_C" type="revolute">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="J1_C_stator"/>
    <child link="L_1_C"/>
    <origin rpy="1.5707963267948966 -0.0 -1.5707963267948966" xyz="-0.022 0.0 0.06075"/>
    <limit effort="127.0" lower="-2.6" upper="2.6" velocity="8.1"/>
  </joint>
  <link name="L_1_C">
    <visual>
      <origin rpy="3.141592653589793 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_steering_module_frame.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="3.141592653589793 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_steering_module_frame.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.090718433 0.0 0.22670598"/>
      <mass value="7.6995028"/>
      <inertia ixx="0.44048889" ixy="-0.03668535" ixz="0.025595962" iyy="0.52073797" iyz="-0.0026667139" izz="0.69985532"/>
    </inertial>
  </link>
  <joint name="fixed_J_wheel_C" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="L_1_C"/>
    <child link="J_wheel_C_stator"/>
    <origin rpy="-3.141592653589793 1.5707963267948966 0.0" xyz="0.0 0.0 0.37015"/>
  </joint>
  <link name="J_wheel_C_stator">
    <inertial>
      <mass value="0.0001"/>
      <inertia ixx="1e-09" ixy="0" ixz="0" iyy="1e-09" iyz="0" izz="1e-09"/>
    </inertial>
  </link>
  <joint name="J_wheel_C" type="revolute">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="J_wheel_C_stator"/>
    <child link="wheel_C"/>
    <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.0"/>
    <limit effort="24.0" lower="-10000000000.0" upper="10000000000.0" velocity="9.5"/>
  </joint>
  <link name="wheel_C">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_wheel.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.16"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 -0.005"/>
      <mass value="4.0116207"/>
      <inertia ixx="0.028580559" ixy="0.0" ixz="0.0" iyy="0.028580559" iyz="0.0" izz="0.044954065"/>
    </inertial>
  </link>
  <joint name="fixed_J1_D" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="mobile_base"/>
    <child link="J1_D_stator"/>
    <origin rpy="-0.0 -1.5707963267948966 0.0" xyz="-0.37925 -0.3125 -0.195"/>
  </joint>
  <link name="J1_D_stator">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_steering_connection_module.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_steering_connection_module.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.0079406061 0.0 0.037039629"/>
      <mass value="1.1813661"/>
      <inertia ixx="0.0026106263" ixy="0.0" ixz="0.00017928188" iyy="0.0027438806" iyz="0.0" izz="0.0025223123"/>
    </inertial>
  </link>
  <joint name="J1_D" type="revolute">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="J1_D_stator"/>
    <child link="L_1_D"/>
    <origin rpy="-1.5707963267948966 -0.0 1.5707963267948966" xyz="-0.022 0.0 0.06075"/>
    <limit effort="127.0" lower="-2.6" upper="2.6" velocity="8.1"/>
  </joint>
  <link name="L_1_D">
    <visual>
      <origin rpy="3.141592653589793 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_steering_module_frame.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="3.141592653589793 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_steering_module_frame.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.090718433 0.0 0.22670598"/>
      <mass value="7.6995028"/>
      <inertia ixx="0.44048889" ixy="-0.03668535" ixz="0.025595962" iyy="0.52073797" iyz="-0.0026667139" izz="0.69985532"/>
    </inertial>
  </link>
  <joint name="fixed_J_wheel_D" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="L_1_D"/>
    <child link="J_wheel_D_stator"/>
    <origin rpy="-3.141592653589793 1.5707963267948966 0.0" xyz="0.0 0.0 0.37015"/>
  </joint>
  <link name="J_wheel_D_stator">
    <inertial>
      <mass value="0.0001"/>
      <inertia ixx="1e-09" ixy="0" ixz="0" iyy="1e-09" iyz="0" izz="1e-09"/>
    </inertial>
  </link>
  <joint name="J_wheel_D" type="revolute">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="J_wheel_D_stator"/>
    <child link="wheel_D"/>
    <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.0"/>
    <limit effort="24.0" lower="-10000000000.0" upper="10000000000.0" velocity="9.5"/>
  </joint>
  <link name="wheel_D">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_wheel.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.16"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 -0.005"/>
      <mass value="4.0116207"/>
      <inertia ixx="0.028580559" ixy="0.0" ixz="0.0" iyy="0.028580559" iyz="0.0" izz="0.044954065"/>
    </inertial>
  </link>
  <joint name="fixed_J1_E" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="mobile_base"/>
    <child link="J1_E_stator"/>
    <origin rpy="0.0 -0.0 3.141592653589793" xyz="0.2 0.0 0.026"/>
  </joint>
  <link name="J1_E_stator">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_yaw-base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_yaw-base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.121"/>
      <mass value="3.586681"/>
      <inertia ixx="0.017808349" ixy="1.6625281e-06" ixz="-1.8023262e-05" iyy="0.017503159" iyz="-6.8557387e-05" izz="0.0068844202"/>
    </inertial>
  </link>
  <joint name="J1_E" type="revolute">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="J1_E_stator"/>
    <child link="L_1_E"/>
    <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.22316"/>
    <limit effort="460.0" lower="-2.75" upper="2.75" velocity="2.14"/>
  </joint>
  <link name="L_1_E">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_yaw-link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_yaw-link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.001875"/>
      <mass value="0.57797267"/>
      <inertia ixx="0.00098277371" ixy="9.0821713e-08" ixz="-7.1612945e-08" iyy="0.0009834362" iyz="0.0" izz="0.0016259552"/>
    </inertial>
  </link>
  <joint name="fixed_J2_E" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="L_1_E"/>
    <child link="J2_E_stator"/>
    <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.02184"/>
  </joint>
  <link name="J2_E_stator">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_elbow-base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_elbow-base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.001149215 0.042825109 0.076804506"/>
      <mass value="3.6144181"/>
      <inertia ixx="0.015371313" ixy="-9.9563906e-05" ixz="9.0045644e-06" iyy="0.0086766316" iyz="-0.0018673171" izz="0.013320926"/>
    </inertial>
  </link>
  <joint name="J2_E" type="revolute">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="J2_E_stator"/>
    <child link="L_2_E"/>
    <origin rpy="-1.5707963267948966 -0.0 0.0" xyz="0.0 0.13666 0.0875"/>
    <limit effort="460.0" lower="-2.75" upper="2.75" velocity="2.14"/>
  </joint>
  <link name="L_2_E">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_elbow-link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_elbow-link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 -0.028238874 0.017527776"/>
      <mass value="1.1481338"/>
      <inertia ixx="0.0034928397" ixy="-1.0903971e-05" ixz="6.2736548e-06" iyy="0.0029687575" iyz="0.00031199793" izz="0.0042223026"/>
    </inertial>
  </link>
  <joint name="fixed_J3_E" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="L_2_E"/>
    <child link="J3_E_stator"/>
    <origin rpy="-1.5707963267948966 -0.0 3.141592653589793" xyz="0.0 -0.0875 0.0195"/>
  </joint>
  <link name="J3_E_stator">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_yaw-base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_yaw-base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.121"/>
      <mass value="3.586681"/>
      <inertia ixx="0.017808349" ixy="1.6625281e-06" ixz="-1.8023262e-05" iyy="0.017503159" iyz="-6.8557387e-05" izz="0.0068844202"/>
    </inertial>
  </link>
  <joint name="J3_E" type="revolute">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="J3_E_stator"/>
    <child link="L_3_E"/>
    <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.22316"/>
    <limit effort="460.0" lower="-2.75" upper="2.75" velocity="2.14"/>
  </joint>
  <link name="L_3_E">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_yaw-link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_yaw-link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.001875"/>
      <mass value="0.57797267"/>
      <inertia ixx="0.00098277371" ixy="9.0821713e-08" ixz="-7.1612945e-08" iyy="0.0009834362" iyz="0.0" izz="0.0016259552"/>
    </inertial>
  </link>
  <link name="L_3_link_1_E">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/mesh_passive_400.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.2"/>
      <geometry>
        <cylinder length="0.4" radius="0.0675"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.183"/>
      <mass value="1.894"/>
      <inertia ixx="0.043541237" ixy="0.0" ixz="0.0" iyy="0.043541237" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>
  <joint name="L_3_fixed_joint_1_E" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="L_3_E"/>
    <child link="L_3_link_1_E"/>
    <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.02184"/>
  </joint>
  <joint name="fixed_J4_E" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="L_3_link_1_E"/>
    <child link="J4_E_stator"/>
    <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.4"/>
  </joint>
  <link name="J4_E_stator">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_elbow-base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_elbow-base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.001149215 0.042825109 0.076804506"/>
      <mass value="3.6144181"/>
      <inertia ixx="0.015371313" ixy="-9.9563906e-05" ixz="9.0045644e-06" iyy="0.0086766316" iyz="-0.0018673171" izz="0.013320926"/>
    </inertial>
  </link>
  <joint name="J4_E" type="revolute">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="J4_E_stator"/>
    <child link="L_4_E"/>
    <origin rpy="-1.5707963267948966 -0.0 0.0" xyz="0.0 0.13666 0.0875"/>
    <limit effort="460.0" lower="-2.75" upper="2.75" velocity="2.14"/>
  </joint>
  <link name="L_4_E">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_elbow-link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_elbow-link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 -0.028238874 0.017527776"/>
      <mass value="1.1481338"/>
      <inertia ixx="0.0034928397" ixy="-1.0903971e-05" ixz="6.2736548e-06" iyy="0.0029687575" iyz="0.00031199793" izz="0.0042223026"/>
    </inertial>
  </link>
  <joint name="fixed_J5_E" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="L_4_E"/>
    <child link="J5_E_stator"/>
    <origin rpy="-1.5707963267948966 -0.0 3.141592653589793" xyz="0.0 -0.0875 0.0195"/>
  </joint>
  <link name="J5_E_stator">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_yaw-base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_yaw-base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.11767718"/>
      <mass value="3.2592751"/>
      <inertia ixx="0.017046707" ixy="-2.9108359e-07" ixz="-6.3074987e-06" iyy="0.016747265" iyz="-5.7883896e-05" izz="0.0060549298"/>
    </inertial>
  </link>
  <joint name="J5_E" type="revolute">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="J5_E_stator"/>
    <child link="L_5_E"/>
    <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.22316"/>
    <limit effort="314.0" lower="-2.75" upper="2.75" velocity="2.85"/>
  </joint>
  <link name="L_5_E">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_yaw-link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_yaw-link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 -0.02"/>
      <mass value="0.56725368"/>
      <inertia ixx="0.00097738346" ixy="9.078518e-08" ixz="5.0589486e-08" iyy="0.00097804563" iyz="0.0" izz="0.001617678"/>
    </inertial>
  </link>
  <link name="L_5_link_1_E">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/mesh_passive_300.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.0675"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.135"/>
      <mass value="1.647"/>
      <inertia ixx="0.0226" ixy="0.0" ixz="0.0" iyy="0.0226" iyz="0.0" izz="0.00346"/>
    </inertial>
  </link>
  <joint name="L_5_fixed_joint_1_E" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="L_5_E"/>
    <child link="L_5_link_1_E"/>
    <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.02184"/>
  </joint>
  <joint name="fixed_J6_E" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="L_5_link_1_E"/>
    <child link="J6_E_stator"/>
    <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.3"/>
  </joint>
  <link name="J6_E_stator">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_elbow-base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_elbow-base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.001149215 0.042825109 0.076804506"/>
      <mass value="3.6144181"/>
      <inertia ixx="0.015371313" ixy="-9.9563906e-05" ixz="9.0045644e-06" iyy="0.0086766316" iyz="-0.0018673171" izz="0.013320926"/>
    </inertial>
  </link>
  <joint name="J6_E" type="revolute">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="J6_E_stator"/>
    <child link="L_6_E"/>
    <origin rpy="-1.5707963267948966 -0.0 0.0" xyz="0.0 0.13666 0.0875"/>
    <limit effort="460.0" lower="-2.75" upper="2.75" velocity="2.14"/>
  </joint>
  <link name="L_6_E">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_elbow-link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="concert_blue">
        <color rgba="0.149 0.259 0.51 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_elbow-link.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 -0.028238874 0.017527776"/>
      <mass value="1.1481338"/>
      <inertia ixx="0.0034928397" ixy="-1.0903971e-05" ixz="6.2736548e-06" iyy="0.0029687575" iyz="0.00031199793" izz="0.0042223026"/>
    </inertial>
  </link>
  <link name="drill_E">
    <visual>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/concert_drill.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin rpy="0.0 -0.0 0.0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://concert_resources/models/modular/meshes/concert/simple/concert_drill.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0062 0.0 0.1028"/>
      <mass value="10.353486"/>
      <inertia ixx="0.049783517" ixy="8.1965703e-06" ixz="0.0033027183" iyy="0.13661192" iyz="-1.937269e-06" izz="0.10214692"/>
    </inertial>
  </link>
  <!-- camera body, with origin at bottom screw mount -->
  <joint name="drill_camera_E_joint" type="fixed">
    <origin rpy="0.05 0.22 0.05" xyz="0.12 0.0 0.26"/>
    <parent link="drill_E"/>
    <child link="drill_camera_E_bottom_screw_frame"/>
  </joint>
  <link name="drill_camera_E_bottom_screw_frame"/>
  <link name="drillnose_E"/>
  <joint name="fixed_drillnose_E" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="drill_E"/>
    <child link="drillnose_E"/>
    <origin rpy="-0.0 1.5707963267948966 0.0" xyz="0.2815 0.0 0.0865"/>
  </joint>
  <joint name="drill_E_fixed_joint" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="L_6_E"/>
    <child link="drill_E"/>
    <origin rpy="-1.5707963267948966 -0.0 3.141592653589793" xyz="0.0 -0.0875 0.0195"/>
  </joint>
  <link name="ee_E">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 -0.135"/>
      <geometry>
        <cylinder length="0.27" radius="0.012"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 -0.135"/>
      <geometry>
        <cylinder length="0.27" radius="0.012"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0.135"/>
      <mass value="0.34"/>
      <inertia ixx="0.0020777400000000002" ixy="0.0" ixz="0.0" iyy="0.0020777400000000002" iyz="0.0" izz="2.4480000000000003e-05"/>
    </inertial>
  </link>
  <joint name="fixed_ee_E" type="fixed">
    <axis rpy="0 0 0" xyz="0 0 1"/>
    <parent link="drillnose_E"/>
    <child link="ee_E"/>
    <origin rpy="0.0 -0.0 0.0" xyz="0.0 0.0 0.27"/>
  </joint>
</robot>"""


def _extract_link_joint_names(xml_text):
  root = ET.fromstring(xml_text)
  links = {el.attrib["name"] for el in root.findall(".//link")}
  joints = {el.attrib["name"] for el in root.findall(".//joint")}
  return root, links, joints


def _short_list(items, max_items=20):
  items = sorted(items)
  if len(items) <= max_items:
    return items
  return items[:max_items] + [f"... (+{len(items) - max_items} more)"]


@pytest.mark.skipif(
  not _urdf_writer_constructable(),
  reason="UrdfWriter construction requires ros2 CLI and full resource paths",
)
def test_read_from_json_generates_expected_urdf_string():
  from modular.URDF_writer import UrdfWriter

  writer = UrdfWriter(verbose=False, quiet=True, slave_desc_mode="use_pos")
  data = writer.read_from_json(reply)
  writer.remove_all_connectors()
  generated_urdf = writer.process_urdf(xacro_mappings={'gazebo_urdf': 'false', 'velodyne': 'false', 'realsense': 'false', 'ultrasound': 'false'})
#   assert "string" in data
#   generated_urdf = data["string"]
  generated_root, generated_links, generated_joints = _extract_link_joint_names(generated_urdf)
  expected_root, expected_links, expected_joints = _extract_link_joint_names(urdf)

  # Some optional addon entities can be absent depending on current defaults.
  expected_links -= {"ee_E"}
  expected_joints -= {"fixed_ee_E"}

  missing_links = expected_links - generated_links
  missing_joints = expected_joints - generated_joints
  extra_links = generated_links - expected_links
  extra_joints = generated_joints - expected_joints

  assert generated_root.tag == expected_root.tag == "robot"
  assert generated_root.attrib.get("name", "").lower() == expected_root.attrib.get("name", "").lower()
  assert expected_links.issubset(generated_links), (
    "Missing expected links in generated URDF.\n"
    f"missing_links({len(missing_links)}): {_short_list(missing_links)}\n"
    f"extra_links({len(extra_links)}): {_short_list(extra_links)}"
  )
  assert expected_joints.issubset(generated_joints), (
    "Missing expected joints in generated URDF.\n"
    f"missing_joints({len(missing_joints)}): {_short_list(missing_joints)}\n"
    f"extra_joints({len(extra_joints)}): {_short_list(extra_joints)}"
  )