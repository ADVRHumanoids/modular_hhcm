"""
Control-plugin strategy classes.

Each plugin encapsulates the control-framework–specific parts of the robot
description: joint transmissions (URDF/xacro snippets), SRDF groups, joint
maps, and low-level configuration files.

Currently three back-ends are supported:
- RosControlPlugin  – ros_control transmissions + MoveIt! srdf/config
- XBotCorePlugin    – XBotCore (legacy) low-level config
- XBot2Plugin       – XBot2 (current) hal/joint-config files

Usage inside UrdfWriter
-----------------------
The active plugin is stored as ``UrdfWriter.control_plugin``.  Every plugin
instance holds a back-reference to the writer via ``self.urdf_writer`` so it
can query the current robot state.

    plugin.urdf_writer = urdf_writer_instance   # set once at init/reset
    plugin.add_plugin()
    plugin.add_joint(joint_name)
    plugin.write_srdf()
    plugin.write_joint_map()
    plugin.write_lowlevel_config()
"""
# pylint: disable=line-too-long, missing-function-docstring

from __future__ import print_function
import os
import errno
import copy
import logging
import xml.etree.ElementTree as ET
import xml.dom.minidom
import yaml
from collections import OrderedDict
from abc import ABCMeta, abstractmethod

from modular.enums import ModuleType, ModuleClass
from modular.yaml_utils import ordered_load, ordered_dump
from modular.urdf_xml_builder import NS_XACRO, ns

# Output directory (same convention as the main writer)
_path_name = "/tmp"


# ---------------------------------------------------------------------------
# Abstract base
# ---------------------------------------------------------------------------

class Plugin(metaclass=ABCMeta):

    @property
    def urdf_writer(self):
        return self._urdf_writer

    @urdf_writer.setter
    def urdf_writer(self, writer):
        self._urdf_writer = writer

    @abstractmethod
    def add_plugin(self):
        pass

    @abstractmethod
    def add_joint(self):
        pass

    @abstractmethod
    def remove_joint(self):
        pass

    # SRDF
    @abstractmethod
    def add_gripper_to_srdf(self):
        pass

    @abstractmethod
    def add_wheel_to_srdf(self):
        pass


    # TODO: This should be fixed. Should not be here, probably a SRDFwriter class could be implemented
    def write_srdf(self, builder_joint_map=None):
        """Generates a basic srdf so that the model can be used right away with XBotCore"""

        root = ET.Element('robot', name="ModularBot")

        active_modules_chains = []

        chains = []
        joints = []
        wheels = []
        end_effectors = []
        groups_in_chains_group = []
        groups_in_arms_group = []

        active_modules_chains = self.urdf_writer.get_actuated_modules_chains()
        self.urdf_writer.print(active_modules_chains)
        
        # Create groups for chains, arms and wheels. Also 'home_group_state' for homing
        chains_group = ET.SubElement(root, 'group', name="chains")
        arms_group = ET.SubElement(root, 'group', name="arms")
        hands_group = ET.SubElement(root, 'group', name="hands")
        wheels_group = ET.SubElement(root, 'group', name="wheels")
        home_group_state = ET.SubElement(root, 'group_state', name="home", group="chains")

        for joints_chain in active_modules_chains:
            # create chain group and add as child the chain element (base_link, tip_link)
            group_name = "chain" + self.urdf_writer.find_chain_tag(joints_chain)
            chain_group = ET.Element('group', name=group_name)
            base_link = self.urdf_writer.find_chain_base_link(joints_chain)
            tip_link = self.urdf_writer.find_chain_tip_link(joints_chain)
            ET.SubElement(chain_group, 'chain', base_link=base_link, tip_link=tip_link)
            # add chain group to srdf chains group. (Will be added to srdf root at the end, so it will appear first in the srdf file) 
            chains.append(chain_group)
            
            for joint_module in joints_chain:
                # add wheel module to srdf wheels group
                if joint_module.type is ModuleType.WHEEL:
                    wheels += filter(lambda item: item is not None, [self.add_wheel_to_srdf(wheels_group, joint_module.name)])
                
                # add homing state for each joint-like module. Also create custom groups for some end-effectors
                if joint_module.type in ModuleClass.joint_modules() | {ModuleType.DAGANA}:
                    joint_name = self.urdf_writer.get_joint_name(joint_module)
                    if builder_joint_map is not None:
                        homing_value = float(builder_joint_map[joint_name])
                    else:
                        homing_value = 0.1
                    joints.append(ET.SubElement(home_group_state, 
                                                'joint', 
                                                name=joint_name, 
                                                value=str(homing_value)))
                    
        index = 0
        for idx, group in enumerate(chains):
            # insert ET.Element group in root at index idx (at the top of srdf xml)
            root.insert(idx, group)
            # add chain group to srdf chains group
            groups_in_chains_group.append(ET.SubElement(chains_group, 'group', name=group.attrib['name']))
            # save last index
            index = idx
        for idx, group in enumerate(end_effectors):
            # insert ET.Element group in root at index idx (at the top of srdf xml)
            root.insert(index + idx + 1, group)
            # add chain group to srdf chains group
            groups_in_chains_group.append(ET.SubElement(chains_group, 'group', name=group.attrib['name']))

        xmlstr = xml.dom.minidom.parseString(ET.tostring(root)).toprettyxml(indent="   ")

        return xmlstr

    # JOINT MAP
    def write_joint_map(self, use_robot_id=False):
        """Creates the joint map needed by XBotCore """

        jointmap_filename = _path_name + '/ModularBot/joint_map/ModularBot_joint_map.yaml'
        i = 0
        joint_map = {'joint_map': {}}
        for hub_module in self.urdf_writer.listofhubs:
            i += 1
            if use_robot_id:
                joint_map['joint_map'][int(hub_module.robot_id)] = "HUB_" + str(hub_module.robot_id)
            else:
                joint_map['joint_map'][i] = "HUB_" + str(i)
        for joints_chain in self.urdf_writer.listofchains:
            for joint_module in joints_chain:
                if joint_module.type in ModuleClass.nonactuated_modules():
                    continue
                i += 1
                if joint_module.type is ModuleType.TOOL_EXCHANGER:
                    name = joint_module.name + '_fixed_joint'
                elif joint_module.type is ModuleType.GRIPPER:
                    name = joint_module.name + '_fixed_joint'
                else:
                    name = joint_module.name

                if use_robot_id:
                    joint_map['joint_map'][int(joint_module.robot_id)] = name
                else:
                    joint_map['joint_map'][i] = name

        # Create folder if doesen't exist
        if not os.path.exists(os.path.dirname(jointmap_filename)):
            try:
                os.makedirs(os.path.dirname(jointmap_filename))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        with open(jointmap_filename, 'w+') as outfile:
            yaml.dump(joint_map, outfile, default_flow_style=False)
        return joint_map

    # CONFIG
    def write_lowlevel_config(self):
        pass


# ---------------------------------------------------------------------------
# RosControl back-end
# ---------------------------------------------------------------------------

class RosControlPlugin(Plugin):
    def add_plugin(self):
        return ET.SubElement(self.urdf_writer.root, "xacro:plugin_ros_control")

    def add_joint(self, joint_name):
        ET.SubElement(self.urdf_writer.root, "xacro:ros_control_transmission",
                      transmission=joint_name+'_tran',
                      joint=joint_name,
                      motor=joint_name+'_mot')

    def remove_joint(self, joint_name):
        for transmission in self.urdf_writer.root.findall('*[@transmission]', ns):
            if transmission.attrib['joint'] == joint_name:
                self.urdf_writer.root.remove(transmission)

    # SRDF
    def add_gripper_to_srdf(self, et_root, module, hand_name, parent_group_name):
        hand_group = ET.SubElement(et_root, "group", name=hand_name)
        if module.type is ModuleType.GRIPPER:
            ET.SubElement(hand_group, "link", name=module.name)
            ET.SubElement(hand_group, "link", name=module.name+"_leftfinger")
            ET.SubElement(hand_group, "link", name=module.name+"_rightfinger")
            ET.SubElement(hand_group, "joint", name=module.name+"_finger_joint1")
            ET.SubElement(hand_group, "passive_joint", name=module.name+"_finger_joint2")
            open_state = ET.SubElement(et_root, "group_state", name="open", group=hand_name)
            ET.SubElement(open_state, "joint", name=module.name+"_finger_joint1", value="0.05")
            ET.SubElement(open_state, "joint", name=module.name+"_finger_joint2", value="0.05")
            close_state = ET.SubElement(et_root, "group_state", name="close", group=hand_name)
            ET.SubElement(close_state, "joint", name=module.name+"_finger_joint1", value="0.0")
            ET.SubElement(close_state, "joint", name=module.name+"_finger_joint2", value="0.0")
            # remove collisions
            ET.SubElement(et_root, "disable_collisions", link1=module.name, link2="TCP_"+module.name, reason="Adjacent")
            ET.SubElement(et_root, "disable_collisions", link1=module.name, link2=module.name+"_leftfinger", reason="Adjacent")
            ET.SubElement(et_root, "disable_collisions", link1=module.name, link2=module.name+"_rightfinger", reason="Adjacent")
            ET.SubElement(et_root, "disable_collisions", link1="TCP_"+module.name, link2=module.name+"_rightfinger", reason="Default")
            ET.SubElement(et_root, "disable_collisions", link1="TCP_"+module.name, link2=module.name+"_leftfinger",reason="Default")
            ET.SubElement(et_root, "disable_collisions", link1=module.name + "_rightfinger", link2=module.name+"_leftfinger", reason="Default")
        elif module.type is ModuleType.TOOL_EXCHANGER:
            tool_exchanger_group = ET.SubElement(et_root, 'group', name="ToolExchanger")
            ET.SubElement(tool_exchanger_group, 'joint', name=module.name + '_fixed_joint')


        endeffector_group = ET.SubElement(et_root, "end-effector", name="TCP", parent_link="TCP_"+module.name,
                      group=hand_name, parent_group=parent_group_name)
        # add arm_hand group
        arm_hand_group = ET.SubElement(et_root, "group", name="arm_" + hand_name)
        ET.SubElement(arm_hand_group, "group", name=parent_group_name)
        ET.SubElement(arm_hand_group, "group", name=hand_name)

        return endeffector_group

    def add_wheel_to_srdf(self, wheel_group_name, wheel_name):
        return None

    def write_srdf(self, builder_joint_map=None):
        """Generates a basic srdf so that the model can be used right away with XBotCore"""
        
        root = ET.Element('robot', name="ModularBot")

        groups = []
        chains = []
        joints = []
        end_effectors = []
        groups_in_chains_group = []
        groups_in_arms_group = []

        # MoveIt
        controller_list = []
        initial_poses = []
        hardware_interface_joints = []

        #kinematics.yaml
        template_kinematics_filename = self.urdf_writer.resource_finder.get_filename('moveit_config/kinematics.yaml', ['data_path'])
        kinematics_filename = _path_name + "/moveit_config/kinematics.yaml"
        tmp_kinematics = OrderedDict([])
        kinematics = OrderedDict([])
        with open(template_kinematics_filename, 'r') as stream:
            try:
                tmp_kinematics = ordered_load(stream, yaml.SafeLoader)
            except yaml.YAMLError as exc:
                self.urdf_writer.print(exc)

        #ompl_planning.yaml
        template_ompl_filename = self.urdf_writer.resource_finder.get_filename('moveit_config/ompl_planning.yaml', ['data_path'])
        ompl_filename = _path_name + "/moveit_config/ompl_planning.yaml"
        tmp_ompl = OrderedDict([])
        ompl = OrderedDict([])
        with open(template_ompl_filename, 'r') as stream:
            try:
                tmp_ompl = ordered_load(stream, yaml.SafeLoader)
            except yaml.YAMLError as exc:
                self.urdf_writer.print(exc)
        ompl.update([('planner_configs', copy.deepcopy(tmp_ompl['planner_configs']))])

        self.urdf_writer.print(self.urdf_writer.listofchains)
        for idx, joints_chain in enumerate(self.urdf_writer.listofchains):
            group_name = "arm" + self.urdf_writer.find_chain_tag(joints_chain)
            groups.append(ET.SubElement(root, 'group', name=group_name))
            base_link = self.urdf_writer.find_chain_base_link(joints_chain)
            tip_link = self.urdf_writer.find_chain_tip_link(joints_chain)
            chains.append(ET.SubElement(groups[idx], 'chain', base_link=base_link, tip_link=tip_link))
        arms_group = ET.SubElement(root, 'group', name="arms")
        group_state = ET.SubElement(root, 'group_state', name="home", group="chains")
        tool_exchanger_group = ET.SubElement(root, 'group', name="ToolExchanger")
        hands_group = ET.SubElement(root, 'group', name="hands")
        # MoveIt
        initial_poses.append(OrderedDict([('group', 'arms'), ('pose', 'home')]))
        for idx, joints_chain in enumerate(self.urdf_writer.listofchains):
            group_name = "arm" + self.urdf_writer.find_chain_tag(joints_chain)
            hand_name = "hand" + self.urdf_writer.find_chain_tag(joints_chain)
            groups_in_arms_group.append(ET.SubElement(arms_group, 'group', name=group_name))
            # MoveIt: create controller list
            controller_list.append(OrderedDict([('name', 'fake_'+group_name+'_controller'), ('joints', [])]))
            kinematics.update([(group_name, copy.deepcopy(tmp_kinematics['group_name']))])
            ompl.update([(group_name, copy.deepcopy(tmp_ompl['group_name']))])
            for joint_module in joints_chain:
                if joint_module.type in ModuleClass.joint_modules() | {ModuleType.DAGANA}:
                    # Homing state
                    if builder_joint_map is not None:
                        homing_value = float(builder_joint_map[joint_module.name])
                    else:
                        homing_value = 0.1
                    joints.append(ET.SubElement(group_state, 'joint', name=joint_module.name, value=str(homing_value)))
                    # MoveIt: add joints to controller
                    controller_list[idx]['joints'].append(joint_module.name)
                    hardware_interface_joints.append(joint_module.name)
                elif joint_module.type is ModuleType.TOOL_EXCHANGER:
                    tool_exchanger_group = ET.SubElement(root, 'group', name="ToolExchanger")
                    end_effectors.append(ET.SubElement(tool_exchanger_group, 'joint',
                                                       name=joint_module.name + '_fixed_joint'))
                elif joint_module.type is ModuleType.GRIPPER:
                    end_effectors += filter(lambda item: item is not None, [self.add_gripper_to_srdf(et_root=root, 
                                                                                                     module=joint_module,
                                                                                                     hand_name=hand_name, 
                                                                                                     parent_group_name=group_name)])
                    controller_list.append(OrderedDict([('name', 'fake_' + hand_name + '_controller'), ('joints', [])]))
                    controller_list[idx+1]['joints'].append(joint_module.name+'_finger_joint1')
                    controller_list[idx+1]['joints'].append(joint_module.name + '_finger_joint2')
                    hardware_interface_joints.append(joint_module.name + '_finger_joint1')
                    initial_poses.append(OrderedDict([('group', hand_name), ('pose', 'open')]))
                    ompl.update([(hand_name, copy.deepcopy(tmp_ompl['group_name']))])
                    ompl.update([('arm_'+hand_name, copy.deepcopy(tmp_ompl['group_name']))])

        # MoveIt disable collisions
        for coll_elem in self.urdf_writer.collision_elements:
            ET.SubElement(root, 'disable_collisions', link1=coll_elem[0], link2=coll_elem[1], reason='Adjacent')

        xmlstr = xml.dom.minidom.parseString(ET.tostring(root)).toprettyxml(indent="   ")

        fake_controllers_filename = _path_name + "/moveit_config/fake_controllers.yaml"
        fake_controllers = OrderedDict([('controller_list', controller_list)])
        fake_controllers.update({'initial': initial_poses})

        # Create folder if doesen't exist
        if not os.path.exists(os.path.dirname(fake_controllers_filename)):
            try:
                os.makedirs(os.path.dirname(fake_controllers_filename))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        with open(fake_controllers_filename, 'w') as outfile:
            ordered_dump(fake_controllers, stream=outfile, default_flow_style=False, line_break='\n\n', indent=4,
                         canonical=False)

        with open(kinematics_filename, 'w') as outfile:
            ordered_dump(kinematics, stream=outfile, default_flow_style=False, line_break='\n\n', indent=4,
                         canonical=False)

        with open(ompl_filename, 'w') as outfile:
            ordered_dump(ompl, stream=outfile, default_flow_style=False, line_break='\n\n', indent=4,
                         canonical=False)

        # ros_controllers.launch
        ros_controllers_launch = _path_name + "/launch/ros_controllers.launch"
        launch_root = ET.Element('launch')
        ET.SubElement(launch_root, "rosparam", file="$(find pino_moveit)/moveit_config/ros_controllers.yaml",
                      command="load")
        controller_list_str = ' '.join((ctrl['name'].replace('fake_', '') for ctrl in controller_list))
        ET.SubElement(launch_root, "node", name="controller_spawner", pkg="controller_manager", type="spawner",
                      respawn="false", output="screen", args="joint_state_controller "+controller_list_str)

        # Create folder if doesen't exist
        if not os.path.exists(os.path.dirname(ros_controllers_launch)):
            try:
                os.makedirs(os.path.dirname(ros_controllers_launch))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        xmlstr_launch = xml.dom.minidom.parseString(ET.tostring(launch_root)).toprettyxml(indent="   ")
        with open(ros_controllers_launch, 'w+') as f:
            f.write(xmlstr_launch)

        return xmlstr


# ---------------------------------------------------------------------------
# XBotCore (legacy) back-end
# ---------------------------------------------------------------------------

class XBotCorePlugin(Plugin):
    def add_plugin(self):
        pass

    def add_joint(self, joint_name):
        pass

    def remove_joint(self, joint_name):
        pass

    # SRDF
    def add_gripper_to_srdf(self, et_root, module, hand_name, parent_group_name):
        return None

    def add_wheel_to_srdf(self, wheel_group_name, wheel_name):
        return None

    # CONFIG
    def write_lowlevel_config(self, use_robot_id=False):
        """Creates the low level config file needed by XBotCore """

        basic_config_filename = self.urdf_writer.resource_finder.get_filename('configs/ModularBot.yaml', ['data_path'])
        lowlevel_config_filename = _path_name + '/ModularBot/configs/ModularBot.yaml'
        lowlevel_config = OrderedDict([])

        with open(basic_config_filename, 'r') as stream:
            try:
                lowlevel_config = ordered_load(stream, yaml.SafeLoader)
                self.urdf_writer.print(list(lowlevel_config.items())[0])
            except yaml.YAMLError as exc:
                self.urdf_writer.print(exc)

        self.urdf_writer.print(lowlevel_config.items())
        self.urdf_writer.print(lowlevel_config['GazeboXBotPlugin'])
        lowlevel_config['GazeboXBotPlugin']['gains'] = OrderedDict([])
        i = 0
        p = 0
        for joints_chain in self.urdf_writer.listofchains:
            # HACK
            p += 1
            for joint_module in joints_chain:
                if joint_module.type in ModuleClass.nonactuated_modules():
                    continue
                if joint_module.type in ModuleClass.joint_modules():
                    i += 1
                    lowlevel_config['GazeboXBotPlugin']['gains'][joint_module.name] = OrderedDict(
                        [('p', 300), ('d', 20)])
                    if use_robot_id:
                        key = 'CentAcESC_' + str(joint_module.robot_id)
                    else:
                        key = 'CentAcESC_' + str(i)
                    value = joint_module.CentAcESC
                    self.urdf_writer.print(yaml.dump(joint_module.CentAcESC))
                    # HACK: Every joint on 2nd, 3rd, etc. chains have the torque loop damping set very low.
                    if p > 1:
                        value.pid.impedance = [500.0, 20.0, 1.0, 0.003, 0.99]
                elif joint_module.type is ModuleType.TOOL_EXCHANGER:
                    if use_robot_id:
                        key = 'AinMsp432ESC_' + str(joint_module.robot_id)
                        xbot_ecat_interface = [[int(joint_module.robot_id)], "libXBotEcat_ToolExchanger"]
                    else:
                        key = 'AinMsp432ESC_' + str(i)
                        xbot_ecat_interface = [[int(i)], "libXBotEcat_ToolExchanger"]
                    value = joint_module.AinMsp432ESC
                    self.urdf_writer.print(yaml.dump(joint_module.AinMsp432ESC))
                    lowlevel_config['HALInterface']['IEndEffectors'].append(xbot_ecat_interface)
                elif joint_module.type is ModuleType.GRIPPER:
                    if use_robot_id:
                        key = 'LpESC_' + str(joint_module.robot_id)
                        xbot_ecat_interface = [[int(joint_module.robot_id)], "libXBotEcat_Gripper"]
                    else:
                        key = 'LpESC_' + str(i)
                        xbot_ecat_interface = [[int(i)], "libXBotEcat_Gripper"]
                    value = joint_module.LpESC
                    self.urdf_writer.print(yaml.dump(joint_module.LpESC))
                    lowlevel_config['HALInterface']['IEndEffectors'].append(xbot_ecat_interface)

                lowlevel_config[key] = value
                self.urdf_writer.print(joint_module.kinematics.__dict__.items())
                self.urdf_writer.print(lowlevel_config[key])

        # Create folder if doesen't exist
        if not os.path.exists(os.path.dirname(lowlevel_config_filename)):
            try:
                os.makedirs(os.path.dirname(lowlevel_config_filename))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        with open(lowlevel_config_filename, 'w') as outfile:
            ordered_dump(lowlevel_config, stream=outfile, default_flow_style=False, line_break='\n\n', indent=4,
                         canonical=False)
        return lowlevel_config


# ---------------------------------------------------------------------------
# XBot2 (current) back-end
# ---------------------------------------------------------------------------

class XBot2Plugin(Plugin):
    def add_plugin(self):
        self.gazebo_node = ET.SubElement(self.urdf_writer.root, "gazebo")
        self.plugin_node = ET.SubElement(self.gazebo_node, "plugin",
                                         name="gz::sim::systems::GzJointServer",
                                         filename="libxbot2_gz_joint_server.so")
        self.gain_node = ET.SubElement(self.plugin_node, "profile", name='small_mot', p='100', d='10')
        self.gain_node = ET.SubElement(self.plugin_node, "profile", name='medium_mot', p='500', d='50')
        self.gain_node = ET.SubElement(self.plugin_node, "profile", name='big_mot', p='1000', d='100')
        return self.gain_node

    def add_joint(self, joint_name, control_params=None):
        if control_params is not None:
            if hasattr(control_params, 'pid'):
                pid_node = ET.SubElement(self.plugin_node, "pid", name=joint_name, p=str(control_params.pid.p), d=str(control_params.pid.d))
            if hasattr(control_params, 'profile'):
                pid_node = ET.SubElement(self.plugin_node, "pid", name=joint_name, profile=str(control_params.profile))
        else:
            pid_node = ET.SubElement(self.plugin_node, "pid", name=joint_name, profile="medium_mot")
        return pid_node

    def remove_joint(self, joint_name):
        for pid in self.plugin_node.findall('./pid'):
            if pid.attrib['name'] == joint_name:
                self.plugin_node.remove(pid)

    # SRDF
    def add_gripper_to_srdf(self, et_root, module, hand_name, parent_group_name):

        # Create custom group for the tool exchanger
        if module.type is ModuleType.TOOL_EXCHANGER:
            tool_exchanger_group = ET.SubElement(et_root, 'group', name="ToolExchanger")
            ET.SubElement(tool_exchanger_group, 'joint', name=module.name + '_fixed_joint')

        # If the module has no fingers, return
        if len(module.finger_names) == 0:
            return None

        # Create chain group for the gripper
        chain_group = ET.Element('group', name=hand_name)
        for finger in module.finger_names:
            base_link = module.base_link_name
            tip_link = finger
            ET.SubElement(chain_group, 'chain', base_link=base_link, tip_link=tip_link)
        return chain_group

    def add_wheel_to_srdf(self, wheel_group, wheel_name):
        wheel = ET.SubElement(wheel_group, 'joint', name=wheel_name)
        return wheel

    # JOINT MAP
    def write_joint_map(self, use_robot_id=False):
        """Creates the joint map needed by XBotCore """

        jointmap_filename = _path_name + '/ModularBot/joint_map/ModularBot_joint_map.yaml'
        i = 0
        joint_map = {'joint_map': {}, 'albero_gripper_map': {}}

        for hub_module in self.urdf_writer.listofhubs:
            i += 1
            if use_robot_id:
                joint_map['joint_map'][int(hub_module.robot_id)] = "HUB_" + str(hub_module.robot_id)
            else:
                joint_map['joint_map'][i] = "HUB_" + str(i)
        for joints_chain in self.urdf_writer.listofchains:
            for joint_module in joints_chain:
                if joint_module.type in ModuleClass.nonactuated_modules():
                    continue
                i += 1
                if joint_module.type is ModuleType.TOOL_EXCHANGER:
                    name = joint_module.name + '_fixed_joint'
                elif joint_module.type is ModuleType.GRIPPER:
                    name = joint_module.name
                    fingers = [name + '_rightfinger', name + '_leftfinger']
                    if use_robot_id:
                        joint_map['albero_gripper_map'][int(joint_module.robot_id)] = {'name': name, 'fingers': fingers}
                    else:
                        joint_map['albero_gripper_map'][i] = {'name': name, 'fingers': fingers}
                else:
                    name = self.urdf_writer.get_joint_name(joint_module)
                
                if use_robot_id:
                    joint_map['joint_map'][int(joint_module.robot_id)] = name
                else:
                    joint_map['joint_map'][i] = name

        # Create folder if doesen't exist
        if not os.path.exists(os.path.dirname(jointmap_filename)):
            try:
                os.makedirs(os.path.dirname(jointmap_filename))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        with open(jointmap_filename, 'w+') as outfile:
            yaml.dump(joint_map, outfile, default_flow_style=False)
        return joint_map

    # CONFIG
    def write_lowlevel_config(self, use_robot_id=False):
        """Creates the low level config file needed by XBotCore """
        # HAL config ModularBot_ec_all
        hal_config_template = self.urdf_writer.resource_finder.get_filename('configs/low_level/hal/ModularBot_ec_all.yaml', ['data_path'])
        hal_config_filename = _path_name + '/ModularBot/config/hal/ModularBot_ec_all.yaml'
        hal_config = OrderedDict([])

        with open(hal_config_template, 'r') as stream:
            try:
                hal_config = ordered_load(stream, yaml.SafeLoader)
                self.urdf_writer.print(list(hal_config.items())[0])
            except yaml.YAMLError as exc:
                self.urdf_writer.print(exc)

        self.urdf_writer.print(hal_config['xbotcore_devices']['joint_ec']['params'].items())
        ids = []
        i = 0
        for joints_chain in self.urdf_writer.listofchains:
            for joint_module in joints_chain:
                if joint_module.type in ModuleClass.nonactuated_modules():
                    continue
                i += 1
                if joint_module.type in {ModuleType.TOOL_EXCHANGER, ModuleType.GRIPPER}:
                    if use_robot_id:
                        mod_id = str(joint_module.robot_id)
                    else:
                        mod_id = str(i)
                    ids.append(mod_id)
        ignore_id = OrderedDict({'ignore_id': {'type': 'vector<int>', 'value': ids}})
        hal_config['xbotcore_devices']['joint_ec']['params'].update(ignore_id)

        #joint_gripper_adapter
        i = 0
        for joints_chain in self.urdf_writer.listofchains:
            for joint_module in joints_chain:
                if joint_module.type is ModuleType.DAGANA:
                    
                    attrs = [a for a in dir(joint_module.joint_gripper_adapter) if not a.startswith('__') and not callable(getattr(joint_module.joint_gripper_adapter, a))]
                    attrs_with_prefix = [joint_module.name+"/" + x for x in attrs]
                    params_dict = {i:getattr(joint_module.joint_gripper_adapter, j) for i, j in zip(attrs_with_prefix, attrs)}
                    params_dict.update({joint_module.name+"/joint_name": {"value": joint_module.dagana_joint_name, "type": "string"}})
                    joint_gripper_adapter_params = OrderedDict(params_dict)
                    hal_config['xbotcore_devices']['joint_gripper_adapter']['params'].update(joint_gripper_adapter_params)

                    hal_config['xbotcore_devices']['joint_gripper_adapter']['names'].append(joint_module.name)

        # Create folder if doesen't exist
        if not os.path.exists(os.path.dirname(hal_config_filename)):
            try:
                os.makedirs(os.path.dirname(hal_config_filename))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        with open(hal_config_filename, 'w') as outfile:
            ordered_dump(hal_config, stream=outfile, default_flow_style=False, line_break='\n\n', indent=4,
                         canonical=False)

        # HAL config ModularBot_idle
        idle_joint_config_template = self.urdf_writer.resource_finder.get_filename(
            'configs/low_level/joint_config/ModularBot_idle.yaml', ['data_path'])
        idle_joint_config_filename = _path_name + '/ModularBot/config/joint_config/ModularBot_idle.yaml'
        # HAL config ModularBot_impd4
        impd4_joint_config_template = self.urdf_writer.resource_finder.get_filename(
            'configs/low_level/joint_config/ModularBot_impd4.yaml', ['data_path'])
        impd4_joint_config_filename = _path_name + '/ModularBot/config/joint_config/ModularBot_impd4.yaml'
        # HAL config ModularBot_pos3b
        pos3b_joint_config_template = self.urdf_writer.resource_finder.get_filename(
            'configs/low_level/joint_config/ModularBot_pos3b.yaml', ['data_path'])
        pos3b_joint_config_filename = _path_name + '/ModularBot/config/joint_config/ModularBot_pos3b.yaml'
        # XBot2 config
        xbot2_config_template = self.urdf_writer.resource_finder.get_filename(
            'configs/ModularBot_xbot2.yaml', ['data_path'])
        xbot2_config_filename = _path_name + '/ModularBot/config/ModularBot.yaml'

        idle_joint_config = OrderedDict([])
        impd4_joint_config = OrderedDict([])
        pos3b_joint_config = OrderedDict([])
        xbot2_config = OrderedDict([])

        with open(idle_joint_config_template, 'r') as stream:
            try:
                idle_joint_config = ordered_load(stream, yaml.SafeLoader)
                self.urdf_writer.print(list(idle_joint_config.items())[0])
            except yaml.YAMLError as exc:
                self.urdf_writer.print(exc)
        with open(impd4_joint_config_template, 'r') as stream:
            try:
                impd4_joint_config = ordered_load(stream, yaml.SafeLoader)
                self.urdf_writer.print(list(impd4_joint_config.items())[0])
            except yaml.YAMLError as exc:
                self.urdf_writer.print(exc)
        with open(pos3b_joint_config_template, 'r') as stream:
            try:
                pos3b_joint_config = ordered_load(stream, yaml.SafeLoader)
                self.urdf_writer.print(list(pos3b_joint_config.items())[0])
            except yaml.YAMLError as exc:
                self.urdf_writer.print(exc)

        i = 0
        p = 0
        for joints_chain in self.urdf_writer.listofchains:
            # HACK
            p += 1
            for joint_module in joints_chain:
                if joint_module.type in ModuleClass.nonactuated_modules():
                    continue
                if joint_module.type in {ModuleType.JOINT, ModuleType.DAGANA}:
                    key = self.urdf_writer.get_joint_name(joint_module)
                    value = joint_module.CentAcESC
                    # Remove parameters that are now not used by XBot2
                    if hasattr(value, 'sign'):
                        del value.sign
                    if hasattr(value, 'pos_offset'):
                        del value.pos_offset
                    if hasattr(value, 'max_current_A'):
                        del value.max_current_A

                    impd4_joint_config[key] = copy.deepcopy(value)
                    impd4_joint_config[key].control_mode = 'D4_impedance_ctrl'
                    if hasattr(impd4_joint_config[key], 'pid'):
                        if hasattr(impd4_joint_config[key].pid, 'position'):
                            del impd4_joint_config[key].pid.position

                    pos3b_joint_config[key] = copy.deepcopy(value)
                    pos3b_joint_config[key].control_mode = '3B_motor_pos_ctrl'
                    if hasattr(pos3b_joint_config[key], 'pid'):
                        if hasattr(pos3b_joint_config[key].pid, 'impedance'):
                            del pos3b_joint_config[key].pid.impedance

                    idle_joint_config[key] = copy.deepcopy(value)
                    idle_joint_config[key].control_mode = 'idle'
                    if hasattr(idle_joint_config[key], 'pid'):
                        if hasattr(idle_joint_config[key].pid, 'position'):
                            del idle_joint_config[key].pid.position
                    if hasattr(idle_joint_config[key], 'pid'):
                        if hasattr(idle_joint_config[key].pid, 'impedance'):
                            del idle_joint_config[key].pid.impedance
                    if hasattr(idle_joint_config[key], 'pid'):
                        if hasattr(idle_joint_config[key].pid, 'velocity'):
                            del idle_joint_config[key].pid.velocity

                elif joint_module.type is ModuleType.WHEEL:
                    key = self.urdf_writer.get_joint_name(joint_module)
                    value = joint_module.CentAcESC
                    if hasattr(value, 'sign'):
                        del value.sign
                    if hasattr(value, 'pos_offset'):
                        del value.pos_offset
                    if hasattr(value, 'max_current_A'):
                        del value.max_current_A

                    impd4_joint_config[key] = copy.deepcopy(value)
                    impd4_joint_config[key].control_mode = '71_motor_vel_ctrl'

                    pos3b_joint_config[key] = copy.deepcopy(value)
                    pos3b_joint_config[key].control_mode = '71_motor_vel_ctrl'

                    idle_joint_config[key] = copy.deepcopy(value)
                    idle_joint_config[key].control_mode = 'idle'
                    if hasattr(idle_joint_config[key], 'pid'):
                        if hasattr(idle_joint_config[key].pid, 'velocity'):
                            del idle_joint_config[key].pid.velocity

                elif joint_module.type is ModuleType.TOOL_EXCHANGER:
                    key = joint_module.name
                    value = joint_module.AinMsp432ESC

                elif joint_module.type is ModuleType.GRIPPER:
                    key = joint_module.name + '_motor'
                    value = joint_module.LpESC
                    if hasattr(value, 'sign'):
                        del value.sign
                    if hasattr(value, 'pos_offset'):
                        del value.pos_offset
                    if hasattr(value, 'max_current_A'):
                        del value.max_current_A

                    impd4_joint_config[key] = copy.deepcopy(value)
                    impd4_joint_config[key].control_mode = '3B_motor_pos_ctrl'

                    idle_joint_config[key] = copy.deepcopy(value)
                    idle_joint_config[key].control_mode = 'idle'

        with open(xbot2_config_template, 'r') as stream:
            try:
                xbot2_config = ordered_load(stream, yaml.SafeLoader)
                self.urdf_writer.print(list(idle_joint_config.items())[0])
            except yaml.YAMLError as exc:
                self.urdf_writer.print(exc)

        if self.urdf_writer.floating_base:
            xbot2_config['ModelInterface']['is_model_floating_base'] = 'true'

        # Create folders if they don't exist
        for fname in [idle_joint_config_filename, impd4_joint_config_filename,
                      pos3b_joint_config_filename, xbot2_config_filename]:
            if not os.path.exists(os.path.dirname(fname)):
                try:
                    os.makedirs(os.path.dirname(fname))
                except OSError as exc:  # Guard against race condition
                    if exc.errno != errno.EEXIST:
                        raise

        with open(idle_joint_config_filename, 'w') as outfile:
            ordered_dump(idle_joint_config, stream=outfile, default_flow_style=False, line_break='\n\n', indent=4,
                         canonical=False)

        with open(impd4_joint_config_filename, 'w') as outfile:
            ordered_dump(impd4_joint_config, stream=outfile, default_flow_style=False, line_break='\n\n', indent=4,
                         canonical=False)

        with open(pos3b_joint_config_filename, 'w') as outfile:
            ordered_dump(pos3b_joint_config, stream=outfile, default_flow_style=False, line_break='\n\n', indent=4,
                         canonical=False)

        with open(xbot2_config_filename, 'w') as outfile:
            ordered_dump(xbot2_config, stream=outfile, default_flow_style=False, line_break='\n\n', indent=4,
                         canonical=False)

        return hal_config
