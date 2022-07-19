from __future__ import print_function
from future.utils import iteritems
from abc import ABCMeta, abstractmethod
import os
import logging


# try:
#     from lxml import etree as ET
#     #print("running with lxml.etree")
# except ImportError:
try:
    import xml.etree.ElementTree as ET
    # print("running with ElementTree on Python 2.5+")
except ImportError:
    # print("Failed to import ElementTree from any known place")
    pass

import xacro
import xml.dom.minidom
import codecs
import yaml
import json
import copy
from collections import OrderedDict

from modular.utils import ResourceFinder
import modular.ModuleNode  as ModuleNode# import module_from_yaml, ModuleNode, mastercube_from_yaml, slavecube_from_yaml
import argparse

import rospy
import roslaunch
import rospkg
import tf

# from anytree import NodeMixin, RenderTree, Node, AsciiStyle
import anytree
from anytree import RenderTree

import subprocess
from shutil import copyfile
import os
import errno
import sys
currDir = os.path.dirname(os.path.realpath(__file__))
# print(currDir)
rootDir = os.path.abspath(os.path.join(currDir, '../..'))
# print(rootDir)
if rootDir not in sys.path:  # add parent dir to paths
    sys.path.append(rootDir)

NS_XACRO = "http://www.ros.org/wiki/xacro"
ET.register_namespace("xacro", NS_XACRO)
ns = {"xacro": NS_XACRO}

import modular
path_name = os.path.dirname(modular.__file__)
# print(path_name)
path_superbuild = os.path.abspath(os.path.join(path_name, '../..'))
# print(path_superbuild)
# #obtaining tree from base file
# resource_path = '/'.join(('modular_data', 'urdf/ModularBot.library.urdf.xacro'))
# basefile_name = pkg_resources.resource_string(resource_package, resource_path)
# urdf_tree = ET.parse(basefile_name)

# Use /tmp folder to store urdf, srdf, etc.
path_name = "/tmp"

# noinspection PyPep8Naming
def ordered_load(stream, Loader=yaml.SafeLoader, object_pairs_hook=OrderedDict):
    class OrderedLoader(Loader):
        pass

    def construct_mapping(loader, node):
        loader.flatten_mapping(node)
        return object_pairs_hook(loader.construct_pairs(node))

    OrderedLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
        construct_mapping)

    return yaml.load(stream, OrderedLoader)


class MyDumper(yaml.Dumper):

    def increase_indent(self, flow=False, indentless=False):
        return super(MyDumper, self).increase_indent(flow, False)


# noinspection PyPep8Naming
def ordered_dump(data, stream=None, Dumper=MyDumper, **kwds):
    class OrderedDumper(Dumper):
        pass

    def _dict_representer(dumper, data):
        return dumper.represent_mapping(
            yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
            data.items())

    OrderedDumper.add_representer(OrderedDict, _dict_representer)

    # to avoid printing tags. TODO: Find a better way to do this. This changes the global yaml emitter
    def noop(self, *args, **kw):
        pass

    yaml.emitter.Emitter.process_tag = noop

    return yaml.dump(data, stream, OrderedDumper, **kwds)


def repl_option():
    parser = argparse.ArgumentParser()
    # parser.add_argument("-f", "--file_yaml", dest="esc_type_yaml", action="store", default="esc_type.yaml")
    parser.add_argument("-f", "--file_yaml", dest="robot_id_yaml", action="store", default="./robot_id.yaml")
    parser.add_argument("-c", dest="cmd_exec_cnt", action="store", type=int, default=1)
    args = parser.parse_args()
    dict_opt = vars(args)
    return dict_opt


class Plugin:
    __metaclass__ = ABCMeta

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
        None

    @abstractmethod
    def add_hand_group_to_srdf(self):
        pass

    # TODO: This should be fixed. Should not be here, probably a SRDFwriter class could be implemented
    def write_srdf(self, builder_joint_map=None):
        """Generates a basic srdf so that the model can be used right away with XBotCore"""
        # global path_name
        srdf_filename = path_name + '/ModularBot/srdf/ModularBot.srdf'
        # srdf_filename = path_superbuild + '/configs/ADVR_shared/ModularBot/srdf/ModularBot.srdf'
        # srdf_filename = self.urdf_writer.resource_finder.get_filename('srdf/ModularBot.srdf', 'modularbot_path')
        # srdf_filename = "/tmp/modular/srdf/ModularBot.srdf"

        root = ET.Element('robot', name="ModularBot")

        groups = []
        chains = []
        joints = []
        end_effectors = []
        groups_in_chains_group = []
        groups_in_arms_group = []
        # base_link = ""
        # tip_link = ""
        i = 0

        self.urdf_writer.print(self.urdf_writer.listofchains)
        for joints_chain in self.urdf_writer.listofchains:
            group_name = "chain" + self.urdf_writer.branch_switcher.get(i + 1)
            # group_name = "arm" + self.urdf_writer.branch_switcher.get(i + 1)
            groups.append(ET.SubElement(root, 'group', name=group_name))
            if "con" in joints_chain[0].parent.name:
                base_link = joints_chain[0].parent.parent.name
            else:
                base_link = joints_chain[0].parent.name
            if joints_chain[-1].children:
                if "con" in joints_chain[-1].children[0].name:
                    tip_link = joints_chain[-1].children[0].children[0].name
                else:
                    tip_link = joints_chain[-1].children[0].name
            else:
                tip_link = 'L_' + str(joints_chain[-1].i) + joints_chain[-1].tag
                if joints_chain[-1].type == 'tool_exchanger':
                    # tip_link = joints_chain[-1].name
                    tip_link = joints_chain[-1].pen_name
                if joints_chain[-1].type == 'gripper':
                    # tip_link = joints_chain[-1].name
                    tip_link = joints_chain[-1].TCP_name
                elif joints_chain[-1].type == 'simple_ee':
                    tip_link = joints_chain[-1].name
            chains.append(ET.SubElement(groups[i], 'chain', base_link=base_link, tip_link=tip_link))
            i += 1
        i = 0
        arms_group = ET.SubElement(root, 'group', name="arms")
        chains_group = ET.SubElement(root, 'group', name="chains")
        group_state = ET.SubElement(root, 'group_state', name="home", group="chains")
        for joints_chain in self.urdf_writer.listofchains:
            group_name = "chain" + self.urdf_writer.branch_switcher.get(i + 1)
            hand_name = "hand" + self.urdf_writer.branch_switcher.get(i + 1)
            groups_in_chains_group.append(ET.SubElement(chains_group, 'group', name=group_name))
            groups_in_arms_group.append(ET.SubElement(arms_group, 'group', name=group_name))
            for joint_module in joints_chain:
                if joint_module.type == 'joint':
                    # Homing state
                    if builder_joint_map is not None:
                        homing_value = float(builder_joint_map[joint_module.name]['angle'])
                    else:
                        homing_value = 0.1
                    # self.urdf_writer.print(homing_value)
                    joints.append(ET.SubElement(group_state, 'joint', name=joint_module.name, value=str(homing_value)))
                elif joint_module.type == 'tool_exchanger':
                    tool_exchanger_group = ET.SubElement(root, 'group', name="ToolExchanger")
                    end_effectors.append(ET.SubElement(tool_exchanger_group, 'joint',
                                                       name=joint_module.name + '_fixed_joint'))
                elif joint_module.type == 'gripper':
                    self.add_hand_group_to_srdf(root, joint_module.name, hand_name)
                    # groups_in_hands_group.append(ET.SubElement(hands_group, 'group', name=hand_name))
                    end_effectors += filter(None, [
                        self.add_gripper_to_srdf(root, joint_module.name, hand_name, group_name)])
            i += 1

        # Create folder if doesen't exist
        if not os.path.exists(os.path.dirname(srdf_filename)):
            try:
                os.makedirs(os.path.dirname(srdf_filename))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        xmlstr = xml.dom.minidom.parseString(ET.tostring(root)).toprettyxml(indent="   ")
        with open(srdf_filename, 'w+') as f:
            f.write(xmlstr)

        return xmlstr

    # JOINT MAP
    def write_joint_map(self, use_robot_id=False):
        """Creates the joint map needed by XBotCore """

        # global path_name
        jointmap_filename = path_name + '/ModularBot/joint_map/ModularBot_joint_map.yaml'
        # jointmap_filename = path_superbuild + '/configs/ADVR_shared/ModularBot/joint_map/ModularBot_joint_map.yaml'
        ## jointmap_filename = self.urdf_writer.resource_finder.get_filename('joint_map/ModularBot_joint_map.yaml',
        ##                                                       'modularbot_path')
        #jointmap_filename="/tmp/modular/joint_map/ModularBot_joint_map.yaml"
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
                i += 1
                if joint_module.type == 'tool_exchanger':
                    name = joint_module.name + '_fixed_joint'
                elif joint_module.type == 'simple_ee':
                    continue
                elif joint_module.type == 'gripper':
                    name = joint_module.name + '_fixed_joint'
                else:
                    name = joint_module.name
                if use_robot_id:
                    joint_map['joint_map'][int(joint_module.robot_id)] = name
                else:
                    joint_map['joint_map'][i] = name
            # self.urdf_writer.print(str(i), joint_module.name)
            # self.urdf_writer.print(joint_map)

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
    def add_gripper_to_srdf(self, root, gripper_name, hand_name, parent_group_name):
        ET.SubElement(root, "end-effector", name="TCP", parent_link="TCP_"+gripper_name,
                      group=hand_name, parent_group=parent_group_name)
        # add arm_hand group
        arm_hand_group = ET.SubElement(root, "group", name="arm_" + hand_name)
        ET.SubElement(arm_hand_group, "group", name=parent_group_name)
        ET.SubElement(arm_hand_group, "group", name=hand_name)

    def add_hand_group_to_srdf(self, root, gripper_name, hand_name):
        hand_group = ET.SubElement(root, "group", name=hand_name)
        ET.SubElement(hand_group, "link", name=gripper_name)
        ET.SubElement(hand_group, "link", name=gripper_name+"_leftfinger")
        ET.SubElement(hand_group, "link", name=gripper_name+"_rightfinger")
        ET.SubElement(hand_group, "joint", name=gripper_name+"_finger_joint1")
        ET.SubElement(hand_group, "passive_joint", name=gripper_name+"_finger_joint2")
        open_state = ET.SubElement(root, "group_state", name="open", group=hand_name)
        ET.SubElement(open_state, "joint", name=gripper_name+"_finger_joint1", value="0.05")
        ET.SubElement(open_state, "joint", name=gripper_name+"_finger_joint2", value="0.05")
        close_state = ET.SubElement(root, "group_state", name="close", group=hand_name)
        ET.SubElement(close_state, "joint", name=gripper_name+"_finger_joint1", value="0.0")
        ET.SubElement(close_state, "joint", name=gripper_name+"_finger_joint2", value="0.0")
        # remove collisions
        ET.SubElement(root, "disable_collisions", link1=gripper_name, link2="TCP_"+gripper_name, reason="Adjacent")
        ET.SubElement(root, "disable_collisions", link1=gripper_name, link2=gripper_name+"_leftfinger", reason="Adjacent")
        ET.SubElement(root, "disable_collisions", link1=gripper_name, link2=gripper_name+"_rightfinger", reason="Adjacent")
        ET.SubElement(root, "disable_collisions", link1="TCP_"+gripper_name, link2=gripper_name+"_rightfinger", reason="Default")
        ET.SubElement(root, "disable_collisions", link1="TCP_"+gripper_name, link2=gripper_name+"_leftfinger",reason="Default")
        ET.SubElement(root, "disable_collisions", link1=gripper_name + "_rightfinger", link2=gripper_name+"_leftfinger", reason="Default")

        return hand_group

    def write_srdf(self, builder_joint_map=None):
        """Generates a basic srdf so that the model can be used right away with XBotCore"""
        # global path_name
        srdf_filename = path_name + '/ModularBot/srdf/ModularBot.srdf'
        # srdf_filename = path_superbuild + '/configs/ADVR_shared/ModularBot/srdf/ModularBot.srdf'
        ## srdf_filename = self.urdf_writer.resource_finder.get_filename('srdf/ModularBot.srdf', 'modularbot_path')
        #srdf_filename = "/tmp/modular/srdf/ModularBot.srdf"

        root = ET.Element('robot', name="ModularBot")

        groups = []
        chains = []
        joints = []
        end_effectors = []
        groups_in_chains_group = []
        groups_in_arms_group = []
        # groups_in_hands_group = []
        # base_link = ""
        # tip_link = ""
        i = 0

        # MoveIt
        controller_list = []
        initial_poses = []
        hardware_interface_joints = []

        #kinematics.yaml
        template_kinematics_filename = self.urdf_writer.resource_finder.get_filename('moveit_config/kinematics.yaml', 'data_path')
        kinematics_filename = path_name + "/moveit_config/kinematics.yaml"
        # kinematics_filename = "/tmp/modular/moveit_config/kinematics.yaml"
        tmp_kinematics = OrderedDict([])
        kinematics = OrderedDict([])
        with open(template_kinematics_filename, 'r') as stream:
            try:
                tmp_kinematics = ordered_load(stream, yaml.SafeLoader)
            except yaml.YAMLError as exc:
                self.urdf_writer.print(exc)

        #ompl_planning.yaml
        template_ompl_filename = self.urdf_writer.get_filename('moveit_config/ompl_planning.yaml', 'data_path')
        ompl_filename = path_name + "/moveit_config/ompl_planning.yaml"
        # ompl_filename = "/tmp/modular/moveit_config/ompl_planning.yaml"
        tmp_ompl = OrderedDict([])
        ompl = OrderedDict([])
        with open(template_ompl_filename, 'r') as stream:
            try:
                tmp_ompl = ordered_load(stream, yaml.SafeLoader)
            except yaml.YAMLError as exc:
                self.urdf_writer.print(exc)
        ompl.update([('planner_configs', copy.deepcopy(tmp_ompl['planner_configs']))])

        self.urdf_writer.print(self.urdf_writer.listofchains)
        for joints_chain in self.urdf_writer.listofchains:
            group_name = "arm" + self.urdf_writer.branch_switcher.get(i + 1)
            #group_name = "chain_"+str(i+1)
            groups.append(ET.SubElement(root, 'group', name=group_name))
            if "con" in joints_chain[0].parent.name:
                base_link = joints_chain[0].parent.parent.name
            else:
                base_link = joints_chain[0].parent.name
            if joints_chain[-1].children:
                if "con" in joints_chain[-1].children[0].name:
                    tip_link = joints_chain[-1].children[0].children[0].name
                else:
                    tip_link = joints_chain[-1].children[0].name
            else:
                tip_link = 'L_' + str(joints_chain[-1].i) + joints_chain[-1].tag
                if joints_chain[-1].type == 'tool_exchanger':
                    # tip_link = joints_chain[-1].name
                    tip_link = joints_chain[-1].pen_name
                if joints_chain[-1].type == 'gripper':
                    # tip_link = joints_chain[-1].name
                    tip_link = joints_chain[-1].TCP_name
                elif joints_chain[-1].type == 'simple_ee':
                    tip_link = joints_chain[-1].name
            chains.append(ET.SubElement(groups[i], 'chain', base_link=base_link, tip_link=tip_link))
            i += 1
        i = 0
        arms_group = ET.SubElement(root, 'group', name="arms")
        #chains_group = ET.SubElement(root, 'group', name="chains")
        group_state = ET.SubElement(root, 'group_state', name="home", group="chains")
        tool_exchanger_group = ET.SubElement(root, 'group', name="ToolExchanger")
        hands_group = ET.SubElement(root, 'group', name="hands")
        # MoveIt
        initial_poses.append(OrderedDict([('group', 'arms'), ('pose', 'home')]))
        for joints_chain in self.urdf_writer.listofchains:
            #group_name = "chain_"+str(i+1)
            #groups_in_chains_group.append(ET.SubElement(chains_group, 'group', name=group_name))
            group_name = "arm" + self.urdf_writer.branch_switcher.get(i + 1)
            hand_name = "hand" + self.urdf_writer.branch_switcher.get(i + 1)
            groups_in_arms_group.append(ET.SubElement(arms_group, 'group', name=group_name))
            # MoveIt: create controller list
            controller_list.append(OrderedDict([('name', 'fake_'+group_name+'_controller'), ('joints', [])]))
            kinematics.update([(group_name, copy.deepcopy(tmp_kinematics['group_name']))])
            ompl.update([(group_name, copy.deepcopy(tmp_ompl['group_name']))])
            for joint_module in joints_chain:
                if joint_module.type == 'joint':
                    # Homing state
                    if builder_joint_map is not None:
                        homing_value = float(builder_joint_map[joint_module.name]['angle'])
                    else:
                        homing_value = 0.1
                    #self.urdf_writer.print(homing_value)
                    joints.append(ET.SubElement(group_state, 'joint', name=joint_module.name, value=str(homing_value)))
                    # Disable collision
                    # disable_collision = ET.SubElement(root, 'disable_collisions', link1=joint_module.stator_name, link2=joint_module.distal_link_name, reason='Adjacent')
                    # MoveIt: add joints to controller
                    controller_list[i]['joints'].append(joint_module.name)
                    hardware_interface_joints.append(joint_module.name)
                elif joint_module.type == 'tool_exchanger':
                    tool_exchanger_group = ET.SubElement(root, 'group', name="ToolExchanger")
                    end_effectors.append(ET.SubElement(tool_exchanger_group, 'joint',
                                                       name=joint_module.name + '_fixed_joint'))
                elif joint_module.type == 'gripper':
                    self.add_hand_group_to_srdf(root, joint_module.name, hand_name)
                    # groups_in_hands_group.append(ET.SubElement(hands_group, 'group', name=hand_name))
                    end_effectors += filter(None, [self.add_gripper_to_srdf(root, joint_module.name, hand_name, group_name)])
                    controller_list.append(OrderedDict([('name', 'fake_' + hand_name + '_controller'), ('joints', [])]))
                    controller_list[i+1]['joints'].append(joint_module.name+'_finger_joint1')
                    controller_list[i+1]['joints'].append(joint_module.name + '_finger_joint2')
                    hardware_interface_joints.append(joint_module.name + '_finger_joint1')
                    initial_poses.append(OrderedDict([('group', hand_name), ('pose', 'open')]))
                    ompl.update([(hand_name, copy.deepcopy(tmp_ompl['group_name']))])
                    ompl.update([('arm_'+hand_name, copy.deepcopy(tmp_ompl['group_name']))])
            i += 1

        # MoveIt: add virtual joint
        # ET.SubElement(root, "virtual_joint", name="virtual_joint", type="floating", parent_frame="world", child_link="base_link")

        # MoveIt disable collisions
        for coll_elem in self.urdf_writer.collision_elements:
            ET.SubElement(root, 'disable_collisions', link1=coll_elem[0], link2=coll_elem[1], reason='Adjacent')

        # Create folder if doesen't exist
        if not os.path.exists(os.path.dirname(srdf_filename)):
            try:
                os.makedirs(os.path.dirname(srdf_filename))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        xmlstr = xml.dom.minidom.parseString(ET.tostring(root)).toprettyxml(indent="   ")
        with open(srdf_filename, 'w+') as f:
            f.write(xmlstr)

        ###################################
        # Moveit configs: TO BE FIXED
        # ros_controllers_template_filename = self.urdf_writer.resource_finder.get_filename(
        #     'ModularBot/moveit_config/ros_controllers.yaml', 'data_path')
        # ros_controllers_filename = "/tmp/modular/moveit_config/ros_controllers.yaml"
        # ros_controllers = OrderedDict([])

        fake_controllers_filename = path_name + "/moveit_config/fake_controllers.yaml"
        # fake_controllers_filename = "/tmp/modular/moveit_config/fake_controllers.yaml"
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
        ros_controllers_launch = path_name + "/launch/ros_controllers.launch"
        # ros_controllers_launch = "/tmp/modular/launch/ros_controllers.launch"
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

        #########################
        # Remove collisions from SRDF by using Moveit collisions_updater
        # THIS WILL BE DONE AFTER DEPLOY FOR NOW

        # rospy.init_node('collisions_node', anonymous=True)
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # # find launch filename
        # update_collisions_launch = self.urdf_writer.resource_finder.get_filename('launch/update_collision.launch', 'data_path')
        # launch = roslaunch.parent.ROSLaunchParent(uuid, [update_collisions_launch])
        # launch.start()
        # rospy.loginfo("started")
        # launch.spin()

        return xmlstr


class XBotCorePlugin(Plugin):
    def add_plugin(self):
        # return ET.SubElement(self.urdf_writer.root, "xacro:plugin_xbotcore")
        pass

    def add_joint(self, joint_name):
        pass

    def remove_joint(self, joint_name):
        pass

    # SRDF
    def add_gripper_to_srdf(self, root, gripper_name, hand_name, parent_group_name):
        pass

    def add_hand_group_to_srdf(self, root, gripper_name, hand_name):
        pass

    # CONFIG
    def write_lowlevel_config(self, use_robot_id=False):
        """Creates the low level config file needed by XBotCore """

        # basic_config_filename = path_name + '/configs/ModularBot.yaml'
        basic_config_filename = self.urdf_writer.resource_finder.get_filename('configs/ModularBot.yaml', 'data_path')
        lowlevel_config_filename = path_name + '/ModularBot/configs/ModularBot.yaml'
        ## lowlevel_config_filename = self.urdf_writer.resource_finder.get_filename('configs/ModularBot.yaml', 'modularbot_path')
        # lowlevel_config_filename = "/tmp/modular/configs/ModularBot.yaml"
        lowlevel_config = OrderedDict([])

        with open(basic_config_filename, 'r') as stream:
            try:
                lowlevel_config = ordered_load(stream, yaml.SafeLoader)
                # cartesio_stack['EE']['base_link'] = self.urdf_writer.listofchains[0]
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
                if joint_module.type == 'joint':
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
                    # This is to handle chains with only one joint and low inertia after it.
                    # If we build two big robots this could have catastrophic effects
                    # TODO: fix this
                    if p > 1:
                        value.pid.impedance = [500.0, 20.0, 1.0, 0.003, 0.99]
                elif joint_module.type == 'tool_exchanger':
                    if use_robot_id:
                        key = 'AinMsp432ESC_' + str(joint_module.robot_id)
                        xbot_ecat_interface = [[int(joint_module.robot_id)], "libXBotEcat_ToolExchanger"]
                    else:
                        key = 'AinMsp432ESC_' + str(i)
                        xbot_ecat_interface = [[int(i)], "libXBotEcat_ToolExchanger"]
                    value = joint_module.AinMsp432ESC
                    self.urdf_writer.print(yaml.dump(joint_module.AinMsp432ESC))
                    lowlevel_config['HALInterface']['IEndEffectors'].append(xbot_ecat_interface)
                elif joint_module.type == 'gripper':
                    if use_robot_id:
                        key = 'LpESC_' + str(joint_module.robot_id)
                        xbot_ecat_interface = [[int(joint_module.robot_id)], "libXBotEcat_Gripper"]
                    else:
                        key = 'LpESC_' + str(i)
                        xbot_ecat_interface = [[int(i)], "libXBotEcat_Gripper"]
                    value = joint_module.LpESC
                    self.urdf_writer.print(yaml.dump(joint_module.LpESC))
                    lowlevel_config['HALInterface']['IEndEffectors'].append(xbot_ecat_interface)
                elif joint_module.type == 'simple_ee':
                    continue
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
            # ordered_dump(lowlevel_config, stream=outfile, Dumper=yaml.SafeDumper,  default_flow_style=False, line_break='\n\n', indent=4)
            ordered_dump(lowlevel_config, stream=outfile, default_flow_style=False, line_break='\n\n', indent=4,
                         canonical=False)
        return lowlevel_config

class XBot2Plugin(Plugin):
    def add_plugin(self):
        #self.urdf_writer.plugin_element = ET.SubElement(self.urdf_writer.root, "xacro:plugin_xbot2")
        self.gazebo_node = ET.SubElement(self.urdf_writer.root, "gazebo")
        self.plugin_node = ET.SubElement(self.gazebo_node, "plugin",
                                         name="xbot2_gz_joint_server",
                                         filename="libxbot2_gz_joint_server.so")
        self.pid_node = ET.SubElement(self.plugin_node, "pid")
        self.gain_node = ET.SubElement(self.pid_node, "gain", name='small_mot', p='100', d='10')
        self.gain_node = ET.SubElement(self.pid_node, "gain", name='medium_mot', p='500', d='50')
        self.gain_node = ET.SubElement(self.pid_node, "gain", name='big_mot', p='1000', d='100')
        return self.pid_node

    def add_joint(self, joint_name):
        #return ET.SubElement(self.pid_node, "xacro:add_xbot2_pid", name=joint_name, profile="small_mot")
        return ET.SubElement(self.pid_node, "gain", name=joint_name, profile="medium_mot")

    def remove_joint(self, joint_name):
        for pid in self.pid_node.findall('./pid'):
            if pid.attrib['name'] == joint_name:
                self.pid_node.remove(pid)

    # SRDF     
    def add_gripper_to_srdf(self, root, gripper_name, hand_name, parent_group_name):
        pass

    def add_hand_group_to_srdf(self, root, gripper_name, hand_name):
        pass

    # JOINT MAP
    def write_joint_map(self, use_robot_id=False):
        """Creates the joint map needed by XBotCore """

        # global path_name
        jointmap_filename = path_name + '/ModularBot/joint_map/ModularBot_joint_map.yaml'
        # jointmap_filename = path_superbuild + '/configs/ADVR_shared/ModularBot/joint_map/ModularBot_joint_map.yaml'
        ## jointmap_filename = self.urdf_writer.resource_finder.get_filename('joint_map/ModularBot_joint_map.yaml',
        ##                                                       'modularbot_path')
        #jointmap_filename="/tmp/modular/joint_map/ModularBot_joint_map.yaml"
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
                i += 1
                if joint_module.type == 'tool_exchanger':
                    name = joint_module.name + '_fixed_joint'
                    if use_robot_id:
                        joint_map['joint_map'][int(joint_module.robot_id)] = name
                    else:
                        joint_map['joint_map'][i] = name
                elif joint_module.type == 'simple_ee':
                    continue
                elif joint_module.type == 'gripper':
                    name = joint_module.name
                    fingers = [name + '_rightfinger', name + '_leftfinger']
                    if use_robot_id:
                        joint_map['albero_gripper_map'][int(joint_module.robot_id)] = {'name': name, 'fingers': fingers}
                    else:
                        joint_map['albero_gripper_map'][i] = {'name': name, 'fingers': fingers}
                else:
                    name = joint_module.name
                    if use_robot_id:
                        joint_map['joint_map'][int(joint_module.robot_id)] = name
                    else:
                        joint_map['joint_map'][i] = name
                
            # self.urdf_writer.print(str(i), joint_module.name)
            # self.urdf_writer.print(joint_map)

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
        hal_config_template = self.urdf_writer.resource_finder.get_filename('configs/low_level/hal/ModularBot_ec_all.yaml', 'data_path')
        hal_config_filename = path_name + '/ModularBot/config/hal/ModularBot_ec_all.yaml'
        # hal_config_filename = self.urdf_writer.resource_finder.get_filename('config/hal/ModularBot_ec_all.yaml', 'modularbot_path')
        # hal_config_filename = "/tmp/modular/config/hal/ModularBot_ec_all.yaml"
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
                i += 1
                if joint_module.type in ('tool_exchanger', 'gripper'):
                    if use_robot_id:
                        mod_id = str(joint_module.robot_id)
                    else:
                        mod_id = str(i)
                    ids.append(mod_id)
        ignore_id = OrderedDict({'ignore_id': {'type': 'vector<int>', 'value': ids}})
        hal_config['xbotcore_devices']['joint_ec']['params'].update(ignore_id)

        # Create folder if doesen't exist
        if not os.path.exists(os.path.dirname(hal_config_filename)):
            try:
                os.makedirs(os.path.dirname(hal_config_filename))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        with open(hal_config_filename, 'w') as outfile:
            # ordered_dump(hal_config, stream=outfile, Dumper=yaml.SafeDumper,  default_flow_style=False, line_break='\n\n', indent=4)
            ordered_dump(hal_config, stream=outfile, default_flow_style=False, line_break='\n\n', indent=4,
                         canonical=False)

        # HAL config ModularBot_idle
        idle_joint_config_template = self.urdf_writer.resource_finder.get_filename(
            'configs/low_level/joint_config/ModularBot_idle.yaml', 'data_path')
        idle_joint_config_filename = path_name + '/ModularBot/config/joint_config/ModularBot_idle.yaml'
        # HAL config ModularBot_impd4
        impd4_joint_config_template = self.urdf_writer.resource_finder.get_filename(
            'configs/low_level/joint_config/ModularBot_impd4.yaml', 'data_path')
        impd4_joint_config_filename = path_name + '/ModularBot/config/joint_config/ModularBot_impd4.yaml'

        idle_joint_config = OrderedDict([])
        impd4_joint_config = OrderedDict([])

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

        i = 0
        p = 0
        for joints_chain in self.urdf_writer.listofchains:
            # HACK
            p += 1
            for joint_module in joints_chain:
                if joint_module.type == 'joint':
                    i += 1
                    key = joint_module.name
                    value = joint_module.CentAcESC
                    impd4_joint_config[key] = copy.deepcopy(value)
                    impd4_joint_config[key].control_mode = 'D4_impedance_ctrl'
                    # Remove parameters that are now not used by XBot2 (they are handled by the EtherCat master on a different config file)
                    del impd4_joint_config[key].sign 
                    del impd4_joint_config[key].pos_offset 
                    del impd4_joint_config[key].max_current_A 

                    idle_joint_config[key] = copy.deepcopy(value)
                    idle_joint_config[key].control_mode = 'idle'
                    # Remove parameters that are now not used by XBot2 (they are handled by the EtherCat master on a different config file)   
                    del idle_joint_config[key].sign 
                    del idle_joint_config[key].pos_offset 
                    del idle_joint_config[key].max_current_A 

                    # HACK: Every joint on 2nd, 3rd, etc. chains have the torque loop damping set very low.
                    # This is to handle chains with only one joint and low inertia after it.
                    # If we build two big robots this could have catastrophic effects
                    # TODO: fix this
                    if p > 1:
                        value.pid.impedance = [500.0, 20.0, 1.0, 0.003, 0.99]

                elif joint_module.type == 'tool_exchanger':
                    key = joint_module.name
                    value = joint_module.AinMsp432ESC

                elif joint_module.type == 'gripper':
                    key = joint_module.name + '_motor'
                    value = joint_module.LpESC
                    impd4_joint_config[key] = copy.deepcopy(value)
                    impd4_joint_config[key].control_mode = '3B_motor_pos_ctrl'
                    # Remove parameters that are now not used by XBot2 (they are handled by the EtherCat master on a different config file)   
                    del impd4_joint_config[key].sign 
                    del impd4_joint_config[key].pos_offset 
                    del impd4_joint_config[key].max_current_A 

                    idle_joint_config[key] = copy.deepcopy(value)
                    idle_joint_config[key].control_mode = 'idle'
                    # Remove parameters that are now not used by XBot2 (they are handled by the EtherCat master on a different config file)   
                    del idle_joint_config[key].sign 
                    del idle_joint_config[key].pos_offset 
                    del idle_joint_config[key].max_current_A 
                    
                elif joint_module.type == 'simple_ee':
                    continue

                # idle_joint_config[key] = copy.deepcopy(value)
                # idle_joint_config[key].control_mode = 'idle'
                # Remove parameters that are now not used by XBot2 (they are handled by the EtherCat master on a different config file)
                # del idle_joint_config[key].sign 
                # del idle_joint_config[key].pos_offset 
                # del idle_joint_config[key].max_current_A 

        # Create folder if doesen't exist
        if not os.path.exists(os.path.dirname(idle_joint_config_filename)):
            try:
                os.makedirs(os.path.dirname(idle_joint_config_filename))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
        if not os.path.exists(os.path.dirname(impd4_joint_config_filename)):
            try:
                os.makedirs(os.path.dirname(impd4_joint_config_filename))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        with open(idle_joint_config_filename, 'w') as outfile:
            # ordered_dump(idle_joint_config, stream=outfile, Dumper=yaml.SafeDumper,  default_flow_style=False, line_break='\n\n', indent=4)
            ordered_dump(idle_joint_config, stream=outfile, default_flow_style=False, line_break='\n\n', indent=4,
                         canonical=False)

        with open(impd4_joint_config_filename, 'w') as outfile:
            # ordered_dump(impd4_joint_config, stream=outfile, Dumper=yaml.SafeDumper,  default_flow_style=False, line_break='\n\n', indent=4)
            ordered_dump(impd4_joint_config, stream=outfile, default_flow_style=False, line_break='\n\n', indent=4,
                         canonical=False)
        return hal_config


# noinspection PyUnresolvedReferences
class UrdfWriter:
    def __init__(self, 
                config_file='config_file.yaml', 
                control_plugin='xbot2', 
                elementree=None, 
                speedup=False, 
                parent=None, 
                verbose=False,
                logger=None):

        # Setting this variable to True, speed up the robot building.
        # To be used when the urdf does not need to be shown at every iteration
        self.speedup = speedup

        self.parent = parent

        self.logger = logger
        
        self.verbose = verbose
        if self.verbose:
            self.logger.setLevel(logging.DEBUG)  
        else:
            pass

        self.config_file = config_file

        self.resource_finder = ResourceFinder(self.config_file)

        self.collision_elements = []

        # Plugin class attribute. Can be XBot2Plugin, XBotCorePlugin or RosControlPlugin
        if control_plugin == 'ros_control':
            self.control_plugin = RosControlPlugin()
        elif control_plugin == 'xbotcore':
            self.control_plugin = XBotCorePlugin()
        elif control_plugin == 'xbot2':
            self.control_plugin = XBot2Plugin()
        self.control_plugin.urdf_writer = self

        if elementree is None:
            ## Open the template xacro file
            template = self.resource_finder.get_string('urdf/template.urdf.xacro', 'data_path')
            self.root = ET.fromstring(template)
            # Open the base xacro file
            # filename = path_name + '/modular_data/urdf/template.urdf.xacro'
            # self.print(filename)
            # with codecs.open(filename, 'r') as f:
            #     string = f.read()
            # Instantiate an Element Tree
            #self.root = ET.fromstring(string)
            
            self.urdf_tree = ET.ElementTree(self.root)
            
            # change path to xacro library
            library_filename = self.resource_finder.get_filename('urdf/ModularBot.library.urdf.xacro', 'data_path')
            control_filename = self.resource_finder.get_filename('urdf/ModularBot.control.urdf.xacro', 'data_path')
            for include in self.root.findall('xacro:include', ns):
                if include.attrib['filename'] == 'ModularBot.library.urdf.xacro':
                    include.attrib['filename'] = library_filename
                elif include.attrib['filename'] == 'ModularBot.control.urdf.xacro':
                    include.attrib['filename'] = control_filename

            self.control_plugin.add_plugin()

        else:
            self.root = elementree
            self.urdf_tree = ET.ElementTree(self.root)

        self.tag_num = 1
        self.branch_switcher = {
            1: '_A',
            2: '_B',
            3: '_C',
            4: '_D',
            5: '_E',
            6: '_F',
            7: '_G',
            8: '_H',
            9: '_I',
            10: '_L',
            11: '_M',
            12: '_N',
            13: '_O',
            14: '_P',
            15: '_Q',
            16: '_R'
        }

        self.inverse_branch_switcher = {y: x for x, y in iteritems(self.branch_switcher)}

        self.n_cubes = 0
        self.cube_switcher = {
            0: 'a',
            1: 'b',
            2: 'c',
            3: 'd',
            4: 'e',
            5: 'f',
            6: 'g',
            7: 'h'
        }

        self.listofchains = []
        self.listofhubs = []

        self.origin, self.xaxis, self.yaxis, self.zaxis = (0, 0, 0.4), (1, 0, 0), (0, 1, 0), (0, 0, 1)

        # add attribute corresponding to the connector transform
        self.T_con = tf.transformations.translation_matrix((0, 0, 0.0))  # self.slavecube.geometry.connector_length))

        data = {'type': "base_link", 'name': "base_link"}

        self.base_link = ModuleNode.ModuleNode(data, "base_link")
        setattr(self.base_link, 'name', "base_link")
        setattr(self.base_link, 'tag', "_A")
        setattr(self.base_link, 'size', 3)
        setattr(self.base_link, 'i', 0)
        setattr(self.base_link, 'p', 0)
        setattr(self.base_link, 'Homogeneous_tf', tf.transformations.identity_matrix())
        setattr(self.base_link, 'robot_id', 0)

        self.parent_module = self.base_link

    def print(self, *args):
        if isinstance(self.logger, logging.Logger):
            self.logger.debug(' '.join(str(a) for a in args))
        else:
            print(args)

    def info_print(self, *args):
        if isinstance(self.logger, logging.Logger):
            self.logger.info(' '.join(str(a) for a in args))
        else:
            print(args)

    @staticmethod
    def find_module_from_id(module_id, modules):
        """Given the module id find the corresponding dictionary entry and return it"""
        found_module = None
        for module in modules:
            if module_id in module.keys():
                found_module = module
                break
            else:
                continue
        return found_module

    @staticmethod
    def find_next_module_in_chain(module_id, modules):
        """Given the module id find the corresponding dictionary entry and return it"""
        found_module = None
        found_module_id = 0
        next_position = 2  # 1 #TODO: remove this hack for not cosidering pwrboard
        for module in modules:
            if module_id in module.keys():
                position = module[module_id]['position']
                next_position = int(position) + 1
                break
        # print('next position:', next_position)
        for next_module in modules:
            next_id = (next_module.keys())[0]
            if next_module[next_id]['position'] == next_position:
                found_module = next_module
                found_module_id = next_id
                break
            else:
                continue
        return found_module, found_module_id
    
    def sort_modules(self, modules_dict):

        ordered_chain = [None] * len(modules_dict)

        for key, item in modules_dict.items():

            module_position = int(item['position'])

            try:
                ordered_chain[module_position - 1] = item
            
            except IndexError:
                self.print('unexpected module position {}, modules number: {}'.format(module_position, len(modules_dict)))
                return list()
        
        return ordered_chain


    # This method will be used when branches in the robot will be supported.
    def read_from_json(self, json_data):

        # If a tree representing the topology was already instantiated, re-initialize and start from scratch
        if self.root != 0:
            self.print("Re-initialization")
            self.__init__(config_file=self.config_file, 
                control_plugin=self.control_plugin, 
                speedup=self.speedup, 
                verbose=self.verbose,
                logger=self.logger)
        
        # # Open the base xacro file
        # filename = path_name + '/urdf/ModularBot_new.urdf.xacro'
        # with codecs.open(filename, 'r') as f:
        #     string = f.read()
        # # Instantiate an Element Tree
        # self.root = ET.fromstring(string)
        # self.urdf_tree = ET.ElementTree(self.root)
        # self.print(ET.tostring(self.urdf_tree.getroot()))

        # Load the robot_id dictionary from yaml file
        # opts = repl_option()
        # robot_id_dict = yaml.safe_load(open(opts["robot_id_yaml"], 'r'))
        robot_id_yaml = self.resource_finder.get_filename('robot_id.yaml', '')
        robot_id_dict = yaml.safe_load(open(robot_id_yaml, 'r'))

        module_params_yaml = self.resource_finder.get_filename('module_params.yaml', '')
        module_params_dict = yaml.safe_load(open(module_params_yaml, 'r'))

        # Process the modules described in the json to create the tree
        modules_dict = yaml.safe_load(json_data)
        modules_list = self.sort_modules(modules_dict)

        for module in modules_list:

            module_id = int(module['robot_id'])

            mod_type = int(module['mod_type'])
            mod_id = int(module['mod_id'])
            mod_size = int(module['mod_size'])
            mod_rev = int(module['mod_rev'])

            module_filename = module_params_dict.get(mod_type, {}).get(mod_id,{}).get(mod_size,{}).get(mod_rev)
            if module_filename is None:
                module_filename = robot_id_dict.get(module_id)
                if module_filename is None:
                    self.info_print("Id not recognized! Skipping add_module() for id", robot_id_dict.get(module_id))          
                    continue

            module_position = int(module['position'])
            module_topology = int(module['topology'])

            self.info_print('Discovered module with ID:', module_id)

            parent_position = None

            # find parent (from OpenEtherCATsociety)
            if (module_position > 1):
                topo_counter = 0
                candidate_position = module_position - 1

                while candidate_position > 0:
                    candidate_parent = modules_list[candidate_position - 1]
                    candidate_parent_id = int(candidate_parent['robot_id'])
                    topology = int(candidate_parent['topology'])

                    if topology == 1:
                        topo_counter -= 1
                    elif topology == 3:
                        topo_counter += 1
                    elif topology == 4:
                        topo_counter += 2

                    if (topo_counter >= 0 and topology > 1) or candidate_position == 1:
                        # parent found
                        parent_position = candidate_position
                        candidate_position = 1

                    candidate_position -= 1

            self.print("module and parent:", module_id, parent_position)

            # select the correct parent module
            if parent_position :
                parent = modules_list[parent_position -1]
                self.print('parent:', parent)
                
                parent_id = int(parent['robot_id'])
                self.print('parent_id:', parent_id)
                
                parent_active_ports = int(parent['active_ports'])
                self.print('parent_active_ports:', parent_active_ports)

                parent_topology = int(parent['topology'])
                self.print('parent_topology:', parent_topology)

                if self.verbose:
                    # Render tree
                    self.print(RenderTree(self.base_link))
                    for pre, _, node in RenderTree(self.base_link):
                        self.print(pre, node, node.name, node.robot_id)
                        # treestr = u"%s%s" % (pre, node.name)
                        # self.print(treestr.ljust(8), node.name, node.robot_id)

                parent_module = anytree.search.findall_by_attr(self.base_link, parent_id, name='robot_id')[0]
                self.print('parent_module:', parent_module, '\nparent name:', parent_module.name)
                self.select_module_from_name(parent_module.name)
                self.print(self.parent_module.name)
                #TODO:replace with select_module_from_id

                # set the selected_port as occupied
                mask = 1 << parent_module.selected_port - 1
                self.print(mask)
                self.print(parent_module.occupied_ports)
                parent_module.occupied_ports = "{0:04b}".format(int(parent_module.occupied_ports, 2) | mask)
                self.print(parent_module.occupied_ports)
                #parent_module.occupied_ports[-selected_port] = 1

                #If the parent is a cube to support non-structural box we add a socket
                if parent_module.type == 'cube':
                    if parent_module.is_structural == False:
                        self.add_socket()

            # get which ports in the ESC slave are active
            active_ports = int(module['active_ports'])
            self.print('active_ports:', active_ports)

            #add the module
            if mod_type == 2 or module_filename=='master_cube.yaml':
                data = self.add_slave_cube(0, is_structural=False, robot_id=module_id, active_ports=active_ports)
            else:
                data = self.add_module(module_filename, 0, robot_id=module_id, active_ports=active_ports)
            
            if self.verbose:
                for pre, _, node in RenderTree(self.base_link):
                    self.print(pre, node, node.name, node.robot_id)

            #TODO: remove this shit

            module_name = data['lastModule_name']
            module_type = data['lastModule_type']

            # if module['connections'][0] == 0:
            #     self.print('\n Module: \n')
            #     self.print(module['id'], module['type'])
            #     if module['type'] == 'master_cube':
            #         data = self.add_slave_cube(0)
            #     else:
            #         data = self.add_module(module['type'], 0)
            #     module_name = data['lastModule_name']
            #     module_type = data['lastModule_type']
            #     self.process_connections(module['connections'], modules, module_name, module_type)

        # doc = xacro.parse(string)
        # xacro.process_doc(doc, in_order=True)
        # string = doc.toprettyxml(indent='  ')
        string = self.process_urdf()

        self.info_print("Discovery completed")

        data = {'string': string}
        return data

    def render_tree(self):
        for pre, _, node in RenderTree(self.base_link):
            self.print(pre, node, node.name, node.robot_id)

        return 0

    # Until the robot can be only an unbranched robot this function needs to be called. No "connections" in json msg
    def read_from_json_alt(self, json_data):
        """Read the JSON msg with info on the topology and produce the URDF

            Parameters
            ----------
            json_data: str
                String containing the JSON msg received from 0MQ

            Returns
            ----------
            data: dict
                Dictionary containing the URDF string
        """

        # If a tree representing the topology was already instantiated, re-initialize and start from scratch
        if self.root != 0:
            self.print("Re-initialization")
            self.__init__()

        # # Open the base xacro file
        # filename = path_name + '/urdf/ModularBot_new.urdf.xacro'
        # with codecs.open(filename, 'r') as f:
        #     string = f.read()
        # # Instantiate an Element Tree
        # self.root = ET.fromstring(string)
        # self.urdf_tree = ET.ElementTree(self.root)
        # self.print(ET.tostring(self.urdf_tree.getroot()))

        # Load the esc_type dictionary from yaml file
        opts = repl_option()
        # robot_id_dict = yaml.safe_load(open(opts["esc_type_yaml"], 'r'))
        robot_id_dict = yaml.safe_load(open(opts["robot_id_yaml"], 'r'))

        # Add a first cube for the initial ethercat test with no Hub.
        # TODO: remove it once the hub is implemented and can be added automatically
        # data = self.add_slave_cube(0)
        # module_name = data['lastModule_name']
        # module_name = 'L_0a_con2'
        # module_type = data['lastModule_type']

        # Process the modules described in the json to create the tree
        modules = json.load(json_data)
        self.print("modules:", modules, type(modules))
        
        # # This loop has only one iteration over the first element of each chain in the module list from the json msg
        # # The others modules in the chain are processed by process_connections_alt
        # for module in modules:
        #     self.print("module:", module)
        module, module_id = self.find_next_module_in_chain(0, modules)
        self.print("module:",module, type(module))
        # TODO: The "IDs" actually are now the "positions" in the ECAT network (from get_ec_positions()).
        #       Need to change when branches are implemented
        # for module_id in module.keys():
        self.print("module id:",module_id)
        # Find the name of the yaml describing the module to be added, searching the dictionary by the esc_type
        #TODO: handle exception if the id is not found in the dict
        module_filename = robot_id_dict.get(module[module_id]['robot_id'])
        data = self.add_module(module_filename, 0, robot_id=module_id)

        module_name = data['lastModule_name']
        module_type = data['lastModule_type']

        # generate the list of connections.
        # Right now only unbranched chains are supported so the only connection is the next module on the list
        # TODO: get the connections list when branches will be present in the robot
        connections = []
        if module[module_id]['topology'] == 2:
            next_module, next_module_id = self.find_next_module_in_chain(module_id, modules)
            connections.append(str(next_module_id))
        # if topology is not 2, the only other option is 1 right now so the connections list is empty
        self.print(connections)

        # Process the connections of the module and add the modules as child of the current one
        self.process_connections_alt(connections, modules, module_name, module_type, robot_id_dict)

        # break

        # doc = xacro.parse(string)
        # xacro.process_doc(doc, in_order=True)
        # string = doc.toprettyxml(indent='  ')
        string = self.process_urdf()

        data = {'string': string}
        return data

    def process_connections(self, connections_list, modules_list, name, m_type):
        """Process connections of the module as described in the JSON as a list"""
        self.print('enter!')
        for child_id in connections_list[1:]:
            self.print('child: ', child_id)
            self.select_module_from_name(name)
            self.print(self.parent_module.name)
            if child_id != -1:
                # Find child module to process searching by id
                child = self.find_module_from_id(child_id, modules_list)
                # If the processed module is a mastercube we need first to select the connector to which attach to
                if m_type == 'mastercube':
                    _connector_index = connections_list.index(child_id) + 1
                    con_name = name + '_con' + str(_connector_index)
                    self.select_module_from_name(con_name)
                # Add the module
                if child['type'] == 'master_cube':
                    data = self.add_slave_cube(0)
                else:
                    data = self.add_module(child['type'] + '.yaml', 0)
                # Update variables and process its connections
                module_name = data['lastModule_name']
                module_type = data['lastModule_type']
                self.process_connections(child['connections'], modules_list, module_name, module_type)

    def process_connections_alt(self, connections_list, modules_list, name, m_type, esc_dict):
        """Process connections of a module as described in the list obtained from the JSON

        Parameters
        ----------
        connections_list: list
            List containing the IDs of the modules connected

        modules_list: object
            Python object obtained deserializing the JSON containing all the modules in the robot

        name: str
            Name of the module we are processing the connections of

        m_type: str
            String identifyng the type of module

        esc_dict: dict
            Dictionary assigning esc_type numbers to the relative modules and the yaml file describing it

        """
        self.print('enter!')
        for child_id in connections_list:
            self.print('child: ', child_id)
            self.select_module_from_name(name)
            self.print(self.parent_module.name)
            if child_id != -1:
                # Find child module to process searching by id in the modules_list
                #child = modules_list[int(child_id) - 1]  # self.find_module_from_id(child_id, modules_list)
                child = self.find_module_from_id(child_id, modules_list)       
                self.print(child[child_id])
                # TODO: Uncomment below when switching to branched robot
                # # If the processed module is a mastercube we need first to select the connector to which attach to
                # if m_type == 'mastercube':
                #     _connector_index = connections_list.index(child_id) + 1
                #     con_name = name + '_con' + str(_connector_index)
                #     self.select_module(con_name)

                # child is an element of the JSON. child_id is the key to access its value.
                # esc_type is a number specifying the type of module
                # esc_type = esc_dict.get(child[child_id]['esc_type'])
                child_filename = esc_dict.get(child[child_id]['robot_id'])
                # Add the module with an angle offset of 0. as default always zero relative orientation is assumed
                data = self.add_module(child_filename, 0, robot_id=child_id)

                # Update variables and process its connections
                module_name = data['lastModule_name']
                module_type = data['lastModule_type']

                connections = []
                # Check topology to determine the num of connections and put them in the list.
                # TODO: add cases for different topologies when switching to branched robots
                if child[child_id]['topology'] == 2:
                    next_module, next_module_id = self.find_next_module_in_chain(child_id, modules_list)
                    connections.append(str(next_module_id))
                self.print('connections:', connections)

                # Recursively call the function.
                # Process the connections of the module and add the modules as child of the current one
                self.process_connections_alt(connections, modules_list, module_name, module_type, esc_dict)

    def read_file(self, file_str):
        """Open the URDF chosen from the front-end and import it as a ElemenTree tree"""
        # global root, urdf_tree
        self.print(file_str)
        self.root = ET.fromstring(file_str.encode('utf-8'))
        self.urdf_tree = ET.ElementTree(self.root)
        self.print(ET.tostring(self.urdf_tree.getroot()))

        # include files necessary for Gazebo&XBot simulation
        # ET.SubElement(root, "xacro:include", filename="$(find modular)/urdf/config.xacro")
        # ET.SubElement(root, "xacro:include", filename="$(find modular)/urdf/modular.gazebo")

        doc = xacro.parse(file_str.encode('utf-8'))
        xacro.process_doc(doc, in_order=True)
        string = doc.toprettyxml(indent='  ')

        data = {'string': string}
        return data

    def process_urdf(self):
        """Process the urdf to convert from xacro and perform macro substitutions. Returns urdf string"""
        # global urdf_tree
        # write the urdf tree to a string
        xmlstr = xml.dom.minidom.parseString(ET.tostring(self.urdf_tree.getroot())).toprettyxml(indent="   ")

        # parse the string to convert from xacro
        doc = xacro.parse(xmlstr)

        # perform macro replacement
        xacro.process_doc(doc)

        string = doc.toprettyxml(indent='  ')

        return string

    def add_to_chain(self, new_joint):
        """Add joint to one of the robot kinematic chains

        Parameters
        ----------
        new_joint: ModuleNode.ModuleNode
            New ModuleNode object representing a joint to be added to a kinematic chain"""

        # get tag_index, an integer representing on which branch of the robot the joint has been added
        tag_index = self.inverse_branch_switcher.get(new_joint.tag)
        chain = [new_joint]
        self.print("tag_index: ", tag_index, "list of chains: ", len(self.listofchains))
        # if tag_index is bigger than the length of the list of chains, it means this chain hasn't been added yet.
        # then we need to append a new list representing the new chain formed by the new joint only
        if tag_index > len(self.listofchains):
            self.listofchains.append(chain)
        # if instead tag_index is not bigger it means the chain the new joint is part of has already beeen added.
        # then the new joint is appended to the list representing the chain it's part of.
        else:
            self.listofchains[tag_index - 1].append(new_joint)

    def remove_from_chain(self, joint):
        """Remove joint from the list of the robot kinematic chains

        Parameters
        ----------
        joint: ModuleNode.ModuleNode
            ModuleNode object representing a joint to be removed to a kinematic chain"""

        for chain in self.listofchains:
            if joint in chain:
                chain.remove(joint)
        self.listofchains = filter(None, self.listofchains)


    # noinspection PyPep8Naming
    def add_slave_cube(self, angle_offset, is_structural=True, robot_id=0, active_ports=1):
        """Method adding slave/master cube to the tree.

        Parameters
        ----------
        angle_offset: float
            Value of the angle between the parent module output frame and the cube input frame

        is_structural: float
            Bool variable indicating if the box is a structural part of the robot or it's just used as computation unit away from the robot.
            In the second case the different chains are placed in default locations in a xy plane 
            (the user will then be queried to specify their actual location once the robot model is recontructed)

        robot_id: int
            Value of the robot_id set in the firmware of the module. 
            This is obtained in Discovery Mode when reading the JSON from the EtherCAT master. This is not used when in Bulding Mode.

        active_ports: int
            The number of active ports of the cube (how many ports have established a connection to a module). 
            The value is the conversion to int of the 4-bit binary string where each bit represent one port (1 if port is active, 0 if port is unactive)

        Returns
        -------
        data: dict
            Dictionary with as entries all the relevant info on the newly added module.
            In particular the updated and newly processed urdf string.

        """
        # global T_con, L_0a, n_cubes, parent_module

        # TODO: This part below the "if" is deprecated. It still uses "connectors" separate module. 
        # It was made to handle creating robots with multiple "boxes", which will be probably never done.
        # If important should be reviewed and modify it as the part below the "else".

        if self.n_cubes > 0:
            # add slave cube

            # Generate name according to the # of cubes already in the tree
            name = 'L_0' + self.cube_switcher.get(self.n_cubes)
            self.n_cubes += 1

            # Get inverse of the transform of the connector
            T_con_inv = tf.transformations.inverse_matrix(self.T_con)

            # Generate name and dict for the 1st connector module
            name_con1 = name + '_con1'
            data1 = {'Homogeneous_tf': T_con_inv, 'type': "con", 'name': name_con1, 'i': 0, 'p': 0, 'size': 3}
            self.print('T_con_inv:', T_con_inv)

            # Add the 1st connector module to the tree
            slavecube_con1 = ModuleNode.ModuleNode(data1, name_con1, parent=self.parent_module)

            # Get transform representing the output frame of the parent module after a rotation of angle_offset
            transform = ModuleNode.get_rototranslation(self.parent_module.Homogeneous_tf,
                                                       tf.transformations.rotation_matrix(angle_offset,
                                                                                          self.zaxis))
            x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(transform)

            # Generate the name of the fixed joint used to connect the cube
            if self.parent_module.type == "cube":
                fixed_joint_name = 'FJ_' + self.parent_module.parent.name + '_' + name
            else:
                fixed_joint_name = 'FJ_' + self.parent_module.name + '_' + name

            # Select the name of the parent to use to generate the urdf. If the parent is a joint use the name of the
            # dummy link attached to it
            if self.parent_module.type == "joint":
                parent_name = 'L_' + str(self.parent_module.i) + self.parent_module.tag
            else:
                parent_name = self.parent_module.name

            # Add the fixed joint to the xml tree
            ET.SubElement(
                self.root,
                "xacro:add_fixed_joint",
                type="fixed_joint",
                name=fixed_joint_name,
                father=parent_name,
                child=name_con1,
                x=x,
                y=y,
                z=z,
                roll=roll,
                pitch=pitch,
                yaw=yaw
            )

            # call the method that reads the yaml file describing the cube and instantiate a new module object
            # cube_path = '/'.join((yaml_path, 'master_cube.yaml'))
            # filename = pkg_resources.resource_string(resource_package, cube_path)
            filename = self.resource_finder.get_filename('master_cube.yaml', 'yaml_path')
            # filename = path_name + '/web/static/yaml/master_cube.yaml'
            slavecube = ModuleNode.slavecube_from_yaml(filename, slavecube_con1)

            # set attributes of the newly added module object
            setattr(slavecube, 'name', name)
            setattr(slavecube, 'i', 0)
            setattr(slavecube, 'p', 0)

            setattr(slavecube, 'robot_id', robot_id)

            # add the slave cube to the xml tree
            ET.SubElement(self.root, "xacro:add_slave_cube", type='cube', name=name, filename=filename)

            # instantate a ModuleNode for branch 2 connector
            name_con2 = name + '_con2'
            data2 = {'Homogeneous_tf': self.T_con, 'type': "con", 'name': name_con2, 'i': 0, 'p': 0, 'size': 3}
            slavecube_con2 = ModuleNode.ModuleNode(data2, name_con2, parent=slavecube)

            # instantate a ModuleNode for branch 3 connector
            name_con3 = name + '_con3'
            data3 = {'Homogeneous_tf': self.T_con, 'type': "con", 'name': name_con3, 'i': 0, 'p': 0, 'size': 3}
            slavecube_con3 = ModuleNode.ModuleNode(data3, name_con3, parent=slavecube)

            # instantate a ModuleNode for branch 4 connector
            name_con4 = name + '_con4'
            data4 = {'Homogeneous_tf': self.T_con, 'type': "con", 'name': name_con4, 'i': 0, 'p': 0, 'size': 3}
            slavecube_con4 = ModuleNode.ModuleNode(data4, name_con4, parent=slavecube)

            if self.verbose:
                # Render tree
                for pre, _, node in anytree.render.RenderTree(self.base_link):
                    self.print("%s%s" % (pre, node.name))

            # new_Link = slavecube_con1
            # past_Link = parent_module
            # new_Link.get_rototranslation(past_Link.Homogeneous_tf, tf.transformations.identity_matrix())
            # self.print(new_Link.z)

            # ET.SubElement(root, "xacro:add_master_cube", name=name)

            # fixed_joint_name = 'cube_joint'
            # ET.SubElement(root,
            #               "xacro:add_fixed_joint",
            #               name=fixed_joint_name,
            #               type="fixed_joint",
            #               father=past_Link.name,
            #               child=new_Link.name,
            #               x=new_Link.x,
            #               y=new_Link.y,
            #               z=new_Link.z,
            #               roll=new_Link.roll,
            #               pitch=new_Link.pitch,
            #               yaw=new_Link.yaw)

            # update the urdf file, adding the new module
            # string = write_urdf(path_name + '/urdf/ModularBot_test.urdf', urdf_tree)

            if self.speedup:
                string = ""
            else:
                # Process the urdf string by calling the process_urdf method.
                # Parse, convert from xacro and write to string.
                string = self.process_urdf()

            # Update the parent_module attribute of the URDF_writer class
            self.parent_module = slavecube

            self.listofhubs.append(slavecube)

            # Create a dictionary containing the urdf string just processed and other parameters needed by the web app
            data = {'result': string,
                    'lastModule_type': 'mastercube',
                    'lastModule_name': name,
                    'size': 3,
                    'count': self.n_cubes}

            return data

        else:
            # add master cube
            self.print('add_master_cube')
            # Generate name according to the # of cubes already in the tree
            name = 'L_0' + self.cube_switcher.get(self.n_cubes)
            self.n_cubes += 1

            # self.T_con = tf.transformations.translation_matrix((0, 0, 0.1))
            # self.T_con = self.mastercube.geometry.connector_length))

            # filename = path_name + '/web/static/yaml/master_cube.yaml'
            filename = self.resource_finder.get_filename('master_cube.yaml', 'yaml_path')

            # call the method that reads the yaml file describing the cube and instantiate a new module object
            mastercube = ModuleNode.mastercube_from_yaml(filename, self.parent_module)

            # set attributes of the newly added module object
            setattr(mastercube, 'name', name)
            setattr(mastercube, 'i', 0)
            setattr(mastercube, 'p', 0)

            setattr(mastercube, 'robot_id', robot_id)

            setattr(mastercube, 'is_structural', is_structural)
            if is_structural:
                # add the master cube to the xml tree
                ET.SubElement(self.root, "xacro:add_master_cube", type='cube', name=name, filename=filename)
                ET.SubElement(self.root, "xacro:add_connectors", type='connectors', name=name, filename=filename)
            else:
                # add the master cube to the xml tree
                #ET.SubElement(self.root, "xacro:add_master_cube", type='cube', name=name, filename=filename)
                #ET.SubElement(self.root, "xacro:add_connectors", type='connectors', name=name, filename=filename)
                pass

            # # instantate a ModuleNode for branch 1 connector
            # name_con1 = name + '_con1'
            # data1 = {'Homogeneous_tf': mastercube.Con_1_tf, 'type': "con", 'name': name_con1, 'i': 0, 'p': 0, 'size': 3}
            # slavecube_con1 = ModuleNode.ModuleNode(data1, name_con1, parent=mastercube)

            # # instantate a ModuleNode for branch 2 connector
            # name_con2 = name + '_con2'
            # data2 = {'Homogeneous_tf': mastercube.Con_2_tf, 'type': "con", 'name': name_con2, 'i': 0, 'p': 0, 'size': 3}
            # slavecube_con2 = ModuleNode.ModuleNode(data2, name_con2, parent=mastercube)

            # # instantate a ModuleNode for branch 3 connector
            # name_con3 = name + '_con3'
            # data3 = {'Homogeneous_tf': mastercube.Con_3_tf, 'type': "con", 'name': name_con3, 'i': 0, 'p': 0, 'size': 3}
            # slavecube_con3 = ModuleNode.ModuleNode(data3, name_con3, parent=mastercube)

            # # instantate a ModuleNode for branch 4 connector
            # name_con4 = name + '_con4'
            # data4 = {'Homogeneous_tf': mastercube.Con_4_tf, 'type': "con", 'name': name_con4, 'i': 0, 'p': 0, 'size': 3}
            # slavecube_con4 = ModuleNode.ModuleNode(data4, name_con4, parent=mastercube)

            # Render tree
            #for pre, _, node in anytree.render.RenderTree(self.base_link):
                #self.print("%s%s" % (pre, node.name))

            # new_Link = slavecube_con1
            # past_Link = parent_module
            # new_Link.get_rototranslation(past_Link.Homogeneous_tf, tf.transformations.identity_matrix())
            # self.print(new_Link.z)

            # ET.SubElement(root, "xacro:add_master_cube", name=name)

            # fixed_joint_name = 'cube_joint'
            # ET.SubElement(root,
            #               "xacro:add_fixed_joint",
            #               name=fixed_joint_name,
            #               type="fixed_joint",
            #               father=past_Link.name,
            #               child=new_Link.name,
            #               x=new_Link.x,
            #               y=new_Link.y,
            #               z=new_Link.z,
            #               roll=new_Link.roll,
            #               pitch=new_Link.pitch,
            #               yaw=new_Link.yaw)

            # update the urdf file, adding the new module
            # string = write_urdf(path_name + '/urdf/ModularBot_test.urdf', urdf_tree)

            if self.speedup:
                string = ""
            else:
                # Process the urdf string by calling the process_urdf method. Parse, convert from xacro and write to string.
                string = self.process_urdf()

            # Update the EtherCAT port connected to the electro-mechanical interface where the new module/slave will be added 
            #    1           2           3           4
            #    o           o           o           o
            #    |           |           |           |
            # com-exp   upper port  front port    nothing
            setattr(mastercube, 'selected_port', 2)
            self.print('mastercube.selected_port :', mastercube.selected_port)

            # save the active ports as a binary string
            setattr(mastercube, 'active_ports', "{0:04b}".format(active_ports))
            self.print('active_ports: ', mastercube.active_ports)

            # save the occupied ports as a binary string
            setattr(mastercube, 'occupied_ports', "0001")
            self.print('occupied_ports: ', mastercube.occupied_ports)

            # # If parent topology is greater than 2 the parent is a switch/hub so we need to find the right port where the module is connected
            # if active_ports >= 3:
            #     for port_idx in range(2, len(mastercube.active_ports) - 1)
            #         if mastercube.active_ports[-port_idx] == 1:
            #             mastercube.selected_port = port_idx
            #             break

            # if parent_active_ports == 3:
            #     self.parent_module.selected_port = 3
            # elif parent_active_ports == 5:
            #     self.parent_module.selected_port = 4
            # self.print('self.parent_module.selected_port: ', self.parent_module.selected_port)

            # Update the parent_module attribute of the URDF_writer class. A default connection is chosen.
            #self.parent_module = slavecube_con3
            self.parent_module = mastercube

            self.listofhubs.append(mastercube)

            # Create a dictionary containing the urdf string just processed and other parameters needed by the web app
            data = {'result': string,
                    'lastModule_type': 'mastercube',
                    'lastModule_name': name,
                    'size': 3,
                    'count': self.n_cubes}

            return data

    def get_ET(self):
        return self.urdf_tree

    def get_parent_module(self):
        return self.parent_module

    def update_generator(self):
        # Generator expression for list of urdf elements without the gazebo tag.
        # This is needed because of the change in the xacro file, as gazebo simulation tags
        # are now added from the start and this creates problems with the search
        nodes = set(self.root.findall("*"))
        gazebo_nodes = set(self.root.findall("./gazebo"))
        xacro_include_nodes = set(self.root.findall('./xacro:include', ns))
        filtered_nodes = nodes.difference(gazebo_nodes).difference(xacro_include_nodes)
        self.gen = (node for node in filtered_nodes)

    # Adds a table for simulation purposes
    def add_table(self):
        data = {'type': "link", 'name': "table"}

        table = ModuleNode.ModuleNode(data, "table", parent=self.base_link)
        setattr(table, 'name', "table")
        setattr(table, 'tag', "_A")
        setattr(table, 'size', 3)
        setattr(table, 'i', 0)
        setattr(table, 'p', 0)
        setattr(table, 'Homogeneous_tf', tf.transformations.identity_matrix())
        setattr(table, 'robot_id', 0)

        ET.SubElement(self.root,
                      "xacro:add_table",
                      type="link",
                      name="table",
                      father=self.parent_module.name)

        self.collision_elements.append((self.parent_module.name, "table"))

        self.parent_module = table

        if self.speedup:
            string = ""
        else:
            # Process the urdf string by calling the process_urdf method. Parse, convert from xacro and write to string
            string = self.process_urdf()

        # Create a dictionary containing the urdf string just processed and other parameters needed by the web app
        data = {'result': string,
                'lastModule_type': table.type,
                'lastModule_name': table.name,
                'size': table.size,
                'count': table.i}

        return data

    def move_socket(self, socket_name, x_offset=0.0, y_offset=0.0, z_offset=0.0, angle_offset=0.0):
        socket = self.access_module(socket_name)
        fixed_joint_name = 'L_' + str(socket.i) + socket.tag + '_fixed_joint_' + str(socket.p)

        # Update generator expression
        self.update_generator()

        # From the list of xml elements find the ones with name corresponding to the relative joint, stator link
        # and fixed joint before the stator link and remove them from the xml tree
        for node in self.gen:
            try:
                if node.attrib['name'] == fixed_joint_name:
                    node.set('x', str(x_offset))
                    node.set('y', str(y_offset))
                    node.set('z', str(z_offset))
                    node.set('yaw', str(angle_offset))
            except KeyError:
                pass

        if self.speedup:
            string = ""
        else:
            # Process the urdf string by calling the process_urdf method. Parse, convert from xacro and write to string
            string = self.process_urdf()

        if self.verbose:
            # Render tree
            for pre, _, node in anytree.render.RenderTree(self.base_link):
                self.print("%s%s" % (pre, node.name))

        # Create a dictionary containing the urdf string just processed and other parameters needed by the web app
        data = {'result': string,
                'lastModule_type': socket.type,
                'lastModule_name': socket.name,
                'size': socket.size,
                'count': socket.i}

        # Update the parent_module attribute of the URDF_writer class
        self.parent_module = socket

        return data

    def add_socket(self, x_offset=0.0, y_offset=0.0, z_offset=0.0, angle_offset=0.0):
        filename = 'socket.yaml'
        # Generate the path to the required YAML file
        # module_name = path_name + '/web/static/yaml/' + filename
        module_name = self.resource_finder.get_filename(filename, 'yaml_path')

        # Set base_link as parent
        self.parent_module = self.base_link

        # create a ModuleNode instance for the socket
        new_socket = ModuleNode.module_from_yaml(module_name, self.parent_module, reverse=0)

        # assign a new tag to the chain
        tag_letter = self.branch_switcher.get(self.tag_num)
        setattr(new_socket, 'tag', tag_letter)
        self.tag_num += 1

        # Set attributes of the newly added module object
        setattr(new_socket, 'i', self.parent_module.i)
        setattr(new_socket, 'p', self.parent_module.p)
        # Size is already set from the YAML file
        # setattr(new_socket, 'size', self.parent_module.size)

        setattr(new_socket, 'angle_offset', angle_offset)
        setattr(new_socket, 'robot_id', 0)

        # Update the EtherCAT port connected to the electro-mechanical interface where the new module/slave will be added
        setattr(new_socket, 'selected_port', 2)
        #self.print('selected_port :', new_socket.selected_port)

        # save the active ports as a binary string
        setattr(new_socket, 'active_ports', "{0:04b}".format(3))  # TODO:change this
        #self.print('active_ports: ', new_socket.active_ports)

        # save the occupied ports as a binary string
        setattr(new_socket, 'occupied_ports', "0001")
        #self.print('occupied_ports: ', new_socket.occupied_ports)

        setattr(new_socket, 'name', 'L_' + str(new_socket.i) + new_socket.tag)
        ET.SubElement(self.root,
                      "xacro:add_socket",
                      type="link",
                      name=new_socket.name,
                      filename=new_socket.filename,
                      size_z=new_socket.link_size_z,
                      size=str(new_socket.size))

        if self.parent_module.type == 'cube':
            if self.parent_module.is_structural:
                parent_name = self.parent_module.name
                if self.parent_module.selected_port == 1:
                    interface_transform = self.parent_module.Con_1_tf
                elif self.parent_module.selected_port == 2:
                    interface_transform = self.parent_module.Con_2_tf
                elif self.parent_module.selected_port == 3:
                    interface_transform = self.parent_module.Con_3_tf
                elif self.parent_module.selected_port == 4:
                    interface_transform = self.parent_module.Con_4_tf
            else:
                parent_name = self.parent_module.parent.name
                interface_transform = tf.transformations.identity_matrix()
        else:
            parent_name = self.parent_module.name
            interface_transform = self.parent_module.Homogeneous_tf

        transform = ModuleNode.get_rototranslation(interface_transform,
                                                   tf.transformations.rotation_matrix(angle_offset,
                                                                                      self.zaxis))
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(transform)
        x = str(x_offset)
        y = str(y_offset)
        z = str(z_offset)

        fixed_joint_name = 'L_' + str(new_socket.i) + new_socket.tag + '_fixed_joint_' + str(new_socket.p)

        ET.SubElement(self.root,
                      "xacro:add_fixed_joint",
                      name=fixed_joint_name,
                      type="fixed_joint",
                      father=parent_name,
                      child=new_socket.name,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)

        self.collision_elements.append((self.parent_module.name, new_socket.name))

        if self.speedup:
            string = ""
        else:
            # Process the urdf string by calling the process_urdf method. Parse, convert from xacro and write to string
            string = self.process_urdf()

        # update the urdf file, adding the new module
        # string = write_urdf(path_name + '/urdf/ModularBot_test.urdf', urdf_tree)

        if self.verbose:
            # Render tree
            for pre, _, node in anytree.render.RenderTree(self.base_link):
                self.print("%s%s" % (pre, node.name))

        # Create a dictionary containing the urdf string just processed and other parameters needed by the web app
        data = {'result': string,
                'lastModule_type': new_socket.type,
                'lastModule_name': new_socket.name,
                'size': new_socket.size,
                'count': new_socket.i}

        # if new_module.name.endswith('_stator'):
        #     new_module.name = selected_module[:-7]
        # last_module = anytree.search.findall_by_attr(L_0a, selected_module)[0]

        # Update the parent_module attribute of the URDF_writer class
        self.parent_module = new_socket

        # self.print(self.parent_module)

        return data

    def add_simple_ee(self, x_offset=0.0, y_offset=0.0, z_offset=0.0, angle_offset=0.0):
        # TODO: treat this as a link in the link_after_* methods!
        data = {'type': "simple_ee", 'name': "simple_ee"}

        #self.print("Parent module:")
        #self.print(self.parent_module.name)
        #self.print(self.parent_module.type)

        simple_ee = ModuleNode.ModuleNode(data, "simple_ee", parent=self.parent_module)
        setattr(simple_ee, 'tag', self.parent_module.tag)
        setattr(simple_ee, 'size', self.parent_module.size)
        setattr(simple_ee, 'i', self.parent_module.i)
        setattr(simple_ee, 'p', self.parent_module.p+1)
        setattr(simple_ee, 'robot_id', 0)
        setattr(simple_ee, 'name', 'ee' + self.parent_module.tag)

        ET.SubElement(self.root,
                      "xacro:add_simple_ee",
                      type="simple_ee",
                      name=simple_ee.name,
                      size_z=str(z_offset))

        # self.add_to_chain(simple_ee)

        trasl = tf.transformations.translation_matrix((x_offset, y_offset, z_offset))
        rot = tf.transformations.euler_matrix(0.0, 0.0, angle_offset, 'sxyz')
        rototrasl = ModuleNode.get_rototranslation(trasl, rot)
        setattr(simple_ee, 'Homogeneous_tf', rototrasl)

        if self.parent_module.type=='joint':
            transform = ModuleNode.get_rototranslation(self.parent_module.Distal_tf, rototrasl)
        else:
            transform = ModuleNode.get_rototranslation(self.parent_module.Homogeneous_tf, rototrasl)
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(transform)

        fixed_joint_name = 'L_' + str(simple_ee.i) + '_fixed_joint_' + str(simple_ee.p) + simple_ee.tag
        
        if self.parent_module.type == 'joint':
            father_name = 'L_' + str(self.parent_module.i) + self.parent_module.tag
        else:
            father_name = self.parent_module.name

        ET.SubElement(self.root,
                      "xacro:add_fixed_joint",
                      type="fixed_joint",
                      name=fixed_joint_name,
                      father=father_name,
                      child=simple_ee.name,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)

        self.collision_elements.append((father_name, simple_ee.name))

        self.parent_module = simple_ee
        self.add_to_chain(simple_ee)

        if self.speedup:
            string = ""
        else:
            # Process the urdf string by calling the process_urdf method. Parse, convert from xacro and write to string
            string = self.process_urdf()

        # Create a dictionary containing the urdf string just processed and other parameters needed by the web app
        data = {'result': string,
                'lastModule_type': simple_ee.type,
                'lastModule_name': simple_ee.name,
                'size': simple_ee.size,
                'count': simple_ee.i}

        return data

    def add_module(self, filename, angle_offset, reverse=False, robot_id=0, active_ports=3):
        """Add a module specified by filename as child of the currently selected module.

        Parameters
        ----------
        filename: str
            String with the name of the YAML file to load, describing the module parameters
        angle_offset: float
            Value of the angle between the parent module output frame and the module input frame
        reverse: bool
            Bool value expressing if the module is mounted in reverse direction (true) or in standard one (false).
            By default it is false.

        Returns
        -------
        data: dict
            Dictionary with as entries all the relevant info on the newly added module.
            In particular the updated and newly processed urdf string.

        """
        # global tag, parent_module
        self.print(path_name)
        self.print(filename)
        # Generate the path to the required YAML file
        module_name = self.resource_finder.get_filename(filename, 'yaml_path')
        # module_name = path_name + '/web/static/yaml/' + filename

        # Load the module from YAML and create a ModuleNode instance
        new_module = ModuleNode.module_from_yaml(module_name, self.parent_module, reverse)
        self.print("Module loaded from YAML: " + new_module.name)

        # self.print(angle_offset)

        # If the parent is a connector module, it means we are starting a new branch from a cube.
        # Then assign the correct tag (A, B, C, ...) to the new module (and therefore start a new branch)
        # by looking at the current tag_num (1, 2, 3, ...) and so at how many branches are already present in the robot.
        # If the parent is any other kind of module, assign as tag the same of his parent.
        if self.parent_module.type == 'cube' :
            tag_letter = self.branch_switcher.get(self.tag_num)
            setattr(new_module, 'tag', tag_letter)
            self.tag_num += 1
        else:
            setattr(new_module, 'tag', self.parent_module.tag)

        self.print('new_module.tag:', new_module.tag)

        # Set attributes of the newly added module object
        setattr(new_module, 'i', self.parent_module.i)
        setattr(new_module, 'p', self.parent_module.p)
        # Size is already set from the YAML file
        # setattr(new_module, 'size', self.parent_module.size)

        setattr(new_module, 'angle_offset', angle_offset)
        setattr(new_module, 'reverse', reverse)
        setattr(new_module, 'robot_id', robot_id)

        self.print("parent module:")
        self.print(self.parent_module)
        self.print(self.parent_module.type)

        # Update the EtherCAT port connected to the electro-mechanical interface where the new module/slave will be added 
        #    1           2           3           4
        #    o           o           o           o
        #    |           |           |           |
        # com-exp   upper port  front port    nothing
        setattr(new_module, 'selected_port', 2)
        self.print('mastercube.selected_port :', new_module.selected_port)

        # save the active ports as a binary string
        setattr(new_module, 'active_ports', "{0:04b}".format(active_ports)) 
        self.print('active_ports: ', new_module.active_ports)

        # save the occupied ports as a binary string
        setattr(new_module, 'occupied_ports', "0001")
        self.print('occupied_ports: ', new_module.occupied_ports)

        # Depending on the type of the parent module and the new module, call the right method to add the new module.
        # If the new module is a joint add it to the correct chain via the 'add_to_chain' method.
        
        #if self.parent_module.type == "base_link":

        if self.parent_module.type == 'joint':
            if new_module.type == 'joint':
                # joint + joint
                self.print("joint + joint")
                self.joint_after_joint(new_module, self.parent_module, angle_offset, reverse=reverse)
                # Add the joint to the list of chains
                self.add_to_chain(new_module)
            else:
                # joint + link
                self.print("joint + link")
                self.link_after_joint(new_module, self.parent_module, angle_offset, reverse=reverse)
        elif self.parent_module.type == 'cube':
            if new_module.type == 'joint':
                # cube + joint
                self.joint_after_cube(new_module, self.parent_module, angle_offset, reverse=reverse)
                # Add the joint to the list of chains
                self.add_to_chain(new_module)
            else:
                # cube + link
                self.link_after_cube(new_module, self.parent_module, angle_offset, reverse=reverse)
        else:
            if new_module.type == 'joint':
                # link + joint
                self.print("link + joint")
                self.joint_after_link(new_module, self.parent_module, angle_offset, reverse=reverse)
                # Add the joint to the list of chains
                self.add_to_chain(new_module)
            else:
                # link + link
                self.print("link + link")
                self.link_after_link(new_module, self.parent_module, angle_offset, reverse=reverse)

        if self.speedup:
            string = ""
        else:
            # Process the urdf string by calling the process_urdf method. Parse, convert from xacro and write to string
            string = self.process_urdf()

        # update the urdf file, adding the new module
        # string = write_urdf(path_name + '/urdf/ModularBot_test.urdf', urdf_tree)

        if self.verbose:
            # Render tree
            for pre, _, node in anytree.render.RenderTree(self.base_link):
                self.print("%s%s" % (pre, node.name))

        # Create a dictionary containing the urdf string just processed and other parameters needed by the web app
        data = {'result': string,
                'lastModule_type': new_module.type,
                'lastModule_name': new_module.name,
                'size': new_module.size,
                'count': new_module.i}

        # if new_module.name.endswith('_stator'):
        #     new_module.name = selected_module[:-7]
        # last_module = anytree.search.findall_by_attr(L_0a, selected_module)[0]

        # Update the parent_module attribute of the URDF_writer class
        self.parent_module = new_module

        # self.print(self.parent_module)

        self.info_print("Module added to URDF: " + new_module.name + " (" + new_module.type + ")")

        return data

    def remove_module(self, selected_module=0):
        """Remove the selected module (and all its childs and descendants) and return info on its parent

        Parameters
        ----------
        selected_module: ModuleNode.ModuleNode
            NodeModule object of the module to remove. Default value is 0, in which case the current parent_module is
            selected as the module to be removed.

        Returns
        -------
        data: dict
            Dictionary with as entries all the relevant info on the parent module of the removed module.
            In particular the updated and newly processed urdf string, so without the removed modules.

        """
        # global tag, n_cubes, parent_module

        # If no selected_module argument was passed to the method,
        # select the current parent module to be the one to remove
        if selected_module == 0:
            selected_module = (self.parent_module)

        # If the selected module is a connector module, select his parent (the cube) instead
        if '_con' in selected_module.name:
            selected_module = selected_module.parent

        self.info_print('Removing module: ' + str(selected_module.name) + ' (and all its descendants)')

        # If the selected module is NOT a cube start by first removing its child and descendants.
        # There is a recursive call to this function inside the loop to remove each module.
        if selected_module.type != 'cube':
            self.print('eliminate child')
            for child in selected_module.children:
                self.remove_module(child)

        self.print(selected_module.children)
        self.print(selected_module.parent.name)

        # update generator expression
        self.update_generator()
        #self.gen = (node for node in self.root.findall("*") if node.tag != 'gazebo')

        # switch depending on module type
        if selected_module.type == 'joint':
            #remove the joints from the list of chains
            self.remove_from_chain(selected_module)

            # save parent of the module to remove. This will be the last element of the chain after removal,
            # and it'll be returned by the function
            father = selected_module.parent

            # Generate names of the stator link and fixed joint to be removed from the xml tree
            stator_name = selected_module.name + '_stator'
            joint_stator_name = "fixed_" + selected_module.name
            distal_link_name = 'L_' + str(selected_module.i) + selected_module.tag

            # From the list of xml elements find the ones with name corresponding to the relative joint, stator link
            # and fixed joint before the stator link and remove them from the xml tree
            for node in self.gen:
                try:
                    if node.attrib['name'] == selected_module.name:
                        self.root.remove(node)
                    elif node.attrib['name'] == selected_module.stator_name:
                        self.root.remove(node)
                    elif node.attrib['name'] == joint_stator_name:
                        self.root.remove(node)
                    elif node.attrib['name'] == selected_module.distal_link_name:
                        self.root.remove(node)
                    elif node.attrib['name'] == selected_module.distal_link_name + '_rotor_fast':
                        self.root.remove(node)
                    elif node.attrib['name'] == 'fixed_' + selected_module.distal_link_name + '_rotor_fast':
                        self.root.remove(node)
                except KeyError:
                    pass

            self.control_plugin.remove_joint(selected_module.name)

        # TODO: This is not working in the urdf. The ModuleNode obj is removed but the elment from the tree is not
        elif selected_module.type == 'cube':
            self.listofhubs.remove(selected_module)
            # save parent of the module to remove. This will be the last element of the chain after removal,
            # and its data will be returned by the function
            father = selected_module.parent.parent
            self.print(selected_module.parent.name)

            # update attribute representing number of cubes present in the model
            self.n_cubes -= 1

            # Remove childs of the cube (so connectors!)
            for child in selected_module.children:
                for node in self.gen:
                    try:
                        if node.attrib['name'] == child.name:
                            self.root.remove(node)
                    except KeyError:
                        pass

                        # Remove the cube module from the xml tree
            for node in self.gen:
                try:
                    if node.attrib['name'] == selected_module.name:
                        self.root.remove(node)
                except KeyError:
                    pass

            # Remove the cube module from the xml tree
            for node in self.gen:
                if node.attrib['name'] == selected_module.name:
                    self.root.remove(node)
                    # gen = (node for node in self.root.findall("*") if node.tag != 'gazebo')

            # selected_module.parent = None

            # select the father module of the cube
            father_module = self.access_module(selected_module.parent.name)

            # Generate the name of the fixed joint between parent and cube
            joint_name = 'FJ_' + father_module.parent.parent.name + '_' + father_module.name
            self.print(joint_name)

            # Remove the fixed joint
            for node in self.gen:
                try:
                    if node.attrib['name'] == joint_name:
                        #self.print(joint_name)
                        self.root.remove(node)
                except KeyError:
                    pass

            # before deleting father_module set his parent property to None. Otherwise this will mess up the obj tree
            father_module.parent = None

            # delete object father_module
            del father_module
        elif selected_module.type == 'gripper':
            #remove the gripper from their chain
            self.remove_from_chain(selected_module)

            # save parent of the module to remove. This will be the last element of the chain after removal,
            # and its data will be returned by the function
            father = selected_module.parent

            # Generate te name of the fixed joint connecting the module with its parent
            fixed_joint_name = str(selected_module.name) + '_fixed_joint'

            for node in self.gen:
                try:
                    if node.attrib['name'] == fixed_joint_name:
                        self.root.remove(node)
                    elif node.attrib['name'] == selected_module.name:
                        self.root.remove(node)
                except KeyError:
                    pass

            # TO BE FIXED: ok for ros_control. How will it be for xbot2?
            self.control_plugin.remove_joint(selected_module.name+'_finger_joint1')
            self.control_plugin.remove_joint(selected_module.name+'_finger_joint2')

        elif selected_module.type == 'tool_exchanger':
            #remove the tool exchanger from the chain
            self.remove_from_chain(selected_module)

            # save parent of the module to remove. This will be the last element of the chain after removal,
            # and its data will be returned by the function
            father = selected_module.parent

            # Generate te name of the fixed joint connecting the module with its parent
            fixed_joint_name = str(selected_module.name) + '_fixed_joint'

            for node in self.gen:
                try:
                    if node.attrib['name'] == fixed_joint_name:
                        self.root.remove(node)
                    elif node.attrib['name'] == selected_module.name:
                        self.root.remove(node)
                    elif node.attrib['name'] == selected_module.pen_name:
                        self.root.remove(node)
                    elif node.attrib['name'] == 'fixed_'+selected_module.pen_name:
                        self.root.remove(node)

                except KeyError:
                    pass

        else:
            # save parent of the module to remove. This will be the last element of the chain after removal,
            # and its data will be returned by the function
            father = selected_module.parent

            # Generate te name of the fixed joint connecting the module with its parent
            fixed_joint_name = 'L_' + str(selected_module.i) + '_fixed_joint_' + str(
                selected_module.p) + selected_module.tag

            for node in self.gen:
                try:
                    if node.attrib['name'] == fixed_joint_name:
                        self.root.remove(node)
                    elif node.attrib['name'] == selected_module.name:
                        self.root.remove(node)
                except KeyError:
                    pass

            # if selected_module.type == 'link':
            #     #root.remove(root.findall("*[@name=selected_module.name]", ns)[-1])
            #     for node in self.gen:
            #         if node.attrib['name'] == selected_module.name:
            #             self.root.remove(node)
            #             self.gen = (node for node in self.root.findall("*") if node.tag != 'gazebo')
            # elif selected_module.type == 'elbow':
            #     #root.remove(root.findall("*[@name=selected_module.name]", ns)[-1])
            #     for node in self.gen:
            #         if node.attrib['name'] == selected_module.name:
            #             self.root.remove(node)
            #             self.gen = (node for node in self.root.findall("*") if node.tag != 'gazebo')
            # elif selected_module.type == 'size_adapter':
            #     #root.remove(root.findall("*[@name=selected_module.name]", ns)[-1])
            #     for node in self.gen:
            #         if node.attrib['name'] == selected_module.name:
            #             self.root.remove(node)
            #             self.gen = (node for node in self.root.findall("*") if node.tag != 'gazebo')

        if self.speedup:
            string = ""
        else:
            # Process the urdf string by calling the process_urdf method. Parse, convert from xacro and write to string
            # Update the urdf file, removing the module
            string = self.process_urdf()

        # Update parent module attribute. TODO: understand why and if it's needed
        if not self.parent_module.children:
            self.parent_module = father

        # Create a dictionary containing the urdf string just processed and other parameters needed by the web app
        if father.type == 'cube':
            data = {'result': string,
                    'lastModule_type': father.type,
                    'lastModule_name': father.name,
                    'size': father.size,
                    'count': self.n_cubes}
        else:
            data = {'result': string,
                    'lastModule_type': father.type,
                    'lastModule_name': father.name,
                    'size': father.size,
                    'count': father.i}
        # data = jsonify(data)

        if '_con' in father.name:
            self.tag_num -= 1

        # before deleting selected_module set his parent property to None. Otherwise this will mess up the obj tree
        selected_module.parent = None
        # selected_module.parent.children = None

        # delete object selected_module
        del selected_module

        if self.verbose:
            # Render tree
            for pre, _, node in anytree.render.RenderTree(self.base_link):
                self.print("%s%s" % (pre, node.name))

        return data

    def access_module(self, queried_module_name):
        """Find the selected module object in the tree and returns it. Moreover, sets it as the current parent_module.

        Parameters
        ----------
        queried_module_name: str
            String with the name of the module to access. It will be used to search the tree and find
            the relative ModuleNode object

        Returns
        -------
        last_module: ModuleNode.ModuleNode
            The object of the module with the name as passed by the string.

        """
        # global parent_module
        self.print('queried_module_name: ', queried_module_name)

        # Serch the tree by name for the selected module
        queried_module = anytree.search.findall_by_attr(self.base_link, queried_module_name)[0]

        self.print('queried_module.type: ', queried_module.type)
        
        # Update parent_module attribute
        self.parent_module = queried_module

        return queried_module

    def select_module_from_name(self, name, selected_port=None):
        """Allows to select a module from the tree. An inner call to access_module sets the selected module as the
        current parent module. Returns info on the selected module, so that the GUI can display it.

        Parameters
        ----------
        name: str
            String with the name of the module to select or the name of the mesh clicked on the GUI. It will be used to call the access_module method.
            The corresponding object module data is then put in a dictionary and returned.

        selected_port: int
            Represent the port selected if the module is a hub/box

        Returns
        -------
        data: dict
            The dictionary containing all necessary data about the selected module.

        """

        self.print(name)

        # If the selected module is the stator of a joint modify the string so to select the joint itself.
        # This is needed because from the GUI when you select a joint by clicking, the mesh corresponding to the stator
        # is selected, while the module we want to access is the joint (the stator is not part of the tree, only urdf).
        if name.endswith('_stator'):
            # Take the joint when the mesh of the joint stator is selected
            selected_module_name = name[:-7]
        elif '_con' in name:
            # Take the box as parent when a connector is selected
            selected_module_name = name[:-5]
            # Save the selected port
            selected_port = int(name[-1])
            self.print(selected_port)
        else:
            selected_module_name = name

        self.print(selected_module_name)

        # Call access_module to get the object with the requested name and sets it as parent.
        # The method doing the real work is actually access_module
        selected_module = self.access_module(selected_module_name)
        self.print(selected_module.type)
        self.print(selected_module.name)
        self.print(selected_module.robot_id)
        self.print(selected_module.active_ports)
        self.print(selected_module.occupied_ports)
        
        # TODO: Replace this with select_ports
        # binary XOR
        free_ports = int(selected_module.active_ports, 2) ^ int(selected_module.occupied_ports, 2)
        self.print("{0:04b}".format(free_ports))

        selected_module.selected_port = self.ffs(free_ports)
        self.print('selected_module.selected_port :', selected_module.selected_port)
        
        # # If parent topology is greater than 2 the parent is a switch/hub so we need to find the right port where the module is connected
        # if active_ports >= 3:
        #     for port_idx in range(2, len(mastercube.active_ports) - 1)
        #         if mastercube.active_ports[-port_idx] == 1:
        #             mastercube.selected_port = port_idx
        #             break
        
        # if parent_active_ports == 3:
        #     self.parent_module.selected_port = 3
        # elif parent_active_ports == 5:
        #     self.parent_module.selected_port = 4
        # self.print('self.parent_module.selected_port: ', self.parent_module.selected_port)

        # Select the correct port where to add the module to
        if '_con' in name:
            selected_module.selected_port = selected_port
        
        # # Update active and occupied port of the ESC and select the right port
        # self.select_ports(selected_module, name)

        # Create the dictionary with the relevant info on the selected module, so that the GUI can dispaly it.
        if selected_module.type == 'cube':
            data = {'lastModule_type': selected_module.type,
                    'lastModule_name': selected_module.name,
                    'size': selected_module.size,
                    'count': self.n_cubes}
        else:
            data = {'lastModule_type': selected_module.type,
                    'lastModule_name': selected_module.name,
                    'size': selected_module.size,
                    'count': selected_module.i}

        return data

    def select_ports(self, module, name):
        # In building mode the port is not set by reading the EtherCAT infos, but from the user which selects the
        # connector mesh with a mouse click
        if '_con' in name:
            # "Deactivate" ports which are not occupied. This is necessary if from the GUI the user selects a port
            # but doesn't attach anything to it.
            module.active_ports = module.occupied_ports  # int(module.active_ports, 2) & int(module.occupied_ports, 2)
            # Save the selected port
            selected_port = int(name[-1])
            #self.print(selected_port)
            # Set the selected_port as active
            mask = 1 << selected_port - 1
            #self.print(mask)
            #self.print(module.active_ports)
            # binary OR
            module.active_ports = "{0:04b}".format(int(module.active_ports, 2) | mask)
            #self.print(module.active_ports)
        #self.print(module.type)
        #self.print(module.name)
        #self.print(module.robot_id)
        #self.print(module.active_ports)
        #self.print(module.occupied_ports)
        # binary XOR
        free_ports = int(module.active_ports, 2) ^ int(module.occupied_ports, 2)
        #self.print("{0:04b}".format(free_ports))

        module.selected_port = self.ffs(free_ports)
        #self.print('module.selected_port :', module.selected_port)

        # # If parent topology is greater than 2 the parent is a switch/hub so we need to find the right port where
        # the module is connected if active_ports >= 3: for port_idx in range(2, len(mastercube.active_ports) - 1) if
        # mastercube.active_ports[-port_idx] == 1: mastercube.selected_port = port_idx break

        # if parent_active_ports == 3:
        #     self.parent_module.selected_port = 3
        # elif parent_active_ports == 5:
        #     self.parent_module.selected_port = 4
        # #self.print('self.parent_module.selected_port: ', self.parent_module.selected_port)

        # Select the correct port where to add the module to
        # if '_con' in name:
        #     #self.print('AAAAAAAAAAAAAAAAAAAAAAA')
        #     module.selected_port = selected_port

        return 0

    @staticmethod
    def ffs(x):
        """Returns the index, counting from 0, of the
        least significant set bit in `x`.
        """
        return (x & -x).bit_length()

    # noinspection PyPep8Naming
    def link_after_cube(self, new_Link, past_Cube, offset, reverse):
        """Adds to the URDF tree a link module as a child of a cube module

        Parameters
        ----------
        new_Link: ModuleNode.ModuleNode
            ModuleNode object of the link module to add

        past_Cube: ModuleNode.ModuleNode
            ModuleNode object of the cube module to which attach the link

        offset: float
            Value of the angle between the parent module output frame and the module input frame
        """

        setattr(new_Link, 'p', past_Cube.p + 1)

        if new_Link.type == 'link':
            setattr(new_Link, 'name', 'L_' + str(new_Link.i) + '_link_' + str(new_Link.p) + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_link",
                          type="link",
                          name=new_Link.name,
                          filename=new_Link.filename,
                          size_z=new_Link.link_size_z,
                          size=str(new_Link.size))
        elif new_Link.type == 'elbow':
            setattr(new_Link, 'name', 'L_' + str(new_Link.i) + '_elbow_' + str(new_Link.p) + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_elbow",
                          type="elbow",
                          name=new_Link.name,
                          size_y=new_Link.link_size_y,
                          size_z=new_Link.link_size_z,
                          size=str(new_Link.size))
        elif new_Link.type == 'tool_exchanger':
            setattr(new_Link, 'name', 'tool_exchanger' + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_tool_exchanger",
                          type="tool_exchanger",
                          name=new_Link.name,
                          filename=new_Link.filename)
            self.add_to_chain(new_Link)
            # HACK: add pen after tool_exchanger
            setattr(new_Link, 'pen_name', 'pen' + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_pen",
                          type="pen",
                          name=new_Link.pen_name,
                          father=new_Link.name)
        elif new_Link.type == 'gripper':
            setattr(new_Link, 'name', 'gripper' + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_gripper",
                          type="gripper",
                          name=new_Link.name,
                          filename=new_Link.filename)
            self.add_to_chain(new_Link)
            # HACK: add pen after gripper
            setattr(new_Link, 'TCP_name', 'TCP_' + new_Link.name)
            # TO BE FIXED: ok for ros_control. How will it be for xbot2?
            self.control_plugin.add_joint(new_Link.name + '_finger_joint1')
            self.control_plugin.add_joint(new_Link.name + '_finger_joint2')
        elif new_Link.type == 'size_adapter':
            setattr(new_Link, 'name', 'L_' + str(new_Link.i) + '_size_adapter_' + str(new_Link.p) + new_Link.tag)
            ET.SubElement(self.root,
                            "xacro:add_size_adapter",
                            type="size_adapter",
                            name=new_Link.name,
                            filename=new_Link.filename,
                            size_z=new_Link.link_size_z,
                        #   size_in=new_Link.size_in,
                        #   size_out=new_Link.size_out
            )
            setattr(new_Link, 'size', new_Link.size_out)
            
        self.print('past_Cube: ', past_Cube)
        self.print('past_Cube.selected_port: ', past_Cube.selected_port)

        if past_Cube.is_structural:
            parent_name = past_Cube.name

            if past_Cube.selected_port == 1:
                interface_transform = past_Cube.Con_1_tf
            elif past_Cube.selected_port == 2:
                interface_transform = past_Cube.Con_2_tf
            elif past_Cube.selected_port == 3:
                interface_transform = past_Cube.Con_3_tf
            elif past_Cube.selected_port == 4:
                interface_transform = past_Cube.Con_4_tf
        else:
            parent_name = past_Cube.parent.name

            interface_transform = tf.transformations.identity_matrix()

        self.print('past_Cube.selected_port:', past_Cube.selected_port)
        self.print('interface_transform: ', interface_transform)

        transform = ModuleNode.get_rototranslation(interface_transform,
                                                   tf.transformations.rotation_matrix(offset,
                                                                                      self.zaxis))
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(transform)

        if new_Link.type == 'tool_exchanger' or new_Link.type == 'gripper':
            fixed_joint_name = new_Link.name + '_fixed_joint'
        else:
            fixed_joint_name = 'L_' + str(new_Link.i) + '_fixed_joint_' + str(new_Link.p) + new_Link.tag

        ET.SubElement(self.root,
                      "xacro:add_fixed_joint",
                      name=fixed_joint_name,
                      type="fixed_joint",
                      father=parent_name,
                      child=new_Link.name,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)


    # noinspection PyPep8Naming
    def joint_after_cube(self, new_Joint, past_Cube, offset, reverse):
        """Adds to the URDF tree a joint module as a child of a cube module

        Parameters
        ----------
        new_Joint: ModuleNode.ModuleNode
            ModuleNode object of the joint module to add

        past_Cube: ModuleNode.ModuleNode
            ModuleNode object of the cube module to which the joint will be attached

        offset: float
            Value of the angle between the parent module output frame and the module input frame
        """

        self.print('past_Cube')

        if past_Cube.is_structural:
            parent_name = past_Cube.name

            if past_Cube.selected_port == 1:
                interface_transform = past_Cube.Con_1_tf
            elif past_Cube.selected_port == 2:
                interface_transform = past_Cube.Con_2_tf
            elif past_Cube.selected_port == 3:
                interface_transform = past_Cube.Con_3_tf
            elif past_Cube.selected_port == 4:
                interface_transform = past_Cube.Con_4_tf
        else:
            parent_name = past_Cube.parent.name

            interface_transform = tf.transformations.identity_matrix()

        self.print('past_Cube.selected_port:', past_Cube.selected_port)
        self.print('interface_transform: ', interface_transform)

        transform = ModuleNode.get_rototranslation(interface_transform,
                                                   tf.transformations.rotation_matrix(offset,
                                                                                      self.zaxis))
        # If the module is mounted in the opposite direction rotate the final frame by 180 deg., as per convention
        if reverse:
            transform = ModuleNode.get_rototranslation(transform,
                                                       tf.transformations.rotation_matrix(3.14, self.yaxis))
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(transform)

        setattr(new_Joint, 'i', 1)
        setattr(new_Joint, 'p', 0)

        setattr(new_Joint, 'name', 'J' + str(new_Joint.i) + new_Joint.tag)
        setattr(new_Joint, 'stator_name', new_Joint.name + '_stator')
        joint_stator_name = "fixed_" + new_Joint.name
        self.print('stator_name: ', new_Joint.stator_name)
        self.print('joint_stator_name: ', joint_stator_name)
        ET.SubElement(self.root, "xacro:add_fixed_joint",
                      type="fixed_joint_stator",
                      name=joint_stator_name,
                      father=parent_name, # past_Cube.name,  # TODO: check!
                      child=new_Joint.stator_name,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)

        # mesh_transform = ModuleNode.get_rototranslation(tf.transformations.rotation_matrix(-1.57, self.zaxis),
        #                                            tf.transformations.rotation_matrix(3.14, self.xaxis))
        mesh_transform = tf.transformations.identity_matrix()

        # If the module is mounted in the opposite direction rotate the final frame by 180 deg., as per convention
        if reverse:
            prox_mesh_transform = ModuleNode.get_rototranslation(mesh_transform, tf.transformations.rotation_matrix(-3.14, self.yaxis))
            prox_mesh_transform = ModuleNode.get_rototranslation(prox_mesh_transform, tf.transformations.inverse_matrix(new_Joint.Proximal_tf))
            # prox_mesh_transform = ModuleNode.get_rototranslation(mesh_transform, tf.transformations.translation_matrix((-0.0591857,0,-0.095508)))#tf.transformations.inverse_matrix(new_Joint.Proximal_tf))
            # prox_mesh_transform = ModuleNode.get_rototranslation(prox_mesh_transform, tf.transformations.rotation_matrix(3.14, self.xaxis))
            # prox_mesh_transform = ModuleNode.get_rototranslation(prox_mesh_transform,
            #                                                      tf.transformations.rotation_matrix(1.57, self.zaxis))
        else:
            prox_mesh_transform = mesh_transform
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(prox_mesh_transform)

        ET.SubElement(self.root,
                      "xacro:add_joint_stator",
                      type="joint_stator",
                      name=new_Joint.stator_name,
                      filename=new_Joint.filename,
                      size_y=new_Joint.joint_size_y,
                      size_z=new_Joint.joint_size_z,
                      size=str(new_Joint.size))

        joint_transform = ModuleNode.get_rototranslation(tf.transformations.identity_matrix(),
                                                         new_Joint.Proximal_tf)
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(joint_transform)

        joint_data = new_Joint.kinematics.joint.joint
        upper_lim = str(joint_data.upper_limit)
        lower_lim = str(joint_data.lower_limit)
        effort = str(joint_data.effort)
        velocity = str(joint_data.velocity)

        ET.SubElement(self.root,
                      "xacro:add_joint",
                      type="joint",
                      name=new_Joint.name,
                      father=new_Joint.stator_name,
                      child='L_' + str(new_Joint.i) + new_Joint.tag,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw,
                      upper_lim=upper_lim,
                      lower_lim=lower_lim,
                      effort=effort,
                      velocity=velocity)

        ####
        #ET.SubElement(self.xbot2_pid, "xacro:add_xbot2_pid", name=new_Joint.name,profile="small_mot")
        self.control_plugin.add_joint(new_Joint.name)
        ####

        if reverse:
            dist_mesh_transform = ModuleNode.get_rototranslation(new_Joint.Distal_tf, mesh_transform)
        else:
            dist_mesh_transform = mesh_transform

        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(dist_mesh_transform)

        setattr(new_Joint, 'distal_link_name', 'L_' + str(new_Joint.i) + new_Joint.tag)
        ET.SubElement(self.root,
                      "xacro:add_distal",
                      type="add_distal",
                      name=new_Joint.distal_link_name,
                      filename=new_Joint.filename)

        if reverse:
            new_Joint.Distal_tf = ModuleNode.get_rototranslation(new_Joint.Distal_tf,
                                                                 tf.transformations.rotation_matrix(3.14, self.yaxis))

        # add the fast rotor part to the inertia of the link/rotor part as a new link
        #NOTE: right now this is attached at the rotating part not to the fixed one (change it so to follow Pholus robot approach)
        ET.SubElement(self.root,
                      "xacro:add_fixed_joint",
                      type="fixed_joint",
                      name="fixed_" + 'L_' + str(new_Joint.i) + new_Joint.tag + '_rotor_fast',
                      father='L_' + str(new_Joint.i) + new_Joint.tag, #stator_name, #
                      child='L_' + str(new_Joint.i) + new_Joint.tag + '_rotor_fast',
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)
        ET.SubElement(self.root,
                      "xacro:add_rotor_fast",
                      type="add_rotor_fast",
                      name='L_' + str(new_Joint.i) + new_Joint.tag + '_rotor_fast',
                      filename=new_Joint.filename,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)

    # noinspection PyPep8Naming
    def link_after_joint(self, new_Link, past_Joint, offset, reverse):
        """Adds to the URDF tree a link module as a child of a joint module

        Parameters
        ----------
        new_Link: ModuleNode.ModuleNode
            ModuleNode object of the link module to add

        past_Joint: ModuleNode.ModuleNode
            ModuleNode object of the joint module to which attach the link

        offset: float
            Value of the angle between the parent module output frame and the module input frame
        """

        setattr(new_Link, 'p', past_Joint.p + 1)

        if new_Link.type == 'link':
            setattr(new_Link, 'name', 'L_' + str(new_Link.i) + '_link_' + str(new_Link.p) + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_link",
                          type="link",
                          name=new_Link.name,
                          filename=new_Link.filename,
                          size_z=new_Link.link_size_z,
                          size=str(new_Link.size))
        elif new_Link.type == 'elbow':
            setattr(new_Link, 'name', 'L_' + str(new_Link.i) + '_elbow_' + str(new_Link.p) + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_elbow",
                          type="elbow",
                          name=new_Link.name,
                          size_y=new_Link.link_size_y,
                          size_z=new_Link.link_size_z,
                          size=str(new_Link.size))
        elif new_Link.type == 'tool_exchanger':
            setattr(new_Link, 'name', 'tool_exchanger' + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_tool_exchanger",
                          type="tool_exchanger",
                          name=new_Link.name,
                          filename=new_Link.filename)
            # the end-effector gets added to the chain although it's not a joint. it's needed in the joint map and in
            # the config!
            self.add_to_chain(new_Link)
            # HACK: add pen after tool_exchanger
            setattr(new_Link, 'pen_name', 'pen' + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_pen",
                          type="pen",
                          name=new_Link.pen_name,
                          father=new_Link.name)
        elif new_Link.type == 'gripper':
            setattr(new_Link, 'name', 'gripper' + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_gripper",
                          type="gripper",
                          name=new_Link.name,
                          filename=new_Link.filename)
            self.add_to_chain(new_Link)
            # HACK: add pen after gripper
            setattr(new_Link, 'TCP_name', 'TCP_' + new_Link.name)
            # TO BE FIXED: ok for ros_control. How will it be for xbot2?
            self.control_plugin.add_joint(new_Link.name + '_finger_joint1')
            self.control_plugin.add_joint(new_Link.name + '_finger_joint2')
        elif new_Link.type == 'size_adapter':
            setattr(new_Link, 'name', 'L_' + str(new_Link.i) + '_size_adapter_' + str(new_Link.p) + new_Link.tag)
            ET.SubElement(self.root,
                            "xacro:add_size_adapter",
                            type="size_adapter",
                            name=new_Link.name,
                            filename=new_Link.filename,
                            size_z=new_Link.link_size_z,
                        #   size_in=new_Link.size_in,
                        #   size_out=new_Link.size_out
            )
            setattr(new_Link, 'size', new_Link.size_out)
            
        transform = ModuleNode.get_rototranslation(past_Joint.Distal_tf,
                                                   tf.transformations.rotation_matrix(offset,
                                                                                      self.zaxis))
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(transform)

        if new_Link.type == 'tool_exchanger' or new_Link.type == 'gripper':
            fixed_joint_name = new_Link.name + '_fixed_joint'
        else:
            fixed_joint_name = 'L_' + str(new_Link.i) + '_fixed_joint_' + str(new_Link.p) + new_Link.tag

        ET.SubElement(self.root,
                      "xacro:add_fixed_joint",
                      type="fixed_joint",
                      name=fixed_joint_name,
                      father='L_' + str(new_Link.i) + new_Link.tag,
                      child=new_Link.name,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)

        self.collision_elements.append(('L_' + str(new_Link.i) + new_Link.tag, new_Link.name))

    # noinspection PyPep8Naming
    def joint_after_joint(self, new_Joint, past_Joint, offset, reverse):
        """Adds to the URDF tree a joint module as a child of a joint module

        Parameters
        ----------
        new_Joint: ModuleNode.ModuleNode
            ModuleNode object of the joint module to add

        past_Joint: ModuleNode.ModuleNode
            ModuleNode object of the joint module to which the joint will be attached

        offset: float
            Value of the angle between the parent module output frame and the module input frame
        """
        transform = ModuleNode.get_rototranslation(past_Joint.Distal_tf,
                                                   tf.transformations.rotation_matrix(offset,
                                                                                      self.zaxis))
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(transform)

        setattr(new_Joint, 'i', past_Joint.i + 1)
        setattr(new_Joint, 'p', 0)

        setattr(new_Joint, 'name', 'J' + str(new_Joint.i) + new_Joint.tag)
        setattr(new_Joint, 'stator_name', new_Joint.name + '_stator')
        joint_stator_name = "fixed_" + new_Joint.name
        father_name = 'L_' + str(past_Joint.i) + past_Joint.tag
        ET.SubElement(self.root,
                      "xacro:add_fixed_joint",
                      type="fixed_joint_stator",
                      name=joint_stator_name,
                      father=father_name,
                      child=new_Joint.stator_name,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)

        self.collision_elements.append((father_name, new_Joint.stator_name))

        # mesh_transform = ModuleNode.get_rototranslation(tf.transformations.rotation_matrix(-1.57, self.zaxis),
        #                                                 tf.transformations.rotation_matrix(3.14, self.xaxis))
        mesh_transform = tf.transformations.identity_matrix()

        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(mesh_transform)

        ET.SubElement(self.root,
                      "xacro:add_joint_stator",
                      type="joint_stator",
                      name=new_Joint.stator_name,
                      filename=new_Joint.filename,
                      size_y=new_Joint.joint_size_y,
                      size_z=new_Joint.joint_size_z,
                      size=str(new_Joint.size))

        transform = ModuleNode.get_rototranslation(tf.transformations.identity_matrix(),
                                                   new_Joint.Proximal_tf)
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(transform)

        joint_data = new_Joint.kinematics.joint.joint
        upper_lim = str(joint_data.upper_limit)
        lower_lim = str(joint_data.lower_limit)
        effort = str(joint_data.effort)
        velocity = str(joint_data.velocity)

        ET.SubElement(self.root,
                      "xacro:add_joint",
                      type="joint",
                      name=new_Joint.name,
                      father=new_Joint.stator_name,
                      child='L_' + str(new_Joint.i) + new_Joint.tag,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw,
                      upper_lim=upper_lim,
                      lower_lim=lower_lim,
                      effort=effort,
                      velocity=velocity)

        ####
        #ET.SubElement(self.xbot2_pid, "xacro:add_xbot2_pid", name=new_Joint.name,profile="small_mot")
        self.control_plugin.add_joint(new_Joint.name)
        ####

        x, y, z, roll, pitch, yaw = '0', '0', '0', '0', '0', '0'

        setattr(new_Joint, 'distal_link_name', 'L_' + str(new_Joint.i) + new_Joint.tag)
        ET.SubElement(self.root,
                      "xacro:add_distal",
                      type="add_distal",
                      name=new_Joint.distal_link_name,
                      filename=new_Joint.filename)

        self.collision_elements.append((new_Joint.stator_name, new_Joint.distal_link_name))

        # add the fast rotor part to the inertia of the link/rotor part as a new link. NOTE: right now this is
        # attached at the rotating part not to the fixed one (change it so to follow Pholus robot approach)
        ET.SubElement(self.root,
                      "xacro:add_fixed_joint",
                      type="fixed_joint",
                      name="fixed_" + 'L_' + str(new_Joint.i) + new_Joint.tag + '_rotor_fast',
                      father='L_' + str(new_Joint.i) + new_Joint.tag,  # stator_name
                      child='L_' + str(new_Joint.i) + new_Joint.tag + '_rotor_fast',
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)
        ET.SubElement(self.root,
                      "xacro:add_rotor_fast",
                      type="add_rotor_fast",
                      name='L_' + str(new_Joint.i) + new_Joint.tag + '_rotor_fast',
                      filename=new_Joint.filename,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)

    # noinspection PyPep8Naming
    def joint_after_link(self, new_Joint, past_Link, offset, reverse):
        """Adds to the URDF tree a joint module as a child of a link module

        Parameters
        ----------
        new_Joint: ModuleNode.ModuleNode
            ModuleNode object of the joint module to add

        past_Link: ModuleNode.ModuleNode
            ModuleNode object of the link module to which the joint will be attached

        offset: float
            Value of the angle between the parent module output frame and the module input frame
        """
        transform = ModuleNode.get_rototranslation(past_Link.Homogeneous_tf,
                                                   tf.transformations.rotation_matrix(offset,
                                                                                      self.zaxis))
        # If the module is mounted in the opposite direction rotate the final frame by 180 deg., as per convention
        if reverse:
            transform = ModuleNode.get_rototranslation(transform,
                                                       tf.transformations.rotation_matrix(3.14, self.yaxis))
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(transform)

        setattr(new_Joint, 'i', past_Link.i + 1)
        setattr(new_Joint, 'p', 0)

        setattr(new_Joint, 'name', 'J' + str(new_Joint.i) + new_Joint.tag)
        setattr(new_Joint, 'stator_name', new_Joint.name + '_stator')
        joint_stator_name = "fixed_" + new_Joint.name
        ET.SubElement(self.root, "xacro:add_fixed_joint",
                      type="fixed_joint_stator",
                      name=joint_stator_name,
                      father=past_Link.name,  # TODO: check!
                      child=new_Joint.stator_name,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)

        self.collision_elements.append((past_Link.name, new_Joint.stator_name))

        # mesh_transform = ModuleNode.get_rototranslation(tf.transformations.rotation_matrix(-1.57, self.zaxis),
        #                                            tf.transformations.rotation_matrix(3.14, self.xaxis))
        mesh_transform = tf.transformations.identity_matrix()

        # If the module is mounted in the opposite direction rotate the final frame by 180 deg., as per convention
        if reverse:
            prox_mesh_transform = ModuleNode.get_rototranslation(mesh_transform, tf.transformations.rotation_matrix(-3.14, self.yaxis))
            prox_mesh_transform = ModuleNode.get_rototranslation(prox_mesh_transform, tf.transformations.inverse_matrix(new_Joint.Proximal_tf))
            # prox_mesh_transform = ModuleNode.get_rototranslation(mesh_transform, tf.transformations.translation_matrix((-0.0591857,0,-0.095508)))#tf.transformations.inverse_matrix(new_Joint.Proximal_tf))
            # prox_mesh_transform = ModuleNode.get_rototranslation(prox_mesh_transform, tf.transformations.rotation_matrix(3.14, self.xaxis))
            # prox_mesh_transform = ModuleNode.get_rototranslation(prox_mesh_transform,
            #                                                      tf.transformations.rotation_matrix(1.57, self.zaxis))
        else:
            prox_mesh_transform = mesh_transform
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(prox_mesh_transform)

        ET.SubElement(self.root,
                      "xacro:add_joint_stator",
                      type="joint_stator",
                      name=new_Joint.stator_name,
                      filename=new_Joint.filename,
                      size_y=new_Joint.joint_size_y,
                      size_z=new_Joint.joint_size_z,
                      size=str(new_Joint.size))

        joint_transform = ModuleNode.get_rototranslation(tf.transformations.identity_matrix(),
                                                         new_Joint.Proximal_tf)
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(joint_transform)

        joint_data = new_Joint.kinematics.joint.joint
        upper_lim = str(joint_data.upper_limit)
        lower_lim = str(joint_data.lower_limit)
        effort = str(joint_data.effort)
        velocity = str(joint_data.velocity)

        ET.SubElement(self.root,
                      "xacro:add_joint",
                      type="joint",
                      name=new_Joint.name,
                      father=new_Joint.stator_name,
                      child='L_' + str(new_Joint.i) + new_Joint.tag,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw,
                      upper_lim=upper_lim,
                      lower_lim=lower_lim,
                      effort=effort,
                      velocity=velocity)

        ####
        #ET.SubElement(self.xbot2_pid, "xacro:add_xbot2_pid", name=new_Joint.name, profile="small_mot")
        self.control_plugin.add_joint(new_Joint.name)
        ####

        if reverse:
            dist_mesh_transform = ModuleNode.get_rototranslation(new_Joint.Distal_tf, mesh_transform)
        else:
            dist_mesh_transform = mesh_transform

        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(dist_mesh_transform)

        setattr(new_Joint, 'distal_link_name', 'L_' + str(new_Joint.i) + new_Joint.tag)
        ET.SubElement(self.root,
                      "xacro:add_distal",
                      type="add_distal",
                      name=new_Joint.distal_link_name,
                      filename=new_Joint.filename)

        self.collision_elements.append((new_Joint.stator_name, new_Joint.distal_link_name))

        if reverse:
            new_Joint.Distal_tf = ModuleNode.get_rototranslation(new_Joint.Distal_tf,
                                                                 tf.transformations.rotation_matrix(3.14, self.yaxis))

        # add the fast rotor part to the inertia of the link/rotor part as a new link. NOTE: right now this is
        # attached at the rotating part not to the fixed one (change it so to follow Pholus robot approach)
        ET.SubElement(self.root,
                      "xacro:add_fixed_joint",
                      type="fixed_joint",
                      name="fixed_" + 'L_' + str(new_Joint.i) + new_Joint.tag + '_rotor_fast',
                      father='L_' + str(new_Joint.i) + new_Joint.tag,  # stator_name, #
                      child='L_' + str(new_Joint.i) + new_Joint.tag + '_rotor_fast',
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)
        ET.SubElement(self.root,
                      "xacro:add_rotor_fast",
                      type="add_rotor_fast",
                      name='L_' + str(new_Joint.i) + new_Joint.tag + '_rotor_fast',
                      filename=new_Joint.filename,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)

    # noinspection PyPep8Naming
    def link_after_link(self, new_Link, past_Link, offset, reverse):
        """Adds to the URDF tree a joint module as a child of a link module

        Parameters
        ----------
        new_Link: ModuleNode.ModuleNode
            ModuleNode object of the link module to add

        past_Link: ModuleNode.ModuleNode
            ModuleNode object of the link module to which the joint will be attached

        offset: float
            Value of the angle between the parent module output frame and the module input frame
        """

        setattr(new_Link, 'p', past_Link.p + 1)

        if new_Link.type == 'link':
            setattr(new_Link, 'name', 'L_' + str(new_Link.i) + '_link_' + str(new_Link.p) + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_link",
                          type="link",
                          name=new_Link.name,
                          filename=new_Link.filename,
                          size_z=new_Link.link_size_z,
                          size=str(new_Link.size))
        elif new_Link.type == 'elbow':
            setattr(new_Link, 'name', 'L_' + str(new_Link.i) + '_elbow_' + str(new_Link.p) + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_elbow",
                          type="elbow",
                          name=new_Link.name,
                          size_y=new_Link.link_size_y,
                          size_z=new_Link.link_size_z,
                          size=str(new_Link.size))
        elif new_Link.type == 'tool_exchanger':
            setattr(new_Link, 'name', 'tool_exchanger' + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_tool_exchanger",
                          type="tool_exchanger",
                          name=new_Link.name,
                          filename=new_Link.filename)
            # the end-effector gets added to the chain although it's not a joint. it's needed in the joint map and in the config!
            self.add_to_chain(new_Link)
            # HACK: add pen after tool_exchanger
            setattr(new_Link, 'pen_name', 'pen' + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_pen",
                          type="pen",
                          name=new_Link.pen_name,
                          father=new_Link.name)
        elif new_Link.type == 'gripper':
            setattr(new_Link, 'name', 'gripper' + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_gripper",
                          type="gripper",
                          name=new_Link.name,
                          filename=new_Link.filename)
            # the end-effector gets added to the chain although it's not a joint. it's needed in the joint map and in the config!
            self.add_to_chain(new_Link)
            # HACK: add tcp after gripper
            setattr(new_Link, 'TCP_name', 'TCP_' + new_Link.name)
            # TO BE FIXED: ok for ros_control. How will it be for xbot2?
            self.control_plugin.add_joint(new_Link.name + '_finger_joint1')
            self.control_plugin.add_joint(new_Link.name + '_finger_joint2')
        elif new_Link.type == 'size_adapter':
            setattr(new_Link, 'name', 'L_' + str(new_Link.i) + '_size_adapter_' + str(new_Link.p) + new_Link.tag)
            ET.SubElement(self.root,
                            "xacro:add_size_adapter",
                            type="size_adapter",
                            name=new_Link.name,
                            filename=new_Link.filename,
                            size_z=new_Link.link_size_z,
                        #   size_in=new_Link.size_in,
                        #   size_out=new_Link.size_out
            )
            setattr(new_Link, 'size', new_Link.size_out)

        transform = ModuleNode.get_rototranslation(past_Link.Homogeneous_tf,
                                                   tf.transformations.rotation_matrix(offset,
                                                                                      self.zaxis))
        self.print(transform)
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(transform)

        if new_Link.type == 'tool_exchanger' or new_Link.type == 'gripper':
            fixed_joint_name = new_Link.name + '_fixed_joint'
        else:
            fixed_joint_name = 'L_' + str(new_Link.i) + '_fixed_joint_' + str(new_Link.p) + new_Link.tag

        ET.SubElement(self.root,
                      "xacro:add_fixed_joint",
                      name=fixed_joint_name,
                      type="fixed_joint",
                      father=past_Link.name,
                      child=new_Link.name,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)

        self.collision_elements.append((past_Link.name, new_Link.name))

    # TODO: remove hard-coded values
    # temporary solution for multi chain robots
    # not used!
    def write_problem_description_multi(self):
        basic_probdesc_filename = self.resource_finder.get_filename('cartesio/ModularBot_cartesio_config.yaml',
                                                          'data_path')
        # basic_probdesc_filename = path_name + '/cartesio/ModularBot_cartesio_config.yaml'
        probdesc_filename = path_name + '/ModularBot/cartesio/ModularBot_cartesio_config.yaml'
        # probdesc_filename = "/tmp/modular/cartesio/ModularBot_cartesio_multichain_config.yaml"
        # probdesc_filename = self.resource_finder.get_filename('cartesio/ModularBot_cartesio_multichain_config.yaml',
        #                                                       'modularbot_path')
        probdesc = OrderedDict([])

        with open(basic_probdesc_filename, 'r') as stream:
            try:
                probdesc = ordered_load(stream, yaml.SafeLoader)
                # cartesio_stack['EE']['base_link'] = self.listofchains[0]
                #self.print(list(probdesc.items())[0])
            except yaml.YAMLError as exc:
                self.print(exc)

        #self.print(probdesc.items())
        i = 0
        tasks = []
        stack = [tasks]
        for joints_chain in self.listofchains:
            ee_name = "EE_" + str(i + 1)
            tasks.append(ee_name)
            probdesc['stack'] = stack
            probdesc[ee_name] = copy.deepcopy(probdesc['EE'])

            if joints_chain[-1].children:
                if "con" in joints_chain[-1].children[0].name:
                    tip_link = joints_chain[-1].children[0].children[0].name
                else:
                    tip_link = joints_chain[-1].children[0].name
            else:
                tip_link = 'L_' + str(joints_chain[-1].i) + joints_chain[-1].tag
                if joints_chain[-1].type == 'tool_exchanger':
                    # tip_link = joints_chain[-1].name
                    tip_link = joints_chain[-1].pen_name
                if joints_chain[-1].type == 'gripper':
                    # tip_link = joints_chain[-1].name
                    tip_link = joints_chain[-1].TCP_name
                elif joints_chain[-1].type == 'simple_ee':
                    tip_link = joints_chain[-1].name
            probdesc[ee_name]['distal_link'] = tip_link

            if "con" in joints_chain[0].parent.name:
                base_link = joints_chain[0].parent.parent.name
            else:
                base_link = joints_chain[0].parent.name
            probdesc[ee_name]['base_link'] = base_link
            probdesc[ee_name]['type'] = "Interaction"
            # probdesc[ee_name]['type'] = "Cartesian"
            # probdesc[ee_name]['lambda'] = 0.1

            i += 1

        probdesc.pop('EE', None)

        # Create folder if doesen't exist
        if not os.path.exists(os.path.dirname(probdesc_filename)):
            try:
                os.makedirs(os.path.dirname(probdesc_filename))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        with open(probdesc_filename, 'w') as outfile:
            # ordered_dump(probdesc, stream=outfile, Dumper=yaml.SafeDumper,  default_flow_style=False, line_break='\n\n', indent=4)
            ordered_dump(probdesc, stream=outfile, default_flow_style=False, line_break='\n\n', indent=4,
                         canonical=False)
        return probdesc

    # temporary solution for single chain robots
    # useful to run CartesianImpedanceController automatically
    def write_problem_description(self):
        basic_probdesc_filename = self.resource_finder.get_filename('cartesio/ModularBot_cartesio_config.yaml',
                                                                   'data_path')
        # basic_probdesc_filename = path_name + '/cartesio/ModularBot_cartesio_config.yaml'
        probdesc_filename = path_name + '/ModularBot/cartesio/ModularBot_cartesio_config.yaml'
        ##probdesc_filename = self.resource_finder.get_filename('cartesio/ModularBot_cartesio_config.yaml',
        ##                                                      'modularbot_path')
        #probdesc_filename = "/tmp/modular/cartesio/ModularBot_cartesio_config.yaml"
        probdesc = OrderedDict([])

        with open(basic_probdesc_filename, 'r') as stream:
            try:
                probdesc = ordered_load(stream, yaml.SafeLoader)
                # cartesio_stack['EE']['base_link'] = self.listofchains[0]
                self.print(list(probdesc.items())[0])
            except yaml.YAMLError as exc:
                self.print(exc)
                
        self.print(probdesc.items())
        joints_chain = self.listofchains[0]
        if joints_chain[-1].children:
            if "con" in joints_chain[-1].children[0].name:
                tip_link = joints_chain[-1].children[0].children[0].name
            else:
                tip_link = joints_chain[-1].children[0].name
        else:
            tip_link = 'L_' + str(joints_chain[-1].i) + joints_chain[-1].tag
            if joints_chain[-1].type == 'tool_exchanger':
                # tip_link = joints_chain[-1].name
                tip_link = joints_chain[-1].pen_name
            if joints_chain[-1].type == 'gripper':
                # tip_link = joints_chain[-1].name
                tip_link = joints_chain[-1].TCP_name
            elif joints_chain[-1].type == 'simple_ee':
                tip_link = joints_chain[-1].name
        probdesc['EE']['distal_link'] = tip_link

         # Create folder if doesen't exist
        if not os.path.exists(os.path.dirname(probdesc_filename)):
            try:
                os.makedirs(os.path.dirname(probdesc_filename))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        with open(probdesc_filename, 'w') as outfile:
            #ordered_dump(probdesc, stream=outfile, Dumper=yaml.SafeDumper,  default_flow_style=False, line_break='\n\n', indent=4)
            ordered_dump(probdesc, stream=outfile, default_flow_style=False, line_break='\n\n', indent=4, canonical = False)
        return probdesc


    def write_lowlevel_config(self, use_robot_id=False):
        """Creates the low level config file needed by XBotCore """
        lowlevel_config = self.control_plugin.write_lowlevel_config(use_robot_id)
        
        return lowlevel_config

    def write_joint_map(self, use_robot_id=False):
        """Creates the joint map needed by XBotCore """
        joint_map = self.control_plugin.write_joint_map(use_robot_id)
        
        return joint_map

    def write_srdf(self, builder_joint_map=None):
        """Generates a basic srdf so that the model can be used right away with XBotCore"""
        srdf = self.control_plugin.write_srdf(builder_joint_map)

        return srdf

    # Function writin the urdf file after converting from .xacro (See xacro/__init__.py for reference)
    def write_urdf(self):
        """Returns the string with the URDF, after writing it to file"""
        global path_name, path_superbuild
        urdf_filename = path_name + '/ModularBot/urdf/ModularBot.urdf'
        # urdf_filename = path_superbuild + '/configs/ADVR_shared/ModularBot/urdf/ModularBot.urdf'
        # urdf_filename = self.resource_finder.get_filename('urdf/ModularBot.urdf', 'modularbot_path')
        # urdf_filename= '/tmp/modular/urdf/ModularBot.urdf'
        out = xacro.open_output(urdf_filename)

        urdf_xacro_filename = path_name + '/ModularBot/urdf/ModularBot.urdf.xacro'
        # urdf_xacro_filename = self.resource_finder.get_filename('urdf/ModularBot.urdf.xacro', 'modularbot_path')
        # urdf_xacro_filename = '/tmp/modular/urdf/ModularBot.urdf.xacro'

        # Create folder if doesen't exist
        if not os.path.exists(os.path.dirname(urdf_xacro_filename)):
            try:
                os.makedirs(os.path.dirname(urdf_xacro_filename))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        # writing .xacro file
        # tree.write(urdf_xacro_filename, xml_declaration=True, encoding='utf-8')
        xmlstr = xml.dom.minidom.parseString(ET.tostring(self.urdf_tree.getroot())).toprettyxml(indent="   ")
        with open(urdf_xacro_filename, 'w+') as f:
            f.write(xmlstr)

        # parse the document into a xml.dom tree
        doc = xacro.parse(None, urdf_xacro_filename)
        # doc = xacro.parse(doc)

        # perform macro replacement
        xacro.process_doc(doc)

        # self.print(doc.lastChild.toprettyxml(indent='  '))

        # add xacro auto-generated banner
        banner = [xml.dom.minidom.Comment(c) for c in
                  [" %s " % ('=' * 83),
                   " |    This document was autogenerated by xacro from %-30s | " % urdf_xacro_filename,
                   " |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED  %-30s | " % "",
                   " %s " % ('=' * 83)]]
        first = doc.firstChild
        for comment in banner:
            doc.insertBefore(comment, first)

        out.write(doc.toprettyxml(indent='  '))

        return doc.toprettyxml(indent='  ')

    # Save URDF/SRDF etc. in a directory with the specified robot_name
    def deploy_robot(self, robot_name):
        script = self.resource_finder.get_filename('deploy.sh', 'data_path')
        deploy_dir = os.path.expanduser(self.resource_finder.cfg['deploy_dir'])
        deploy_dir = os.path.expandvars(deploy_dir)

        if self.verbose:
            output = subprocess.check_output([script, robot_name, "--destination-folder", deploy_dir, "-v"])
        else:
            output = subprocess.check_output([script, robot_name, "--destination-folder", deploy_dir])
        
        self.info_print(str(output, 'utf-8', 'ignore'))

        self.add_connectors()

        return robot_name

    # Remove connectors when deploying the robot
    def remove_connectors(self):

        # update generator expression
        self.update_generator()

        # Remove the fixed joints that join the connectors to the boxes (by checking if 'con' is in the name of the child)
        # Catch KeyError when the node has no child element and continue with the loop.
        for node in self.gen:
            try:
                node_type = node.attrib['type']
                if node_type == 'connectors':
                    self.print('removing node:', node.attrib)
                    self.root.remove(node)
            except KeyError:
                #self.print('missing type', node.attrib['name'])
                continue

        # Process the urdf string by calling the process_urdf method. Parse, convert from xacro and write to string
        # Update the urdf file, removing the module
        string = self.process_urdf()

        if self.verbose:
            # Render tree
            for pre, _, node in anytree.render.RenderTree(self.base_link):
                self.print("%s%s" % (pre, node.name))

        return string

    def add_connectors(self):

        # update generator expression
        self.update_generator()

        for node in self.gen:
            try:
                node_type = node.attrib['type']
                if node_type == 'cube':
                    name = node.attrib['name']
                    filename = path_name + '/web/static/yaml/master_cube.yaml'
                    #filename = self.resource_finder.get_filename('master_cube.yaml', 'yaml_path')

                    ET.SubElement(self.root, "xacro:add_connectors", type='connectors', name=name, filename=filename)
            except KeyError:
                #self.print('missing type', node.attrib['name'])
                continue
        
        
