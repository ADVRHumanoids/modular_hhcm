#!/usr/bin/env python3
# Disable some of the pylint violations in this file
# see https://pylint.pycqa.org/en/latest/user_guide/messages/message_control.html#block-disables
# pylint: disable=line-too-long, missing-function-docstring, missing-module-docstring

from __future__ import print_function
import math
import inspect
import numpy as np
from future.utils import iteritems
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

from modular.utils import ResourceFinder, ModularResourcesManager
from modular.enums import ModuleType, ModuleClass
from modular.ModelStats import ModelStats
import modular.ModuleNode as ModuleNode
import argparse

# import rospy
# import roslaunch
# import rospkg
# import tf


# from anytree import NodeMixin, RenderTree, Node, AsciiStyle
import anytree
from anytree import RenderTree

import subprocess
from shutil import copyfile
import os
import errno
import sys

# tf_transformations is imported transitively via ModuleNode (single source of truth)
tf_transformations = ModuleNode.tf_transformations

currDir = os.path.dirname(os.path.realpath(__file__))
# print(currDir)
rootDir = os.path.abspath(os.path.join(currDir, '../..'))
# print(rootDir)
if rootDir not in sys.path:  # add parent dir to paths
    sys.path.append(rootDir)

# ---------------------------------------------------------------------------
# Components extracted from this module for better maintainability
# ---------------------------------------------------------------------------
# YAML utilities
from modular.yaml_utils import ordered_load, ordered_dump  # noqa: F401 (re-exported)
# Enums
from modular.enums import SlaveDescMode  # noqa: F401 (re-exported)
# Control-plugin strategy classes
from modular.plugins import Plugin, RosControlPlugin, XBotCorePlugin, XBot2Plugin  # noqa: F401 (re-exported)
# Low-level XML element builder
from modular.urdf_xml_builder import URDFXmlBuilder, NS_XACRO, ns  # noqa: F401 (re-exported)
# Kinematic-chain manager
from modular.chain_manager import ChainManager

ET.register_namespace("xacro", NS_XACRO)

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

def repl_option():
    parser = argparse.ArgumentParser()
    # parser.add_argument("-f", "--file_yaml", dest="esc_type_yaml", action="store", default="esc_type.yaml")
    parser.add_argument("-f", "--file_yaml", dest="robot_id_yaml", action="store", default="./robot_id.yaml")
    parser.add_argument("-c", dest="cmd_exec_cnt", action="store", type=int, default=1)
    args = parser.parse_args()
    dict_opt = vars(args)
    return dict_opt

# noinspection PyUnresolvedReferences
class UrdfWriter:
    def __init__(self,
                config_file='config_file.yaml',
                control_plugin='xbot2',
                elementree=None,
                speedup=False,
                parent=None,
                floating_base=False,
                verbose=False,
                quiet=False,
                logger=None,
                slave_desc_mode='use_pos'):
        self.config_file = config_file
        self.resource_finder = ResourceFinder(self.config_file)
        self.modular_resources_manager = ModularResourcesManager(self.resource_finder)
        self.reset(control_plugin,
                    elementree,
                    speedup,
                    parent,
                    floating_base,
                    verbose,
                    quiet,
                    logger,
                    slave_desc_mode)

    def reset(self,
                control_plugin='xbot2',
                elementree=None,
                speedup=False,
                parent=None,
                floating_base=False,
                verbose=False,
                quiet=False,
                logger=None,
                slave_desc_mode='use_pos'):

        # Setting this variable to True, speed up the robot building.
        # To be used when the urdf does not need to be shown at every iteration
        self.speedup = speedup

        self.parent = parent

        # xacro mappings to perform args substitution (see template.urdf.xacro)
        self.default_xacro_mappings = {'modular_path': os.path.dirname(os.path.realpath(__file__)),
                                'floating_base': 'false',
                                'gazebo_urdf': 'false',
                                'velodyne': 'false',
                                'realsense': 'false',
                                'ultrasound': 'false',
                                'use_gpu_ray': 'false',
                                'reflect_rotor_inertia': 'false',}

        # additional xacro mappings for addons, external xacro files, etc.
        self.additional_xacro_mappings = {}
        # Keep track of discovered sensor names by type (e.g. camera, lidar).
        self.sensor_names = {}

        self.set_floating_base(floating_base)


        if logger is None:
            # Use the module's named logger so it fits into the 'modular.*' hierarchy.
            self.logger = logging.getLogger(__name__)
            if not self.logger.handlers:
                handler = logging.StreamHandler(sys.stderr)
                handler.setFormatter(logging.Formatter('%(levelname)s:%(name)s:%(message)s'))
                self.logger.addHandler(handler)
                # Keep logs local to this logger to avoid duplicate messages from root handlers.
                self.logger.propagate = False
        else:
            self.logger = logger

        self.verbose = verbose
        self.quiet = quiet
        # Set logging level: CRITICAL if quiet, DEBUG if verbose, INFO otherwise.
        if self.quiet:
            self.logger.setLevel(logging.CRITICAL)
        elif self.verbose:
            self.logger.setLevel(logging.DEBUG)
        else:
            self.logger.setLevel(logging.INFO)

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
            template = self.resource_finder.get_string('urdf/template.urdf.xacro', ['data_path'])
            # we load the template as a ET Element. Store the 'robot' element as root
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
            library_filename = self.resource_finder.get_filename('urdf/ModularBot.library.urdf.xacro', ['data_path'])
            control_filename = self.resource_finder.get_filename('urdf/ModularBot.control.urdf.xacro', ['data_path'])
            for include in self.root.findall('xacro:include', ns):
                if include.attrib['filename'] == 'ModularBot.library.urdf.xacro':
                    include.attrib['filename'] = library_filename
                elif include.attrib['filename'] == 'ModularBot.control.urdf.xacro':
                    include.attrib['filename'] = control_filename

            self.control_plugin.add_plugin()

        else:
            self.root = elementree
            self.urdf_tree = ET.ElementTree(self.root)

        self.tag_num = 0
        self.branch_switcher = {
            0: '',
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

        self.origin, self.xaxis, self.yaxis, self.zaxis = (0, 0, 0.4), (1, 0, 0), (0, 1, 0), (0, 0, 1)

        data = {'type': "base_link", 'name': "base_link", 'kinematics_convention': "urdf"}

        self.base_link = ModuleNode.ModuleNode(data, "base_link")
        setattr(self.base_link, 'name', "base_link")
        setattr(self.base_link, 'tag', self.branch_switcher.get(self.tag_num))
        setattr(self.base_link, 'flange_size', '3')
        setattr(self.base_link, 'i', 0)
        setattr(self.base_link, 'p', 0)
        setattr(self.base_link, 'Homogeneous_tf', tf_transformations.identity_matrix())
        setattr(self.base_link, 'robot_id', 0)
        setattr(self.base_link, 'current_port', 1)
        #HACK: base_link does not have ports.
        setattr(self.base_link, 'active_ports', "0011")
        setattr(self.base_link, 'occupied_ports', "0001")
        setattr(self.base_link, 'connectors', ['connector_0'])
        setattr(self.base_link, 'connector_idx', 1)
        setattr(self.base_link, 'is_structural', False)
        setattr(self.base_link, 'mesh_names', [])

        self.listofchains = [[self.base_link]]
        self.listofhubs = []
        # Delegation targets for extracted submodules.
        self.chain_manager = ChainManager(self)
        self.xml_builder = URDFXmlBuilder(self)

        self.parent_module = self.base_link

        # update generator expression
        self.update_generators()

        # map between name of the mesh and the module
        self.mesh_to_module_map = {}

        self.process_urdf()
        self.urdf_dirty = False

        self.model_stats = ModelStats(self)

        # set the slave description mode
        try:
            self.slave_desc_mode = SlaveDescMode(slave_desc_mode)
        except ValueError:
            self.slave_desc_mode = SlaveDescMode.USE_POSITIONS
            self.info_print('Slave description mode not recognized! Defaulting to USE_POSITIONS')

    def set_floating_base(self, floating_base):
        """Set the floating base flag"""
        self.floating_base = floating_base
        self.update_default_xacro_mappings()

    def update_default_xacro_mappings(self):
        """Update the xacro mappings to perform args substitution (see template.urdf.xacro).
        To be called when the floating base status changes."""
        if self.floating_base:
            self.default_xacro_mappings['floating_base'] = 'true'
        else:
            self.default_xacro_mappings['floating_base'] = 'false'

    def print(self, *args):
        msg = ' '.join(str(a) for a in args)
        if isinstance(self.logger, logging.Logger):
            self.logger.debug(msg)
        else:
            print(msg)

    def info_print(self, *args):
        msg = ' '.join(str(a) for a in args)
        if isinstance(self.logger, logging.Logger):
            self.logger.info(msg)
        else:
            print(msg)

    def error_print(self, *args):
        msg = ' '.join(str(a) for a in args)
        if isinstance(self.logger, logging.Logger):
            self.logger.error(msg)
        else:
            print(msg, file=sys.stderr)

    def warning_print(self, *args):
        msg = ' '.join(str(a) for a in args)
        if isinstance(self.logger, logging.Logger):
            self.logger.warning(msg)
        else:
            print(msg, file=sys.stderr)

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


    def sort_modules_by_pos(self, modules_dict):

        ordered_chain = [None] * len(modules_dict)

        for key, item in modules_dict.items():

            module_position = key

            try:
                ordered_chain[module_position - 1] = item
            
            except IndexError:
                self.print('unexpected module position {}, modules number: {}'.format(module_position, len(modules_dict)))
                return list()
        
        return ordered_chain


    # This method will be used when branches in the robot will be supported.
    def read_from_json(self, json_data):
        # HACK: we keep track of how many sockets we have to add to insert a default offset
        socket_counter = 0

        # If a tree representing the topology was already instantiated, re-initialize and start from scratch
        if self.root != 0:
            self.print("Re-initialization")
            self.__init__(config_file=self.config_file,
                control_plugin=self.control_plugin,
                speedup=self.speedup,
                verbose=self.verbose,
                quiet=self.quiet,
                logger=self.logger,
                slave_desc_mode=self.slave_desc_mode)

        robot_id_yaml = self.resource_finder.get_filename('robot_id.yaml')
        robot_id_dict = yaml.safe_load(open(robot_id_yaml, 'r'))

        module_params_yaml = self.resource_finder.get_filename('module_params.yaml')
        module_params_dict = yaml.safe_load(open(module_params_yaml, 'r'))

        # Process the modules described in the json to create the tree
        modules_dict = yaml.safe_load(json_data)
        
        if self.slave_desc_mode is SlaveDescMode.USE_POSITIONS:
            # Sort the modules by position
            modules_list = self.sort_modules_by_pos(modules_dict)
        elif self.slave_desc_mode is SlaveDescMode.USE_IDS:
            # Sort the modules by id
            modules_list = self.sort_modules(modules_dict)
        else:
            raise ValueError('Slave description mode not recognized!')

        for module in modules_list:

            module_position = int(module['position'])
            module['robot_id'] = int(module['robot_id']) if module['robot_id'] != -1 else module_position*(-1)
            robot_id = module['robot_id']
            active_ports = int(module['active_ports'])

            mod_type = int(module['mod_type'])
            mod_id = int(module['mod_id'])
            mod_size = int(module['mod_size'])
            mod_rev = int(module['mod_rev'])

            module_filename = module_params_dict.get(mod_type, {}).get(mod_id,{}).get(mod_size,{}).get(mod_rev)
            if module_filename is None:
                module_filename = robot_id_dict.get(robot_id)
                if module_filename is None:
                    self.info_print("Id not recognized! Skipping add_module() for id", robot_id_dict.get(robot_id))          
                    continue

            self.info_print('Discovered module with ID:', robot_id)

            parent_position = None

            # find parent (from OpenEtherCATsociety)
            if (module_position > 1):
                topo_counter = 0
                candidate_position = module_position - 1

                while candidate_position > 0:
                    candidate_parent = modules_list[candidate_position - 1]
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

            self.print("module and parent:", robot_id, parent_position)

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

                # select the correct parent module from its id and sets the correct current port
                self.select_module_from_id(parent_id)

                # set the current_port as occupied
                mask = 1 << self.eth_to_physical_port_idx(self.parent_module.current_port)
                self.print(mask)
                self.print(self.parent_module.occupied_ports)
                self.parent_module.occupied_ports = "{0:04b}".format(int(self.parent_module.occupied_ports, 2) | mask)
                self.print(self.parent_module.occupied_ports)

            #HACK: If the parent is a cube to support non-structural box we add a socket
            if self.parent_module.type is ModuleType.CUBE:
                if not self.parent_module.is_structural:
                    self.add_module('socket.yaml', {'x': float(socket_counter)}, reverse=False)
                    socket_counter += 1

            # HACK: manually set name of mobile platform to be 'mobile_base', instead of auto-generated name
            module_name = None
            if module_filename=='concert/mobile_platform_concert.json':
                module_name = 'mobile_base'

            #add the module
            data = self.add_module(module_filename, {}, reverse=False, robot_id=robot_id, active_ports=active_ports, module_name=module_name)

        self.process_urdf()

        self.info_print("Discovery completed")

        data = {'string': self.urdf_string}
        return data

    def render_tree(self):
        for pre, _, node in RenderTree(self.base_link):
            self.print(pre, node, node.name, node.robot_id)

        return 0

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

    def process_urdf(self, xacro_mappings={}):
        """Process the urdf to convert from xacro and perform macro substitutions. Returns urdf string"""

        # write the urdf tree to a string
        xmlstr = xml.dom.minidom.parseString(ET.tostring(self.urdf_tree.getroot())).toprettyxml(indent="   ")
        # self.print(xmlstr)

        # parse the string to convert from xacro
        doc = xacro.parse(xmlstr)

        # mappings = self.default_xacro_mappings if xacro_mappings else xacro_mappings
        mappings = copy.deepcopy(self.default_xacro_mappings)
        mappings.update(xacro_mappings)
        # we pass the additioonal mappings to the process_doc function as well
        mappings.update(self.additional_xacro_mappings)

        # perform macro replacement
        xacro.process_doc(doc, mappings=mappings)

        # string = doc.toprettyxml(indent='  ')
        string = doc.toprettyxml(indent='  ', encoding='utf-8').decode('utf-8')

        self.urdf_string = string
        self.urdf_dirty = False

        return string

    def update_urdf_cache(self):
        """Regenerate URDF immediately unless speedup mode is enabled."""
        if self.speedup:
            self.urdf_dirty = True
            return self.urdf_string

        return self.process_urdf()

    def add_to_chain(self, new_joint):
        """Add joint to one of the robot kinematic chains."""
        return self.chain_manager.add_to_chain(new_joint)

    def remove_from_chain(self, joint):
        """Remove joint from the list of the robot kinematic chains."""
        return self.chain_manager.remove_from_chain(joint)

    def get_actuated_modules_chains(self):
        return self.chain_manager.get_actuated_modules_chains()

    def get_ET(self):
        return self.urdf_tree

    def get_parent_module(self):
        return self.parent_module
    
    @staticmethod
    def find_chain_tip_link(chain):
        return ChainManager.find_chain_tip_link(chain)
    
    @staticmethod
    def find_chain_base_link(chain):
        return ChainManager.find_chain_base_link(chain)

    @staticmethod
    def find_chain_tag(chain):
        return ChainManager.find_chain_tag(chain)

    def update_generators(self):
        # Generator expression for list of urdf elements without the gazebo tag.
        # This is needed because of the change in the xacro file, as gazebo simulation tags
        # are now added from the start and this creates problems with the search
        nodes = set(self.root.findall("*"))
        gazebo_nodes = set(self.root.findall("./gazebo"))
        xacro_include_nodes = set(self.root.findall('./xacro:include', ns))
        xacro_if_nodes = set(self.root.findall('./xacro:if', ns))
        # xacro_macro_nodes = set(self.root.findall('./xacro:macro', ns))
        xacro_property_nodes = set(self.root.findall('./xacro:property', ns))
        xacro_arg_nodes = set(self.root.findall('./xacro:arg', ns))
        filtered_nodes = nodes.difference(gazebo_nodes).difference(xacro_include_nodes).difference(xacro_if_nodes).difference(xacro_property_nodes).difference(xacro_arg_nodes)
        self.urdf_nodes_generator = (node for node in filtered_nodes)
        self.gazebo_nodes_generator = (node for node in gazebo_nodes)

    # Adds a table for simulation purposes
    def add_table(self):
        data = {'type': "link", 'name': "table"}

        table = ModuleNode.ModuleNode(data, "table", parent=self.base_link)
        setattr(table, 'name', "table")
        setattr(table, 'tag', "_A")
        setattr(table, 'flange_size', 3)
        setattr(table, 'i', 0)
        setattr(table, 'p', 0)
        setattr(table, 'Homogeneous_tf', tf_transformations.identity_matrix())
        setattr(table, 'robot_id', 0)

        ET.SubElement(self.root,
                      "xacro:add_table",
                      type="link",
                      name="table",
                      father=self.parent_module.name)

        self.collision_elements.append((self.parent_module.name, "table"))

        self.parent_module = table

        # Select the current connector of the new module
        selected_connector = self.select_connector(table)
        # Select the meshes to highlight in the GUI
        selected_meshes = self.select_meshes(selected_connector, table)

        self.update_urdf_cache()

        # Create the dictionary with the relevant info on the selected module, so that the GUI can dispaly it.
        data = {'name': table.name,
                'type': table.type,
                'flange_size': table.flange_size,
                'selected_connector': selected_connector,
                'selected_meshes': selected_meshes,
                'urdf_string': self.urdf_string}

        return data
    
    def add_drillbit(self, length=0.27, radius=0.012, mass=0.1):
        drillbit_name = 'ee'+ self.parent_module.tag
        ET.SubElement(self.root,
            "xacro:add_cylinder",
            type="drillbit",
            name=drillbit_name,
            size_z=str(length),
            mass=str(mass),
            radius=str(radius))
        self.parent_module.mesh_names.append(drillbit_name)

        trasl = tf_transformations.translation_matrix((0.0, 0.0, length))
        rot = tf_transformations.euler_matrix(0.0, 0.0, 0.0, 'sxyz')
        transform = ModuleNode.get_rototranslation(trasl, rot)
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(transform)

        father_name = self.parent_module.tcp_name

        ET.SubElement(self.root,
                      "xacro:add_fixed_joint",
                      type="fixed_joint",
                      name="fixed_" + drillbit_name,
                      father=father_name,
                      child=drillbit_name,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)
        
        self.collision_elements.append((father_name, drillbit_name))

        self.update_urdf_cache()

        return [drillbit_name, "fixed_" + drillbit_name]

    def add_handle(self, x_offset=0.0, y_offset=0.25, z_offset=-0.18, mass=0.330, radius=0.025):
        handle_name = 'handle'+ self.parent_module.tag
        ET.SubElement(self.root,
            "xacro:add_cylinder",
            type="handle",
            name=handle_name,
            size_z=str(abs(y_offset)),
            mass=str(mass),
            radius=str(radius))
        self.parent_module.mesh_names.append(handle_name)

        trasl = tf_transformations.translation_matrix((x_offset, y_offset, z_offset))
        if y_offset >= 0.0:
            rot = tf_transformations.euler_matrix(-1.57, 0.0, 0.0, 'sxyz')
        else:
            rot = tf_transformations.euler_matrix(1.57, 0.0, 0.0, 'sxyz')
        transform = ModuleNode.get_rototranslation(trasl, rot)
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(transform)

        father_name = self.parent_module.tcp_name

        ET.SubElement(self.root,
                      "xacro:add_fixed_joint",
                      type="fixed_joint",
                      name="fixed_" + handle_name,
                      father=father_name,
                      child=handle_name,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)
        
        self.collision_elements.append((father_name, handle_name))

        # Add also a frame on the handle gripping point
        trasl = tf_transformations.translation_matrix((0.0, y_offset/2, z_offset))
        rot = tf_transformations.euler_matrix(0.0, 0.0, 0.0, 'sxyz')
        transform = ModuleNode.get_rototranslation(trasl, rot)
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(transform)

        handle_gripping_point_name = 'handle_gripping_point'+ self.parent_module.tag
        ET.SubElement(self.root,
                      "xacro:add_fixed_joint",
                      type="fixed_joint",
                      name="fixed_" + handle_gripping_point_name,
                      father=father_name,
                      child=handle_gripping_point_name,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)
        
        ET.SubElement(self.root,
                        "link",
                        name=handle_gripping_point_name)

        self.update_urdf_cache()

        return [handle_name, "fixed_" + handle_name, handle_gripping_point_name, "fixed_" + handle_gripping_point_name]

    def add_dagana_claws(self, type='centauro_claws'):
        if type == 'centauro_claws':
            self.additional_xacro_mappings['dagana_claws_type'] = 'centauro_claws'
        elif type == 'concert_formwork_claws':
            self.additional_xacro_mappings['dagana_claws_type'] = 'concert_formwork_claws'
        elif type == 'concert_tube_claws':
            self.additional_xacro_mappings['dagana_claws_type'] = 'concert_tube_claws'
        
        # return empty list since all xml elemnts are added from xacro
        return []
    
    def add_camera(self, xyz_offset=[0.0, 0.0, 0.0], rpy_offset=[0.0, 0.0, 0.0], camera_name: str | None = None,
                   parent_name: str | None = None,
                   gazebo_urdf: str | bool = "${GAZEBO_URDF}",
                   variant: str | None = None,
                   publish_tf: str | bool = "${ADD_CAMERAS}"):
        _ = variant  # reserved for future camera-specific specializations
        camera_name = camera_name or ('camera'+ self.parent_module.tag)
        parent_name = parent_name or self.parent_module.name
        ET.SubElement(self.root, 
                      "xacro:include",
                      filename="${MODULAR_PATH}/modular_data/urdf/concert.sensors.urdf.xacro")
        et = ET.SubElement(self.root,
                      "xacro:add_rgbd_camera",
                      name=camera_name,
                      parent_name=parent_name,
                      publish_tf=str(publish_tf).lower() if isinstance(publish_tf, bool) else str(publish_tf),
                      gazebo_urdf=str(gazebo_urdf).lower() if isinstance(gazebo_urdf, bool) else str(gazebo_urdf))
        ET.SubElement(et,
                      "origin",
                      xyz=" ".join([str(x) for x in xyz_offset]),
                      rpy=" ".join([str(x) for x in rpy_offset]))
        self.parent_module.mesh_names.append(camera_name + "_link")
        self.add_sensor_name('camera', camera_name)
        return [camera_name]
    
    def add_realsense(self, camera: str = "d435", align_depth: bool = False,
                        add_plug: bool = False, use_mesh: bool = True,
                        enable_infrared: bool = False,
                        publish_pointcloud:bool = False,
                        color_image: dict = {"width": 640, "height": 480, "fps": 30},
                        depth_image: dict = {"width": 640, "height": 480, "fps": 30},
                        xyz_offset=[0.0, 0.0, 0.0], rpy_offset=[0.0, 0.0, 0.0],
                        camera_name: str | None = None,
                        parent_name: str | None = None,
                        variant: str | None = None,
                        publish_tf: str | bool = "${ADD_CAMERAS}",
                        gazebo_urdf: str | bool = "${GAZEBO_URDF}"):
        """
                        Add a Realsense camera to the URDF.
                        Supported camera types are: 'd435', 'd435i'.
                        Args:
                            camera (str): Camera model type. Supported types are 'd435' or 'd435i'. Defaults to "d435".
                            align_depth (bool): Whether to align depth image to color image. Defaults to True.
                            add_plug (bool): Whether to add camera connector plug to the model. Defaults to False.
                            use_mesh (bool): Whether to use mesh geometry for the camera model. Defaults to True.
                            enable_infrared (bool): Whether to enable infrared sensors. Defaults to False.
                            color_image (dict): Color camera configuration with keys 'width', 'height', and 'fps'.
                                Defaults to {"width": 640, "height": 480, "fps": 30}.
                            depth_image (dict): Depth camera configuration with keys 'width', 'height', and 'fps'.
                                Defaults to {"width": 640, "height": 480, "fps": 30}.
                            xyz_offset (list): XYZ positional offset from parent module in meters [x, y, z].
                                Defaults to [0.0, 0.0, 0.0].
                            rpy_offset (list): Roll, pitch, yaw rotational offset from parent module in radians [r, p, y].
                                Defaults to [0.0, 0.0, 0.0].
                            camera_name (str | None): Optional explicit camera frame/name used in the URDF.
                                If omitted, defaults to "camera" + parent tag.
                            parent_name (str | None): Optional explicit parent link name for the camera macro.
                                If omitted, defaults to the currently selected parent module name.
                        Returns:
                            list: List containing the camera name identifier.
                        Raises:
                            ValueError: If camera type is not one fo the supported types.
                        Examples:
                            Add a d435 camera with custom resolution:
                                add_realsense(camera="d435", color_image={"width": 1280, "height": 720, "fps": 30})
                            Add a d435i camera with offset positioning:
                                add_realsense(camera="d435i", xyz_offset=[0.05, 0.0, 0.1], rpy_offset=[0.0, 0.785, 0.0])
                        """

        _ = variant  # reserved for future camera-family customizations

        # Filter supported camera types
        supported_cameras = ["d435", "d435i"]
        if camera not in supported_cameras:
            raise ValueError(f"Camera type '{camera}' not supported. Supported types are: {', '.join(supported_cameras)}.")

        camera_name = camera_name or ('camera'+ self.parent_module.tag)
        parent_name = parent_name or self.parent_module.name
        macro_name = 'sensor_' + camera
        xacro_filename = f"_{camera}.urdf.xacro"
        realsense_xacro_path = self.resource_finder.get_external_resource_filename(
            'realsense_gazebo_description_path',
            [f'urdf/{xacro_filename}', xacro_filename],
        )
        ET.SubElement(self.root, 
                      "xacro:include",
                      filename=realsense_xacro_path)
        et = ET.SubElement(self.root,
                      f"xacro:{macro_name}",
                      parent=parent_name,
                      name=camera_name,
                      use_nominal_extrinsics="true",
                      add_plug=str(add_plug).lower(),
                      use_mesh=str(use_mesh).lower(),
                      publish_tf=str(publish_tf).lower() if isinstance(publish_tf, bool) else str(publish_tf),
                      gazebo_urdf=str(gazebo_urdf).lower() if isinstance(gazebo_urdf, bool) else str(gazebo_urdf),
                      align_depth=str(align_depth).lower(),
                      enable_infrared=str(enable_infrared).lower(),
                      publish_pointcloud=str(publish_pointcloud).lower(),
                      visualize="true",
                      color_width=str(color_image.get("width", 640)),
                      color_height=str(color_image.get("height", 480)),
                      color_fps=str(color_image.get("fps", 30)),
                      depth_width=str(depth_image.get("width", 640)),
                      depth_height=str(depth_image.get("height", 480)),
                      depth_fps=str(depth_image.get("fps", 30)))
        ET.SubElement(et,
                      "origin",
                      xyz=" ".join([str(x) for x in xyz_offset]),
                      rpy=" ".join([str(x) for x in rpy_offset]))
        if not align_depth:
            self.add_sensor_name('camera/realsense', camera_name)
        else:
            self.add_sensor_name('camera/realsense_depth_aligned', camera_name)
        self.parent_module.mesh_names.append(camera_name + "_link")
        return [camera_name]
    
    def add_imu(self, xyz_offset=[0.0, 0.0, 0.0], rpy_offset=[0.0, 0.0, 0.0], imu_name: str | None = None,
                parent_name: str | None = None, gazebo_urdf: str | bool = "${GAZEBO_URDF}",
                variant: str | None = None,
                publish_tf: str | bool = "${ADD_IMU}"):
        variant_name = variant if variant else "generic"
        imu_name = imu_name or ('imu'+ self.parent_module.tag)
        imu_link_name = imu_name + "_link"
        parent_name = parent_name or self.parent_module.name
        ET.SubElement(self.root,
                      "xacro:include",
                      filename="${MODULAR_PATH}/modular_data/urdf/concert.sensors.urdf.xacro")
        et = ET.SubElement(self.root,
                      "xacro:add_imu",
                      name=imu_name,
                      parent_name=parent_name,
                      gazebo_urdf=str(gazebo_urdf).lower() if isinstance(gazebo_urdf, bool) else str(gazebo_urdf),
                      publish_tf=str(publish_tf).lower() if isinstance(publish_tf, bool) else str(publish_tf))
        ET.SubElement(et,
                      "origin",
                      xyz=" ".join([str(x) for x in xyz_offset]),
                      rpy=" ".join([str(x) for x in rpy_offset]))
        self.parent_module.mesh_names.append(imu_link_name)
        self.add_sensor_name("imu/"+variant_name, imu_link_name)
        return [imu_name]

    def add_sensor_name(self, sensor_type: str, sensor_name: list[str] | str):
        """
        Add one or more sensor names to the sensor registry.

        Args:
            sensor_type (str): Sensor category used to group the sensor
                names. Uses '/' as nested subcategory separator. For example, `camera/realsense` 
                identifies a realsense subclass in camera category, while `force_torque_sensor` 
                identifies a generic force-torque sensor category.
            sensor_name (list[str] | str): Sensor name to add. If a list is
                provided, each sensor name in the list is added.

        Returns:
            None
        """
        if sensor_name is None:
            return

        if isinstance(sensor_name, (list, tuple, set)):
            for name in sensor_name:
                self.add_sensor_name(sensor_type, name)
            return

        sensor_list = self.sensor_names.setdefault(sensor_type, [])
        if sensor_name not in sensor_list:
            sensor_list.append(sensor_name)

    def get_sensor_configuration(self, xacro_mappings=None):
        _ = xacro_mappings  # reserved for future extensions
        return {
            'sensor_names': copy.deepcopy(self.sensor_names),
        }

    # Add a cylinder as a fake end-effector
    def add_simple_ee(self, x_offset=0.0, y_offset=0.0, z_offset=0.0, angle_offset=0.0, mass=1.0, radius=0.02, gazebo=None):
        data = {'type': "simple_ee", 'name': "simple_ee", 'kinematics_convention': "urdf"}
        simple_ee = ModuleNode.ModuleNode(data, "simple_ee", parent=self.parent_module)

        setattr(simple_ee, 'tag', self.parent_module.tag)
        setattr(simple_ee, 'flange_size', self.parent_module.flange_size)
        setattr(simple_ee, 'robot_id', 0)
        setattr(simple_ee, 'addon_elements', [])
        setattr(simple_ee, 'xml_tree_elements', [])
        setattr(simple_ee, 'mesh_names', [])
        setattr(simple_ee, 'connectors', [])
        setattr(simple_ee, 'active_ports', '0000')
        setattr(simple_ee, 'occupied_ports', '0000')
        setattr(simple_ee, 'current_port', 0)

        if gazebo is not None:
            if isinstance(gazebo, dict):
                setattr(simple_ee, 'gazebo', ModuleNode.Module.Attribute({'body_1': gazebo}))
            else:
                setattr(simple_ee, 'gazebo', ModuleNode.Module.Attribute({'body_1': vars(gazebo)}))

        length = abs(z_offset) if abs(z_offset) > 0.0 else 2.0 * radius
        setattr(simple_ee, 'simple_ee_radius', radius)
        setattr(simple_ee, 'simple_ee_mass', mass)
        setattr(simple_ee, 'simple_ee_length', length)

        offsets = {
            'x': x_offset,
            'y': y_offset,
            'z': z_offset,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': angle_offset,
        }

        if self.parent_module.type == 'joint':
            self.link_after_joint(simple_ee, self.parent_module, offsets=offsets, reverse=False)
        elif self.parent_module.type in {'cube', 'mobile_base'}:
            self.link_after_hub(simple_ee, self.parent_module, offsets=offsets, reverse=False)
        else:
            self.link_after_link(simple_ee, self.parent_module, offsets=offsets, reverse=False)

        self.add_to_chain(simple_ee)
        self.parent_module = simple_ee

        selected_connector = simple_ee.name
        selected_meshes = [simple_ee.name]

        self.update_urdf_cache()

        return {'name': simple_ee.name,
                'type': simple_ee.type,
                'flange_size': simple_ee.flange_size,
                'selected_connector': selected_connector,
                'selected_meshes': selected_meshes,
                'urdf_string': self.urdf_string}


    def add_wheel_module(self, wheel_filename, steering_filename, offsets={}, reverse=False, robot_id=(0,0)):
        steering_data = self.add_module(steering_filename, offsets, reverse, robot_id=robot_id[0])
        wheel_data = self.add_module(wheel_filename, offsets, reverse, robot_id=robot_id[1])

        return wheel_data, steering_data

    def _filter_callable_kwargs(self, func, params, context_name):
        func_name = getattr(func, '__name__', str(func))
        if not isinstance(params, dict):
            raise TypeError(f"Parameters for '{func_name}' must be a dictionary")

        supported = set(inspect.signature(func).parameters.keys())
        filtered_params = {k: v for k, v in params.items() if k in supported}
        ignored_params = [k for k in params if k not in supported]
        if ignored_params:
            self.warning_print(
                f"Ignoring unsupported parameters for '{func_name}' in '{context_name}': {ignored_params}"
            )

        return filtered_params

    def add_addon(self, addon_filename, target_module=None, addon_name=None):
        module = target_module if target_module is not None else self.parent_module

        try:
            addons_dict = self.modular_resources_manager.get_available_addons_dict()
            new_addon = addons_dict[addon_filename]
        except KeyError:
            diagnostics, root_cause = self.modular_resources_manager.get_resource_discovery_diagnostics()
            if diagnostics:
                msg = (
                    addon_filename
                    + ' was not found in the available resources.\n'
                    + 'Resource discovery errors:\n'
                    + diagnostics
                )
                raise FileNotFoundError(msg) from root_cause
            raise FileNotFoundError(addon_filename+' was not found in the available resources')

        # Map addon type to handler function.
        # Keep addon `type` generic and use `variant` to select implementation details
        # (e.g. type=camera, variant=realsense).
        addon_header = new_addon.get('header', {})
        params = copy.deepcopy(new_addon.get('parameters', {}))
        addon_type = addon_header.get('type')
        addon_variant = addon_header.get('variant')
        addon_variant = addon_variant.strip().lower() if isinstance(addon_variant, str) else None
        # `variant` is defined in header; inject it into kwargs so handlers can use it.
        params['variant'] = addon_variant

        handler_func = None
        
        if addon_type == 'drillbit':
            handler_func = self.add_drillbit
        elif addon_type == 'handle':
            handler_func = self.add_handle
        elif addon_type == 'dagana_claws':
            handler_func = self.add_dagana_claws
        elif addon_type == 'camera':
            if addon_variant in {None, 'rgbd', 'generic'}:
                handler_func = self.add_camera
            elif addon_variant in {'realsense'}:
                handler_func = self.add_realsense
            else:
                self.warning_print(
                    f"Unknown camera variant '{addon_variant}' in addon '{addon_filename}'. "
                    "Falling back to generic camera handler."
                )
                handler_func = self.add_camera
        elif addon_type == 'imu':
            handler_func = self.add_imu
        else:
            self.logger.info('Addon type not supported')
            return []
        
        # Common parameter filtering and calling pattern
        filtered_params = self._filter_callable_kwargs(
            handler_func,
            params,
            f"addon '{addon_filename}'",)
        
        # Apply type-specific defaults for camera implementations.
        if addon_type == 'camera':
            filtered_params.setdefault('camera_name', 'camera' + module.tag)
            if addon_name is not None:
                filtered_params['camera_name'] = addon_name
            filtered_params.setdefault('parent_name', module.name)
        
        # Call the handler function and add the resulting elements to the module's addon_elements
        added_elements = handler_func(**filtered_params)
        module.addon_elements += added_elements

        return added_elements


    def add_module(self, filename, offsets={}, reverse=False, addons =[], robot_id=0, active_ports=3, is_structural=True, module_name=None):
        """Add a module specified by filename as child of the currently selected module.

        Parameters
        ----------
        filename: str
            String with the name of the YAML file to load, describing the module parameters
        offsets: dict
            Dictionary containing the various offsets ('x','y','z','roll','pitch','yaw') between the parent module output frame and the module input frame. Es. offsets = {'x': 1.0, 'y': 2.0, 'yaw': 1.57}
        reverse: bool
            Bool value expressing if the module is mounted in reverse direction (true) or in standard one (false).
            By default it is false.
            Not used for hub types (cube, mobile base, etc.).
        robot_id: int
            Value of the robot_id set in the firmware of the module.
            This is obtained in Discovery Mode when reading the JSON from the EtherCAT master. This is not used when in Bulding Mode.
        active_ports: int
            The number of active ports of the module (how many ports have established a connection to a module).
            This is the integer conversion of the 4-bit binary string where each bit represent one port (1 if port is active, 0 if port is unactive)
        is_structural: bool
            Bool value expressing if the module is structural (true) or not (false).
            Used only for hub types (cube, mobile base, etc.).

        Returns
        -------
        data: dict
            Dictionary with as entries all the relevant info on the newly added module.
            In particular the updated and newly processed urdf string.

        """
        # global tag, parent_module
        self.print(path_name)
        self.print(filename)

        try:
            module_dict = self.modular_resources_manager.get_available_modules_dict()[filename]
            template_dict = self.modular_resources_manager.get_available_modules_dict()['template.yaml']
        except KeyError:
            diagnostics, root_cause = self.modular_resources_manager.get_resource_discovery_diagnostics()
            if diagnostics:
                msg = (
                    filename
                    + ' was not found in the available resources.\n'
                    + 'Resource discovery errors:\n'
                    + diagnostics
                )
                raise FileNotFoundError(msg) from root_cause
            raise FileNotFoundError(filename+' was not found in the available resources')

        if filename.lower().endswith(('.yaml', '.yml')):
            # Load the module from YAML and create a ModuleNode instance
            new_module = ModuleNode.module_from_yaml_dict(module_dict, self.parent_module, template_dict, reverse)
            self.print("Module loaded from YAML: " + new_module.name)
        elif filename.lower().endswith(('.json')):
            # Load the module from YAML and create a ModuleNode instance
            new_module = ModuleNode.module_from_json_dict(module_dict, self.parent_module, template_dict, reverse)
            self.print("Module loaded from JSON: " + new_module.name)

        # Socket module is a custom type. It behaves differently from other link modules because it has no electronics onboard. Its parent should always be the base_link. On the hardware it will actually be connected to a non-structural hub, which therefore will not be part of the URDF, so we consider the base_link to be the parent in any case. This means the ports of the hub will not actually be occupied, so potentially there is no limit to how many sockets could be connected (>3).
        if new_module.type is ModuleType.SOCKET:
            self.parent_module = new_module.parent = self.base_link

        # If the parent is a hub module, it means we are starting a new branch.
        # Then assign the correct tag (A, B, C, ...) to the new module (and therefore start a new branch)
        # by looking at the current tag_num (1, 2, 3, ...) and so at how many branches are already present in the robot.
        # If the parent is any other kind of module, assign as tag the same of his parent.
        if (
            self.parent_module.type in ModuleClass.hub_modules() | {ModuleType.BASE_LINK} 
            and new_module.type not in ModuleClass.hub_modules()
        ):
            self.tag_num += 1
            tag_letter = self.branch_switcher.get(self.tag_num)
            setattr(new_module, 'tag', tag_letter)
        else:
            setattr(new_module, 'tag', self.parent_module.tag)

        self.print('new_module.tag:', new_module.tag)

        # Set attributes of the newly added module object
        setattr(new_module, 'i', self.parent_module.i)
        setattr(new_module, 'p', self.parent_module.p)
        # flange_size is already set from the YAML file
        # setattr(new_module, 'flange_size', self.parent_module.flange_size)

        setattr(new_module, 'offsets', offsets)
        setattr(new_module, 'reverse', reverse)
        setattr(new_module, 'robot_id', robot_id)

        # Attribute specifying if the module is structural or not. Useful for those types of modules that are not structural (e.g. hubs), in particular if they are present in the network and discovered by EtherCAT, but they are not part of the robot structure.
        if hasattr(new_module.header, 'is_structural'):
            setattr(new_module, 'is_structural', new_module.header.is_structural)
        else:
            # if the attribute is not present, set it, otherwise leave it as it is (it has been set from the YAML/JSON file)
            setattr(new_module, 'is_structural', is_structural)

        self.print("parent module:", self.parent_module.name, ", type :", self.parent_module.type)

        # Update the EtherCAT port connected to the electro-mechanical interface where the new module/slave will be added 
        #################################################
        # non-hub modules:
        #      1            2           3           4
        #      o            o           o           o
        #      |            |           |           |
        # input port   output port   nothing    nothing
        #################################################
        # cube modules:
        #    1           2           3           4
        #    o           o           o           o
        #    |           |           |           |
        # com-exp   upper port  front port    nothing
        #################################################
        setattr(new_module, 'current_port', 1)
        self.print('new_module.current_port :', new_module.current_port)

        # save the active ports as a binary string
        setattr(new_module, 'active_ports', "{0:04b}".format(active_ports))
        self.print('active_ports: ', new_module.active_ports)

        # save the occupied ports as a binary string
        setattr(new_module, 'occupied_ports', "0001")
        self.print('occupied_ports: ', new_module.occupied_ports)

        # add list of addons as attribute
        setattr(new_module, 'addon_elements', [])

        # add list of urdf elements as attribute
        setattr(new_module, 'xml_tree_elements', [])

        # add list of xml elements with an associated visual mesh as attribute
        setattr(new_module, 'mesh_names', [])

        # Ensure module-level default addons is always present and list-typed.
        setattr(new_module, 'default_addons', list(getattr(new_module, 'default_addons', [])))

        # add list of connectors names as attribute. The 0 connector is added by default. The others will be added by the add_connectors() method
        setattr(new_module, 'connectors', ['connector_0'])

        # For certain types of modules, the model is considered as floating base, i.e. the base_link is not fixed to the world frame, but is connected through a floating joint.
        # WARNING: this changes the behavior of the whole URDF not only for this module
        if new_module.type in {'mobile_base'}:
            self.set_floating_base(True)

        # Depending on the type of the parent module and the new module, call the right method to add the new module.
        # Add the module to the correct chain via the 'add_to_chain' method.
        if self.parent_module.type == 'joint':
            if new_module.type in { 'joint', 'wheel' }:
                # joint + joint
                self.print("joint + joint")
                self.joint_after_joint(new_module, self.parent_module, offsets=offsets, reverse=reverse)
            elif new_module.type in { 'cube', 'mobile_base' }:
                # joint + hub
                self.print("joint + hub")
                self.hub_after_joint(new_module, self.parent_module, offsets=offsets, reverse=reverse, module_name=module_name)
            else:
                # joint + link
                self.print("joint + link")
                self.link_after_joint(new_module, self.parent_module, offsets=offsets, reverse=reverse)
        elif self.parent_module.type == 'wheel':
            # TODO: prevent adding modules after wheels in a better way (raise exception?)
            return {'result': 'ERROR: module cannot be added after wheel module. Select another chain or remove the wheel module.'}
        elif self.parent_module.type in {'cube', "mobile_base"}:
            if new_module.type in { 'joint', 'wheel' }:
                # hub + joint
                self.print("hub + joint")
                self.joint_after_hub(new_module, self.parent_module, offsets=offsets, reverse=reverse)
            elif new_module.type in { 'cube', 'mobile_base' }:
                # hub + hub
                self.print("hub + hub")
                self.hub_after_hub(new_module, self.parent_module, offsets=offsets, reverse=reverse, module_name=module_name)
            else:
                # hub + link
                self.print("hub + link")
                self.link_after_hub(new_module, self.parent_module, offsets=offsets, reverse=reverse)
        else:
            if new_module.type in { 'joint', 'wheel' }:
                # link + joint
                self.print("link + joint")
                self.joint_after_link(new_module, self.parent_module, offsets=offsets, reverse=reverse)
            elif new_module.type in { 'cube', 'mobile_base' }:
                # link + hub
                self.print("link + hub")
                self.hub_after_link(new_module, self.parent_module, offsets=offsets, reverse=reverse, module_name=module_name)
            else:
                # link + link
                self.print("link + link")
                self.link_after_link(new_module, self.parent_module, offsets=offsets, reverse=reverse)

        # # TODO: check if this is correct
        # # Add the module to the list of chains if there is at least a joint before it
        # if new_module.i > 0:
        #     self.add_to_chain(new_module)
        self.add_to_chain(new_module)

        # Update the parent_module attribute of the URDF_writer class
        self.parent_module = new_module

        # Select the current connector of the new module
        selected_connector = self.select_connector(new_module, port_idx=new_module.current_port)
        # Select the meshes to highlight in the GUI
        selected_meshes = self.select_meshes(selected_connector, new_module)

        default_addons = list(getattr(new_module, 'default_addons', []))
        explicit_addons = [
            addon if isinstance(addon, dict) else {'addon_filename': addon}
            for addon in addons
        ]

        for addon_spec in default_addons + explicit_addons:
            try:
                self.add_addon(
                    addon_filename=addon_spec['addon_filename'],
                    addon_name=addon_spec.get('addon_name'),
                )
            except FileNotFoundError:
                self.logger.error(f"Addon {addon_spec['addon_filename']} not found, skipping it")

        # add meshes to the map
        self.mesh_to_module_map.update({k: new_module.name for k in new_module.mesh_names})

        self.update_urdf_cache()

        # update the urdf file, adding the new module
        # string = write_urdf(path_name + '/urdf/ModularBot_test.urdf', urdf_tree)

        if self.verbose:
            # Render tree
            for pre, _, node in anytree.render.RenderTree(self.base_link):
                self.print("%s%s: %d" % (pre, node.name, node.robot_id))

        # Create a dictionary containing the urdf string just processed and other parameters needed by the web app
        data = {'name': new_module.name,
                'type': new_module.type,
                'flange_size': new_module.flange_size,
                'selected_connector': selected_connector,
                'selected_meshes': selected_meshes,
                'urdf_string': self.urdf_string}          

        self.info_print("Module added to URDF: " + new_module.name + " (" + new_module.type + ")")

        return data

    def add_gazebo_element(self, new_module_obj, gazebo_obj, new_module_name):
        return self.xml_builder.add_gazebo_element(new_module_obj, gazebo_obj, new_module_name)


    def add_gazebo_element_children(self, gazebo_child_obj, gazebo_element):
        return self.xml_builder.add_gazebo_element_children(gazebo_child_obj, gazebo_element)

    
    def update_module(self, selected_module=0, offsets={}, reverse=False, addons=[]):
        if selected_module == 0:
            selected_module = (self.parent_module)
        # If the selected module is a connector module, select his parent (the hub) instead
        if '_con' in selected_module.name:
            selected_module = selected_module.parent

        self.info_print('Updating module: ' + str(selected_module.name))
        
        # Update generator expression
        self.update_generators()

        # Update offsets. MEMO: to be fixed! must apply offsets not overwrite
        for node in self.urdf_nodes_generator:
            try:
                if node.attrib['name'] == selected_module.fixed_joint_name:
                    for key in offsets.keys():
                        node.set(key, str(offsets[key]))
            except (KeyError, AttributeError):
                pass

        # update generator expression
        self.update_generators()

        # remove addons
        if(getattr(selected_module, 'addon_elements')):
            for node in self.urdf_nodes_generator:
                try:
                    if node.attrib['name'] in selected_module.addon_elements:
                        # remove mesh from list of meshes and from the map
                        if node.attrib['name'] in selected_module.mesh_names:
                            selected_module.mesh_names.remove(node.attrib['name'])
                            self.mesh_to_module_map.pop(node.attrib['name'])
                        # remove node from tree
                        self.root.remove(node)
                except KeyError:
                    pass

        for addon in addons:
            try:
                self.add_addon(addon_filename=addon)
            except FileNotFoundError:
                self.logger.error(f'Addon {addon} not found, skipping it')

        self.mesh_to_module_map.update({k: selected_module.name for k in selected_module.mesh_names})

        # Select the current connector of the new module
        selected_connector = self.select_connector(selected_module, port_idx=selected_module.current_port)
        # Select the meshes to highlight in the GUI
        selected_meshes = self.select_meshes(selected_connector, selected_module)

        self.update_urdf_cache()

        # Create a dictionary containing the urdf string just processed and other parameters needed by the web app
        data = {'name': selected_module.name,
                'type': selected_module.type,
                'flange_size': selected_module.flange_size,
                'selected_connector': selected_connector,
                'selected_meshes': selected_meshes,
                'urdf_string': self.urdf_string}          
        
        return data


    def remove_module(self, selected_module=0):
        """Remove the selected module (and all its childs and descendants) and return info on its parent

        Parameters
        ----------
        selected_module: ModuleNode.ModuleNode
            NodeModule object of the module to remove. Default value is 0, in which case the current parent_module is selected as the module to be removed.

        Returns
        -------
        data: dict
            Dictionary with as entries all the relevant info on the parent module of the removed module.
            In particular the updated and newly processed urdf string, so without the removed modules.

        """
        # If no selected_module argument was passed to the method,
        # select the current parent module to be the one to remove
        if selected_module == 0:
            selected_module = (self.parent_module)

        self.info_print('Removing module: ' + str(selected_module.name) + ' (and all its descendants)')

        # Remove the module childs and its descendants recursively
        for child in selected_module.children:
            self.print('eliminate child: ' + child.name + ' of type: ' + child.type + ' of parent: ' + selected_module.name)
            self.remove_module(child)

        # update generator expression
        self.update_generators()

        xml_elements_to_remove = []
        # remove addons
        if(getattr(selected_module, 'addon_elements')):
            xml_elements_to_remove += selected_module.addon_elements
        # remove module xml elements
        if(getattr(selected_module, 'xml_tree_elements')):
            xml_elements_to_remove += selected_module.xml_tree_elements
            
        # remove all required xml elements from the tree
        for node in self.urdf_nodes_generator:
            try:
                if node.attrib['name'] in xml_elements_to_remove:
                    # remove mesh from list of meshes and from the map
                    if node.attrib['name'] in selected_module.mesh_names:
                        selected_module.mesh_names.remove(node.attrib['name'])
                        self.mesh_to_module_map.pop(node.attrib['name'])
                    # remove node from tree
                    self.root.remove(node)
            except KeyError:
                pass

        # save parent of the module to remove. This will be the last element of the chain after removal,
        # and its data will be returned by the function
        father = selected_module.parent

        # remove the module from the list of chains
        self.remove_from_chain(selected_module)

        # switch depending on module type
        if selected_module.type in ModuleClass.joint_modules():
            self.control_plugin.remove_joint(selected_module.name)

        elif selected_module.type is ModuleType.GRIPPER:
            # TO BE FIXED: ok for ros_control. How will it be for xbot2?
            self.control_plugin.remove_joint(selected_module.name+'_finger_joint1')
            self.control_plugin.remove_joint(selected_module.name+'_finger_joint2')

        elif selected_module.type in ModuleClass.hub_modules():
            # if the module is a hub, remove it from the list of hubs
            self.listofhubs.remove(selected_module)

        # if the parent module is a hub, decrease the tag number. A chain has been removed, tag should be reset accordingly
        if (
            father.type in ModuleClass.hub_modules() | {ModuleType.BASE_LINK} 
            and selected_module.type not in ModuleClass.hub_modules()
        ):
            self.tag_num -= 1

        self.update_urdf_cache()

        # Update the parent_module attribute of the URDF_writer class
        self.parent_module = father

        # Select the current connector of the new module
        selected_connector = self.select_connector(father, port_idx=self.connector_to_port_idx(father.connector_idx, father))
        # Select the meshes to highlight in the GUI
        selected_meshes = self.select_meshes(selected_connector, father)
        

       # Create a dictionary containing the urdf string just processed and other parameters needed by the web app
        data = {'name': father.name,
                'type': father.type,
                'flange_size': father.flange_size,
                'selected_connector': selected_connector,
                'selected_meshes': selected_meshes,
                'urdf_string': self.urdf_string}  

        # before deleting selected_module set his parent property to None. Otherwise this will mess up the obj tree
        selected_module.parent = None

        # delete object selected_module
        del selected_module

        if self.verbose:
            # Render tree
            for pre, _, node in anytree.render.RenderTree(self.base_link):
                self.print("%s%s" % (pre, node.name))

        return data

    def access_module_by_id(self, queried_module_id):
        """Find the selected module object by searching its ID in the tree and returns it. Moreover, sets it as the current parent_module.

        Parameters
        ----------
        queried_module_id: int
            The id of the module to access. It will be used to search the tree and find the relative ModuleNode object

        Returns
        -------
        last_module: ModuleNode.ModuleNode
            The object of the module with the id as passed by the arg.

        """
        # global parent_module
        self.print('queried_module_id: ', queried_module_id)

        # Serch the tree by id for the selected module
        queried_module = anytree.search.findall_by_attr(self.base_link, queried_module_id, name='robot_id')[0]

        self.print('queried_module.type: ', queried_module.type)

        # Update parent_module attribute
        self.parent_module = queried_module

        return queried_module

    def access_module_by_name(self, queried_module_name):
        """Find the selected module object in the tree and returns it. Moreover, sets it as the current parent_module.

        Parameters
        ----------
        queried_module_name: str
            String with the name of the module to access. It will be used to search the tree and find the relative ModuleNode object

        Returns
        -------
        last_module: ModuleNode.ModuleNode
            The object of the module with the name as passed by the arg.

        """
        # global parent_module
        self.print('queried_module_name: ', queried_module_name)

        # Serch the tree by name for the selected module
        queried_module = anytree.search.findall_by_attr(self.base_link, queried_module_name)[0]

        self.print('queried_module.type: ', queried_module.type)

        # Update parent_module attribute
        self.parent_module = queried_module

        return queried_module
    
    def select_module_from_id(self, id, current_port=None):
        """Allows to select a module from the tree. An inner call to access_module_by_id sets the selected module as the
        current parent module. Returns info on the selected module, so that the GUI can display it.

        Parameters
        ----------
        id: int
            The id of the module to select. It will be used to call the access_module_by_id method.
            The corresponding object module data is then put in a dictionary and returned.

        current_port: int
            Represent the current port. If the module is a hub/box it is used to select tjhe connector to be used.

        Returns
        -------
        data: dict
            The dictionary containing all necessary data about the selected module.

        """
        # global parent_module
        self.print('id: ', id)

        # Call the access_module_by_id method to find the selected module
        selected_module = self.access_module_by_id(id)

        # Select the current connector of the selected module
        selected_connector = self.select_connector(selected_module, port_idx=current_port)
        # Select the meshes to highlight in the GUI
        selected_meshes = self.select_meshes(selected_connector, selected_module)

        # Create the dictionary with the relevant info on the selected module, so that the GUI can dispaly it.
        data = {'name': selected_module.name,
                'type': selected_module.type,
                'flange_size': selected_module.flange_size,
                'selected_connector': selected_connector,
                'selected_meshes': selected_meshes,
                'urdf_string': self.urdf_string}          

        return data

    def select_module_from_name(self, name, current_port=None):
        """Allows to select a module from the tree. An inner call to access_module_by_name sets the selected module as the
        current parent module. Returns info on the selected module, so that the GUI can display it.

        Parameters
        ----------
        name: str
            String with the name of the module to select or the name of the mesh clicked on the GUI. It will be used to call the access_module_by_name method.
            The corresponding object module data is then put in a dictionary and returned.

        current_port: int
            Represent the current port. If the module is a hub/box it is used to select tjhe connector to be used.

        Returns
        -------
        data: dict
            The dictionary containing all necessary data about the selected module.

        """

        self.print(name)

        # If the name of the mesh clicked on the GUI is not the name of the module, but the name of the mesh, we need to
        # find the module object from the mesh name. The mesh_to_module_map dictionary is used for this. Default value
        # is the name itself, so if the name is not in the dictionary, it is the name of the module itself.
        selected_module_name = self.mesh_to_module_map.get(name, name)

        # Call access_module_by_name to get the object with the requested name and sets it as parent.
        # The method doing the real work is actually access_module_by_name
        selected_module = self.access_module_by_name(selected_module_name)

        # Select the current connector of the selected module
        selected_connector = self.select_connector(selected_module, connector_name=name, port_idx=current_port)
        # Select the meshes to highlight in the GUI
        selected_meshes = self.select_meshes(selected_connector, selected_module)

        # Create the dictionary with the relevant info on the selected module, so that the GUI can dispaly it.
        data = {'name': selected_module.name,
                'type': selected_module.type,
                'flange_size': selected_module.flange_size,
                'selected_connector': selected_connector,
                'selected_meshes': selected_meshes,
                'urdf_string': self.urdf_string}          

        return data
    

    def port_to_connector_idx(self, port_idx, module):
        """Convert the port index to the connector index, for the given module.

        Parameters
        ----------
        port_idx: int
            The index of the port to convert.

        module: ModuleNode.ModuleNode
            The module to compute the connector index for.

        Returns
        -------
        connector_idx: int
            The index of the connector corresponding to the port index.

        """
        # MYNOTE: this part is used only in Discovery mode for now.
        # TODO: this works only for hub trees of maximum depth 2. For deeper trees, this should be revised.
        idx_offset = 0
        # If the hub is not structural, we need to take into account the other children the parent hub might have.
        # If the hub is not structural, by default its parent must be a hub as well. Non-structural hubs can only be
        # connected to structural hubs (or other non-structural hubs) and be a "children hub". Their only purpose is to 
        # "increase" the number of ports of its parent hub (and therefore its available connectors). 
        # The connectors are shared between the parent and children hubs, so their index must be computed accordingly.
        if module.type in ModuleClass.hub_modules() and module.parent.type in ModuleClass.hub_modules() and not module.is_structural:
            # # The current port used by the parent hub to connect to the hub we are computing the transforms of.
            # parent_current_port = 1 << self.eth_to_physical_port_idx(module.parent.current_port)
            # # The ports of the parent hub already occupied before adding the current hub.
            # parent_already_occupied_ports = int(module.parent.occupied_ports, 2) & ~parent_current_port
            # # The ports 1, 2 and 3
            # non_zero_ports = int("1110", 2)
            # # The ports of the parent hub already occupied before adding the current hub, excluding the port 0.
            # parent_ports_occupied_before_hub = (parent_already_occupied_ports & non_zero_ports)
            # # The number of children the parent hub has before adding the current hub.
            # n_children_before_hub = 0
            # for i in range(4):
            #     if parent_ports_occupied_before_hub & (1 << i):
            #         n_children_before_hub += 1

            # The number of hubs children the parent hub has after adding the current hub.
            parent_elder_hubs_children = module.parent.n_children_hubs - 1
            # The index offset is computed by looking at the number of children hubs and non-hubs the parent hub has before adding the current hub.
            if parent_elder_hubs_children > 0:
                # Count the number of non-structural hubs children the parent hub has (sieblings of the current hub)
                # Remove the current hub from the count, should not be taken into account when counting the index.
                nonstructural_hub_children = self.count_children_non_structural_hubs(module.parent, count=-1)
                # We take into account the other hubs connected to get the right index. We have 4 connectors per hub, but since port 0 is occupied by the hub-hub connection, each child hub increase the index by 3
                idx_offset += (4-1)*nonstructural_hub_children
                # We take into account that one port on the current hub (parent) is used to establish a connection with a second hub, and that should not be taken into account when counting the index
                idx_offset -= 1*nonstructural_hub_children
                # # Each hub sibling increases the index offset by 1!
                # idx_offset += (1)*parent_elder_hubs_children 
                
            # The number of non-hubs children the parent hub has before adding the current hub.
            #parent_elder_nonhubs_children = n_children_before_hub - parent_elder_hubs_children
            # # the number of non-hubs children increases the index offset by 1 (since each non-hub has 2 connectors, but one is used to connect to the parent hub).
            # idx_offset +=(parent_elder_nonhubs_children)*1
            #MYNOTE: this is kind of magic
            idx_offset += module.parent.current_port - 1
        
        # indexes for connector and selected port are the same, unless the hub is connected to another non-structural hub
        connector_idx = port_idx + idx_offset
        
        # We need to add an offset to the connector index introduced by the other non-structural hubs already connected to the current hub
        if module.type in ModuleClass.hub_modules():
            # Count the number of non-structural hubs children the current hub has
            nonstructural_hub_children = self.count_children_non_structural_hubs(module)
            # We take into account the other hubs connected to get the right index. We have 4 connectors per hub, but since port 0 is occupied by the hub-hub connection, each child hub increase the index by 3
            connector_idx += (4-1)*nonstructural_hub_children
            # We take into account that one port on the current hub (parent) is used to establish a connection with a second hub, and that should not be taken into account when counting the index
            connector_idx -= 1*nonstructural_hub_children

        return connector_idx
    

    def count_children_non_structural_hubs(self, module, count=0):
        """Count all non-strucural hubs children and grandchildren of the given module recursively.

        Parameters
        ----------
        module: ModuleNode.ModuleNode
            The module to compute the offset for.

        count: int
            The current offset count. Default value is 0.

        Returns
        -------
        count: int
            The offset of the connector index for the given module.

        """
        for child in module.children:
            count = self.count_children_non_structural_hubs(child, count)
            if child.type in ModuleClass.hub_modules() and not child.is_structural:
                count += 1
        return count
    

    def connector_to_port_idx(self, connector_idx, module):
        """Convert the connector index to the port index.

        Parameters
        ----------
        connector_idx: int
            The index of the connector to convert.

        Returns
        -------
        port_idx: int
            The index of the port corresponding to the connector index.

        """
        # MYNOTE: this part is used only in Building mode for now. When the differences between the two modes will be removed, the implemantation of this function will be the reverse of the port_to_connector_idx function.

        # connector index is the same as the port index for the first 4 ports
        port_idx = connector_idx

        return port_idx
    

    def eth_to_physical_port_idx(self, eth_port_idx):
        """Convert the EtherCAT port index to the physical port index. This is necessary because the EtherCAT master
        scans the ports in a different order than the physical one.

        Parameters
        ----------

        eth_port_idx: int
            The index of the EtherCAT port to convert.

        Returns
        -------
        physical_port_idx: int
            The index of the physical port corresponding to the EtherCAT port index.

        """
        eth_to_physical_port_map = {
            0: 0,
            1: 3,
            2: 1,
            3: 2
        }
        return eth_to_physical_port_map[eth_port_idx]
    

    def set_current_port(self, module, port_idx=None):
        """Set the current port of the module, i.e. the one where the new module will be added to.
        The current port is the first free one, i.e. the first one seen from the EtherCAT master scan.
        If the port_idx argument is passed, the current port is set to the one specified by it.

        Parameters
        ----------
        module: ModuleNode.ModuleNode
            The object of the module to set the current port to.

        port_idx: int
            The index of the port to select as the current one.

        Returns
        -------
        module.current_port: int
            The port selected as the current one.

        """
        # If the port index is not None, it means that the user has selected it from the GUI. Value is overwritten
        if port_idx is not None:
            module.current_port = port_idx
        else:
            #MYNOTE: this part is used only in Discovery mode for now. occupied ports gets updated only in Discovery mode for now
            # binary XOR: the free ports are the ones that are active but not occupied
            free_ports = int(module.active_ports, 2) ^ int(module.occupied_ports, 2)
            self.print(module.name + " active_ports: " + module.active_ports + " - " + "occupied_ports: " + module.occupied_ports + " =")
            self.print("{0:04b}".format(free_ports))

            # remap the ports from the physical order to the EtherCAT order: 3, 2, 1, 0 -> 2, 1, 3, 0. 
            # See EtherCAT slave documentation for more info 
            free_ports_remapped = ((free_ports & int("0110", 2)) << 1) + ((free_ports & int("1000", 2)) >> 2)
            self.print("{0:04b}".format(free_ports_remapped))

            # By default the selected port is the first free one (the firt one seen from the EtherCAT master scan)
            selected_eth_port = self.ffs(free_ports_remapped)
            self.print('selected EtherCAT port :', selected_eth_port)

            # MYNOTE: this part is not needed. We are not interested in the physical port index, but in the EtherCAT port index, so that it matches the order of the ports in the EtherCAT master scan.
            # # remap the ports from the EtherCAT order to the physical order: 2, 1, 3, 0 -> 3, 2, 1, 0.
            # selected_physical_port = self.eth_to_physical_port_idx(selected_eth_port)
            # self.print('selected physical port :', selected_physical_port)

            # Set the current_port attribute of the module
            module.current_port = selected_eth_port

        self.print('module.current_port :', module.current_port)

        return module.current_port 
    

    def select_connector(self, module, connector_name=None, port_idx=None):
        """Select the connector of the module to which the new module will be added to.
        If no `connector_name` is passed, the connector is the first free one, i.e. the first one seen from the EtherCAT master scan. (Discovery mode)
        Otherwise the connector name is selected as the one passed as argument. (Building mode)

        Parameters
        ----------
        module: ModuleNode.ModuleNode
            The object of the module to set the current port to.

        connector_name: str
            String with the name of the connector. Could come from the name of the mesh clicked on the GUI and associated to the connector.

        port_idx: int
            The index of the port to select as the current one.

        Returns
        -------
        selected_connector: str
            The name of the connector currently selected.
        """

        # If the selected mesh is the one of a connector, we need to set the right port
        if connector_name in module.connectors:
            # The connector index is retrieved from the list of connectors of the module
            connector_idx = module.connectors.index(connector_name)
            # Convert the connector index to the port index
            current_port = self.connector_to_port_idx(connector_idx, module)
            # TODO: the current port is not at the moment. This is currently done only in Discovery mode. In Building mode the current port is not used.
            # Set the name of the selected connector
            selected_connector = connector_name
        else:
            # Set the current port of the module
            self.set_current_port(module, port_idx=port_idx)
            # Convert the port index to the connector index
            connector_idx = self.port_to_connector_idx(module.current_port, module) 
            # Set the name of the selected connector. If the index is out of range, it means that the module has only one connector, so we use as name the module name itself.
            # TODO: add connectors also for modules with only one connector, so to avoid this and have a uniform behavior
            if 0 <= connector_idx < len(module.connectors):
                selected_connector = module.connectors[connector_idx]
            else:
                selected_connector = module.name

        setattr(module, 'connector_idx', connector_idx)

        self.print('selected_connector :', selected_connector)

        return selected_connector
    

    def select_meshes(self, selected_connector, module):
        """Select the mesh of the module to be highlighted on the GUI.

        Parameters
        ----------
        selected_connector: str
            String with the name of the connector.

        module: ModuleNode.ModuleNode
            The object of the module to select the mesh from.

        Returns
        -------
        selected_mesh: list
            The names of the meshes currently selected.
        """

        if selected_connector in module.connectors:
            # If the selected mesh is the one of a connector, we select as mesh only the one associated to the connector
            selected_meshes = [selected_connector]
        else:
            # Otherwise we select all the meshes of the module
            selected_meshes = module.mesh_names

        self.print('selected_mesh :', selected_meshes)

        return selected_meshes


    @staticmethod
    def ffs(x):
        """Returns the index, counting from 0, of the
        least significant set bit in `x`.
        """
        return (x & -x).bit_length() - 1

    # TODO: handle reverse also for links
    def add_link(self, new_Link, parent_name, transform, reverse):
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(transform)

        if new_Link.type in ModuleClass.link_modules() - {ModuleType.SIZE_ADAPTER}:
            setattr(new_Link, 'name', 'L_' + str(new_Link.i) + '_link_' + str(new_Link.p) + new_Link.tag)
            self.add_link_element(new_Link.name, new_Link, 'body_1') #  , type='link')
        
        elif new_Link.type is ModuleType.DAGANA:
            setattr(new_Link, 'name', 'dagana' + new_Link.tag)
            dagana_xacro_path = self.resource_finder.get_external_resource_filename(
                'dagana_path',
                ['urdf/dagana_macro.urdf.xacro', 'dagana_macro.urdf.xacro'],
            )
            ET.SubElement(self.root, "xacro:include", filename=dagana_xacro_path)
            ET.SubElement(self.root,
                          "xacro:add_dagana",
                          type="link",
                          name=new_Link.name,
                          father=parent_name,
                          x=x,
                          y=y,
                          z=z,
                          roll=roll,
                          pitch=pitch,
                          yaw=yaw)
            # add the xacro:add_dagana element to the list of urdf elements
            new_Link.xml_tree_elements.append(new_Link.name)
            new_Link.mesh_names += [new_Link.name + '_top_link', new_Link.name + '_bottom_link', new_Link.name + '_top_link_jaw']

            setattr(new_Link, 'dagana_joint_name', new_Link.name + '_claw_joint')
            setattr(new_Link, 'base_link_name', new_Link.name + '_top_link')
            setattr(new_Link, 'dagana_tcp_name', new_Link.name + '_tcp')
            setattr(new_Link, 'tcp_name', 'ee' + new_Link.tag)
            # this list will contain the names of the fingers or any moving extremity of the end effector
            setattr(new_Link, 'finger_names', [new_Link.name + '_bottom_link_jaw'])
            new_Link.mesh_names += new_Link.finger_names

            ET.SubElement(self.root,
                          "xacro:add_tcp",
                          type="pen",
                          name=new_Link.tcp_name,
                          father=new_Link.dagana_tcp_name,
                          x="0.0",
                          y="0.05",
                          z="0.0",
                          roll="0.0",
                          pitch="0.0",
                          yaw="0.0")
            # add the xacro:add_tcp element to the list of urdf elements
            new_Link.xml_tree_elements.append(new_Link.tcp_name)

            # the dagana gets added to the chain. it's needed in the joint map and in the config!
            # self.add_to_chain(new_Link)
            self.control_plugin.add_joint(new_Link.dagana_joint_name, control_params=new_Link.xbot_gz if hasattr(new_Link, 'xbot_gz') else None)                           

            return

        elif new_Link.type is ModuleType.DRILL:
            setattr(new_Link, 'name', 'drill' + new_Link.tag)
            self.add_link_element(new_Link.name, new_Link, 'body_1') #  , type='link')

            setattr(new_Link, 'base_link_name', new_Link.name)
            # this list will contain the names of the fingers or any moving extremity of the end effector
            setattr(new_Link, 'finger_names', [])
            for addon_spec in new_Link.default_addons:
                if (
                    isinstance(addon_spec, dict)
                    and addon_spec.get('addon_filename') == 'concert/drill_camera.json'
                ):
                    addon_spec['addon_name'] = 'drill_camera' + new_Link.tag
                    break

            x_ee, y_ee, z_ee, roll_ee, pitch_ee, yaw_ee = ModuleNode.get_xyzrpy(np.array(new_Link.kinematics.link.pose))
            setattr(new_Link, 'tcp_name', 'drillnose' + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_tcp",
                          type="pen",
                          name=new_Link.tcp_name,
                          father=new_Link.name,
                          x=x_ee,
                          y=y_ee,
                          z=z_ee,
                          roll=roll_ee,
                          pitch=pitch_ee,
                          yaw=yaw_ee)
            # add the xacro:add_tcp element to the list of urdf elements
            new_Link.xml_tree_elements.append(new_Link.tcp_name)

        elif new_Link.type is ModuleType.SPRAYING_TOOL:
            setattr(new_Link, 'name', 'spraying_tool' + new_Link.tag)
            self.add_link_element(new_Link.name, new_Link, 'body_1') #  , type='link')

            setattr(new_Link, 'base_link_name', new_Link.name)
            # this list will contain the names of the fingers or any moving extremity of the end effector
            setattr(new_Link, 'finger_names', [])

            x_ee, y_ee, z_ee, roll_ee, pitch_ee, yaw_ee = ModuleNode.get_xyzrpy(np.array(new_Link.kinematics.link.pose))
            setattr(new_Link, 'tcp_name', 'ee' + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_tcp",
                          type="pen",
                          name=new_Link.tcp_name,
                          father=new_Link.name,
                          x=x_ee,
                          y=y_ee,
                          z=z_ee,
                          roll=roll_ee,
                          pitch=pitch_ee,
                          yaw=yaw_ee)
            # add the xacro:add_tcp element to the list of urdf elements
            new_Link.xml_tree_elements.append(new_Link.tcp_name)

        elif new_Link.type is ModuleType.END_EFFECTOR:
            setattr(new_Link, 'name', 'end_effector' + new_Link.tag)
            self.add_link_element(new_Link.name, new_Link, 'body_1') #   , type='link')

            setattr(new_Link, 'base_link_name', new_Link.name)
            # this list will contain the names of the fingers or any moving extremity of the end effector
            setattr(new_Link, 'finger_names', [])

            x_ee, y_ee, z_ee, roll_ee, pitch_ee, yaw_ee = ModuleNode.get_xyzrpy(np.array(new_Link.kinematics.link.pose))
            setattr(new_Link, 'tcp_name', 'ee' + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_tcp",
                          type="pen",
                          name=new_Link.tcp_name,
                          father=new_Link.name,
                          x=x_ee,
                          y=y_ee,
                          z=z_ee,
                          roll=roll_ee,
                          pitch=pitch_ee,
                          yaw=yaw_ee)
            # add the xacro:add_tcp element to the list of urdf elements
            new_Link.xml_tree_elements.append(new_Link.tcp_name)

        elif new_Link.type is ModuleType.SIMPLE_EE:
            setattr(new_Link, 'name', 'simple_ee' + new_Link.tag)
            setattr(new_Link, 'base_link_name', new_Link.name)
            setattr(new_Link, 'finger_names', [])
            setattr(new_Link, 'tcp_name', 'ee' + new_Link.tag)

            radius = getattr(new_Link, 'simple_ee_radius', 0.02)
            mass = getattr(new_Link, 'simple_ee_mass', 1.0)
            length = getattr(new_Link, 'simple_ee_length', 2.0 * radius)

            ET.SubElement(self.root,
                          "xacro:add_cylinder",
                          type="simple_ee",
                          name=new_Link.name,
                          size_z=str(length),
                          mass=str(mass),
                          radius=str(radius))
            new_Link.xml_tree_elements.append(new_Link.name)
            new_Link.mesh_names.append(new_Link.name)

            ET.SubElement(self.root,
                          "xacro:add_tcp",
                          type="pen",
                          name=new_Link.tcp_name,
                          father=new_Link.name,
                          x="0.0",
                          y="0.0",
                          z=str(length),
                          roll="0.0",
                          pitch="0.0",
                          yaw="0.0")
            new_Link.xml_tree_elements.append(new_Link.tcp_name)
            
        elif new_Link.type is ModuleType.TOOL_EXCHANGER:
            setattr(new_Link, 'name', 'tool_exchanger' + new_Link.tag)
            self.add_link_element(new_Link.name, new_Link, 'body_1') #  , type='tool_exchanger')

            setattr(new_Link, 'base_link_name', new_Link.name)
            # this list will contain the names of the fingers or any moving extremity of the end effector
            setattr(new_Link, 'finger_names', [])

            # the end-effector gets added to the chain although it's not a joint. it's needed in the joint map and in the config!
            # self.add_to_chain(new_Link)
            # HACK: add pen after tool_exchanger
            setattr(new_Link, 'tcp_name', 'pen' + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_tcp",
                          type="pen",
                          name=new_Link.tcp_name,
                          father=new_Link.name,
                          x="0.0",
                          y="0.0",
                          z="0.222",
                          roll="0.0",
                          pitch="0.0",
                          yaw="0.0")
            # add the xacro:add_tcp element to the list of urdf elements
            new_Link.xml_tree_elements.append(new_Link.tcp_name)
            
        elif new_Link.type is ModuleType.GRIPPER:
            setattr(new_Link, 'name', 'gripper' + new_Link.tag)
            self.add_link_element(new_Link.name, new_Link, 'body_1') #   , type='gripper_body')
            
            # the end-effector gets added to the chain although it's not a joint. it's needed in the joint map and in the config!
            # self.add_to_chain(new_Link)
            # add fingers and tcp after gripper
            setattr(new_Link, 'tcp_name', 'TCP_' + new_Link.name)
            setattr(new_Link, 'joint_name_finger1', new_Link.name + '_finger_joint1')
            setattr(new_Link, 'joint_name_finger2', new_Link.name + '_finger_joint2')
            setattr(new_Link, 'name_finger1', new_Link.name + '_finger1')
            setattr(new_Link, 'name_finger2', new_Link.name + '_finger2')

            setattr(new_Link, 'base_link_name', new_Link.name)
            # this list will contain the names of the fingers or any moving extremity of the end effector
            setattr(new_Link, 'finger_names', [new_Link.name_finger1, new_Link.name_finger2])
            
            finger1 = self.add_link_element(new_Link.name_finger1, new_Link, 'body_2')
            finger2 = self.add_link_element(new_Link.name_finger2, new_Link, 'body_3')
            self.add_gazebo_element(new_Link, new_Link.gazebo.body_2, new_Link.name_finger1)
            self.add_gazebo_element(new_Link, new_Link.gazebo.body_3, new_Link.name_finger2)

            self.add_joint_element(new_Link.joint_name_finger1, new_Link, new_Link.base_link_name, new_Link.name_finger1)

            # rotate the finger transform by 180 deg. around z
            new_Link.Proximal_tf = tf_transformations.concatenate_matrices(
                new_Link.Proximal_tf,
                tf_transformations.rotation_matrix(math.pi, [0, 0, 1], point=[0, 0, 0])
            )
            # mirror the mesh on the xy directions
            new_Link.Proximal_tf[0,3] = -1*new_Link.Proximal_tf[0,3]
            new_Link.Proximal_tf[1,3] = -1*new_Link.Proximal_tf[1,3]

            self.add_joint_element(new_Link.joint_name_finger2, new_Link, new_Link.base_link_name, new_Link.name_finger2, mimic_joint=new_Link.joint_name_finger1)

            x_ee, y_ee, z_ee, roll_ee, pitch_ee, yaw_ee = ModuleNode.get_xyzrpy(np.array(new_Link.kinematics.link.pose))
            setattr(new_Link, 'tcp_name', 'ee' + new_Link.tag)
            ET.SubElement(self.root,
                          "xacro:add_tcp",
                          type="pen",
                          name=new_Link.tcp_name,
                          father=new_Link.name,
                          x=x_ee,
                          y=y_ee,
                          z=z_ee,
                          roll=roll_ee,
                          pitch=pitch_ee,
                          yaw=yaw_ee)
            # add the xacro:add_tcp element to the list of urdf elements
            new_Link.xml_tree_elements.append(new_Link.tcp_name)

        elif new_Link.type is ModuleType.SIZE_ADAPTER:
            setattr(new_Link, 'name', 'L_' + str(new_Link.i) + '_size_adapter_' + str(new_Link.p) + new_Link.tag)
            ET.SubElement(self.root,
                            "xacro:add_size_adapter",
                            type="size_adapter",
                            name=new_Link.name,
                            filename=new_Link.filename,
                            size_z=new_Link.kinematics.link.n_l,
                        #   size_in=new_Link.size_in,
                        #   size_out=new_Link.size_out
            )
            # add the xacro:add_size_adapter element to the list of urdf elements
            new_Link.xml_tree_elements.append(new_Link.name)
            new_Link.mesh_names.append(new_Link.name)
            setattr(new_Link, 'flange_size', new_Link.size_out)

        gazebo_body_1 = getattr(getattr(new_Link, 'gazebo', None), 'body_1', None)
        self.add_gazebo_element(new_Link, gazebo_body_1, new_Link.name)

        if new_Link.type in ModuleClass.end_effector_modules():
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
        # add the xacro:add_gripper_fingers element to the list of urdf elements
        new_Link.xml_tree_elements.append(fixed_joint_name)
        setattr(new_Link, 'fixed_joint_name', fixed_joint_name)
        
        return


    def get_hub_output_transform(self, hub):

        connector_name = 'Con_' + str(hub.connector_idx) + '_tf'
        try:
            interface_transform = getattr(hub, connector_name)
        except AttributeError:
            self.error_print('AttributeError: ' + connector_name + ' not found in hub ' + hub.name + '. Either something went wrong during the discovery or the resources should be updated.')
            raise AttributeError
        # if not hub.is_structural:
        #     interface_transform = tf_transformations.identity_matrix()

        self.print('hub.current_port:', hub.current_port)
        self.print('interface_transform: ', interface_transform)

        return interface_transform


    def get_joint_output_transform(self, past_Joint):
        # if the joint is reversed, rotate the distal link frame by 180 deg. around y (as per convention)
        if past_Joint.reverse:
            past_Joint.Distal_tf = ModuleNode.get_rototranslation(past_Joint.Distal_tf,
                                                                 tf_transformations.rotation_matrix(3.14, self.yaxis))
        return past_Joint.Distal_tf


    def get_link_output_transform(self, past_Link):
        return past_Link.Homogeneous_tf


    def get_joint_name(self, module):
        if module.type in ModuleClass.joint_modules() | {ModuleType.DRILL, ModuleType.SPRAYING_TOOL}:
            return module.name
        elif module.type is ModuleType.DAGANA:
            return module.dagana_joint_name
        else:
            return None

    def get_proximal_transform(self, interface_transform, offsets, reverse):
        # compute offset
        T = tf_transformations.translation_matrix((offsets.get('x', 0.0), offsets.get('y', 0.0), offsets.get('z', 0.0)))
        R = tf_transformations.euler_matrix(offsets.get('roll', 0.0), offsets.get('pitch', 0.0), offsets.get('yaw', 0.0), 'sxyz')
        offset_transform = tf_transformations.concatenate_matrices(T, R)
        
        transform = ModuleNode.get_rototranslation(interface_transform,
                                                   offset_transform)
        # If the module is mounted in the opposite direction rotate the final frame by 180 deg., as per convention
        if reverse:
            transform = ModuleNode.get_rototranslation(transform,
                                                       tf_transformations.rotation_matrix(3.14, self.yaxis))

        return transform
    
    # HACK: to handle 90° offset between PINO and CONCERT flanges
    def apply_adapter_transform_rotation(self, interface_transform, size1, size2):
        if size2 < size1:
            self.info_print("Size mismatch: " + size1 + " vs " + size2 + " ---> Rotating input connector of 90°")
            transform = ModuleNode.get_rototranslation(interface_transform,
                                                        tf_transformations.rotation_matrix(1.57,
                                                        self.zaxis))
        else:
            transform = interface_transform

        return transform

    def add_origin(self, parent_el, pose):
        return self.xml_builder.add_origin(parent_el, pose)


    def add_geometry(self, parent_el, geometry):
        return self.xml_builder.add_geometry(parent_el, geometry)

    
    def add_material(self, parent_el, color):
        return self.xml_builder.add_material(parent_el, color)


    def add_inertial(self, parent_el, dynamics, gear_ratio=1.0):
        return self.xml_builder.add_inertial(parent_el, dynamics, gear_ratio=gear_ratio)


    def add_link_element(self, link_name, module_obj, body_name, root=None, is_geared=False):
        return self.xml_builder.add_link_element(link_name, module_obj, body_name, root=root, is_geared=is_geared)


    def add_joint_element(self, joint_name, module_obj, parent_name, child_name, mimic_joint=None):
        return self.xml_builder.add_joint_element(joint_name, module_obj, parent_name, child_name, mimic_joint=mimic_joint)


    def add_rotor_element(self, new_Joint):
        return self.xml_builder.add_rotor_element(new_Joint)


    def add_joint(self, new_Joint, parent_name, transform, reverse):
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(transform)

        if new_Joint.type is ModuleType.JOINT:
            setattr(new_Joint, 'name', 'J' + str(new_Joint.i) + new_Joint.tag)
            setattr(new_Joint, 'distal_link_name', 'L_' + str(new_Joint.i) + new_Joint.tag)
        elif new_Joint.type is ModuleType.WHEEL:
            setattr(new_Joint, 'name', 'J_wheel' + new_Joint.tag)
            setattr(new_Joint, 'distal_link_name', 'wheel' + new_Joint.tag)
        setattr(new_Joint, 'stator_name', new_Joint.name + '_stator')
        setattr(new_Joint, 'fixed_joint_name', "fixed_" + new_Joint.name)
        ET.SubElement(self.root, "xacro:add_fixed_joint",
                      type="fixed_joint_stator",
                      name=new_Joint.fixed_joint_name,
                      father=parent_name,  
                      child=new_Joint.stator_name,
                      x=x,
                      y=y,
                      z=z,
                      roll=roll,
                      pitch=pitch,
                      yaw=yaw)
        # add the xacro:add_fixed_joint element to the list of urdf elements
        new_Joint.xml_tree_elements.append(new_Joint.fixed_joint_name)

        self.collision_elements.append((parent_name, new_Joint.stator_name))

        # Add proximal link
        self.add_link_element(new_Joint.stator_name, new_Joint, 'body_1')
        self.add_gazebo_element(new_Joint, new_Joint.gazebo.body_1, new_Joint.stator_name)

        self.add_joint_element(new_Joint.name, new_Joint, new_Joint.stator_name, new_Joint.distal_link_name)

        # Add distal link
        self.add_link_element(new_Joint.distal_link_name, new_Joint, 'body_2')
        self.add_gazebo_element(new_Joint, new_Joint.gazebo.body_2, new_Joint.distal_link_name)
       
        # Add proximal/distal links pair to the list of collision elements to ignore
        self.collision_elements.append((new_Joint.stator_name, new_Joint.distal_link_name))

        # Add rotor part if present in the module_description. The REFLECT_ROTOR_INERTIA xacro mapping will determine if the rotor inertia is reflected or not
        if hasattr(new_Joint.dynamics, 'body_2_fast'):
            self.add_rotor_element(new_Joint)


    def add_hub(self, new_Hub, parent_name, transform, hub_name=None):
        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(transform)

        if hub_name:
            setattr(new_Hub, 'name', hub_name)
        else:
            setattr(new_Hub, 'name', 'L_' + str(new_Hub.i) + '_link_' + str(new_Hub.p) + new_Hub.tag)

        self.add_link_element(new_Hub.name, new_Hub, 'body_1')
        self.add_gazebo_element(new_Hub, new_Hub.gazebo.body_1, new_Hub.name)
        
        self.add_connectors(new_Hub)

        fixed_joint_name = 'fixed_' + new_Hub.name

        ET.SubElement(self.root,
                        "xacro:add_fixed_joint",
                        name=fixed_joint_name,
                        type="fixed_joint",
                        father=parent_name,
                        child=new_Hub.name,
                        x=x,
                        y=y,
                        z=z,
                        roll=roll,
                        pitch=pitch,
                        yaw=yaw)
        # add the xacro:add_fixed_joint element to the list of urdf elements
        new_Hub.xml_tree_elements.append(fixed_joint_name)
        setattr(new_Hub, 'fixed_joint_name', fixed_joint_name)

        if new_Hub.type is ModuleType.MOBILE_BASE:
            ET.SubElement(self.root, 
                        "xacro:add_mobile_base_sensors",
                        name=new_Hub.name + '_sensors',
                        parent_name=new_Hub.name)
            # add the xacro:add_mobile_base_sensors element to the list of urdf elements
            new_Hub.xml_tree_elements.append(new_Hub.name + '_sensors')

            self.add_sensor_name('lidar/velodyne', ['VLP16_lidar_front', 'VLP16_lidar_back'])
            self.add_sensor_name('ultrasound/bosch_uss5', [
                'ultrasound_fl_sag',
                'ultrasound_fr_sag',
                'ultrasound_rl_sag',
                'ultrasound_rr_sag',
                'ultrasound_fl_lat',
                'ultrasound_fr_lat',
                'ultrasound_rl_lat',
                'ultrasound_rr_lat',
            ])

        # Add hub and parent links pair to the list of collision elements to ignore
        self.collision_elements.append((parent_name, new_Hub.name))


    # noinspection PyPep8Naming
    def link_after_hub(self, new_Link, past_Hub, offsets, reverse):
        """Adds to the URDF tree a link module as a child of a hub module

        Parameters
        ----------
        new_Link: ModuleNode.ModuleNode
            ModuleNode object of the link module to add

        past_Hub: ModuleNode.ModuleNode
            ModuleNode object of the hub module to which attach the link

        offsets: dict
            Dictionary containing the various offsets ('x','y','z','roll','pitch','yaw') between the parent module output frame and the module input frame. Es. offsets = {'x': 1.0, 'y': 2.0, 'yaw': 1.57}
        """
        setattr(new_Link, 'p', 0)
        setattr(new_Link, 'i', 0)

        if past_Hub.is_structural:
            parent_name = past_Hub.name
        else:
            parent_name = past_Hub.parent.name

        interface_transform = self.get_hub_output_transform(past_Hub)

        transform = self.get_proximal_transform(interface_transform, offsets, reverse)

        # HACK: to handle 90 deg offset between PINO and CONCERT flanges
        transform = self.apply_adapter_transform_rotation(transform, past_Hub.flange_size, new_Link.flange_size)

        self.add_link(new_Link, parent_name, transform, reverse)

        self.collision_elements.append((past_Hub.name, new_Link.name))


    # noinspection PyPep8Naming
    def joint_after_hub(self, new_Joint, past_Hub, offsets, reverse):
        """Adds to the URDF tree a joint module as a child of a hub module

        Parameters
        ----------
        new_Joint: ModuleNode.ModuleNode
            ModuleNode object of the joint module to add

        past_Hub: ModuleNode.ModuleNode
            ModuleNode object of the hub module to which the joint will be attached

        offsets: dict
            Dictionary containing the various offsets ('x','y','z','roll','pitch','yaw') between the parent module output frame and the module input frame. Es. offsets = {'x': 1.0, 'y': 2.0, 'yaw': 1.57}
        """
        if past_Hub.is_structural:
            parent_name = past_Hub.name
        else:
            parent_name = past_Hub.parent.name

        interface_transform = self.get_hub_output_transform(past_Hub)

        transform = self.get_proximal_transform(interface_transform, offsets, reverse)

        # HACK: to handle 90 deg offset between PINO and CONCERT flanges
        transform = self.apply_adapter_transform_rotation(transform, past_Hub.flange_size, new_Joint.flange_size)

        setattr(new_Joint, 'i', 1)
        setattr(new_Joint, 'p', 0)

        self.add_joint(new_Joint, parent_name, transform, reverse)

    
    def hub_after_hub(self, new_Hub, past_Hub, offsets, reverse, module_name=None):
        """Adds to the URDF tree a hub module as a child of a hub module

        Parameters
        ----------
        new_Hub: ModuleNode.ModuleNode
            ModuleNode object of the hub module to add

        past_Hub: ModuleNode.ModuleNode
            ModuleNode object of the hub module to which the hub will be attached

        offsets: dict
            Dictionary containing the various offsets ('x','y','z','roll','pitch','yaw') between the parent module output frame and the module input frame. Es. offsets = {'x': 1.0, 'y': 2.0, 'yaw': 1.57}
        """
        if past_Hub.is_structural:
            parent_name = past_Hub.name
        else:
            parent_name = past_Hub.parent.name

        setattr(new_Hub, 'i', 0)
        setattr(new_Hub, 'p', past_Hub.p + 1)

        interface_transform = self.get_hub_output_transform(past_Hub)

        transform = self.get_proximal_transform(interface_transform, offsets, reverse)

        # HACK: to handle 90° offset between PINO and CONCERT flanges
        transform = self.apply_adapter_transform_rotation(transform, past_Hub.flange_size, new_Hub.flange_size)

        # Set the number of child hubs to 0 (it will be incremented when a child hub is added)
        setattr(new_Hub, 'n_children_hubs', 0)

        if new_Hub.is_structural:
            self.add_hub(new_Hub, parent_name, transform, hub_name=module_name)
        else:
            # HACK: we set the name of the non-structural hub to be the same as the parent. This is needed to correctly write the SRDF chains!
            setattr(new_Hub, 'name', parent_name)
            # HACK: we set the connectors of the non-structural hub to be the same as the parent.
            new_Hub.connectors = past_Hub.connectors
            # if the parent is a hub, the n_children_hubs attribute is incremented, in order to keep track of the number of hubs connected to the parent hub and therefore the number of ports occupied. This is needed to select the right connector where to connect the new module 
            self.parent_module.n_children_hubs += 1
        
        #  Add the hub to the list of hubs
        self.listofhubs.append(new_Hub)   


    def hub_after_link(self, new_Hub, past_Link, offsets, reverse, module_name=None):
        """Adds to the URDF tree a hub module as a child of a link module

        Parameters
        ----------
        new_Hub: ModuleNode.ModuleNode
            ModuleNode object of the hub module to add

        past_Link: ModuleNode.ModuleNode
            ModuleNode object of the link module to which the hub will be attached

        offsets: dict
            Dictionary containing the various offsets ('x','y','z','roll','pitch','yaw') between the parent module output frame and the module input frame. Es. offsets = {'x': 1.0, 'y': 2.0, 'yaw': 1.57}
        """
        setattr(new_Hub, 'i', past_Link.i)
        setattr(new_Hub, 'p', past_Link.p + 1)

        interface_transform = self.get_link_output_transform(past_Link)

        transform = self.get_proximal_transform(interface_transform, offsets, reverse=reverse)

        # HACK: to handle 90° offset between PINO and CONCERT flanges
        transform = self.apply_adapter_transform_rotation(transform, past_Link.flange_size, new_Hub.flange_size)

        parent_name = past_Link.name

        # Set the number of child hubs to 0 (it will be incremented when a child hub is added)
        setattr(new_Hub, 'n_children_hubs', 0)

        if new_Hub.is_structural:
            self.add_hub(new_Hub, parent_name, transform, hub_name=module_name)

        #  Add the hub to the list of hubs
        self.listofhubs.append(new_Hub) 


    def hub_after_joint(self, new_Hub, past_Joint, offsets, reverse, module_name=None):
        """Adds to the URDF tree a hub module as a child of a joint module

        Parameters
        ----------
        new_Hub: ModuleNode.ModuleNode
            ModuleNode object of the hub module to add

        past_Joint: ModuleNode.ModuleNode
            ModuleNode object of the joint module to which the hub will be attached

        offsets: dict
            Dictionary containing the various offsets ('x','y','z','roll','pitch','yaw') between the parent module output frame and the module input frame. Es. offsets = {'x': 1.0, 'y': 2.0, 'yaw': 1.57}
        """
        setattr(new_Hub, 'i', past_Joint.i)
        setattr(new_Hub, 'p', past_Joint.p + 1)

        interface_transform = self.get_joint_output_transform(past_Joint)

        transform = self.get_proximal_transform(interface_transform, offsets, reverse=reverse)

        # HACK: to handle 90° offset between PINO and CONCERT flanges
        transform = self.apply_adapter_transform_rotation(transform, past_Joint.flange_size, new_Hub.flange_size)

        parent_name = past_Joint.distal_link_name

        # Set the number of child hubs to 0 (it will be incremented when a child hub is added)
        setattr(new_Hub, 'n_children_hubs', 0)

        if new_Hub.is_structural:
            self.add_hub(new_Hub, parent_name, transform, hub_name=module_name)
        
        #  Add the hub to the list of hubs
        self.listofhubs.append(new_Hub) 


    # noinspection PyPep8Naming
    def link_after_joint(self, new_Link, past_Joint, offsets, reverse):
        """Adds to the URDF tree a link module as a child of a joint module

        Parameters
        ----------
        new_Link: ModuleNode.ModuleNode
            ModuleNode object of the link module to add

        past_Joint: ModuleNode.ModuleNode
            ModuleNode object of the joint module to which attach the link

        offsets: dict
            Dictionary containing the various offsets ('x','y','z','roll','pitch','yaw') between the parent module output frame and the module input frame. Es. offsets = {'x': 1.0, 'y': 2.0, 'yaw': 1.57}
        """
        setattr(new_Link, 'i', past_Joint.i)
        setattr(new_Link, 'p', past_Joint.p + 1)

        interface_transform = self.get_joint_output_transform(past_Joint)

        transform = self.get_proximal_transform(interface_transform, offsets, reverse)

        # HACK: to handle 90° offset between PINO and CONCERT flanges
        transform = self.apply_adapter_transform_rotation(transform, past_Joint.flange_size, new_Link.flange_size)

        parent_name = past_Joint.distal_link_name

        self.add_link(new_Link, parent_name, transform, reverse)

        self.collision_elements.append((past_Joint.distal_link_name, new_Link.name))

    # noinspection PyPep8Naming
    def joint_after_joint(self, new_Joint, past_Joint, offsets, reverse):
        """Adds to the URDF tree a joint module as a child of a joint module

        Parameters
        ----------
        new_Joint: ModuleNode.ModuleNode
            ModuleNode object of the joint module to add

        past_Joint: ModuleNode.ModuleNode
            ModuleNode object of the joint module to which the joint will be attached

        offsets: dict
            Dictionary containing the various offsets ('x','y','z','roll','pitch','yaw') between the parent module output frame and the module input frame. Es. offsets = {'x': 1.0, 'y': 2.0, 'yaw': 1.57}
        """
        interface_transform = self.get_joint_output_transform(past_Joint)

        transform = self.get_proximal_transform(interface_transform, offsets, reverse)

        # HACK: to handle 90° offset between PINO and CONCERT flanges
        transform = self.apply_adapter_transform_rotation(transform, past_Joint.flange_size, new_Joint.flange_size)

        setattr(new_Joint, 'i', past_Joint.i + 1)
        setattr(new_Joint, 'p', 0)

        parent_name = 'L_' + str(past_Joint.i) + past_Joint.tag

        self.add_joint(new_Joint, parent_name, transform, reverse)

    # noinspection PyPep8Naming
    def joint_after_link(self, new_Joint, past_Link, offsets, reverse):
        """Adds to the URDF tree a joint module as a child of a link module

        Parameters
        ----------
        new_Joint: ModuleNode.ModuleNode
            ModuleNode object of the joint module to add

        past_Link: ModuleNode.ModuleNode
            ModuleNode object of the link module to which the joint will be attached

        offsets: dict
            Dictionary containing the various offsets ('x','y','z','roll','pitch','yaw') between the parent module output frame and the module input frame. Es. offsets = {'x': 1.0, 'y': 2.0, 'yaw': 1.57}
        """
        interface_transform = self.get_link_output_transform(past_Link)

        transform = self.get_proximal_transform(interface_transform, offsets, reverse)

        # HACK: to handle 90° offset between PINO and CONCERT flanges
        transform = self.apply_adapter_transform_rotation(transform, past_Link.flange_size, new_Joint.flange_size)

        setattr(new_Joint, 'i', past_Link.i + 1)
        setattr(new_Joint, 'p', 0)

        parent_name = past_Link.name

        self.add_joint(new_Joint, parent_name, transform, reverse)

    # noinspection PyPep8Naming
    def link_after_link(self, new_Link, past_Link, offsets, reverse):
        """Adds to the URDF tree a joint module as a child of a link module

        Parameters
        ----------
        new_Link: ModuleNode.ModuleNode
            ModuleNode object of the link module to add

        past_Link: ModuleNode.ModuleNode
            ModuleNode object of the link module to which the joint will be attached

        offsets: dict
            Dictionary containing the various offsets ('x','y','z','roll','pitch','yaw') between the parent module output frame and the module input frame. Es. offsets = {'x': 1.0, 'y': 2.0, 'yaw': 1.57}
        """

        setattr(new_Link, 'i', past_Link.i)
        setattr(new_Link, 'p', past_Link.p + 1)

        interface_transform = self.get_link_output_transform(past_Link)

        transform = self.get_proximal_transform(interface_transform, offsets, reverse)

        # HACK: to handle 90° offset between PINO and CONCERT flanges
        transform = self.apply_adapter_transform_rotation(transform, past_Link.flange_size, new_Link.flange_size)

        parent_name = past_Link.name

        self.add_link(new_Link, parent_name, transform, reverse)

        self.collision_elements.append((past_Link.name, new_Link.name))


    # TODO: remove hard-coded values
    def write_problem_description_multi(self):
        basic_probdesc_filename = self.resource_finder.get_filename('cartesio/ModularBot_cartesio_IK_config.yaml',
                                                          ['data_path'])
        # basic_probdesc_filename = path_name + '/cartesio/ModularBot_cartesio_config.yaml'
        probdesc_filename = path_name + '/ModularBot/cartesio/ModularBot_cartesio_IK_config.yaml'
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
        active_modules_chains = self.get_actuated_modules_chains()
        for joints_chain in active_modules_chains:
            ee_name = "EE_" + str(i + 1)
            tasks.append(ee_name)
            probdesc['stack'] = stack
            probdesc[ee_name] = copy.deepcopy(probdesc['EE'])

            tip_link = self.find_chain_tip_link(joints_chain)
            probdesc[ee_name]['distal_link'] = tip_link

            base_link = self.find_chain_base_link(joints_chain)
            probdesc[ee_name]['base_link'] = base_link
            # probdesc[ee_name]['type'] = "Interaction"
            probdesc[ee_name]['type'] = "Cartesian"
            probdesc[ee_name]['lambda'] = 0.1

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
                                                                   ['data_path'])
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
        active_modules_chains = self.get_actuated_modules_chains()
        joints_chain = active_modules_chains[0]
        tip_link = self.find_chain_tip_link(joints_chain)
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

    def write_srdf(self, builder_joint_map=None, compute_acm=True, num_trials=1e5):
        """Generates a basic srdf so that the model can be used right away with XBotCore"""
        
        global path_name
        srdf_filename = path_name + '/ModularBot/srdf/ModularBot.srdf'
        # srdf_filename = path_superbuild + '/configs/ADVR_shared/ModularBot/srdf/ModularBot.srdf'
        # srdf_filename = self.urdf_writer.resource_finder.get_filename('srdf/ModularBot.srdf', 'modularbot_path')
        # srdf_filename = "/tmp/modular/srdf/ModularBot.srdf"
        
        # Generate srdf string
        srdf_string = self.control_plugin.write_srdf(builder_joint_map)

        # Compute Allowed Collision Matrix (ACM) using MoveIt!
        if compute_acm:
            srdf_string = self.add_acm_to_srdf(srdf_string, num_trials=num_trials)

        # Create folder if doesen't exist
        if not os.path.exists(os.path.dirname(srdf_filename)):
            try:
                os.makedirs(os.path.dirname(srdf_filename))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        with open(srdf_filename, 'w+') as f:
            f.write(srdf_string)

        return srdf_string
    
    def add_acm_to_srdf(self, srdf, num_trials=1e5):
        """Compute Allowed Collision Matrix (ACM) using MoveIt!"""
        
        try:
            # Prefer current binding name (top-level module), keep legacy fallback.
            try:
                import pymcdc
            except ImportError:
                import importlib
                pymcdc = importlib.import_module('moveit_compute_default_collisions.pymcdc')

            # Ensure URDF string is available before ACM computation.
            if getattr(self, 'urdf_dirty', False) or not getattr(self, 'urdf_string', None) or not self.urdf_string.strip():
                self.process_urdf()

            if not self.urdf_string or not self.urdf_string.strip():
                raise RuntimeError("URDF string is empty, cannot compute ACM")

            # set verbosity level of the mcdc module
            if self.logger.level == logging.DEBUG:
                mcdc_verbose = True
                self.print("Allowed Collision Matrix (ACM) computation")
            else:
                mcdc_verbose = False

            # generate acm and write it into srdf
            acm = pymcdc.MoveitComputeDefaultCollisions()
            acm.setVerbose(mcdc_verbose)
            acm.initFromString(self.urdf_string, srdf, False)
            acm.computeDefaultCollisions(int(num_trials))
            if mcdc_verbose:
                acm.printDisabledCollisions()
            srdf_with_acm = acm.getXmlString()
            srdf = srdf_with_acm 

        except ImportError:
            self.info_print("Cannot import pymcdc bindings, skipping ACM computation")

        return srdf

    # Function writin the urdf file after converting from .xacro (See xacro/__init__.py for reference)
    def write_urdf(self):
        """Returns the string with the URDF, after writing it to file"""
        global path_name  # , path_superbuild

        urdf_filename = path_name + '/ModularBot/urdf/ModularBot.urdf'
        # urdf_filename = path_superbuild + '/configs/ADVR_shared/ModularBot/urdf/ModularBot.urdf'
        # urdf_filename = self.resource_finder.get_filename('urdf/ModularBot.urdf', 'modularbot_path')
        # urdf_filename= '/tmp/modular/urdf/ModularBot.urdf'
        out = xacro.open_output(urdf_filename)

        gazebo_urdf_filename = path_name + '/ModularBot/urdf/ModularBot.gazebo.urdf'
        gazebo_out = xacro.open_output(gazebo_urdf_filename)

        # get xml string from ET
        xmlstr = xml.dom.minidom.parseString(ET.tostring(self.urdf_tree.getroot())).toprettyxml(indent="   ")

        # write preprocessed xml to file
        urdf_xacro_filename = path_name + '/ModularBot/urdf/ModularBot.urdf.xacro'
        # # urdf_xacro_filename = self.resource_finder.get_filename('urdf/ModularBot.urdf.xacro', 'modularbot_path')
        # # urdf_xacro_filename = '/tmp/modular/urdf/ModularBot.urdf.xacro'
        preprocessed_out = xacro.open_output(urdf_xacro_filename)
        preprocessed_out.write(xmlstr)
        preprocessed_out.close()

        # write the URDF for Gazebo
        string_urdf_gz = self.process_urdf(xacro_mappings={'gazebo_urdf': 'true', 'velodyne': 'true', 'realsense': 'true', 'ultrasound': 'true'})
        gazebo_out.write(string_urdf_gz)
        gazebo_out.close()

        # write the URDF
        string_urdf_xbot = self.process_urdf(xacro_mappings={'gazebo_urdf': 'false', 'velodyne': 'false', 'realsense': 'false', 'ultrasound': 'false'})
        out.write(string_urdf_xbot)
        out.close()

        self.urdf_string = string_urdf_xbot

        return string_urdf_xbot

    def write_sensor_config(self, output_path=None, xacro_mappings=None):
        def nest_sensor_names(sensor_names):
            tree = {}

            def add_unique_names(target_list, new_names):
                for name in new_names:
                    if name not in target_list:
                        target_list.append(name)

            def format_node(node):
                names = copy.deepcopy(node.get('_names', []))
                children = {
                    key: format_node(value)
                    for key, value in node.items()
                    if key != '_names'
                }

                if not children:
                    return names

                if names:
                    children = {'generic': names, **children}

                return children

            for sensor_type, names in sensor_names.items():
                sensor_levels = [level.strip() for level in str(sensor_type).split('/') if level.strip()]
                if not sensor_levels:
                    continue

                node = tree
                for level in sensor_levels:
                    node = node.setdefault(level, {'_names': []})

                add_unique_names(node.setdefault('_names', []), list(names))

            return {
                key: format_node(value)
                for key, value in tree.items()
            }

        global path_name
        if output_path is None:
            output_path = path_name + '/ModularBot/sensors/ModularBot.sensors.yaml'
        sensor_config = self.get_sensor_configuration(xacro_mappings=xacro_mappings)
        sensor_config['sensor_names'] = nest_sensor_names(sensor_config.get('sensor_names', {}))
        content = yaml.safe_dump(sensor_config, default_flow_style=False, sort_keys=False)

        output_dir = os.path.dirname(output_path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)

        with open(output_path, 'w') as f:
            f.write(content)

        return content

    # Save URDF/SRDF etc. in a directory with the specified robot_name
    def deploy_robot(self, robot_name='modularbot', deploy_dir=None):
        script = self.resource_finder.get_filename('deploy.sh', ['data_path'])

        try:
            if deploy_dir is None:
                deploy_dir = self.resource_finder.get_expanded_path(['deploy_dir'])
            if self.verbose:
                output = subprocess.check_output([script, robot_name, "--destination-folder", deploy_dir, "-v"])
            else:
                output = subprocess.check_output([script, robot_name, "--destination-folder", deploy_dir])
        except (subprocess.CalledProcessError, FileNotFoundError, RuntimeError) as e:
            self.error_print(f"An error occurred when executing deploy script: {script}. Aborting.")
            raise e

        self.info_print(str(output, 'utf-8', 'ignore'))

        hubs = self.findall_by_type(types=ModuleClass.hub_modules())
        if hubs is not None:
            for hub_module in (hub for hub in hubs if hub.is_structural):
                self.add_connectors(hub_module)

        for post_processing_script in self.resource_finder.cfg['post_processing_scripts']:
            try:
                script_path = self.resource_finder.get_expanded_path(['post_processing_scripts', post_processing_script, 'file'])
                required = self.resource_finder.nested_access(['post_processing_scripts', post_processing_script, 'required'])
                self.info_print(f"Running post processing script {post_processing_script}: {script_path}")
                output = subprocess.check_output([script_path, "--destination-folder", deploy_dir, "--package-name", robot_name])
            except (subprocess.CalledProcessError, FileNotFoundError, RuntimeError) as e:
                if required:
                    raise(e)
                else:
                    self.error_print(e)
                    self.error_print(f"Skipping post processing script {post_processing_script}")
                    pass

        return robot_name

    # Remove connectors when deploying the robot
    def remove_all_connectors(self):
        return self.xml_builder.remove_all_connectors()


    def findall_by_type(self, types=[]):
        # Serch the tree by name for the selected module
        modulenodes = anytree.search.findall(self.base_link, filter_=lambda node: node.type in types)
        return modulenodes

    def add_connectors(self, modulenode):
        return self.xml_builder.add_connectors(modulenode)
                
    def compute_payload(self, samples):
        self.model_stats.update_model()
        return self.model_stats.compute_payload(n_samples=samples)
    

    def compute_stats(self, samples=1000):
        self.model_stats.update_model()
        return self.model_stats.compute_stats(n_samples=samples)


    @staticmethod
    def parse_generator_cli_args(argv=None, known_only=False):
        """Parse generator CLI arguments used by example and deployment scripts.

        Args:
            argv: Optional iterable of CLI tokens. If None, argparse reads from
                sys.argv.
            known_only: When True, return ``(args, unknown_args)`` using
                ``parse_known_args``. When False, return only ``args`` using
                ``parse_args``.

        Returns:
            argparse.Namespace or tuple[argparse.Namespace, list[str]]:
                Parsed arguments, with optional unknown tokens when
                ``known_only`` is enabled.
        """
        parser = argparse.ArgumentParser(
            prog='Modular URDF/SRDF generator and deployer',
            usage='./script_name.py --output urdf writes URDF to stdout '\
                  '\n./script_name.py --deploy deploy_dir generates a ros package at deploy_dir',
        )

        parser.add_argument('--output', '-o', required=False, choices=('urdf', 'srdf', 'sensors'),
                            help='write requested file to stdout and exit')
        parser.add_argument('--xacro-args', '-a', required=False, nargs='*',
                            help='xacro arguments in key:=value format')
        parser.add_argument('--deploy', '-d', required=False,
                            help='directory where to deploy the package')
        parser.add_argument('--robot-name', '-r', required=False,
                            help='name of the robot')
        parser.add_argument('--quiet', action='store_true', default=False,
                            help='suppress logger output while generating files')

        if known_only:
            return parser.parse_known_args(argv)
        return parser.parse_args(argv)


    def write_file_to_stdout(self, homing_map, robot_name='modularbot', args=None):
        """Execute CLI-driven generation actions and print selected output.

        This method is the entrypoint used by generator scripts after creating
        a ``UrdfWriter`` instance. Depending on parsed CLI arguments, it can:
        generate URDF/SRDF/sensor YAML content, write a copy under ``/tmp``,
        print generated content to stdout, and optionally deploy a full robot
        package.

        Args:
            homing_map: Joint homing map used when generating SRDF.
            robot_name: Default robot name used for output filenames and deploy.
                Overridden by ``--robot-name`` when present.
            args: Optional pre-parsed CLI namespace from
                ``parse_generator_cli_args``. If None, arguments are parsed from
                ``sys.argv``.

        Side effects:
            - May set logger level to ``CRITICAL`` when ``--quiet`` is enabled.
            - Writes generated files under ``/tmp`` for selected outputs.
            - May deploy URDF/SRDF/config artifacts when ``--deploy`` is set.
        """
        if args is None:
            args = self.parse_generator_cli_args()

        if args.robot_name is not None:
            robot_name = args.robot_name

        xacro_mappings = {}
        if args.xacro_args:
            for arg in args.xacro_args:
                key, value = arg.split(':=', 1)
                xacro_mappings[key] = value

        if args.quiet and isinstance(self.logger, logging.Logger):
            self.quiet = True
            self.logger.setLevel(logging.CRITICAL)

        content = None
        self.remove_all_connectors()

        if args.output == 'urdf':
            content = self.process_urdf(xacro_mappings=xacro_mappings)
            open(f'/tmp/{robot_name}.urdf', 'w').write(content)

        elif args.output == 'srdf':
            self.process_urdf(xacro_mappings=xacro_mappings)
            content = self.write_srdf(homing_map, num_trials=1e3)
            open(f'/tmp/{robot_name}.srdf', 'w').write(content)

        elif args.output == 'sensors':
            self.process_urdf(xacro_mappings=xacro_mappings)
            content = self.write_sensor_config(f'/tmp/{robot_name}.sensors.yaml', xacro_mappings)

        if content is not None:
            print(content)

        if args.deploy is not None:
            self.write_urdf()
            self.write_lowlevel_config()
            self.write_problem_description_multi()
            self.write_srdf(homing_map, num_trials=1e3)
            self.write_joint_map()
            self.write_sensor_config()

            self.deploy_robot(robot_name, args.deploy)


# Temporary functions to maintain backwards compatibility with the old generator scripts
from contextlib import contextmanager
import sys, os

def parse_generator_cli_args(argv=None, known_only=False):
    return UrdfWriter.parse_generator_cli_args(argv=argv, known_only=known_only)
