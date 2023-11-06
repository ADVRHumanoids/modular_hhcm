"""
This module defines the 'Module' class, used to describe any link or joint module, and some functions to instantiate
a module reading from a YAML file.
The 'Module' class is actually extended by the 'ModuleNode' class which make the
object a node of a tree, so that a robot can be represented by a tree of 'Module' objects.
"""
import yaml
import json
import sys
import os
import tf
from enum import Enum
import anytree

from modular.utils import ResourceFinder

class ModuleDescriptionFormat(Enum):
    YAML = 1  # YAML format used internally in HHCM
    JSON = 2  # JSON format used for the CONCERT project  

class KinematicsConvention(str, Enum):
    """Kinematic convention used to define the rototranslation between two modules"""
    DH_EXT = 'DH_ext'  # extended Denavit-Hartenberg convention. See https://mediatum.ub.tum.de/doc/1280464/file.pdf
    URDF = 'urdf'  # URDF convention
    AFFINE = 'affine_tf_matrix'  # Affine trasformation matrix convention

class ModuleType(str, Enum):
    """Type of module"""
    LINK = 'link'
    JOINT = 'joint'
    CUBE = 'cube'
    WHEEL = 'wheel'
    TOOL_EXCHANGER = 'tool_exchanger'
    GRIPPER = 'gripper'
    MOBILE_BASE = 'mobile_base'
    BASE_LINK = 'base_link'
    SIZE_ADAPTER = 'size_adapter'
    SIMPLE_EE = 'simple_ee'
    END_EFFECTOR = 'end_effector'
    DRILL = 'drill'
    DAGANA = 'dagana'

# import collections.abc
def update_nested_dict(d, u):
    for k, v in u.items():
        if isinstance(v, dict):
            d[k] = update_nested_dict(d.get(k, {}), v)
        else:
            d[k] = v
    return d

def as_dumpable_dict(obj):
    """ Convert object to nested Python dictionary dumpable to YAML """
    props = {}
    if isinstance(obj, (Module, Module.Attribute)):
       for k,v in vars(obj).items():
        if isinstance(v, Module.Attribute):
            props[k] = as_dumpable_dict(v)
        elif isinstance(v, (list, tuple)):
            props[k] = as_dumpable_dict(v)
        elif isinstance(v, (KinematicsConvention, ModuleType)):
            props[k] = v.value
        else:
            props[k] = v
    elif isinstance(obj, (list, tuple)):
        props = []
        for x in obj:
            if isinstance(x, (list, tuple)):
                props.append(as_dumpable_dict(x))
            elif isinstance(x, Module.Attribute):
                props.append(as_dumpable_dict(x))
            elif isinstance(x, (KinematicsConvention, ModuleType)):
                props.append(x.value)
            else:
                props.append(x)
    else:
        props = obj
    return props

class JSONInterpreter(object):
    """Class used to interpret the JSON file describing a module"""
    def __init__(self, owner, d, json_file=None):
        self.owner = owner
        self.parse_dict(d)
        self.owner.set_flange_size()

        # set filename
        if json_file is not None:
            path_without_extension = os.path.splitext(json_file)[0]
            filename_without_extension = path_without_extension.split('/')[-1]
            new_filename = '/tmp/ModulesDescriptions/' + filename_without_extension + '.yaml'
            os.makedirs(os.path.dirname(new_filename), exist_ok=True)
            out = as_dumpable_dict(self.owner)
            with open(new_filename, 'w') as outfile:
                yaml.safe_dump(out, outfile, default_flow_style=False)
            self.owner.filename = new_filename
        else:
            self.owner.filename = None

    def parse_dict(self, d):
        """Parse the dictionary d and set the attributes of the owner module"""
        self.owner.kinematics_convention = KinematicsConvention.AFFINE
        self.owner.type = ModuleType(d['header']['type'])
        self.type_dispatcher(d)

    def type_dispatcher(self, d):
        """Dispatch the parsing of the dictionary d according to the module type"""
        header_obj = getattr(self.owner, "header")
        update_nested_dict(header_obj.__dict__, d['header'])
        if self.owner.type in {ModuleType.LINK, ModuleType.GRIPPER, ModuleType.TOOL_EXCHANGER, ModuleType.SIZE_ADAPTER, ModuleType.END_EFFECTOR, ModuleType.DRILL}:
            if len(d['joints']) != 0:
                raise ValueError('A link must have no joints')
            if len(d['bodies']) != 1:
                raise ValueError('A link must have exactly one body')
            if len(d['bodies'][0]['connectors']) != 2:
                raise ValueError('A link must have exactly 2 connectors')
            dict_body_1 = d['bodies'][0]
            # kinematics
            #HACK: We assume that the output connector is the last one. Otherwise, we should check the ID
            output_connector = dict_body_1['connectors'][-1]
            # for con in dict_body_1['Connectors']:
            #     if con['ID'] == 'out':
            #         output_connector = con
            self.owner.kinematics.link.pose = output_connector['pose']
            # dynamics
            self.set_dynamic_properties(self.owner.dynamics.body_1, dict_body_1)
            # visual
            self.set_visual_properties(self.owner.visual, 'body_1', dict_body_1)
            # collision
            self.set_collision_properties(self.owner.collision, 'body_1', dict_body_1)
            # gazebo
            self.set_gazebo_properties(self.owner.gazebo.body_1, dict_body_1)
            # flange_size
            #HACK: We assume the flange_size is the same for all the connectors and load it from the output connector
            output_flange_size = output_connector['size']
            if self.owner.type is ModuleType.SIZE_ADAPTER:
                self.owner.size_out = output_flange_size
                self.owner.size_in = d['bodies'][0]['connectors'][0]['size']
            else:
                self.owner.flange_size = output_flange_size 
        elif self.owner.type in {ModuleType.JOINT, ModuleType.WHEEL}:
            if len(d['joints']) != 1:
                raise ValueError('A joint must have exactly one joint')
            if len(d['bodies']) != 2:
                raise ValueError('A joint must have exactly two bodies')
            if len(d['bodies'][0]['connectors']) != 1 and len(d['bodies'][1]['connectors']) != 1:
                raise ValueError('A joint must have exactly one connector for each body')
            dict_joint = d['joints'][0]
            dict_body_1 = d['bodies'][0]
            dict_body_2 = d['bodies'][-1]

            # kinematics
            proximal_pose = Module.Attribute({'pose': dict_joint['pose_parent']}) 
            # x, y, z, roll, pitch, yaw = get_xyzrpy(tf.transformations.numpy.array(dict_joint['pose_parent']))
            # proximal_pose = Module.Attribute({'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch, 'yaw': yaw})
            update_nested_dict(self.owner.kinematics.joint.proximal.__dict__, proximal_pose.__dict__)
            
            distal_pose = Module.Attribute({'pose': dict_joint['pose_child']}) 
            # x, y, z, roll, pitch, yaw = get_xyzrpy(tf.transformations.numpy.array(dict_joint['pose_child']))
            # distal_pose = Module.Attribute({'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch, 'yaw': yaw})
            update_nested_dict(self.owner.kinematics.joint.distal.__dict__, distal_pose.__dict__)
            
            # joint data
            self.owner.actuator_data.type = dict_joint['type']
            self.owner.actuator_data.upper_limit = dict_joint['limits']['positionUpper']
            self.owner.actuator_data.lower_limit = dict_joint['limits']['positionLower']
            self.owner.actuator_data.velocity = dict_joint['limits']['velocity']
            self.owner.actuator_data.effort = dict_joint['limits']['peak_torque']
            self.owner.actuator_data.gear_ratio = dict_joint['gear_ratio']
            self.owner.actuator_data.zero_offset = 0.0
            # dynamics
            self.set_dynamic_properties(self.owner.dynamics.body_1, dict_body_1)
            self.set_dynamic_properties(self.owner.dynamics.body_2, dict_body_2)
            # visual
            self.set_visual_properties(self.owner.visual, 'body_1', dict_body_1)
            self.set_visual_properties(self.owner.visual, 'body_2', dict_body_2)
            # collision
            self.set_collision_properties(self.owner.collision, 'body_1', dict_body_1)
            self.set_collision_properties(self.owner.collision, 'body_2', dict_body_2)
            # gazebo
            self.set_gazebo_properties(self.owner.gazebo.body_1, dict_body_1)
            self.set_gazebo_properties(self.owner.gazebo.body_2, dict_body_2)
            # flange_size
            output_connector = dict_body_2['connectors'][-1]
            self.owner.flange_size = output_connector['size']
            # CentAcESC
            self.owner.CentAcESC = Module.Attribute(dict_joint['control_parameters']['xbot'])
            # xbot_gz
            self.owner.xbot_gz = Module.Attribute(dict_joint['control_parameters']['xbot_gz'])
        elif self.owner.type in {ModuleType.CUBE, ModuleType.MOBILE_BASE}:
            if len(d['joints']) != 0:
                raise ValueError('A hub must have no joints')
            if len(d['bodies']) != 1:
                raise ValueError('A hub must have exactly one body')
            # if len(d['Bodies'][0]['Connectors']) != 5:
            #     raise ValueError('A hub must have exactly 5 connectors')
            body_1 = d['bodies'][0]
            for idx, con in enumerate(body_1['connectors']):             
                attr = Module.Attribute({'pose': con['pose']})
                prefix = d['header']['ID']+'_'
                con_name = con['ID'][len(prefix):] if con['ID'].startswith(prefix) else con['ID']
                setattr(self.owner.kinematics, con_name, Module.Attribute({}))
                con_obj = getattr(self.owner.kinematics, con_name)
                update_nested_dict(con_obj.__dict__, attr.__dict__)
                # Module.Attribute(con_obj, con['pose'])
            # dynamics
            self.set_dynamic_properties(self.owner.dynamics.body_1, body_1)
            # visual
            self.set_visual_properties(self.owner.visual, 'body_1', body_1)
            # collision
            self.set_collision_properties(self.owner.collision, 'body_1', body_1)
            # gazebo
            self.set_gazebo_properties(self.owner.gazebo.body_1, body_1)
            # flange_size
            #HACK: We assume the flange_size is the same for all the connectors and load it from the last connector (which we assume to be the connector for the arm)
            self.owner.flange_size = body_1['connectors'][-1]['size']
        elif self.owner.type in {ModuleType.DAGANA}:
            dict_joint = d['joints'][0]
            # joint data
            self.owner.actuator_data.type = dict_joint['type']
            self.owner.actuator_data.upper_limit = dict_joint['limits']['positionUpper']
            self.owner.actuator_data.lower_limit = dict_joint['limits']['positionLower']
            self.owner.actuator_data.velocity = dict_joint['limits']['velocity']
            self.owner.actuator_data.effort = dict_joint['limits']['peak_torque']
            self.owner.actuator_data.gear_ratio = dict_joint['gear_ratio']
            self.owner.actuator_data.zero_offset = 0.0
            # CentAcESC
            self.owner.CentAcESC = Module.Attribute(dict_joint['control_parameters']['xbot'])
            # xbot_gz
            self.owner.xbot_gz = Module.Attribute(dict_joint['control_parameters']['xbot_gz'])
            self.owner.joint_gripper_adapter = Module.Attribute(dict_joint['control_parameters']['joint_gripper_adapter'])
            #HACK: We hard-code the value for the flange_size of the dagana
            self.owner.flange_size = 'big'

    @staticmethod
    def set_dynamic_properties(body, dict_body):
        """Set the dynamic properties of a body from a dictionary"""
        body.mass = dict_body['mass']
        body.inertia_tensor.I_xx = dict_body['inertia'][0][0]
        body.inertia_tensor.I_yy = dict_body['inertia'][1][1]
        body.inertia_tensor.I_zz = dict_body['inertia'][2][2]
        body.inertia_tensor.I_xy = dict_body['inertia'][0][1]
        body.inertia_tensor.I_xz = dict_body['inertia'][0][2]
        body.inertia_tensor.I_yz = dict_body['inertia'][1][2]
        body.CoM.x = dict_body['r_com'][0]
        body.CoM.y = dict_body['r_com'][1]
        body.CoM.z = dict_body['r_com'][2]

    @staticmethod
    def set_visual_properties(visual_obj, body_name, dict_body):
        """Set the visual properties of a body from a dictionary"""
        try:
            visual_properties = []
            for visual in dict_body['visual']:
                attr = Module.Attribute(visual)
                if visual:
                    # NOTE: pose of visual properties should be expressed in urdf format at the moment
                    x, y, z, roll, pitch, yaw = get_xyzrpy(tf.transformations.numpy.array(visual['pose']))
                    attr.pose = Module.Attribute({'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch, 'yaw': yaw})
                visual_properties.append(attr)
            visual_attr = Module.Attribute({body_name: visual_properties})
            update_nested_dict(visual_obj.__dict__, visual_attr.__dict__)
        except KeyError:
            pass

    @staticmethod
    def set_collision_properties(collision_obj, body_name, dict_body):
        """Set the collision properties of a body from a dictionary"""
        try:
            collision_properties = []
            for collision in dict_body['collision']:
                attr = Module.Attribute(collision) 
                if collision:
                    # NOTE: pose of collision properties should be expressed in urdf format at the moment
                    x, y, z, roll, pitch, yaw = get_xyzrpy(tf.transformations.numpy.array(collision['pose']))
                    attr.pose = Module.Attribute({'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch, 'yaw': yaw})
                collision_properties.append(attr)
            collision_attr = Module.Attribute({body_name: collision_properties})
            update_nested_dict(collision_obj.__dict__, collision_attr.__dict__)
        except KeyError:
            pass

    @staticmethod
    def set_gazebo_properties(body, dict_body):
        """Set the gazebo properties of a body from a dictionary"""
        try:
            if len(dict_body['gazebo']) != 1:
                raise ValueError('A body must have exactly one gazebo property')
            gazebo = dict_body['gazebo'][0]
            attr = Module.Attribute(gazebo)  
            update_nested_dict(body.__dict__, attr.__dict__)
        except KeyError:
            pass


class YAMLInterpreter(object):
    """Class used to interpret the YAML file describing a module"""
    def __init__(self, owner, d, yaml_file=None):
        self.owner = owner
        self.parse_dict(d)
        self.owner.set_flange_size()
        # set filename
        self.owner.filename = yaml_file

    def parse_dict(self, d):
        """Parse the dictionary describing the module"""
        attr = Module.Attribute(d)  
        update_nested_dict(self.owner.__dict__, attr.__dict__)
        self.owner.type = ModuleType(self.owner.type)
        self.owner.kinematics_convention = KinematicsConvention(self.owner.kinematics_convention)

#
class Module(object):
    """
    Class describing all the properties of a module
    - Kinematics and dynamics paramters are loaded from a YAML file when instantiated
    - Methods are provided to compute frame transformations and store them as class attributes
    """
    def __init__(self, d, filename=None, format=ModuleDescriptionFormat.YAML, template_d={}):   
        """ Initialize a Module object from a dictionary either in YAML or JSON format

        Parameters:
        d (dict): dictionary containing all the basic attributes of the module.
        filename (str): path to the YAML or JSON file containing the module description
        format (ModuleDescriptionFormat): format of the file containing the module description

        """
        template_obj = Module.Attribute(template_d)
        update_nested_dict(self.__dict__, template_obj.__dict__)
        #
        interpreter_map = {ModuleDescriptionFormat.YAML: YAMLInterpreter, ModuleDescriptionFormat.JSON: JSONInterpreter}
        self.interpreter = interpreter_map[format](self, d, filename)

    class Attribute(object):
        """ Class to store the attributes of a module """
        def __init__(self, d):      
            """ Convert nested Python dictionary (obtained from the YAML file) to object"""
            for a, b in d.items():
                if isinstance(b, (list, tuple)):
                    setattr(self, a, [Module.Attribute(x) if isinstance(x, dict) else x for x in b])
                else:
                    setattr(self, a, Module.Attribute(b) if isinstance(b, dict) else b)

    # 
    def set_flange_size(self):
        """Set the flange_size of the module"""
        switcher = {
                'small': '1',
                'medium': '2',
                'big': '3',
                'large': '4',
                'extra-large': '5',
            }
        if self.type == "size_adapter":
            #print(self.size_in)
            setattr(self, 'size_in', switcher.get(self.size_in, "Invalid flange_size"))
            #print(self.size_out)
            setattr(self, 'size_out', switcher.get(self.size_out, "Invalid flange_size"))
        else:
            if hasattr(self, 'flange_size'):
                setattr(self, 'flange_size', switcher.get(self.flange_size, "Invalid flange_size"))
            else:
                pass
    #
    # noinspection PyPep8Naming
    def get_proximal_distal_matrices(self, reverse):
        """Computes the homogeneous transformation matrices for the distal and proximal part of the joint"""
        origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)

        proximal = self.kinematics.joint.proximal
        distal = self.kinematics.joint.distal

        if self.kinematics_convention is KinematicsConvention.URDF:
            T = tf.transformations.translation_matrix((proximal.x, proximal.y, proximal.z))
            R = tf.transformations.euler_matrix(proximal.roll, proximal.pitch, proximal.yaw, 'sxyz')
            P = tf.transformations.concatenate_matrices(T, R)

            T = tf.transformations.translation_matrix((distal.x, distal.y, distal.z))
            R = tf.transformations.euler_matrix(distal.roll, distal.pitch, distal.yaw, 'sxyz')
            D = tf.transformations.concatenate_matrices(T, R)

            if reverse:
                P = tf.transformations.inverse_matrix(D)
                D = tf.transformations.inverse_matrix(P)

        elif self.kinematics_convention is KinematicsConvention.DH_EXT:
            H1 = tf.transformations.rotation_matrix(proximal.delta_pl, zaxis)
            H2 = tf.transformations.translation_matrix((0, 0, proximal.p_pl))
            H3 = tf.transformations.translation_matrix((proximal.a_pl, 0, 0))
            H4 = tf.transformations.rotation_matrix(proximal.alpha_pl, xaxis)
            H5 = tf.transformations.translation_matrix((0, 0, proximal.n_pl))
            P = tf.transformations.concatenate_matrices(H1, H2, H3, H4, H5)

            H1 = tf.transformations.translation_matrix((0, 0, distal.p_dl))
            H2 = tf.transformations.translation_matrix((distal.a_dl, 0, 0))
            H3 = tf.transformations.rotation_matrix(distal.alpha_dl, xaxis)
            H4 = tf.transformations.translation_matrix((0, 0, distal.n_dl))
            H5 = tf.transformations.rotation_matrix(distal.delta_dl, zaxis)
            D = tf.transformations.concatenate_matrices(H1, H2, H3, H4, H5)

            if reverse:
                P = tf.transformations.inverse_matrix(D)
                D = tf.transformations.inverse_matrix(P)

        elif self.kinematics_convention is KinematicsConvention.AFFINE:
            P = tf.transformations.numpy.array(proximal.pose)
            D = tf.transformations.numpy.array(distal.pose)
            
            if reverse:
                P = tf.transformations.inverse_matrix(D)
                D = tf.transformations.inverse_matrix(P)

        else:
            raise ValueError("Unknown kinematic convention")
            
        # Add the transformation matrix for the Proximal part as attribute of the class
        setattr(self, 'Proximal_tf', P)
        # Add the transformation matrix for the Distal part as attribute of the class
        setattr(self, 'Distal_tf', D)
    
    # 
    # noinspection PyPep8Naming
    def get_homogeneous_matrix(self, reverse):
        """Computes the homogeneous transformation matrix for the link"""
        origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)

        link = self.kinematics.link

        if self.kinematics_convention is KinematicsConvention.URDF:
            T = tf.transformations.translation_matrix((link.x, link.y, link.z))
            R = tf.transformations.euler_matrix(link.roll, link.pitch, link.yaw, 'sxyz')    
            H = tf.transformations.concatenate_matrices(T, R)
            if reverse:
                H = tf.transformations.inverse_matrix(H)

        elif self.kinematics_convention is KinematicsConvention.DH_EXT:
            H1 = tf.transformations.rotation_matrix(link.delta_l_in, zaxis)
            H2 = tf.transformations.translation_matrix((0, 0, link.p_l))
            H3 = tf.transformations.translation_matrix((link.a_l, 0, 0))
            H4 = tf.transformations.rotation_matrix(link.alpha_l, xaxis)
            H5 = tf.transformations.translation_matrix((0, 0, link.n_l))
            H6 = tf.transformations.rotation_matrix(link.delta_l_out, zaxis)
            H = tf.transformations.concatenate_matrices(H1, H2, H3, H4, H5, H6)
            if reverse:
                # TO BE CHECKED!!!
                H1 = tf.transformations.rotation_matrix(-link.delta_l_out, zaxis)
                H2 = tf.transformations.translation_matrix((0, 0, -link.n_l))
                H3 = tf.transformations.rotation_matrix(-link.alpha_l, xaxis)
                H4 = tf.transformations.translation_matrix((-link.a_l, 0, 0))
                H5 = tf.transformations.translation_matrix((0, 0, -link.p_l))
                H6 = tf.transformations.rotation_matrix(-link.delta_l_in, zaxis)

                # H = tf.transformations.inverse_matrix(H)

        elif self.kinematics_convention is KinematicsConvention.AFFINE:
            H = tf.transformations.numpy.array(link.pose)
            if reverse:
                H = tf.transformations.inverse_matrix(H)

        else:
            raise ValueError("Unknown kinematic convention")

        # Add the transformation matrix for the Proximal part as attribute of the class
        setattr(self, 'Homogeneous_tf', H)

        #TODO: To be implemented
        if not reverse:
            pass
        else:
            # TBD!!!
            pass

    def get_hub_connections_tf(self, reverse):
        """Computes the homogeneous transformation matrices for the 4 cube connections"""
        origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
        max_num_con = 10

        for i in range(1, max_num_con):
            if hasattr(self.kinematics, "connector_{}".format(i)):
                con = getattr(self.kinematics, "connector_{}".format(i))
                if self.kinematics_convention is KinematicsConvention.URDF:
                    trasl = tf.transformations.translation_matrix((con.x, con.y, con.z))
                    rot = tf.transformations.euler_matrix(con.roll, con.pitch, con.yaw, 'sxyz')
                    tf_con = tf.transformations.concatenate_matrices(trasl, rot)

                elif self.kinematics_convention is KinematicsConvention.DH_EXT:
                    H1 = tf.transformations.rotation_matrix(con.delta_l_in, zaxis)
                    H2 = tf.transformations.translation_matrix((0, 0, con.p_l))
                    H3 = tf.transformations.translation_matrix((con.a_l, 0, 0))
                    H4 = tf.transformations.rotation_matrix(con.alpha_l, xaxis)
                    H5 = tf.transformations.translation_matrix((0, 0, con.n_l))
                    H6 = tf.transformations.rotation_matrix(con.delta_l_out, zaxis)
                    
                    tf_con = tf.transformations.concatenate_matrices(H1, H2, H3, H4, H5, H6)
                    if reverse:
                        # TO BE CHECKED!!!
                        H1 = tf.transformations.rotation_matrix(-con.delta_l_out, zaxis)
                        H2 = tf.transformations.translation_matrix((0, 0, -con.n_l))
                        H3 = tf.transformations.rotation_matrix(-con.alpha_l, xaxis)
                        H4 = tf.transformations.translation_matrix((-con.a_l, 0, 0))
                        H5 = tf.transformations.translation_matrix((0, 0, -con.p_l))
                        H6 = tf.transformations.rotation_matrix(-con.delta_l_in, zaxis)

                        tf_con = tf.transformations.concatenate_matrices(H1, H2, H3, H4, H5, H6)
                        # tf_con = tf.transformations.inverse_matrix(tf_con)
                
                elif self.kinematics_convention is KinematicsConvention.AFFINE:
                    tf_con = tf.transformations.numpy.array(con.pose)
                    if reverse:
                        tf_con = tf.transformations.inverse_matrix(tf_con)

                else:
                    raise ValueError("Unknown kinematic convention")

                # Add the transformation matrix for the Proximal part as attribute of the class
                setattr(self, 'Con_{}_tf'.format(i), tf_con)

        # if not reverse:
        #     pass
        # else:
        #     # TBD!!!
        #     pass

    # 
    def get_transform(self, reverse):
        """Computes the correct transformation depending on the module type"""
        x = self.type
        #print('module_type', x)
        switcher = {
            'joint': self.get_proximal_distal_matrices,
            'wheel': self.get_proximal_distal_matrices,
            'link': self.get_homogeneous_matrix,
            'size_adapter': self.get_homogeneous_matrix,
            'tool_exchanger': self.get_homogeneous_matrix,
            'end_effector': self.get_homogeneous_matrix,
            'drill': self.get_homogeneous_matrix,
            'dagana': lambda reverse: None,
            'gripper': self.get_homogeneous_matrix,
            'cube': self.get_hub_connections_tf, #  self.get_cube_connections_tf,
            'mobile_base': self.get_hub_connections_tf, #  self.get_mobile_base_connections_tf,
            'base_link': tf.transformations.identity_matrix()
        }
        return switcher.get(x, 'Invalid type')(reverse)

# 
class ModuleNode(Module, anytree.NodeMixin):
    """Class that extends the Module class to be a tree node (using anytree NodeMixin)"""
    def __init__(self, dictionary, filename=None, format=ModuleDescriptionFormat.YAML, parent=None, template_dictionary={}):
        """When instantiated the __init__ from the inherited class is called. Name and parent attributes are updated"""
        # when calling a method of a class that has been extended super is needed
        super(ModuleNode, self).__init__(dictionary, filename, format=format, template_d=template_dictionary)
        self.name = filename
        # self.filename = filename
        self.parent = parent


# noinspection PyPep8Naming
def get_rototranslation(distal_previous, proximal):
    """Computes the rototranslation from the frame centered at the previous joint
    to the frame at the center of this joint module.

    Parameters:
    distal_previous: transform relative to the distal part of the previous module
    proximal: transform relative to the proximal part of the current module

    """
    F = tf.transformations.concatenate_matrices(distal_previous, proximal)

    return F


def get_xyzrpy(transform):
    #print(transform)
    t = transform[:3, -1].transpose()
    r = transform[:3, :3]
    roll, pitch, yaw = tf.transformations.euler_from_matrix(r, 'sxyz')

    x = str(t[0])
    y = str(t[1])
    z = str(t[2])
    roll = str(roll)
    pitch = str(pitch)
    yaw = str(yaw)

    return x, y, z, roll, pitch, yaw


def inverse(transform_matrix):
    inv = tf.transformations.inverse_matrix(transform_matrix)

    return inv

# 
def module_from_yaml(filename, father=None, yaml_template=None, reverse=False):
    """Function parsing YAML file describing a generic module and returning an instance of a Module class"""
    if yaml_template:
        with open(yaml_template, 'r') as stream:
            try:
                template_data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
    else:
        template_data = {}
    with open(filename, 'r') as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    # Create an instance of a ModuleNode class from the dictionary obtained from YAML
    result = ModuleNode(data, filename, format=ModuleDescriptionFormat.YAML, parent=father, template_dictionary=template_data)
    # result.set_type(filename)
    result.get_transform(reverse)
    return result 


# 
def module_from_json(filename, father=None, yaml_template=None, reverse=False):
    """Function parsing JSON file describing a generic module and returning an instance of a Module class"""
    if yaml_template:
        with open(yaml_template, 'r') as stream:
            try:
                template_data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
    else:
        template_data = {}
    with open(filename, 'r') as stream:
        try:
            data = json.load(stream)
        except json.JSONDecodeError as exc:
            print(exc)
    # Create an instance of a ModuleNode class from the dictionary obtained from JSON
    result = ModuleNode(data, filename, format=ModuleDescriptionFormat.JSON, parent=father, template_dictionary=template_data)
    result.get_transform(reverse)
    return result 


def main():
    # module = module_from_yaml(sys.argv[1], None, False)
    module = module_from_json(sys.argv[1], None, False)
    print(module.__dict__)


if __name__ == '__main__':
    main()
