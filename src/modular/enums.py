from enum import Enum

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
    SOCKET = 'socket'

class ModuleClass(set, Enum):
    """Class of module"""
    LINKS = {ModuleType.LINK, ModuleType.SIZE_ADAPTER, ModuleType.BASE_LINK, ModuleType.SOCKET}
    JOINTS = {ModuleType.JOINT, ModuleType.WHEEL}
    HUBS = {ModuleType.CUBE, ModuleType.MOBILE_BASE}
    END_EFFECTORS = {ModuleType.GRIPPER, ModuleType.TOOL_EXCHANGER, ModuleType.END_EFFECTOR, ModuleType.DRILL, ModuleType.DAGANA, ModuleType.SIMPLE_EE}
    PASSIVE_MODULES = {ModuleType.SIZE_ADAPTER, ModuleType.BASE_LINK, ModuleType.END_EFFECTOR, ModuleType.SIMPLE_EE, ModuleType.SOCKET}
    # 
    @classmethod
    def link_modules(cls):
        return cls.LINKS
    @classmethod
    def joint_modules(cls):
        return cls.JOINTS
    @classmethod
    def hub_modules(cls):
        return cls.HUBS
    @classmethod
    def end_effector_modules(cls):
        return cls.END_EFFECTORS
    @classmethod
    def all_modules(cls):
        return cls.LINKS | cls.JOINTS | cls.HUBS | cls.END_EFFECTORS
    @classmethod
    def passive_modules(cls):
        return cls.PASSIVE_MODULES
    @classmethod
    def active_modules(cls):
        return cls.all_modules() - cls.passive_modules()
    @classmethod
    def nonactuated_modules(cls):
        return cls.LINKS | cls.HUBS | cls.PASSIVE_MODULES
    @classmethod
    def actuated_modules(cls):
        return cls.all_modules() - cls.nonactuated_modules()