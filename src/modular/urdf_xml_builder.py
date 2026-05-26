"""
Low-level URDF XML building utilities.

``URDFXmlBuilder`` provides the primitives that translate the abstract module
representation (``ModuleNode`` objects) into concrete ``xml.etree.ElementTree``
elements appended to the URDF tree.

All state it needs is accessed via the ``writer`` back-reference so that the
single source of truth (the XML tree stored in ``UrdfWriter.root``) is never
duplicated.
"""
from __future__ import print_function
import math
import xml.etree.ElementTree as ET

from modular import ModuleNode

# tf_transformations is imported transitively via ModuleNode (single source of truth)
tf_transformations = ModuleNode.tf_transformations
from modular.enums import ModuleClass
from modular.yaml_utils import NS_XACRO, ns


class URDFXmlBuilder:
    """Constructs XML elements and appends them to the shared URDF tree.

    Parameters
    ----------
    writer : UrdfWriter
        Back-reference to the owning writer.  Used to access ``writer.root``,
        ``writer.control_plugin``, ``writer.logger`` and the
        ``writer.collision_elements`` list.
    """

    def __init__(self, writer):
        self._writer = writer

    # ------------------------------------------------------------------
    # Convenience accessors (avoid repeating self._writer everywhere)
    # ------------------------------------------------------------------

    @property
    def root(self):
        return self._writer.root

    @property
    def control_plugin(self):
        return self._writer.control_plugin

    # ------------------------------------------------------------------
    # Primitive XML helpers (origin, geometry, material, inertial)
    # ------------------------------------------------------------------

    def add_origin(self, parent_el, pose):
        ET.SubElement(parent_el, "origin",
                      xyz=str(pose.x) + " " + str(pose.y) + " " + str(pose.z),
                      rpy=str(pose.roll) + " " + str(pose.pitch) + " " + str(pose.yaw))

    def add_geometry(self, parent_el, geometry):
        geometry_el = ET.SubElement(parent_el, "geometry")
        if geometry.type == "mesh":
            ET.SubElement(geometry_el, "mesh",
                          filename=geometry.parameters.file,
                          scale=' '.join(str(x) for x in geometry.parameters.scale))
        elif geometry.type == "box":
            ET.SubElement(geometry_el, "box",
                          size=' '.join(str(x) for x in geometry.parameters.size))
        elif geometry.type == "cylinder":
            ET.SubElement(geometry_el, "cylinder",
                          radius=str(geometry.parameters.radius),
                          length=str(geometry.parameters.length))
        elif geometry.type == "sphere":
            ET.SubElement(geometry_el, "sphere",
                          radius=str(geometry.parameters.radius))

    def add_material(self, parent_el, color):
        material_el = ET.SubElement(parent_el, "material",
                                    name=color.material_name)
        if hasattr(color, 'rgba'):
            ET.SubElement(material_el, "color",
                          rgba=' '.join(str(x) for x in color.rgba))
        if hasattr(color, 'texture'):
            ET.SubElement(material_el, "texture",
                          filename=color.texture.filename)

    def add_inertial(self, parent_el, dynamics, gear_ratio=1.0):
        inertial_el = ET.SubElement(parent_el, "inertial")
        #  We interpret the mass as a flag to enable/disable the inertial properties
        if dynamics.mass:
            ET.SubElement(inertial_el, "origin",
                          xyz=str(dynamics.CoM.x) + " " + str(dynamics.CoM.y) + " " + str(dynamics.CoM.z),
                          rpy=str(0) + " " + str(0) + " " + str(0))
            ET.SubElement(inertial_el, "mass",
                          value=str(dynamics.mass))
            ET.SubElement(inertial_el, "inertia",
                          ixx=str(dynamics.inertia_tensor.I_xx),
                          ixy=str(dynamics.inertia_tensor.I_xy),
                          ixz=str(dynamics.inertia_tensor.I_xz),
                          iyy=str(dynamics.inertia_tensor.I_yy),
                          iyz=str(dynamics.inertia_tensor.I_yz),
                          izz=str(gear_ratio * gear_ratio * dynamics.inertia_tensor.I_zz))
        # If the mass is 0.0 we set the inertial properties to a default value to avoid issues with
        # dynamics libraries that consume the URDF
        else:
            ET.SubElement(inertial_el, "mass",
                          value=str(1e-04))
            ET.SubElement(inertial_el, "inertia",
                          ixx=str(1e-09),
                          ixy=str(0),
                          ixz=str(0),
                          iyy=str(1e-09),
                          iyz=str(0),
                          izz=str(1e-09))

    # ------------------------------------------------------------------
    # Composite XML element builders
    # ------------------------------------------------------------------

    def add_link_element(self, link_name, module_obj, body_name, root=None, is_geared=False):
        """Add a URDF ``<link>`` element with visual, collision and inertial children.

        Parameters
        ----------
        link_name : str
        module_obj : ModuleNode.ModuleNode
        body_name : str
            Key used to look up the visual/collision/dynamics sub-objects on *module_obj*.
        root : ET.Element, optional
            Parent XML element.  Defaults to the URDF tree root.
        is_geared : bool
            If ``True`` the rotor inertia is scaled by the gear ratio squared.
        """
        if root is None:
            root = self.root
        link_el = ET.SubElement(root, 'link', name=link_name)
        # Add the link to the list of urdf elements of the module
        module_obj.xml_tree_elements.append(link_name)
        module_obj.mesh_names.append(link_name)

        visual_bodies = getattr(module_obj.visual, body_name, None)
        collision_bodies = getattr(module_obj.collision, body_name, None)
        dynamics_body = getattr(module_obj.dynamics, body_name, None)

        for body in visual_bodies or []:
            visual_el = ET.SubElement(link_el, 'visual')
            self.add_origin(visual_el, body.pose)
            self.add_geometry(visual_el, body)
            if hasattr(body.parameters, 'color'):
                self.add_material(visual_el, body.parameters.color)

        for body in collision_bodies or []:
            collision_el = ET.SubElement(link_el, 'collision')
            self.add_origin(collision_el, body.pose)
            self.add_geometry(collision_el, body)

        if dynamics_body:
            if is_geared:
                self.add_inertial(link_el, dynamics_body, gear_ratio=module_obj.actuator_data.gear_ratio)
            else:
                self.add_inertial(link_el, dynamics_body)

        return link_el

    def add_joint_element(self, joint_name, module_obj, parent_name, child_name, mimic_joint=None):
        """Add a URDF ``<joint>`` element including limits, axis, and control-plugin snippet."""
        joint_el = ET.SubElement(self.root, 'joint',
                                 name=joint_name,
                                 type=module_obj.actuator_data.type)
        # Add the joint to the list of urdf elements of the module
        module_obj.xml_tree_elements.append(joint_name)

        ET.SubElement(joint_el, "parent", link=parent_name)
        ET.SubElement(joint_el, "child", link=child_name)

        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(module_obj.Proximal_tf)
        joint_pose = ModuleNode.Module.Attribute({'x': x, 'y': y, 'z': z,
                                                   'roll': roll, 'pitch': pitch, 'yaw': yaw})
        self.add_origin(joint_el, joint_pose)

        actuator_data = module_obj.actuator_data
        if actuator_data.type != 'fixed':
            ET.SubElement(joint_el, "limit",
                          effort=str(actuator_data.effort),
                          velocity=str(actuator_data.velocity),
                          lower=str(actuator_data.lower_limit),
                          upper=str(actuator_data.upper_limit))

            if hasattr(actuator_data, 'axis'):
                axis = actuator_data.axis
                ET.SubElement(joint_el, "axis",
                              xyz=str(axis[0]) + " " + str(axis[1]) + " " + str(axis[2]))
            else:
                ET.SubElement(joint_el, "axis", xyz="0 0 1")

            if mimic_joint is not None:
                ET.SubElement(joint_el, "mimic",
                              joint=mimic_joint,
                              multiplier="1",
                              offset="0")

            self.control_plugin.add_joint(
                joint_name,
                control_params=module_obj.xbot_gz if hasattr(module_obj, 'xbot_gz') else None
            )

        return joint_el

    def add_rotor_element(self, new_Joint):
        """Add the rotor link/joint pair that represents reflected rotor inertia.

        The ``REFLECT_ROTOR_INERTIA`` xacro mapping (defaulting to *false*)
        controls whether the inertia is reflected to the rotor link or lumped
        into the stator.
        """
        setattr(new_Joint, 'fixed_joint_rotor_name', "fixed_" + new_Joint.distal_link_name + '_rotor')
        setattr(new_Joint, 'rotor_name', new_Joint.distal_link_name + '_rotor')

        # --- REFLECT_ROTOR_INERTIA = True branch ---
        reflect_if_joint_el = ET.SubElement(self.root,
                                             'xacro:xacro_if_guard',
                                             value="${REFLECT_ROTOR_INERTIA}",
                                             name=new_Joint.fixed_joint_rotor_name + '_if')
        new_Joint.xml_tree_elements.append(new_Joint.fixed_joint_rotor_name + '_if')

        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(tf_transformations.identity_matrix())
        ET.SubElement(reflect_if_joint_el,
                      "xacro:add_fixed_joint",
                      type="fixed_joint",
                      name=new_Joint.fixed_joint_rotor_name,
                      father=new_Joint.distal_link_name,
                      child=new_Joint.rotor_name,
                      x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)

        reflect_if_link_el = ET.SubElement(self.root,
                                            'xacro:xacro_if_guard',
                                            value="${REFLECT_ROTOR_INERTIA}",
                                            name=new_Joint.rotor_name + '_if')
        new_Joint.xml_tree_elements.append(new_Joint.rotor_name + '_if')
        self.add_link_element(new_Joint.rotor_name, new_Joint, 'body_2_fast',
                              root=reflect_if_link_el, is_geared=True)

        # --- REFLECT_ROTOR_INERTIA = False branch ---
        reflect_if_not_joint_el = ET.SubElement(self.root,
                                                 'xacro:xacro_if_guard',
                                                 value="${not REFLECT_ROTOR_INERTIA}",
                                                 name=new_Joint.fixed_joint_rotor_name + '_if_not')
        new_Joint.xml_tree_elements.append(new_Joint.fixed_joint_rotor_name + '_if_not')

        x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(new_Joint.Proximal_tf)
        ET.SubElement(reflect_if_not_joint_el,
                      "xacro:add_fixed_joint",
                      type="fixed_joint",
                      name=new_Joint.fixed_joint_rotor_name,
                      father=new_Joint.stator_name,
                      child=new_Joint.rotor_name,
                      x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)

        reflect_if_not_link_el = ET.SubElement(self.root,
                                                'xacro:xacro_if_guard',
                                                value="${not REFLECT_ROTOR_INERTIA}",
                                                name=new_Joint.rotor_name + '_if_not')
        new_Joint.xml_tree_elements.append(new_Joint.rotor_name + '_if_not')
        self.add_link_element(new_Joint.rotor_name, new_Joint, 'body_2_fast',
                              root=reflect_if_not_link_el, is_geared=False)

    def add_gazebo_element(self, new_module_obj, gazebo_obj, new_module_name):
        """Wrap a ``<gazebo>`` block in an ``xacro:xacro_if_guard`` and append it."""
        if gazebo_obj is not None:
            gazebo_el_name = 'gazebo_' + new_module_name
            gazebo_if_el = ET.SubElement(self.root,
                                         'xacro:xacro_if_guard',
                                         value="${GAZEBO_URDF}",
                                         name=gazebo_el_name)
            new_module_obj.xml_tree_elements.append(gazebo_el_name)

            gazebo_el = ET.SubElement(gazebo_if_el, 'gazebo', reference=new_module_name)
            self.add_gazebo_element_children(gazebo_obj, gazebo_el)

    def add_gazebo_element_children(self, gazebo_child_obj, gazebo_element):
        """Recursively populate the ``<gazebo>`` element from a module attribute object."""
        for key, value in vars(gazebo_child_obj).items():
            gazebo_child_el = ET.SubElement(gazebo_element, key)
            if isinstance(value, ModuleNode.Module.Attribute):
                self.add_gazebo_element_children(value, gazebo_child_el)
            else:
                gazebo_child_el.text = str(value)

    # ------------------------------------------------------------------
    # Connector management (XML-level)
    # ------------------------------------------------------------------

    def add_connectors(self, modulenode):
        """Append ``xacro:add_connector`` elements for each connector port on *modulenode*."""
        max_num_con = 20
        for i in range(1, max_num_con):
            if hasattr(modulenode, 'Con_{}_tf'.format(i)):
                con_tf = getattr(modulenode, 'Con_{}_tf'.format(i))
                x, y, z, roll, pitch, yaw = ModuleNode.get_xyzrpy(con_tf)
                con_name = modulenode.name + '_con{}'.format(i)
                ET.SubElement(self.root,
                              "xacro:add_connector",
                              name=con_name,
                              type='connectors',
                              parent_name=modulenode.name,
                              x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)
                modulenode.xml_tree_elements.append(con_name)
                modulenode.mesh_names.append(con_name)
                modulenode.connectors.append(con_name)

    def remove_all_connectors(self):
        """Remove every ``type='connectors'`` element from the URDF tree."""
        # Refresh the generator before iterating
        self._writer.update_generators()
        for node in self._writer.urdf_nodes_generator:
            try:
                node_type = node.attrib['type']
                if node_type == 'connectors':
                    self._writer.print('removing node:', node.attrib)
                    self.root.remove(node)
            except KeyError:
                continue
