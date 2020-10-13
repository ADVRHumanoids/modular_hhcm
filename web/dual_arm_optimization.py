from itertools import product, ifilterfalse
import numpy

from generator import generate_robot_set
from ik import cartesio_ik

import URDF_writer

if __name__ == "__main__":
    # Instance of UrdfWriter class
    urdf_writer = URDF_writer.UrdfWriter(speedup=True)

    required_dofs = 5
    robot_set = generate_robot_set(required_dofs)

    # Create grid
    x = list(numpy.linspace(0.0, 0.5, 3))
    y = list(numpy.linspace(-0.2, 0.2, 3))
    coordinates = list(product(x, y))
    print coordinates
    coordinates.remove((0.0, 0.0))
    print len(coordinates)

    for robot in robot_set:
        print robot[0]

        angle_offset = 3.14

        # Create joint map to store homing values
        homing_joint_map = {}
        # homing_value = float(builder_joint_map[joint_module.name]['angle'])


        urdf_writer.__init__()

        # Add a table
        urdf_writer.add_table()

        # Add 1st chain base
        urdf_writer.add_socket(0.0, 0.0, 0.0, 0.0)

        # Add 1st chain
        for module in robot[0]:
            if module:
                data = urdf_writer.add_module('module_joint_double_elbow_AVOCADO_B.yaml', 0, False)
            else:
                data = urdf_writer.add_module('module_joint_yaw_AVOCADO_B.yaml', 0, False)
            # set an homing value for the joint
            homing_joint_map[data['lastModule_name']] = {'angle': 0.0}

        # urdf_writer.select_module_from_name(parent)
        urdf_writer.access_module("table")

        # Add 2nd chain base. We place it at (0.0, 0.0) for now
        urdf_writer.add_socket(0.0, 0.0, 0.0, 0.0)

        for module in robot[1]:
            if module:
                data = urdf_writer.add_module('module_joint_double_elbow_AVOCADO_B.yaml', 0, False)
            else:
                data = urdf_writer.add_module('module_joint_yaw_AVOCADO_B.yaml', 0, False)
            homing_joint_map[data['lastModule_name']] = {'angle': 0.0}

        # Modify position and orientation of the 2nd base
        for xy in coordinates:
            for angle_offset in [3.14]:  # [0.0, 1.57, 3.14, -1.57]
                urdf_writer.move_socket("L_0_B", xy[0], xy[1], 0.0, angle_offset)
                print "Moving socket!!!!!!!!!!!!", xy[0], xy[1],
                string = urdf_writer.process_urdf()

                data = urdf_writer.write_urdf()
                srdf = urdf_writer.write_srdf(homing_joint_map)
                joint_map = urdf_writer.write_joint_map()
                lowlevel_config = urdf_writer.write_lowlevel_config()
                probdesc = urdf_writer.write_problem_description_dual_arm()
                # cartesio_stack = urdf_writer.write_cartesio_stack()

                data = urdf_writer.deploy_robot("ModularBot_opt")


