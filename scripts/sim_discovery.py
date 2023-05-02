from modular.URDF_writer import *

urdfwriter_kwargs_dict={
    'verbose': True
}

# 2nd instance of UrdfWriter class for the robot got from HW
urdf_writer_fromHW = UrdfWriter(**urdfwriter_kwargs_dict)

reply = "{201: {active_ports: 15, esc_type: 50, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 1, robot_id: 201, topology: 4}, -1: {active_ports: 15, esc_type: 256, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 2, robot_id: -1, topology: 4}, 56: {active_ports: 3, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 3, robot_id: 56, topology: 2}, 53: {active_ports: 3, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 4, robot_id: 53, topology: 2}, 55: {active_ports: 1, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 5, robot_id: 55, topology: 1}, 11: {active_ports: 3, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 6, robot_id: 11, topology: 2}, 12: {active_ports: 1, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 7, robot_id: 12, topology: 1}, 31: {active_ports: 3, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 8, robot_id: 31, topology: 2}, 32: {active_ports: 1, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 9, robot_id: 32, topology: 1}, 41: {active_ports: 3, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 10, robot_id: 41, topology: 2}, 42: {active_ports: 1, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 11, robot_id: 42, topology: 1}, 21: {active_ports: 3, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 12, robot_id: 21, topology: 2}, 22: {active_ports: 1, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 13, robot_id: 22, topology: 1}}"

data = urdf_writer_fromHW.read_from_json(reply)

write_file_to_stdout(urdf_writer_fromHW, None, robot_name='sim_discovery')