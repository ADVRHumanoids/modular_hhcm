from modular.URDF_writer import *

import logging
FORMAT = '[%(levelname)s] [%(module)s]:  %(message)s'
logging.basicConfig(format=FORMAT)
applogger = logging.getLogger("sim_discovery")

urdfwriter_kwargs_dict={
    'verbose': True,
    'slave_desc_mode': 'use_pos',
    'logger': applogger,
}

urdf_writer_fromHW = UrdfWriter(**urdfwriter_kwargs_dict)

# old discovery with ids
# reply = "{201: {active_ports: 15, esc_type: 50, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 1, robot_id: 201, topology: 4}, -1: {active_ports: 15, esc_type: 256, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 2, robot_id: -1, topology: 4}, 56: {active_ports: 3, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 3, robot_id: 56, topology: 2}, 53: {active_ports: 3, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 4, robot_id: 53, topology: 2}, 55: {active_ports: 1, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 5, robot_id: 55, topology: 1}, 11: {active_ports: 3, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 6, robot_id: 11, topology: 2}, 12: {active_ports: 1, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 7, robot_id: 12, topology: 1}, 31: {active_ports: 3, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 8, robot_id: 31, topology: 2}, 32: {active_ports: 1, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 9, robot_id: 32, topology: 1}, 41: {active_ports: 3, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 10, robot_id: 41, topology: 2}, 42: {active_ports: 1, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 11, robot_id: 42, topology: 1}, 21: {active_ports: 3, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 12, robot_id: 21, topology: 2}, 22: {active_ports: 1, esc_type: 21, mod_id: 0, mod_rev: 0, mod_size: 0, mod_type: 0, position: 13, robot_id: 22, topology: 1}}"

reply = "{1: {active_ports: 15, esc_type: 50, mod_id: 3, mod_rev: 0, mod_size: 0, mod_type: 2, position: 1, robot_id: 201, topology: 4}, 2: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 2, robot_id: -1, topology: 4}, 3: {active_ports: 3, esc_type: 21, mod_id: 8, mod_rev: 0, mod_size: 5, mod_type: 1, position: 3, robot_id: 21, topology: 2}, 4: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 4, robot_id: 22, topology: 1}, 5: {active_ports: 3, esc_type: 21, mod_id: 7, mod_rev: 0, mod_size: 5, mod_type: 1, position: 5, robot_id: 11, topology: 2}, 6: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 6, robot_id: 12, topology: 1}, 7: {active_ports: 3, esc_type: 21, mod_id: 7, mod_rev: 0, mod_size: 5, mod_type: 1, position: 7, robot_id: 31, topology: 2}, 8: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 8, robot_id: 32, topology: 1}, 9: {active_ports: 3, esc_type: 21, mod_id: 8, mod_rev: 0, mod_size: 5, mod_type: 1, position: 9, robot_id: 41, topology: 2}, 10: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 10, robot_id: 42, topology: 1}, 11: {active_ports: 3, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 1, position: 11, robot_id: 55, topology: 2}, 12: {active_ports: 3, esc_type: 21, mod_id: 6, mod_rev: 0, mod_size: 4, mod_type: 1, position: 12, robot_id: 53, topology: 2}, 13: {active_ports: 3, esc_type: 21, mod_id: 4, mod_rev: 0, mod_size: 4, mod_type: 1, position: 13, robot_id: 51, topology: 2}, 14: {active_ports: 3, esc_type: 1280, mod_id: 6, mod_rev: 0, mod_size: 4, mod_type: 4, position: 14, robot_id: -1, topology: 2}, 15: {active_ports: 3, esc_type: 21, mod_id: 6, mod_rev: 0, mod_size: 4, mod_type: 1, position: 15, robot_id: 52, topology: 2}, 16: {active_ports: 3, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 1, position: 16, robot_id: 56, topology: 2}, 17: {active_ports: 3, esc_type: 1280, mod_id: 6, mod_rev: 0, mod_size: 4, mod_type: 4, position: 17, robot_id: -1, topology: 2}, 18: {active_ports: 1, esc_type: 21, mod_id: 5, mod_rev: 0, mod_size: 4, mod_type: 1, position: 18, robot_id: 54, topology: 1}}"

# CoNCERT robot without 3 legs
# reply = "{1: {active_ports: 13, esc_type: 50, mod_id: 3, mod_rev: 0, mod_size: 0, mod_type: 2, position: 1, robot_id: 201, topology: 3}, 2: {active_ports: 3, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 2, robot_id: -1, topology: 2}, 3: {active_ports: 3, esc_type: 21, mod_id: 7, mod_rev: 0, mod_size: 5, mod_type: 1, position: 3, robot_id: 11, topology: 2}, 4: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 4, robot_id: 12, topology: 1}, 5: {active_ports: 3, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 1, position: 5, robot_id: 55, topology: 2}, 6: {active_ports: 3, esc_type: 21, mod_id: 6, mod_rev: 0, mod_size: 4, mod_type: 1, position: 6, robot_id: 53, topology: 2}, 7: {active_ports: 3, esc_type: 21, mod_id: 4, mod_rev: 0, mod_size: 4, mod_type: 1, position: 7, robot_id: 51, topology: 2}, 8: {active_ports: 3, esc_type: 1280, mod_id: 6, mod_rev: 0, mod_size: 4, mod_type: 4, position: 8, robot_id: -1, topology: 2}, 9: {active_ports: 3, esc_type: 21, mod_id: 6, mod_rev: 0, mod_size: 4, mod_type: 1, position: 9, robot_id: 52, topology: 2}, 10: {active_ports: 3, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 1, position: 10, robot_id: 56, topology: 2}, 11: {active_ports: 3, esc_type: 1280, mod_id: 6, mod_rev: 0, mod_size: 4, mod_type: 4, position: 11, robot_id: -1, topology: 2}, 12: {active_ports: 1, esc_type: 21, mod_id: 5, mod_rev: 0, mod_size: 4, mod_type: 1, position: 12, robot_id: 54, topology: 1}}"

# multi-hubs tests
# reply = "{1: {active_ports: 15, esc_type: 50, mod_id: 3, mod_rev: 0, mod_size: 0, mod_type: 2, position: 1, robot_id: 201, topology: 4}, 2: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 2, robot_id: -1, topology: 4}, 3: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 3, robot_id: -1, topology: 4}, 4: {active_ports: 3, esc_type: 21, mod_id: 8, mod_rev: 0, mod_size: 5, mod_type: 1, position: 4, robot_id: 21, topology: 2}, 5: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 5, robot_id: 22, topology: 1}, 6: {active_ports: 3, esc_type: 21, mod_id: 7, mod_rev: 0, mod_size: 5, mod_type: 1, position: 6, robot_id: 11, topology: 2}, 7: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 7, robot_id: 12, topology: 1}, 8: {active_ports: 3, esc_type: 21, mod_id: 7, mod_rev: 0, mod_size: 5, mod_type: 1, position: 8, robot_id: 31, topology: 2}, 9: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 9, robot_id: 32, topology: 1}, 10: {active_ports: 3, esc_type: 21, mod_id: 8, mod_rev: 0, mod_size: 5, mod_type: 1, position: 10, robot_id: 41, topology: 2}, 11: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 11, robot_id: 42, topology: 1}, 12: {active_ports: 1, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 1, position: 12, robot_id: 55, topology: 1}, 13: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 13, robot_id: -1, topology: 4}, 14: {active_ports: 1, esc_type: 21, mod_id: 6, mod_rev: 0, mod_size: 4, mod_type: 1, position: 14, robot_id: 53, topology: 1}, 15: {active_ports: 1, esc_type: 21, mod_id: 2, mod_rev: 0, mod_size: 2, mod_type: 1, position: 15, robot_id: 51, topology: 1}, 16: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 2, mod_type: 1, position: 16, robot_id: 61, topology: 1}, 17: {active_ports: 1, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 3, position: 17, robot_id: 71, topology: 1}}"

# reply = "{1: {active_ports: 15, esc_type: 50, mod_id: 3, mod_rev: 0, mod_size: 0, mod_type: 2, position: 1, robot_id: 201, topology: 4}, 2: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 2, robot_id: -1, topology: 4}, 3: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 3, robot_id: -1, topology: 4}, 4: {active_ports: 3, esc_type: 21, mod_id: 8, mod_rev: 0, mod_size: 5, mod_type: 1, position: 4, robot_id: 21, topology: 2}, 5: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 5, robot_id: 22, topology: 1}, 6: {active_ports: 3, esc_type: 21, mod_id: 7, mod_rev: 0, mod_size: 5, mod_type: 1, position: 6, robot_id: 11, topology: 2}, 7: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 7, robot_id: 12, topology: 1}, 8: {active_ports: 3, esc_type: 21, mod_id: 7, mod_rev: 0, mod_size: 5, mod_type: 1, position: 8, robot_id: 31, topology: 2}, 9: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 9, robot_id: 32, topology: 1}, 10: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 10, robot_id: -1, topology: 4}, 11: {active_ports: 3, esc_type: 21, mod_id: 8, mod_rev: 0, mod_size: 5, mod_type: 1, position: 11, robot_id: 41, topology: 2}, 12: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 12, robot_id: 42, topology: 1}, 13: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 13, robot_id: 42, topology: 1}, 14: {active_ports: 1, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 1, position: 14, robot_id: 55, topology: 1}, 15: {active_ports: 1, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 1, position: 15, robot_id: 55, topology: 1}, 16: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 16, robot_id: -1, topology: 4}, 17: {active_ports: 1, esc_type: 21, mod_id: 6, mod_rev: 0, mod_size: 4, mod_type: 1, position: 17, robot_id: 53, topology: 1}, 18: {active_ports: 1, esc_type: 21, mod_id: 2, mod_rev: 0, mod_size: 2, mod_type: 1, position: 18, robot_id: 51, topology: 1}, 19: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 2, mod_type: 1, position: 19, robot_id: 61, topology: 1}, 20: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 20, robot_id: -1, topology: 4}, 21: {active_ports: 1, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 3, position: 21, robot_id: 71, topology: 1}, 22: {active_ports: 1, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 1, position: 22, robot_id: 71, topology: 1}, 23: {active_ports: 1, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 1, position: 23, robot_id: 71, topology: 1}}"

# test scenario with hub tree depth > 2. not working yet!
# reply = "{1: {active_ports: 13, esc_type: 50, mod_id: 3, mod_rev: 0, mod_size: 0, mod_type: 2, position: 1, robot_id: 201, topology: 3}, 2: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 2, robot_id: -1, topology: 4}, 3: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 3, robot_id: -1, topology: 4}, 4: {active_ports: 3, esc_type: 21, mod_id: 8, mod_rev: 0, mod_size: 5, mod_type: 1, position: 4, robot_id: 21, topology: 2}, 5: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 5, robot_id: 22, topology: 1}, 6: {active_ports: 3, esc_type: 21, mod_id: 7, mod_rev: 0, mod_size: 5, mod_type: 1, position: 6, robot_id: 11, topology: 2}, 7: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 7, robot_id: 12, topology: 1}, 8: {active_ports: 3, esc_type: 21, mod_id: 7, mod_rev: 0, mod_size: 5, mod_type: 1, position: 8, robot_id: 31, topology: 2}, 9: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 9, robot_id: 32, topology: 1}, 10: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 10, robot_id: -1, topology: 4}, 11: {active_ports: 3, esc_type: 21, mod_id: 8, mod_rev: 0, mod_size: 5, mod_type: 1, position: 11, robot_id: 41, topology: 2}, 12: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 12, robot_id: 42, topology: 1}, 13: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 13, robot_id: -1, topology: 4}, 14: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 14, robot_id: 42, topology: 1}, 15: {active_ports: 1, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 1, position: 15, robot_id: 55, topology: 1}, 16: {active_ports: 1, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 1, position: 16, robot_id: 55, topology: 1}, 17: {active_ports: 1, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 1, position: 17, robot_id: 71, topology: 1}, 18: {active_ports: 1, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 1, position: 18, robot_id: 71, topology: 1}, 19: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 19, robot_id: -1, topology: 4}, 20: {active_ports: 1, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 3, position: 20, robot_id: 71, topology: 1}, 21: {active_ports: 1, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 1, position: 21, robot_id: 71, topology: 1}, 22: {active_ports: 1, esc_type: 21, mod_id: 3, mod_rev: 0, mod_size: 4, mod_type: 1, position: 22, robot_id: 71, topology: 1}}"

# mobile base only
# reply = "{1: {active_ports: 11, esc_type: 50, mod_id: 3, mod_rev: 0, mod_size: 0, mod_type: 2, position: 1, robot_id: 201, topology: 4}, 2: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2, position: 2, robot_id: -1, topology: 4}, 3: {active_ports: 3, esc_type: 21, mod_id: 8, mod_rev: 0, mod_size: 5, mod_type: 1, position: 3, robot_id: 21, topology: 2}, 4: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 4, robot_id: 22, topology: 1}, 5: {active_ports: 3, esc_type: 21, mod_id: 7, mod_rev: 0, mod_size: 5, mod_type: 1, position: 5, robot_id: 11, topology: 2}, 6: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 6, robot_id: 12, topology: 1}, 7: {active_ports: 3, esc_type: 21, mod_id: 7, mod_rev: 0, mod_size: 5, mod_type: 1, position: 7, robot_id: 31, topology: 2}, 8: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 8, robot_id: 32, topology: 1}, 9: {active_ports: 3, esc_type: 21, mod_id: 8, mod_rev: 0, mod_size: 5, mod_type: 1, position: 9, robot_id: 41, topology: 2}, 10: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5, position: 10, robot_id: 42, topology: 1}}"

data = urdf_writer_fromHW.read_from_json(reply)

write_file_to_stdout(urdf_writer_fromHW, None, robot_name='sim_discovery')