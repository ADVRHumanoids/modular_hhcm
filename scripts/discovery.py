from modular.URDF_writer import *

import logging
import modular
modular.setup_logging()
applogger = logging.getLogger("discovery")

urdfwriter_kwargs_dict={
    'verbose': True,
    'slave_desc_mode': 'use_pos',
    'logger': applogger,
}

urdf_writer_fromHW = UrdfWriter(**urdfwriter_kwargs_dict)

try:
    from xbot2_cli.ecat_context import Context as EcatContext, Arguments as EcatArguments
except ImportError as e:
    applogger.error('xbot2_cli not found: EtherCAT client package is missing. Discovery is unavailable.')
    raise e

# same as `ecat describe`, uri is the one stored by `ecat config uri` (default: localhost:5555)
reply = EcatContext().get_slave_description(EcatArguments(), verbose=False)

if not reply:
    raise RuntimeError('No slave description received from the EtherCAT master')

applogger.debug("Exit")

urdf_writer_fromHW.read_from_json(reply)

urdf_writer_fromHW.remove_all_connectors()

urdf = urdf_writer_fromHW.write_urdf()
srdf = urdf_writer_fromHW.write_srdf()
joint_map = urdf_writer_fromHW.write_joint_map(use_robot_id=True)
lowlevel_config = urdf_writer_fromHW.write_lowlevel_config(use_robot_id=True)
probdesc = urdf_writer_fromHW.write_problem_description_multi()

applogger.debug("\nURDF\n")
applogger.debug(urdf)
applogger.debug("\nSRDF\n")
applogger.debug(srdf)
applogger.debug("\nJoint Map\n")
applogger.debug(joint_map)

urdf_writer_fromHW.deploy_robot("modularbot_test")