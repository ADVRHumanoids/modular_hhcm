from modular.URDF_writer import *

import logging
FORMAT = '[%(levelname)s] [%(module)s]:  %(message)s'
logging.basicConfig(format=FORMAT)
applogger = logging.getLogger("discovery")

urdfwriter_kwargs_dict={
    'verbose': True,
    'slave_desc_mode': 'use_pos',
    'logger': applogger,
}

urdf_writer_fromHW = UrdfWriter(**urdfwriter_kwargs_dict)

from importlib import util
ec_srvs_spec = util.find_spec('ec_srvs')
if ec_srvs_spec is not None:
    from ec_srvs.srv import GetSlaveInfo
    
srv_name = '/ec_client/get_slaves_description'
rospy.wait_for_service(srv_name, 5)

try:
    slave_description = rospy.ServiceProxy(srv_name, GetSlaveInfo) # pylint: disable=undefined-variable

except rospy.ServiceException as e:
    applogger.debug("Service call failed: %s",e)
    raise e

reply = slave_description()
reply = reply.cmd_info.msg
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