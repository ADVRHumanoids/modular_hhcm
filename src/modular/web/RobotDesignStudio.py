#!/usr/bin/env python3
# Disable some of the pylint violations in this file
# see https://pylint.pycqa.org/en/latest/user_guide/messages/message_control.html#block-disables
# pylint: disable=logging-too-many-args
# pylint: disable=protected-access, global-statement, broad-except, unused-variable
# pylint: disable=line-too-long, missing-function-docstring

# import yaml
import json
import os
import logging
import sys
from importlib import reload, util

import rospy
from flask import Flask, Response, render_template, request, jsonify, send_from_directory, abort
from flask.logging import create_logger
import zmq
import werkzeug

from modular.URDF_writer import UrdfWriter
ec_srvs_spec = util.find_spec('ec_srvs')
if ec_srvs_spec is not None:
    from ec_srvs.srv import GetSlaveInfo, GetSlaveInfoRequest, GetSlaveInfoResponse

# initialize ros node
rospy.init_node('robot_builder', disable_signals=True) # , log_level=rospy.DEBUG)

# set if ROS logger should be used
use_ros_logger = False
if use_ros_logger:
    # roslogger = logging.getLogger('rosout')
    roslogger = logging.getLogger(f'rosout.{__name__}')
    logger = roslogger
else:
    # Since initializing a ros node overrides the logging module behavior, we reload it here.
    reload(logging)
    FORMAT = '[%(levelname)s] [%(module)s]:  %(message)s'
    logging.basicConfig(format=FORMAT)
    applogger = logging.getLogger("RobotDesignStudio")
    logger = applogger

# get werkzeug logger
werkzeug_logger = logging.getLogger('werkzeug')

# set verbosity levels
verbose=True
if verbose:
    logger.setLevel(logging.DEBUG)
    logger.debug('Starting server')
    werkzeug_logger.setLevel(logging.INFO)
else:
    logger.setLevel(logging.INFO)
    werkzeug_logger.setLevel(logging.ERROR)
    
# determine if it's running on a Pyinstaller bundle
if getattr(sys, 'frozen', False) and hasattr(sys, '_MEIPASS'):
    template_folder = os.path.join(sys._MEIPASS, 'modular/web/templates')
    static_folder = os.path.join(sys._MEIPASS, 'modular/web/static')
    is_pyinstaller_bundle=True
else:
    static_folder='static'
    template_folder='templates'
    static_url_path=''
    is_pyinstaller_bundle=False

app = Flask(__name__, static_folder=static_folder, template_folder=template_folder, static_url_path='')
app.logger = logger

if is_pyinstaller_bundle:
    app.logger.debug('running in a PyInstaller bundle')
    app.logger.debug(sys._MEIPASS) # pylint: disable= protected-access, no-member
    app.logger.debug(template_folder)
    app.logger.debug(static_folder)
else:
    app.logger.debug('running in a normal Python process')
    
urdfwriter_kwargs_dict={
    'verbose': verbose,
    'logger': logger
}

# Instance of UrdfWriter class
urdf_writer = UrdfWriter(**urdfwriter_kwargs_dict)

# 2nd instance of UrdfWriter class for the robot got from HW
urdf_writer_fromHW = UrdfWriter(**urdfwriter_kwargs_dict)

# Flags defining which mode are in
building_mode_ON = True

# load view_urdf.html
@app.route('/')
def index():
    return render_template('view_urdf.html')

@app.route('/test')
def test():
    return render_template('test.html')


# call URDF_writer.py to modify the urdf
@app.route('/changeURDF/', methods=['POST'])
def changeURDF():
    filename = request.form.get('module_name', 0)
    app.logger.debug(filename)
    parent = request.form.get('parent', 0)
    app.logger.debug(parent)
    offset = float(request.form.get('angle_offset', 0))
    app.logger.debug(offset)
    reverse = True if request.form.get('reverse', 0) == 'true' else False
    app.logger.debug(reverse)
    data = urdf_writer.add_module(filename, offset, reverse)
    data = jsonify(data)
    return data 

# call URDF_writer.py to modify the urdf
@app.route('/addWheel/', methods=['POST'])
def addWheel():
    wheel_filename = request.form.get('wheel_module_name', 0)
    app.logger.debug(wheel_filename)
    steering_filename = request.form.get('steering_module_name', 0)
    app.logger.debug(steering_filename)
    parent = request.form.get('parent', 0)
    app.logger.debug(parent)
    offset = float(request.form.get('angle_offset', 0))
    app.logger.debug(offset)
    reverse = True if request.form.get('reverse', 0) == 'true' else False
    app.logger.debug(reverse)
    wheel_data, steering_data = urdf_writer.add_wheel_module(wheel_filename, steering_filename, offset, reverse)
    data = jsonify(wheel_data)
    return data 


@app.route('/writeURDF/', methods=['POST'])
def writeURDF():
    global building_mode_ON
    #string = request.form.get('string', 0)
    # app.logger.debug(string)
    # app.logger.debug (building_mode_ON)
    app.logger.debug('jointMap')
    json_jm = request.form.get('jointMap', 0)
    app.logger.debug(json_jm)
    builder_jm = json.loads(json_jm)
    building_mode_ON = True if request.form.get('buildingModeON', 0) == 'true' else False
    if building_mode_ON :
        data = urdf_writer.write_urdf()
        srdf = urdf_writer.write_srdf(builder_jm)
        app.logger.debug(srdf)
        joint_map = urdf_writer.write_joint_map()
        lowlevel_config = urdf_writer.write_lowlevel_config()
        # probdesc = urdf_writer.write_problem_description()
        probdesc = urdf_writer.write_problem_description_multi()
        # cartesio_stack = urdf_writer.write_cartesio_stack()
    else:
        data = urdf_writer_fromHW.write_urdf()
        srdf = urdf_writer_fromHW.write_srdf(builder_jm)
        app.logger.debug(srdf)
        joint_map = urdf_writer_fromHW.write_joint_map(use_robot_id=True)
        lowlevel_config = urdf_writer_fromHW.write_lowlevel_config(use_robot_id=True)
        # probdesc = urdf_writer_fromHW.write_problem_description()
        probdesc = urdf_writer_fromHW.write_problem_description_multi()
        # cartesio_stack = urdf_writer_fromHW.write_cartesio_stack()
    # app.logger.debug("\nSRDF\n")
    # app.logger.debug(srdf)
    # app.logger.debug("\nJoint Map\n")
    # app.logger.debug(joint_map)
    # app.logger.debug("\nCartesIO stack\n")
    # app.logger.debug(cartesio_stack)
    # data = jsonify(data)
    return data 


# call URDF_writer.py to add another master cube
@app.route('/addMasterCube/', methods=['POST'])
def addCube():
    filename = request.form.get('module_name', 0)
    app.logger.debug(filename)
    parent = request.form.get('parent', 0)
    app.logger.debug(parent)
    offset = float(request.form.get('angle_offset', 0))
    app.logger.debug(offset)
    data = urdf_writer.add_slave_cube(offset)
    data = jsonify(data)
    return data 

# call URDF_writer.py to add another master cube
@app.route('/addMobilePlatform/', methods=['POST'])
def addMobilePlatform():
    filename = request.form.get('module_name', 0)
    app.logger.debug(filename)
    parent = request.form.get('parent', 0)
    app.logger.debug(parent)
    offset = float(request.form.get('angle_offset', 0))
    app.logger.debug(offset)
    data = urdf_writer.add_mobile_platform(offset)
    data = jsonify(data)
    return data 


# call URDF_writer.py to add another socket
@app.route('/addSocket/', methods=['POST'])
def addSocket():
    values = json.loads(request.form.get('values'))
    offset = values['offset']
    app.logger.debug(offset)
    angle_offset = values['angle_offset']
    app.logger.debug(angle_offset)

    global building_mode_ON

    building_mode_ON = True if request.form.get('buildingModeON', 0) == 'true' else False
    
    if building_mode_ON :
        data = urdf_writer.add_socket(float(offset.get('x_offset')), float(offset.get('y_offset')),
                                  float(offset.get('z_offset')), float(angle_offset))
    else:
         data = urdf_writer_fromHW.move_socket("L_0_B", float(offset.get('x_offset')), float(offset.get('y_offset')),
                                  float(offset.get('z_offset')), float(angle_offset))

    data = jsonify(data)
    return data


#TODO: to be included in the next versions
# call URDF_writer.py to move socket. TODO: remove hard-code of L_0_B socket for AutomationWare demo
@app.route('/moveSocket/', methods=['POST'])
def moveSocket():
    values = json.loads(request.form.get('values'))
    app.logger.debug(values)
    offset = values['offset']
    app.logger.debug(offset)
    angle_offset = values['angle_offset']
    app.logger.debug(angle_offset)
    data = urdf_writer_fromHW.move_socket("L_0_B", float(offset.get('x_offset')), float(offset.get('y_offset')),
                                  float(offset.get('z_offset')), float(angle_offset))
    data = jsonify(data)
    return data


# call URDF_writer.py to remove the last module
@app.route('/removeModule/', methods=['POST'])
def remove():
    parent = request.form.get('parent', 0)
    data = urdf_writer.remove_module()
    data = jsonify(data)
    return data


# update "last module" (and so shown buttons) when clicking on it
@app.route('/updateLastModule/', methods=['POST'])
def accessModule():
    parent = request.form.get('parent', 0)
    data = urdf_writer.select_module_from_name(parent)
    data = jsonify(data)
    return data


#TODO: to be included in the next versions
# upload a URDF file and display it
@app.route('/openFile/', methods=['POST'])
def openFile():
    file_str = request.form.get('file', 0)
    app.logger.debug(file_str)
    data = urdf_writer.read_file(file_str)
    app.logger.debug('data: %s', data)
    data = jsonify(data)
    return data


# request the urdf generated from the currently stored tree
@app.route('/requestURDF/', methods=['POST'])
def requestURDF():
    # building_mode_on_str = request.form.get('mode', 0)
    # app.logger.debug(building_mode_on_str)
    urdf_string = urdf_writer.process_urdf()
    data = {'string': urdf_string}
    app.logger.debug('data: %s', data)
    data = jsonify(data)
    return data

# NOTE: this should not be needed anymore! Setting the static folder in the app constructor should be enough
# # upload on the server the /static folder
# @app.route('/<path:path>')
# def send_file(path):
#     return send_from_directory(app.static_folder, path)


# upload on the server the /modular_resources folder. 
# This is needed to load the meshes of the modules (withot the need to put them in the /static folder)
@app.route('/modular_resources/<path:path>')
def send_file(path):
    resources_paths = []
    resources_paths += [urdf_writer.resource_finder.find_resource_absolute_path('', ['resources_path'])]

    # upload also external resources (concert_resources, etc.)
    external_paths_dict = urdf_writer.resource_finder.nested_access(['external_resources'])
    external_paths = [ urdf_writer.resource_finder.get_expanded_path(['external_resources', p]) for p in external_paths_dict]
    resources_paths += external_paths

    # if isinstance(resources_path, str):
    #     return send_from_directory(resources_path, path)
    # else:
    for res_path in resources_paths:    
        try:
            return send_from_directory(res_path, path)
        except werkzeug.exceptions.NotFound:
            continue
    abort(404)


#TODO: to be included in the next versions (requires ROS etc.)
# send a request to the poller thread to get ECat topology and synchronize with hardware
@app.route('/syncHW/', methods=['POST'])
def syncHW():
    srv_name = '/ec_client/get_slaves_description'

    rospy.wait_for_service(srv_name)

    try:
        slave_description = rospy.ServiceProxy(srv_name, GetSlaveInfo) # pylint: disable=undefined-variable

    except rospy.ServiceException as e:
        app.logger.debug("Service call failed: %s",e)

    reply = slave_description()
    reply = reply.cmd_info.msg
    app.logger.debug("%s", reply)
    app.logger.debug("Exit")

    data = urdf_writer_fromHW.read_from_json(reply)
    # data = urdf_writer_fromHW.read_from_json_alt(reply)
    if urdf_writer_fromHW.verbose:
        urdf_writer_fromHW.render_tree()
    app.logger.debug('data: %s', data)
    data = jsonify(data)
    return data


# Change mode and reset
@app.route('/changeMode/', methods=['POST'])
def changeMode():
    global building_mode_ON
    
    # Get the state of the toggle switch. Convert the boolean from Javascript to Python
    building_mode_ON = True if request.form.get('buildingModeON', 0) == 'true' else False

    app.logger.debug(building_mode_ON)

    # Re-initialize the two object instances
    urdf_writer.__init__(**urdfwriter_kwargs_dict)
    urdf_writer_fromHW.__init__(**urdfwriter_kwargs_dict)

    #data = urdf_writer_fromHW.read_file(file_str)
    data = {'building mode': building_mode_ON}

    data = jsonify(data)
    return data


# deploy the package of the built robot
@app.route('/deployRobot/', methods=['POST'])
def deployRobot():
    global building_mode_ON

    building_mode_ON = True if request.form.get('buildingModeON', 0) == 'true' else False
    
    if building_mode_ON :
        name = request.form.get('name', 'ModularBot')
        app.logger.debug(name)
        data = urdf_writer.deploy_robot(name)
    else:
        data = urdf_writer_fromHW.deploy_robot('modularbot')
    #time.sleep(10)
    return data


@app.route('/removeConnectors/', methods=['POST'])
def removeConnectors():
    global building_mode_ON

    building_mode_ON = True if request.form.get('buildingModeON', 0) == 'true' else False
    
    if building_mode_ON :
        data = urdf_writer.remove_connectors()
    else:
        data = urdf_writer_fromHW.remove_connectors()    
    return data
    

def byteify(input):
    if isinstance(input, dict):
        return {byteify(key): byteify(value)
                for key, value in input.items()}
    elif isinstance(input, list):
        return [byteify(element) for element in input]
    elif isinstance(input, str):
        return input.encode('utf-8')
    else:
        return input


def main():
    # Start Flask web-server
    app.run(host='0.0.0.0', port=5000, debug=True, threaded=True)
    # app.run(debug=False, threaded=True)


if __name__ == '__main__':
    main()
