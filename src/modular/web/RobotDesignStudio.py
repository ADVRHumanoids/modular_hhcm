#!/usr/bin/env python
import time
from flask import Flask, render_template, request, jsonify, send_from_directory, url_for
#import hierarchy_server as hp
# import flask
# roslogger.debug(flask.__file__, flask.__version__)

# import logging
from modular.URDF_writer import UrdfWriter

import zmq
import yaml
import json
import os
import logging

import rospy

from ec_srvs.srv import GetSlaveInfo, GetSlaveInfoRequest, GetSlaveInfoResponse

import sys

roslogger = logging.getLogger('rosout')


if getattr(sys, 'frozen', False) and hasattr(sys, '_MEIPASS'):
    roslogger.debug('running in a PyInstaller bundle')
    template_folder = os.path.join(sys._MEIPASS, 'modular/web/templates')
    static_folder = os.path.join(sys._MEIPASS, 'modular/web/static')
    roslogger.debug(sys._MEIPASS)
    roslogger.debug(template_folder)
    roslogger.debug(static_folder)
    app = Flask(__name__, static_folder=static_folder, template_folder=template_folder, static_url_path='')
else:
    roslogger.debug('running in a normal Python process')
    app = Flask(__name__, static_folder='static', template_folder='templates', static_url_path='')
    

# app = Flask(__name__)

# Instance of UrdfWriter class
urdf_writer = UrdfWriter(logger=roslogger)

# 2nd instance of UrdfWriter class for the robot got from HW
urdf_writer_fromHW = UrdfWriter(logger=roslogger)

# Initialize json parser server manager
#manager = hp.MANAGER(config_namespace='/state_machine')

#manager.publish_to_frontend()
# Prepare context and sockets. When not using Poller class

# Prepare our context and sockets
context = zmq.Context()#.instance()

# Socket to talk to DEMO SM
publisher = context.socket(zmq.PUB)
publisher.connect('tcp://192.168.9.211:5563')
#publisher.connect('tcp://10.255.36.12:5555')

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
    roslogger.debug(filename)
    parent = request.form.get('parent', 0)
    roslogger.debug(parent)
    offset = float(request.form.get('angle_offset', 0))
    roslogger.debug(offset)
    reverse = True if request.form.get('reverse', 0) == 'true' else False
    roslogger.debug(reverse)
    data = urdf_writer.add_module(filename, offset, reverse)
    data = jsonify(data)
    return data 


@app.route('/writeURDF/', methods=['POST'])
def writeURDF():
    global building_mode_ON
    #string = request.form.get('string', 0)
    # roslogger.debug(string)
    # roslogger.debug (building_mode_ON)
    roslogger.debug('jointMap')
    json_jm = request.form.get('jointMap', 0)
    roslogger.debug(json_jm)
    builder_jm = json.loads(json_jm)
    building_mode_ON = True if request.form.get('buildingModeON', 0) == 'true' else False
    if building_mode_ON :
        data = urdf_writer.write_urdf()
        srdf = urdf_writer.write_srdf(builder_jm)
        roslogger.debug(srdf)
        joint_map = urdf_writer.write_joint_map()
        lowlevel_config = urdf_writer.write_lowlevel_config()
        #probdesc = urdf_writer.write_problem_description()
        probdesc = urdf_writer.write_problem_description_multi()
        # cartesio_stack = urdf_writer.write_cartesio_stack()
    else:
        data = urdf_writer_fromHW.write_urdf()
        srdf = urdf_writer_fromHW.write_srdf(builder_jm)
        roslogger.debug(srdf)
        joint_map = urdf_writer_fromHW.write_joint_map(use_robot_id=True)
        lowlevel_config = urdf_writer_fromHW.write_lowlevel_config(use_robot_id=True)
        #probdesc = urdf_writer_fromHW.write_problem_description()
        probdesc = urdf_writer_fromHW.write_problem_description_multi()
        # cartesio_stack = urdf_writer_fromHW.write_cartesio_stack()
    # roslogger.debug("\nSRDF\n")
    # roslogger.debug(srdf)
    # roslogger.debug("\nJoint Map\n")
    # roslogger.debug(joint_map)
    # roslogger.debug("\nCartesIO stack\n")
    # roslogger.debug(cartesio_stack)
    # data = jsonify(data)
    return data 


# call URDF_writer.py to add another master cube
@app.route('/addMasterCube/', methods=['POST'])
def addCube():
    filename = request.form.get('module_name', 0)
    roslogger.debug(filename)
    parent = request.form.get('parent', 0)
    roslogger.debug(parent)
    offset = float(request.form.get('angle_offset', 0))
    roslogger.debug(offset)
    data = urdf_writer.add_slave_cube(offset)
    data = jsonify(data)
    return data 


# call URDF_writer.py to add another socket
@app.route('/addSocket/', methods=['POST'])
def addSocket():
    values = json.loads(request.form.get('values'))
    offset = values['offset']
    roslogger.debug(offset)
    angle_offset = values['angle_offset']
    roslogger.debug(angle_offset)

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


# call URDF_writer.py to move socket. TODO: remeve hard-code of L_0_B socket for AutomationWare demo
@app.route('/moveSocket/', methods=['POST'])
def moveSocket():
    values = json.loads(request.form.get('values'))
    roslogger.debug(values)
    offset = values['offset']
    roslogger.debug(offset)
    angle_offset = values['angle_offset']
    roslogger.debug(angle_offset)
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


# update "last module" (and so shown buttons) when clicking on it
@app.route('/openFile/', methods=['POST'])
def openFile():
    file_str = request.form.get('file', 0)
    roslogger.debug(file_str)
    data = urdf_writer.read_file(file_str)
    roslogger.debug('data:', data)
    data = jsonify(data)
    return data

# request the urdf generated from the currently stored tree
@app.route('/requestURDF/', methods=['POST'])
def requestURDF():
    # building_mode_on_str = request.form.get('mode', 0)
    # roslogger.debug(building_mode_on_str)
    urdf_string = urdf_writer.process_urdf()
    data = {'string': urdf_string}
    roslogger.debug('data:', data)
    data = jsonify(data)
    return data

# upload on the server the /static folder
@app.route('/<path:path>')
def send_file(path):
    return send_from_directory(app.static_folder, path)

# publish through ZMQ socket to signal user inputs to DEMO GUI
@app.route('/pub_cmd/', methods=['POST'])
def pub_cmd():
    key = 'Builder'
    cmd = request.form.get('cmd', 0)
    roslogger.debug('cmd:', cmd.encode())
    publisher.send_multipart([key, cmd.encode()])

    return jsonify(cmd)

# send a request to the poller thread to get ECat topology and synchronize with hardware
@app.route('/syncHW/', methods=['POST'])
def syncHW():
    srv_name = '/ec_client/get_slaves_description'

    rospy.wait_for_service(srv_name)

    try:
        slave_description = rospy.ServiceProxy(srv_name, GetSlaveInfo)

    except rospy.ServiceException as e:
        roslogger.debug("Service call failed: %s"%e)

    reply = slave_description()
    reply = reply.cmd_info.msg
    roslogger.debug("Exit")

    data = urdf_writer_fromHW.read_from_json(reply)
    # data = urdf_writer_fromHW.read_from_json_alt(reply)
    if urdf_writer_fromHW.verbose:
        urdf_writer_fromHW.render_tree()
    roslogger.debug('data:', data)
    data = jsonify(data)
    return data

# Change mode and reset
@app.route('/changeMode/', methods=['POST'])
def changeMode():
    global building_mode_ON
    
    # Get the state of the toggle switch. Convert the boolean from Javascript to Python
    building_mode_ON = True if request.form.get('buildingModeON', 0) == 'true' else False

    roslogger.debug(building_mode_ON)

    # Re-initialize the two object instances
    urdf_writer.__init__()
    urdf_writer_fromHW.__init__()

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
        roslogger.debug(name)
        data = urdf_writer.deploy_robot(name)
    else:
        data = urdf_writer_fromHW.deploy_robot('ModularBot')
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

@app.route('/sendCommands/', methods=['POST'])
def sendCommands():
    jsonData = request.get_json()
    roslogger.debug(jsonData)
    roslogger.debug(byteify(jsonData))
    # res = manager.parse_json(jsonData)
    
    res = 'if you read this something wrong is happening: fi commented out the hierarchy server!'
    roslogger.debug(res)
    # roslogger.debug jsonData['descriptors']
    # roslogger.debug jsonData['name']
    # dictData = json.loads(jsonData)
    # roslogger.debug(dictData)
    # roslogger.debug(prova["descriptors"])
    # data = request.form.to_dict
    # roslogger.debug data
    return res

def byteify(input):
    if isinstance(input, dict):
        return {byteify(key): byteify(value)
                for key, value in input.iteritems()}
    elif isinstance(input, list):
        return [byteify(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input

def main():
     # initialize ros node
    rospy.init_node('robot_builder')
    # Start Flask web-server
    app.run(host='0.0.0.0', port=5000, debug=True, threaded=True)
    # app.run(debug=False, threaded=True)

    #main()

    # from gevent.pywsgi import WSGIServer
    # from geventwebsocket.handler import WebSocketHandler
    # http_server = WSGIServer(('', 5000), app, handler_class=WebSocketHandler)
    # logger.info('Starting serving')
    # roslogger.debug('Starting serving')
    # http_server.serve_forever()


if __name__ == '__main__':
    main()
   
