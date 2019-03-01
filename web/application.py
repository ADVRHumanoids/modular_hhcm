from flask import Flask, render_template, request, jsonify, send_from_directory
# import flask
# print(flask.__file__, flask.__version__)

# import logging
import URDF_writer
import poller
import zmq

app = Flask(__name__, static_folder='static', static_url_path='')

# Instance of ZMQ Poller class (create sockets, etc.)
zmq_poller = poller.ZmqPoller()

# Instance of UrdfWriter class
urdf_writer = URDF_writer.UrdfWriter()

# 2nd instance of UrdfWriter class for the robot got from HW
urdf_writer_fromHW = URDF_writer.UrdfWriter()

# Prepare context and sockets. When not using Poller class

# Prepare our context and sockets
context = zmq.Context()#.instance()

# Socket to talk to poller
requester = context.socket(zmq.REQ)
requester.connect('tcp://localhost:5555')

# Flags defining which mode are in
building_mode_ON = True
discovery_mode_ON = False

# load view_urdf.html
@app.route('/')
def index():
    return render_template('view_urdf.html')


# call URDF_writer.py to modify the urdf
@app.route('/changeURDF/', methods=['POST'])
def changeURDF():
    filename = request.form.get('module_name', 0)
    print(filename)
    parent = request.form.get('parent', 0)
    print(parent)
    offset = float(request.form.get('angle_offset', 0))
    print(offset)
    data = urdf_writer.add_module(filename, offset)
    data = jsonify(data)
    return data 


@app.route('/writeURDF/', methods=['POST'])
def writeURDF():
    string = request.form.get('string', 0)
    print(string)
    print (building_mode_ON)
    print (discovery_mode_ON)
    print (building_mode_ON and (not discovery_mode_ON))
    if building_mode_ON and (not discovery_mode_ON):
        data = urdf_writer.write_urdf()
        srdf = urdf_writer.write_srdf()
        joint_map = urdf_writer.write_joint_map()
    else:
        data = urdf_writer_fromHW.write_urdf()
        srdf = urdf_writer_fromHW.write_srdf()
        joint_map = urdf_writer_fromHW.write_joint_map()
    print("\nSRDF\n")
    print(srdf)
    print("\nJoint Map\n")
    print(joint_map)
    # data = jsonify(data)
    return data 


# call URDF_writer.py to add another master cube
@app.route('/addMasterCube/', methods=['POST'])
def addCube():
    filename = request.form.get('module_name', 0)
    print(filename)
    parent = request.form.get('parent', 0)
    print(parent)
    offset = float(request.form.get('angle_offset', 0))
    print(offset)
    data = urdf_writer.add_slave_cube(offset)
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
    data = urdf_writer.select_module(parent)
    data = jsonify(data)
    return data


# update "last module" (and so shown buttons) when clicking on it
@app.route('/openFile/', methods=['POST'])
def openFile():
    file_str = request.form.get('file', 0)
    print(file_str)
    data = urdf_writer.read_file(file_str)
    print('data:', data)
    data = jsonify(data)
    return data


# upload on the server the /static folder
@app.route('/<path:path>')
def send_file(path):
    return send_from_directory(app.static_folder, path)


# send a request to the poller thread to get ECat topology and synchronize with hardware
@app.route('/syncHW/', methods=['POST'])
def syncHW():
    ## Method using the poller
    # zmq_poller.requester.send(b"Topology_REQ")
    # message = zmq_poller.requester.recv_json()
    # print("Received reply: %s" % (message))
    # data = urdf_writer_fromHW.read_from_json(message)
    # #print('data:', data)
    # data = jsonify(data)
    # return data

    requester.send(b"Topology_REQ")
    message = requester.recv_json()
    print("Received reply: %s" % (message))
    
    data = urdf_writer_fromHW.read_from_json(message)
    #print('data:', data)
    data = jsonify(data)
    return data


# Change mode and reset
@app.route('/changeMode/', methods=['POST'])
def changeMode():
    global building_mode_ON, discovery_mode_ON
    
    # Get the state of the toggle switch. Convert the boolean from Javascript to Python
    state = True if request.form.get('state', 0) == 'true' else False

    # Change Mode flag
    if state:
        building_mode_ON = False
        discovery_mode_ON = True
    else:
        building_mode_ON = True
        discovery_mode_ON = False

    print(building_mode_ON)
    print(discovery_mode_ON)

    # Re-initialize the two object instances
    urdf_writer.__init__()
    urdf_writer_fromHW.__init__()

    #data = urdf_writer_fromHW.read_file(file_str)
    data = {'building mode': building_mode_ON, 'discovery mode': discovery_mode_ON}

    data = jsonify(data)
    return data




if __name__ == '__main__':

    # Start Flask web-server
    app.run(debug=True, threaded=True)

    #main()

    # from gevent.pywsgi import WSGIServer
    # from geventwebsocket.handler import WebSocketHandler
    # http_server = WSGIServer(('', 5000), app, handler_class=WebSocketHandler)
    # logger.info('Starting serving')
    # print('Starting serving')
    # http_server.serve_forever()
