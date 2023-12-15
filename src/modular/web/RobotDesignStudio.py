#!/usr/bin/env python3
# Disable some of the pylint violations in this file
# see https://pylint.pycqa.org/en/latest/user_guide/messages/message_control.html#block-disables
# pylint: disable=logging-too-many-args
# pylint: disable=protected-access, global-statement, broad-except, unused-variable
# pylint: disable=line-too-long, missing-function-docstring, missing-module-docstring
# pylint: disable=wrong-import-position
# pylint: disable=import-error

# import yaml
import json
import os
import logging
import sys
from importlib import reload, util
from configparser import ConfigParser, ExtendedInterpolation

import numpy as np

import rospy
from flask import Flask, Response, render_template, request, jsonify, send_from_directory, abort
import werkzeug

from modular.URDF_writer import UrdfWriter
import modular.ModuleNode  as ModuleNode

ec_srvs_spec = util.find_spec('ec_srvs')
if ec_srvs_spec is not None:
    from ec_srvs.srv import GetSlaveInfo

# Add Mock Resources lists
import modular.web.mock_resources as mock_resources

import argparse

parser = argparse.ArgumentParser(prog='robot-design-studio', description='Robot Design Studio server')

parser.add_argument('-d', '--debug', required=False, action='store_true')
parser.add_argument('-v', '--verbose', required=False, action='store_true')
parser.add_argument('--use_ros_logger', required=False, action='store_true')
parser.add_argument('--slave_desc_mode', required=False, choices=('use_ids', 'use_pos'))
parser.set_defaults(debug=False, verbose=False, use_ros_logger=False, slave_desc_mode='use_pos')

args = parser.parse_args()

# import custom server config (if any)
base_path, _ = os.path.split(__file__)
config = ConfigParser(interpolation=ExtendedInterpolation(), allow_no_value=True)
config.read(os.path.join(base_path, 'web_config.ini'))
host = config.get('MODULAR_SERVER', 'host', fallback='0.0.0.0')
port = config.getint('MODULAR_SERVER','port',fallback=5003)
gui_route = config.get('MODULAR_API','gui_route',fallback='')
api_base_route = config.get('MODULAR_API','base_route',fallback='')

# initialize ros node
rospy.init_node('robot_builder', disable_signals=True) # , log_level=rospy.DEBUG)

# set if ROS logger should be used
if args.use_ros_logger:
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
if args.verbose:
    logger.setLevel(logging.DEBUG)
    logger.debug('Starting server')
    werkzeug_logger.setLevel(logging.INFO)
else:
    logger.setLevel(logging.INFO)
    werkzeug_logger.setLevel(logging.ERROR)


template_folder='modular_frontend'
static_folder = 'modular_frontend'  # '/static'
static_url_path = ''
# determine if it's running on a Pyinstaller bundle
if getattr(sys, 'frozen', False) and hasattr(sys, '_MEIPASS'):
    template_folder = os.path.join(sys._MEIPASS, 'modular/web',template_folder)
    static_folder = os.path.join(sys._MEIPASS, 'modular/web',static_folder)
    is_pyinstaller_bundle=True
else:
    is_pyinstaller_bundle=False

app = Flask(__name__, static_folder=static_folder, template_folder=template_folder, static_url_path=static_url_path)
app.logger = logger

if is_pyinstaller_bundle:
    app.logger.debug('running in a PyInstaller bundle')
    app.logger.debug(sys._MEIPASS) # pylint: disable= protected-access, no-member
    app.logger.debug(template_folder)
    app.logger.debug(static_folder)
else:
    app.logger.debug('running in a normal Python process')

urdfwriter_kwargs_dict={
    'verbose': args.verbose,
    'logger': logger,
    'slave_desc_mode': args.slave_desc_mode
}

# Instance of UrdfWriter class
urdf_writer = UrdfWriter(**urdfwriter_kwargs_dict)

# 2nd instance of UrdfWriter class for the robot got from HW
urdf_writer_fromHW = UrdfWriter(**urdfwriter_kwargs_dict)

# Flags defining which mode are in
building_mode_ON = True

def get_writer():
    if building_mode_ON :
        return urdf_writer
    else:
        return urdf_writer_fromHW

# load view_urdf.html
@app.route(f'{gui_route}/', methods=['GET'])
def index():
    return render_template('index.html')

@app.route('/test')
def test():
    return render_template('test.html')

# Get workspace mode
@app.route(f'{api_base_route}/mode', methods=['GET'])
def getMode():
    mode = 'Build' if building_mode_ON else 'Discover'
    return jsonify({'mode': mode}), 200

# Change mode and reset
# Request payload:
#   - mode:             [string]: "Build" or "Discover"
@app.route(f'{api_base_route}/mode', methods=['POST'])
def setMode():
    global building_mode_ON
    try:
        # Get the state of the toggle switch. Convert the boolean from Javascript to Python
        mode = request.get_json()['mode']
        if mode!='Build' and mode!= 'Discover':
            raise ValueError(f"Illegal value for mode: exprected 'Build' or 'Discover' but found {mode}.")

        building_mode_ON = mode == 'Build'
        app.logger.debug(building_mode_ON)

        # Re-initialize the two object instances
        urdf_writer.reset(**urdfwriter_kwargs_dict)
        urdf_writer_fromHW.reset(**urdfwriter_kwargs_dict)

        app.logger.info("Switched workspace mode to '%s'", mode)
        return Response(status=204)

    except ValueError as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=400,
            mimetype="application/json"
        )
    except Exception as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=500,
            mimetype="application/json"
        )

# Get a list of the available modules
@app.route(f'{api_base_route}/resources/modules', methods=['GET'])
def resources_modules_get():
    """Get available modules

    :param families: Optionally the returned list can be filtered by their family of module.
    :type families: List[str]
    :param types: Optionally the returned list can filter results by their type of module.
    :type types: List[str]

    :rtype: List[ModuleBase]
    """
    query_params = request.args
    try:
        # Get the right writer instance depending on the mode
        writer = get_writer()

        #get complete list
        modules = writer.modular_resources_manager.get_available_modules()

        # filter by family (from query params)
        valid_families = writer.modular_resources_manager.get_available_family_ids()

        filter_families = query_params.getlist('families[]')
        for t in filter_families:
            if t not in valid_families:
                raise ValueError(f"Illegal value for filter families: expected one of {valid_families} but found '{t}'.")
        if len(filter_families) > 0:
            modules = [el for el in modules if el['family'] in filter_families]

        # filter by type (from query params)
        valid_types = writer.modular_resources_manager.get_available_module_types()
        filter_types = query_params.getlist('types[]')
        for t in filter_types:
            if t not in valid_types:
                raise ValueError(f"Illegal value for filter types: expected one of {valid_types} but found '{t}'.")
        if len(filter_types) > 0:
            modules = [el for el in modules if el['type'] in filter_types]

        # return filtered list
        return Response(
                response=json.dumps({"modules": modules}),
                status=200,
                mimetype="application/json"
            )

    except ValueError as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=400,
            mimetype="application/json"
        )
    except Exception as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=500,
            mimetype="application/json"
        )

# Get a list of the module types that can currently be added to the model.
@app.route(f'{api_base_route}/resources/modules/allowed', methods=['GET'])
def resources_modules_allowed_get():
    """Get a list of the module types that can currently be added to the model.

    :param ids: Optionally, you can provide one or more IDs of modules. (Currently not supported)
    :type ids: List[str]

    :rtype: List[str]
    """
    query_params = request.args
    try:
        # Get the right writer instance depending on the mode
        writer = get_writer()

        # get complete list of modules
        # modules = writer.modular_resources_manager.get_available_modules()

        ids = query_params.getlist('ids[]')
        if len(ids)>1:
            return Response(
                response=json.dumps({"message": 'Use of multiple ids at once is currently not supported'}),
                status=501,
                mimetype="application/json"
            )
        elif len(ids)==1:
            writer.select_module_from_name(ids[0], None)

        if building_mode_ON:
            valid_types = writer.modular_resources_manager.get_available_module_types()
        else:
            # TODO: this list should come from a config file
            valid_types = ['end_effector', 'interface_adapter']


        return Response(
                response=json.dumps({"type": valid_types}),
                status=200,
                mimetype="application/json"
            )

    except Exception as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=500,
            mimetype="application/json"
        )

# Get a list of the available module addons
@app.route(f'{api_base_route}/resources/addons', methods=['GET'])
def resources_addons_get():
    """Get available addons

    :param families: Optionally the returned list can be filtered by their family of addons.
    :type families: List[str]
    :param types: Optionally the returned list can filter results by their type of addons.
    :type types: List[str]

    :rtype: List[ModuleAddonsBase]
    """
    query_params = request.args
    try:
        # Get the right writer instance depending on the mode
        writer = get_writer()

        #get complete list
        addons = writer.modular_resources_manager.get_available_addons()

        # filter by family (from query params)
        valid_families = writer.modular_resources_manager.get_available_family_ids()

        filter_families = query_params.getlist('families[]')
        for t in filter_families:
            if t not in valid_families:
                raise ValueError(f"Illegal value for filter families: expected one of {valid_families} but found '{t}'.")
        if len(filter_families) > 0:
            addons = [el for el in addons if el['family'] in filter_families]

        # filter by type (from query params)
        valid_types = writer.modular_resources_manager.get_available_addon_types()
        filter_types = query_params.getlist('types[]')
        for t in filter_types:
            if t not in valid_types:
                raise ValueError(f"Illegal value for filter types: expected one of {valid_types} but found '{t}'.")
        if len(filter_types) > 0:
            addons = [el for el in addons if el['type'] in filter_types]

        # return filtered list
        return Response(
                response=json.dumps({"addons": addons}),
                status=200,
                mimetype="application/json"
            )

    except ValueError as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=400,
            mimetype="application/json"
        )
    except Exception as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=500,
            mimetype="application/json"
        )

# Get a list of the available families of modules
@app.route(f'{api_base_route}/resources/families', methods=['GET'])
def resources_families_get():
    """Get a list of families of the available modules

    :param families: Optionally the returned list can be filtered by their family of module.
    :type families: List[str]
    :param groups: Optionally the returned list can filter results by their gruop.
    :type groups: List[str]

    :rtype: List[ModuleFamilies]
    """
    query_params = request.args
    try:
        # Get the right writer instance depending on the mode
        writer = get_writer()

        #get complete list
        families = writer.modular_resources_manager.get_available_families()

        # filter by family (from query params)
        valid_families = writer.modular_resources_manager.get_available_family_ids()
        filter_families = query_params.getlist('families')
        for t in filter_families:
            if t not in valid_families:
                raise ValueError(f"Illegal value for filter families: expected one of {valid_families} but found '{t}'.")
        if len(filter_families) > 0:
            families = [el for el in families if el['family'] in filter_families]

        # filter by group (from query params)
        valid_groups = writer.modular_resources_manager.get_available_family_groups()
        filter_groups = query_params.getlist('groups')
        for t in filter_groups:
            if t not in valid_groups:
                raise ValueError(f"Illegal value for filter groups: expected one of {valid_groups} but found '{t}'.")
        if len(filter_groups) > 0:
            families = [el for el in families if el['group'] in filter_groups]

        # return filtered list
        return Response(
                response=json.dumps({"families": families}),
                status=200,
                mimetype="application/json"
            )

    except ValueError as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=400,
            mimetype="application/json"
        )
    except Exception as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=500,
            mimetype="application/json"
        )

@app.route(f'{api_base_route}/model/urdf/modules', methods=['POST'])
def addNewModule():
    req = request.get_json()
    writer = get_writer()

    try:
        if building_mode_ON:
            valid_types = writer.modular_resources_manager.get_available_module_types()
        else:
            # TODO: this list should come from a config file
            valid_types = ['end_effector', 'interface_adapter']

        if req['type'] not in valid_types:
            return Response(
                response=json.dumps({"message": f"In {'Build' if building_mode_ON else 'Discovery'} mode, modules of type {req['type']} cannot be added"}),
                status=409,
                mimetype="application/json"
            )

        # Get the right writer instance depending on the mode
        writer = get_writer()

        filename = req['name']
        app.logger.debug(filename)

        parent = req['parent'] if 'parent' in req else None
        app.logger.debug(parent)

        # We update the selected module to the one selected in the GUI. If no module is selected, we don't update it, since the BE will keep track of the current parent
        if parent:
            writer.select_module_from_name(parent, None)

        offset = float(req['offset']['yaw'] if 'offset' in req and 'yaw' in req['offset'] else 0) # we user RPY notation
        app.logger.debug(offset)

        reverse = True if 'reverse' in req and req['reverse'] == 'true' else False
        app.logger.debug(reverse)

        addons = req['addons'] if 'addons' in req else []
        app.logger.debug(addons)

        module_data = writer.add_module(filename, offset, reverse, addons)

        return Response(response=json.dumps({'id': module_data['selected_connector'],
                                             'meshes': module_data['selected_meshes']}),
                        status=200,
                        mimetype="application/json")

    except ValueError as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=400,
            mimetype="application/json"
        )
    except Exception as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=500,
            mimetype="application/json"
        )

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
    # Get the right writer instance depending on the mode
    writer = get_writer()
    data = writer.add_module(filename, offset, reverse)
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
    # Get the right writer instance depending on the mode
    writer = get_writer()
    wheel_data, steering_data = writer.add_wheel_module(wheel_filename, steering_filename, offset, reverse)
    data = jsonify(wheel_data)
    return data


def writeRobotURDF(builder_jm):
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
    app.logger.debug("\nSRDF\n")
    app.logger.debug(srdf)
    app.logger.debug("\nJoint Map\n")
    app.logger.debug(joint_map)
    # app.logger.debug("\nCartesIO stack\n")
    # app.logger.debug(cartesio_stack)
    return data

@app.route('/writeURDF/', methods=['POST'])
def writeURDF():
    app.logger.debug('jointMap')
    json_jm = request.form.get('jointMap', 0)
    app.logger.debug(json_jm)
    builder_jm = json.loads(json_jm)

    data = writeRobotURDF(builder_jm)
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
    #TODO: fix this hack
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
    # Get the right writer instance depending on the mode
    writer = get_writer()
    data = writer.move_socket("L_0_B", float(offset.get('x_offset')), float(offset.get('y_offset')),
                                  float(offset.get('z_offset')), float(angle_offset))
    data = jsonify(data)
    return data

@app.route('/removeModule/', methods=['POST'])
def remove():
    parent = request.form.get('parent', 0)
    # Get the right writer instance depending on the mode
    writer = get_writer()
    data = writer.remove_module()
    data = jsonify(data)
    return data


# update "last module" (and so shown buttons) when clicking on it
@app.route('/updateLastModule/', methods=['POST'])
def accessModule():
    parent = request.form.get('parent', 0)
    # Get the right writer instance depending on the mode
    writer = get_writer()
    data = writer.select_module_from_name(parent)
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
@app.route(f'{api_base_route}/model/urdf', methods=['GET'])
def getURDF():
    try:

        # Get the right writer instance depending on the mode
        writer = get_writer()
        urdf_string = writer.urdf_string

        # replace path for remote access of STL meshes that will be served with '/meshes/<path:path>' route
        # urdf= urdf_string.replace('package://modular/src/modular/web/static/models/modular/,'package://')
        urdf= urdf_string\
                .replace('package://modular_resources',f'package:/{api_base_route}/resources/meshes')\
                .replace('package://concert_resources',f'package:/{api_base_route}/resources/meshes')


        return  Response(
            response=urdf,
            status=200,
            mimetype='application/xml'
        )
    except Exception as e:
        # validation failed
        app.logger.error( f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=500,
            mimetype="application/json"
        )

@app.route('/requestURDF/', methods=['POST'])
def requestURDF():
    # building_mode_on_str = request.form.get('mode', 0)
    # app.logger.debug(building_mode_on_str)
    # Get the right writer instance depending on the mode
    writer = get_writer()
    urdf_string = writer.process_urdf()
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
@app.route(f'{api_base_route}/resources/meshes/<path:path>', methods=['GET'])
@app.route('/modular_resources/<path:path>')
def send_file(path):
    # Get the right writer instance depending on the mode
    writer = get_writer()

    resources_paths = []
    resources_paths += [writer.resource_finder.find_resource_absolute_path('', ['resources_path'])]

    # upload also external resources (concert_resources, etc.)
    external_paths_dict = writer.resource_finder.nested_access(['external_resources'])
    external_paths = [ writer.resource_finder.get_expanded_path(['external_resources', p]) for p in external_paths_dict]
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
@app.route(f'{api_base_route}/model/urdf', methods=['PUT'])
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
    app.logger.debug("Exit")

    data = urdf_writer_fromHW.read_from_json(reply)
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
    urdf_writer.reset(**urdfwriter_kwargs_dict)
    urdf_writer_fromHW.reset(**urdfwriter_kwargs_dict)

    #data = urdf_writer_fromHW.read_file(file_str)
    data = {'building mode': building_mode_ON}

    data = jsonify(data)
    return data

def getModulesMap():
    chains=[]

    writer = get_writer()
    chains = writer.listofchains

    modules={}
    for chain in chains:
        for el in chain:
            modules[el.name] = ModuleNode.as_dumpable_dict(el.header)
    return modules

# get list of modules of robot
@app.route(f'{api_base_route}/model/urdf/modules/map', methods=['GET'])
def getModelModules():
    try:
        ids = request.args.getlist('ids[]')

        modules = getModulesMap()

        if len(ids)==0:
            filtered_modules = modules  # if no ids are provided, return all modules
        else:
            filtered_modules = {key: modules[key] for key in ids}  # filter modules by ids

        return Response(
            response=json.dumps({'modules': filtered_modules}),
            status=200,
            mimetype="application/json"
        )

    except Exception as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=500,
            mimetype="application/json"
        )

# Get the id of the module associated to a mesh and the list of all meshes associated to a module
@app.route(f'{api_base_route}/model/urdf/modules/meshes', methods=['GET'])
def module_meshes_get():
        writer = get_writer()

        mesh_ids = request.args.getlist('ids[]')

        if len(mesh_ids)>1:
            return Response(
                response=json.dumps({"message": 'Only one id at a time should be provided'}),
                status=501,
                mimetype="application/json"
            )
        elif len(mesh_ids)==1:
            mesh_id = mesh_ids[0]
            # From the name of the mesh clicked on the GUI select the module associated to it
            # Also sets the selected_connector to the one associated to the mesh
            associated_module_data = writer.select_module_from_name(mesh_id, None)

        return Response(response=json.dumps({'id': associated_module_data['selected_connector'],
                                            'meshes': associated_module_data['selected_meshes']}),
                        status=200,
                        mimetype="application/json")

# call URDF_writer.py to remove the last module
@app.route(f'{api_base_route}/model/urdf/modules', methods=['DELETE'])
def removeModules():
    """Delete one or more modules from the robot model. By default it removes the last element.

    :param ids: Optionally, you can provide one or more IDs of modules to remove. (Currently not supported)
    :type ids: List[str]
    """
    if not building_mode_ON:
        return Response(
            response=json.dumps({"message": 'Cannot delete modules in Discovery mode.'}),
            status=409,
            mimetype="application/json"
    )

    try:
        # Get the right writer instance depending on the mode
        writer = get_writer()

        ids = request.args.getlist('ids[]')
        if len(ids)>1:
            return Response(
                response=json.dumps({"message": 'Deletion of multiple ids at once is currently not supported'}),
                status=501,
                mimetype="application/json"
            )
        elif len(ids)==1:
            writer.select_module_from_name(ids[0], None)
        father_module_data = writer.remove_module()
        return Response(response=json.dumps({'id': father_module_data['selected_connector'],
                                            'meshes': father_module_data['selected_meshes']}),
                        status=200,
                        mimetype="application/json")

    except Exception as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=500,
            mimetype="application/json"
        )


# call URDF_writer.py to update the last module
@app.route(f'{api_base_route}/model/urdf/modules', methods=['PUT'])
def updateModule():
    req = request.get_json()

    # Get the right writer instance depending on the mode
    writer = get_writer()

    try:
        ids = request.args.getlist('ids[]')
        if len(ids)>1:
            return Response(
                response=json.dumps({"message": 'Deletion of multiple ids at once is currently not supported'}),
                status=501,
                mimetype="application/json"
            )
        elif len(ids)==1:
            writer.select_module_from_name(ids[0], None)

        app.logger.debug(req['parent'] if 'parent' in req else 'no parent')

        offset = float(req['offset']['yaw'] if 'offset' in req and 'yaw' in req['offset'] else 0) # we user RPY notation
        app.logger.debug(offset)

        reverse = True if 'reverse' in req and req['reverse'] == 'true' else False
        app.logger.debug(reverse)

        addons = req['addons'] if 'addons' in req else []

        updated_module_data = writer.update_module(angle_offset=offset, reverse=reverse, addons=addons)

        return Response(response=json.dumps({'id': updated_module_data['selected_connector'],
                                            'meshes': updated_module_data['selected_meshes']}),
                        status=200,
                        mimetype="application/json")
    except Exception as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=500,
            mimetype="application/json"
        )

# deploy the package of the built robot
@app.route('/deployRobot/', methods=['POST'])
def deployRobot():
    name = request.form.get('name', 'modularbot')
    app.logger.debug(name)

    # Get the right writer instance depending on the mode
    writer = get_writer()

    data = writer.deploy_robot(name)
    #time.sleep(10)
    return data


@app.route('/removeConnectors/', methods=['POST'])
def removeConnectors():
    # Get the right writer instance depending on the mode
    writer = get_writer()

    writer.remove_all_connectors()

    urdf_string = writer.process_urdf()

    return urdf_string

# deploy the package of the built robot
@app.route(f'{api_base_route}/model/urdf', methods=['POST'])
def deployROSModel():
    try:
        req = request.get_json()
        name = req['name']
        builder_jm =  req['jointMap']

        # Get the right writer instance depending on the mode
        writer = get_writer()

        writer.remove_all_connectors() # taken from removeConnectors(), to be removed and itergrated inside .deploy_robot()

        writeRobotURDF(builder_jm)

        app.logger.debug(name)
        writer.deploy_robot(name)

        return Response(status=204)

    except Exception as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=500,
            mimetype="application/json"
        )

# get current model stats
@app.route(f'{api_base_route}/model/stats', methods=['GET'])
def getModelStats():
    """Returns a set of statistics for the curent robot model.
    """
    try:
        # Get the right writer instance depending on the mode
        writer = get_writer()

        stats = writer.compute_stats(samples=1000)

        response = dict()
        if  stats['modules']:
            response["modules"]= { "label": 'Modules', "value": str(stats['modules']) }
        if  stats['payload'] and np.isfinite(stats['payload']):
            response["payload"]= { "label": 'Payload', "value":"{:.2f}".format(stats['payload']), "unit": 'Kg' }
        if  stats['max_reach'] and np.isfinite(stats['max_reach']):
            response["max_reach"]= { "label": 'Reach', "value": "{:.2f}".format(stats['max_reach']), "unit": 'm' }

        return Response(
            response=json.dumps(response),
            status=200,
            mimetype="application/json"
        )
    except RuntimeError as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'Error in computing stats -> {type(e).__name__}: {e}'}),
            status=400,
            mimetype="application/json"
        )
    except Exception as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=500,
            mimetype="application/json"
        )

def byteify(input_raw):
    if isinstance(input_raw, dict):
        return {byteify(key): byteify(value)
                for key, value in input_raw.items()}
    elif isinstance(input_raw, list):
        return [byteify(element) for element in input_raw]
    elif isinstance(input_raw, str):
        return input_raw.encode('utf-8')
    else:
        return input_raw


def main():
    app.run(host=host, port=port, debug=args.debug, threaded=True)


if __name__ == '__main__':
    main()
