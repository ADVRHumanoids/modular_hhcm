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
import re
import argparse
import subprocess
from importlib import reload, util
from configparser import ConfigParser, ExtendedInterpolation
from typing import TypedDict
from datetime import datetime, timedelta
from uuid import uuid4

import numpy as np

import rospy
from flask import Flask, Response, render_template, request, jsonify, send_from_directory, abort, session, send_file
from apscheduler.schedulers.background import BackgroundScheduler
import werkzeug

from modular.URDF_writer import UrdfWriter
import modular.ModuleNode  as ModuleNode
from modular.enums import ModuleClass

ec_srvs_spec = util.find_spec('ec_srvs')
if ec_srvs_spec is not None:
    from ec_srvs.srv import GetSlaveInfo


# get backend version from git
try:
    backend_version = subprocess.check_output(['git', 'describe', '--abbrev=1', '--always', '--dirty'], cwd=os.path.dirname(__file__)).decode().strip()
except subprocess.CalledProcessError as e:
    backend_version = 'Unknown'

# get fronetend version from manisfest file
frontend_version = 'Unknown'
manifest_path = os.path.join(os.path.dirname(__file__), 'modular_frontend', 'manifest.json')
with open(manifest_path) as f:
    manifest_data = json.load(f)
    if 'version' in manifest_data:
        frontend_version = manifest_data['version']
    elif 'version_name' in manifest_data:
        frontend_version = manifest_data['version_name']


parser = argparse.ArgumentParser(prog='robot-design-studio', description='Robot Design Studio server')
parser.add_argument('-d', '--debug', required=False, action='store_true', default=False)
parser.add_argument('-v', '--verbose', required=False, action='store_true', default=False)
parser.add_argument('--use_ros_logger', required=False, action='store_true', default=False)
parser.add_argument('--slave_desc_mode', required=False, choices=('use_ids', 'use_pos'), default='use_pos')

# parse only known args, see https://stackoverflow.com/a/59067873/22225741
args = parser.parse_known_args()[0]

# import custom server config (if any)
base_path, _ = os.path.split(__file__)
config = ConfigParser(interpolation=ExtendedInterpolation(), allow_no_value=True)
config.read(os.path.join(base_path, 'web_config.ini'))
host = config.get('MODULAR_SERVER', 'host', fallback='0.0.0.0')
port = config.getint('MODULAR_SERVER','port',fallback=5003)
gui_route = config.get('MODULAR_API','gui_route',fallback='')
api_base_route = config.get('MODULAR_API','base_route',fallback='')
secret_key = config.get('MODULAR_API','secret_key',fallback='secret_key')
enable_sessions = config.getboolean('MODULAR_API','enable_sessions',fallback=False)
enable_discovery = config.getboolean('MODULAR_API','enable_discovery',fallback=True)
download_on_deploy = config.getboolean('MODULAR_API','download_on_deploy',fallback=False)

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
app.secret_key = secret_key
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


class SessionData(TypedDict):
    # Instance of UrdfWriter class
    urdf_writer: UrdfWriter
    # 2nd instance of UrdfWriter class for the robot got from HW
    urdf_writer_fromHW: UrdfWriter
    # Flags defining which mode are in
    building_mode_ON: bool
    # Last time the session was updated
    last_updated: datetime

# dictionary of sessions data
# sessions:dict[str, SessionData] = {}
sessions = {}


def cleanup():
    now = datetime.now()
    expired_sessions = [sid for sid, session_data in sessions.items() if now - session_data['last_updated'] > timedelta(minutes=30)]
    for sid in expired_sessions:
        del sessions[sid]

scheduler = BackgroundScheduler(daemon=True)
scheduler.add_job(cleanup, 'interval', minutes=30)
scheduler.start()

def get_writer(sid:str) -> UrdfWriter:
    if sid not in sessions:
        raise 'No session found, refresh the page to start a new one'

    if sessions[sid]['building_mode_ON'] :
        return sessions[sid]['urdf_writer']
    else:
        return sessions[sid]['urdf_writer_fromHW']

def get_building_mode_ON(sid:str) -> bool:
    if sid not in sessions:
        raise 'No session found, refresh the page to start a new one'
    return sessions[sid]['building_mode_ON']

# load view_urdf.html
@app.route(f'{gui_route}/', methods=['GET'])
def index():
    if enable_sessions :
        if'session_id' not in session:
            session['session_id'] = str(uuid4())
        sid = session['session_id']
    else:
        sid = 'default'

    if sid not in sessions:
        sessions[sid] = SessionData(
            urdf_writer= UrdfWriter(**urdfwriter_kwargs_dict),
            urdf_writer_fromHW= UrdfWriter(**urdfwriter_kwargs_dict),
            building_mode_ON= True,
            last_updated= datetime.now(),
            )

    return render_template('index.html')

# Get workspace mode
@app.route(f'{api_base_route}/mode', methods=['GET'])
def getMode():
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True

    building_mode_ON=get_building_mode_ON(sid)
    mode = 'Build' if building_mode_ON else 'Discover'
    return jsonify({'mode': mode}), 200

# Get info about dicovery mode status
@app.route(f'{api_base_route}/mode/discovery', methods=['GET'])
def getDiscoveryStatus():
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True

    try:
        return Response(
            status=200,
            response=json.dumps({ 'available': enable_discovery}),
        )
    except Exception as e:
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=500,
            mimetype="application/json"
        )

# Change mode and reset
# Request payload:
#   - mode:             [string]: "Build" or "Discover"
@app.route(f'{api_base_route}/mode', methods=['POST'])
def setMode():
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True

    try:
        # Get the state of the toggle switch. Convert the boolean from Javascript to Python
        mode = request.get_json()['mode']
        if mode!='Build' and mode!= 'Discover':
            raise ValueError(f"Illegal value for mode: exprected 'Build' or 'Discover' but found {mode}.")

        sessions[sid]['building_mode_ON'] = mode == 'Build'
        app.logger.debug(sessions[sid]['building_mode_ON'])

        # Re-initialize the two object instances
        sessions[sid]['urdf_writer'].reset(**urdfwriter_kwargs_dict)
        sessions[sid]['urdf_writer_fromHW'].reset(**urdfwriter_kwargs_dict)

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

# Get details about the current version of the app
@app.route(f'{api_base_route}/info', methods=['GET'])
def getInfo():
    """Get details about the current version of the app"""
    try:
        return Response(
            status=200,
            response=json.dumps({ "backend_version": backend_version,
                                  "frontend_version": frontend_version }),
        )
    except Exception as e:
        # validation failed
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
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True

    query_params = request.args
    try:
        # Get the right writer instance depending on the mode
        writer = get_writer(sid)

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
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True

    query_params = request.args
    try:
        # Get the right writer instance depending on the mode
        writer = get_writer(sid)
        building_mode_ON=get_building_mode_ON(sid)

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
            parent_type = writer.parent_module.type

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
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True

    query_params = request.args
    try:
        # Get the right writer instance depending on the mode
        writer = get_writer(sid)
        building_mode_ON=get_building_mode_ON(sid)

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
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True

    query_params = request.args
    try:
        # Get the right writer instance depending on the mode
        writer = get_writer(sid)
        building_mode_ON=get_building_mode_ON(sid)

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
    
# Validate the offsets provided by the user
def validate_offsets(writer, filename, offsets):
    default_offsets = writer.modular_resources_manager.get_default_offset_values(filename)
    allowed_offsets = writer.modular_resources_manager.get_allowed_offset_values(filename)

    # Analyze provided offsets and fill empty values
    for key in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
        # if empty use default value
        if key not in offsets:
            offsets[key] = default_offsets[key]

        #otherwise the offset value must comply with the definition (when applicable)
        elif key in allowed_offsets and offsets[key] not in allowed_offsets[key]:
            raise ValueError('Offset value for '+key+' is inconsistent with the definition provided in'+filename+'!')
        
    return offsets

@app.route(f'{api_base_route}/model/urdf/modules', methods=['POST'])
def addNewModule():
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True

    req = request.get_json()
    try:
        writer = get_writer(sid)
        building_mode_ON=get_building_mode_ON(sid)

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


        filename = req['name']
        app.logger.debug(filename)

        parent = req['parent'] if 'parent' in req else None
        app.logger.debug(parent)

        # We update the selected module to the one selected in the GUI. If no module is selected, we don't update it, since the BE will keep track of the current parent
        if parent:
            writer.select_module_from_name(parent, None)

        offsets_requested = req['offsets_requested'] if 'offsets_requested' in req else {} # we use RPY notation
        offsets_requested = validate_offsets(writer, filename, offsets_requested)
        app.logger.debug(offsets_requested)

        reverse = True if 'reverse' in req and req['reverse'] == 'true' else False
        app.logger.debug(reverse)

        addons = req['addons'] if 'addons' in req else []
        app.logger.debug(addons)

        module_data = writer.add_module(filename, offsets_requested, reverse, addons)

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

def writeRobotURDF(building_mode_ON, writer, builder_jm):
    data = writer.write_urdf()
    srdf = writer.write_srdf(builder_jm)
    app.logger.debug(srdf)
    if building_mode_ON :
        joint_map = writer.write_joint_map()
        lowlevel_config = writer.write_lowlevel_config()
    else:
        joint_map = writer.write_joint_map(use_robot_id=True)
        lowlevel_config = writer.write_lowlevel_config(use_robot_id=True)
    # probdesc = writer.write_problem_description()
    probdesc = writer.write_problem_description_multi()
    # cartesio_stack = writer.write_cartesio_stack()

    app.logger.debug("\nSRDF\n")
    app.logger.debug(srdf)
    app.logger.debug("\nJoint Map\n")
    app.logger.debug(joint_map)
    # app.logger.debug("\nCartesIO stack\n")
    # app.logger.debug(cartesio_stack)
    return data

# request the urdf generated from the currently stored tree
@app.route(f'{api_base_route}/model/urdf', methods=['GET'])
def getURDF():
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True

    try:
        # Get the right writer instance depending on the mode
        writer = get_writer(sid)
        urdf_string = writer.urdf_string

        # replace path for remote access of STL meshes that will be served with '/meshes/<path:path>' route
        frontend_urdf_string = re.sub(r"(package:/)(/[^/]+)(/.*)", fr"\1{api_base_route}/resources/meshes\3", urdf_string)

        return  Response(
            response=frontend_urdf_string,
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

# upload on the server the /modular_resources folder and the ones under cfg['esternal_resources']
# This is needed to load the meshes of the modules (withot the need to put them in the /static folder)
@app.route(f'{api_base_route}/resources/meshes/<path:path>', methods=['GET'])
def get_resources_meshes_file(path):
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True

    # Get the right writer instance depending on the mode
    writer = get_writer(sid)

    resources_paths = []
    for res_path in writer.resource_finder.resources_paths:
        resources_paths += [writer.resource_finder.find_resource_absolute_path('', res_path)]

    for res_path in resources_paths:
        try:
            return send_from_directory(res_path, path)
        except werkzeug.exceptions.NotFound:
            continue
    abort(404)

# send a request to the poller thread to get ECat topology and synchronize with hardware
@app.route(f'{api_base_route}/model/urdf', methods=['PUT'])
def generateUrdfModelFromHardware():
    if not enable_discovery:
        return Response(
            response=json.dumps({"message": 'Robot discovery is disabled.'}),
            status=409,
            mimetype="application/json"
        )

    srv_name = '/ec_client/get_slaves_description'

    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True
    building_mode_ON = get_building_mode_ON(sid)

    if building_mode_ON:
        return Response(
            response=json.dumps({"message": 'Cannot generate model from connected hardware in Building mode.'}),
            status=409,
            mimetype="application/json"
    )

    try:
        rospy.wait_for_service(srv_name, 5)

        try:
            slave_description = rospy.ServiceProxy(srv_name, GetSlaveInfo) # pylint: disable=undefined-variable

        except rospy.ServiceException as e:
            app.logger.debug("Service call failed: %s",e)
            raise e

        reply = slave_description()
        reply = reply.cmd_info.msg
        app.logger.debug("Exit")

        writer = get_writer(sid)
        writer.read_from_json(reply)
        if writer.verbose:
            writer.render_tree()

        return Response(status=204)

    except Exception as e:
        # validation failed
        app.logger.error(f'{type(e).__name__}: {e}')
        return Response(
            response=json.dumps({"message": f'{type(e).__name__}: {e}'}),
            status=500,
            mimetype="application/json"
        )

def getModulesMap(sid:str):
    chains=[]

    writer = get_writer(sid)
    chains = writer.listofchains

    module_map={}
    for chain in chains:
        for el in chain:
            try:
                module_map[el.name] = ModuleNode.as_dumpable_dict(el.header)
            except AttributeError as e:
                continue
    return module_map

def getJointMap(sid:str):
    chains=[]

    writer = get_writer(sid)
    chains = writer.get_actuated_modules_chains()

    joint_map={}
    for chain in chains:
        for el in chain:
            if el.type in ModuleClass.actuated_modules():
                try:
                    joint_name = writer.get_joint_name(el)
                    joint_data = {
                        'type': el.actuator_data.type,  # revolute, continuos, prismatic, etc.
                        'value': 0.0,  # homing position
                        'min': el.actuator_data.lower_limit,  # lower limit
                        'max': el.actuator_data.upper_limit  # upper limit
                    }
                    joint_map[joint_name] = joint_data
                except AttributeError as e:
                    continue
    return joint_map

# get map of modules of robot: module_name -> module_data (header)
@app.route(f'{api_base_route}/model/urdf/modules/map', methods=['GET'])
def getModelModules():
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True

    try:
        ids = request.args.getlist('ids[]')

        module_map = getModulesMap(sid)

        if len(ids)==0:
            filtered_module_map = module_map  # if no ids are provided, return all modules
        else:
            filtered_module_map = {key: module_map[key] for key in ids}  # filter modules by ids

        return Response(
            response=json.dumps(filtered_module_map),
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

# get map of joints of robot: joint_name -> joint_data (type, value, min, max)
@app.route(f'{api_base_route}/model/urdf/joints/map', methods=['GET'])
def getModelJointMap():
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True

    try:
        joint_map = getJointMap(sid)

        return Response(
            response=json.dumps(joint_map),
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
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True

    writer = get_writer(sid)
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
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True
    building_mode_ON=get_building_mode_ON(sid)

    if not building_mode_ON:
        return Response(
            response=json.dumps({"message": 'Cannot delete modules in Discovery mode.'}),
            status=409,
            mimetype="application/json"
    )

    try:
        # Get the right writer instance depending on the mode
        writer = get_writer(sid)

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
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True

    req = request.get_json()

    # Get the right writer instance depending on the mode
    writer = get_writer(sid)

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

        offsets_requested = req['offsets_requested'] if 'offsets_requested' in req  else {} # we use RPY notation
        app.logger.debug(offsets_requested)

        reverse = True if 'reverse' in req and req['reverse'] == 'true' else False
        app.logger.debug(reverse)

        addons = req['addons'] if 'addons' in req else []

        updated_module_data = writer.update_module(offsets=offsets_requested, reverse=reverse, addons=addons)

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
@app.route(f'{api_base_route}/model/urdf', methods=['POST'])
def deployROSModel():
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True

    try:
        req = request.get_json()
        name = req['name']
        builder_jm =  req['jointMap']

        # Get the right writer instance depending on the mode
        writer = get_writer(sid)
        building_mode_ON=get_building_mode_ON(sid)

        writer.remove_all_connectors() # taken from removeConnectors(), to be removed and itergrated inside .deploy_robot()

        writeRobotURDF(building_mode_ON, writer, builder_jm)

        app.logger.debug(name)
        writer.deploy_robot(name)

        if download_on_deploy:
            return send_file(f'/tmp/{name}.zip', as_attachment=True)
        else:
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
    sid = session.get('session_id') if enable_sessions else 'default'
    if sid not in sessions:
        return 'No session found, refresh the page to start a new one', 404
    sessions[sid]['last_updated'] = datetime.now()
    session.modified = True

    try:
        # Get the right writer instance depending on the mode
        writer = get_writer(sid)

        stats = writer.compute_stats(samples=1000)

        response = dict()
        if  stats['modules']:
            response["modules"]= { "label": 'Modules', "value": str(stats['modules']) }
        if  stats['payload'] and np.isfinite(stats['payload']):
            response["payload"]= { "label": 'Payload', "value":"{:.2f}".format(stats['payload']), "unit": 'Kg' }
        if  stats['max_reach'] and np.isfinite(stats['max_reach']):
            response["max_reach"]= { "label": 'Reach', "value": "{:.2f}".format(stats['max_reach']), "unit": 'm' }
        if  stats['joint_modules']:
            response["joint_modules"]= { "label": 'Joints', "value": str(stats['modules']) }

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
