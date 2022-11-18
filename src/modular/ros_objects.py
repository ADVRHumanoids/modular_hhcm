import time
import rosgraph
import rospy
from xbot_msgs.msg import PingPong
from xbot_msgs.srv import PluginStatus, PluginStatusRequest
from std_msgs.msg import Int32
from signal import SIGINT, SIGTERM
from psutil import Popen, TimeoutExpired
from subprocess import PIPE
from threading import Event
from process_launhers import RunSomething
from utils.utility_functions import clean_ros_nodes
from utils.master_logger import logger
from process_launhers import SwModuleState
from std_srvs.srv import SetBool, SetBoolRequest


class Roscore(RunSomething, object):

    """
    ros core wrapped into a subprocess.
    """

    _initialized = False

    def __init__(self, command_string, process_name, wait_timeout=None, **kwargs):
        RunSomething.__init__(self, command_string, process_name, wait_timeout, **kwargs)

        if Roscore._initialized:
            logger.fail('trying to create two instances o Ros Core')
            raise Exception('double instance of ROS Core')

        if rosgraph.is_master_online():
            logger.warning('ROS Core already running (init)')

        Roscore._initialized = True
        logger.debug('ROS Core object instance created')

    def start(self):

        logger.info('trying to start ROS Core...')

        if RunSomething.start(self) is not SwModuleState.On:
            logger.error('cannot start ROS Core')
            return self.state

        timeout = time.time() + self.wait_timeout

        while not rosgraph.is_master_online():

            if time.time() > timeout:
                logger.error('cannot ping ROS Core after starting')
                self.state = SwModuleState.Error
                return self.state

            logger.warning('ROS Core not ready...')
            time.sleep(1)

        logger.success('ROS Core started!')
        return self.state

    def terminate(self, sig=SIGINT):

        logger.warning('terminating ROS Core...')
        RunSomething.terminate(self, sig)
        return self.state


class RosNode(RunSomething, object):

    def __init__(self, package, node, arg_string='', wait_timeout=None, **kwargs):

        command_string = 'rosrun' + ' ' + package + ' ' + node + ' ' + arg_string
        RunSomething.__init__(self, command_string, wait_timeout, **kwargs)
        self.package = package
        self.node_name = node

    def start(self):

        if self.state is SwModuleState.Error:
            logger.error('ROS node {0} is in error state, cannot start'.format(self.node_name))
            return self.state

        if not rosgraph.is_master_online():
            logger.error('ROS Core offline, cannot start node {0}'.format(self.node_name))
            self.state = SwModuleState.Error
            return self.state

        logger.info('starting ROS node {0}'.format(self.node_name))
        RunSomething.start(self)

        return self.state

    def terminate(self, sig=SIGINT):

        logger.info("try to kill ROS node {0}".format(self.node_name))
        RunSomething.terminate(self, sig)
        clean_ros_nodes()
        return self.state

    def update_status(self):

        if self.state is not SwModuleState.Error:

            node_name = self.node_name.split('.')[0]
            response = Popen(['rosnode', 'ping', '-c', '1', node_name], stdout=PIPE).communicate()[0]
            logger.debug(response)
            self.state = SwModuleState.On if 'time=' in response else SwModuleState.Off
        
        logger.debug('ros node {0} pinged, state {1}'.format(self.node_name, self.state.name))
        return self.state


class RosLauncher(RunSomething, object):

    def __init__(self, package, launcher, arg_string='', wait_timeout=None, **kwargs):

        command_string = 'roslaunch' + ' ' + package + ' ' + launcher + ' ' + arg_string
        RunSomething.__init__(self, command_string, wait_timeout, **kwargs)
        self.package = package
        self.launcher_name = launcher
        self.node_launched_list = []

    def start(self):

        if self.state is SwModuleState.Error:
            logger.error('ROS launcher {0} is in error state, cannot start'.format(self.launcher_name))
            return self.state

        if not rosgraph.is_master_online():
            logger.error('ROS Core offline, cannot start launcher {0}'.format(self.launcher_name))
            self.state = SwModuleState.Error
            return self.state

        logger.info('starting ROS node {0}'.format(self.launcher_name))
        if RunSomething.start(self) is not SwModuleState.On:
            return self.state

        std_output = Popen("roslaunch --nodes {0} {1}".format(self.package, self.launcher_name),
                           shell=True, stdout=PIPE).communicate()[0]

        self.node_launched_list = std_output.replace('/', '')
        self.node_launched_list = self.node_launched_list.split('\n')

        while '' in self.node_launched_list:
            self.node_launched_list.remove('')

        logger.info('nodes launched by launcher {0}: {1}'.format(self.launcher_name, self.node_launched_list))
        logger.info('ROS launcher {0} started!'.format(self.launcher_name))

        return self.state

    def terminate(self, sig=SIGINT):

        logger.info("try to kill ros nodes associated to the launcher {0}".format(self.launcher_name))

        for node in self.node_launched_list:

            logger.info('trying to kill node {0}'.format(node))
            
            if not node:
                continue

            try:
                Popen("rosnode kill {0}".format(node), shell=True, stdout=PIPE).wait(timeout=self.wait_timeout)

            except TimeoutExpired:

                logger.error('timeout in killing node')
                self.state = SwModuleState.Error
                return self.state

            time.sleep(0.5)

        self.state = SwModuleState.Off
        return self.state

    def update_status(self):

        if self.state is SwModuleState.Error:
            logger.warning('ROS launcher {0} is in error state'.format(self.launcher_name))
            return self.state

        nodes_online = []

        for node in self.node_launched_list:

            if not node:
                continue

            response = Popen(['rosnode', 'ping', '-c', '1', node], stdout=PIPE).communicate()[0]
            logger.debug(response)
            nodes_online.append('time=' in response)

        if not nodes_online:
            return self.state

        if not all(x == nodes_online[0] for x in nodes_online):
            self.state = SwModuleState.Error
            return self.state

        self.state = SwModuleState.On if nodes_online[0] else SwModuleState.Off
        logger.debug('launcher pinged, state {0}'.format(self.state.name))
        return self.state


class XBotCore(RunSomething, object):
    """
    xbot core wrapped into a subprocess.
    Singleton implementation prevents from creating more than one instance.
    """
    __initialized = False

    def __init__(self, command_string, process_name, wait_timeout=None, **kwargs):

        if XBotCore.__initialized:
            XBotCore.__state = SwModuleState.Error
            logger.error('cannot create more than one instance of xbot core, exception thrown')
            raise Exception("You can't create more than one instance of XBotCore.")

        if not rosgraph.is_master_online():
            logger.warning('ROS Core is offline')

        RunSomething.__init__(self, command_string, process_name, wait_timeout, **kwargs)
        self.ping_options = {'interval': 0.1, 'timeout': 0.5}
        self.ready = Event()
        self.observers_list = []
        self.ping_publisher = None
        self.pong_subscriber = None
        self.wkc_subscriber = None
        self.node_started = False
        self.ping_timer = None
        XBotCore.__initialized = True

    def start_node(self):

        logger.info('starting xbot_manager node...')
        rospy.init_node('xbot_manager')

        self.pong_subscriber = rospy.Subscriber("/xbotcore/pong", PingPong, self.ping_answer)
        self.ping_publisher = rospy.Publisher('/xbotcore/ping', PingPong, queue_size=10)

        self.node_started = True

        logger.success('xbot_manager node started!')

    def start(self):

        if self.state is SwModuleState.Error:
            logger.error('XBotCore is in error state, cannot start')
            return self.state

        if not self.node_started:
            self.start_node()
        
        if RunSomething.start(self) is not SwModuleState.On:
            logger.warning('something happened when starting XBotCore')
            return self.state
        
        self.state = self.wait_for_xbot_operative(self.wait_timeout)

        if self.state is not SwModuleState.On:
            logger.error('could not start XBot Core')
            return self.state

        logger.info("XBot Core started!")

        self.wkc_subscriber = rospy.Subscriber("/xbotcore/wkc", Int32, self.offline_detection)

        return self.state

    def terminate(self, sig=SIGTERM):

        logger.info("closing XBot Core...")
        RunSomething.terminate(self, sig)
        # clean_ros_nodes()
        return self.state

    def wait_for_xbot_operative(self, timeout):

        self.ready.clear()
        self.ping_timer = rospy.Timer(rospy.Duration(self.ping_options['interval']), self.ping_request, oneshot=False)
        result = self.ready.wait(timeout)
        self.ping_timer.shutdown()

        return SwModuleState.On if result else SwModuleState.Error

    def ping_request(self, tmr):
        
        self.ping_publisher.publish(PingPong())

    def ping_answer(self, answer):

        logger.debug('answer from ping received')
        self.ready.set()

    def offline_detection(self, msg):

        if msg.data == -1:

            logger.warning('modules disconnected, terminating XBot Core session...wkc = {0}'.format(msg.data))
            self.wkc_subscriber.unregister()
            self.terminate()
        '''
        if msg.data != self.expected_wkc:

            logger.warning('wkc anomaly: expected value: {0}, actual value: {1}'.format(self.expected_wkc, msg.data))
        '''


class Homing(RunSomething, object):

    def __init__(self, service_name, homing_time=6, **kwargs):

        wait_timeout = 2
        command_string = 'rosservice' + ' ' + 'call' + ' ' + service_name
        RunSomething.__init__(self, command_string, wait_timeout, **kwargs)
        self.homing_time = homing_time
        self.base_command = command_string

    def wait_for_homing_end(self):

        logger.debug('waiting for homing end')
        status = rospy.ServiceProxy('xbotcore/HomingExample_status', PluginStatus)

        while True:

            try:
                response = status(PluginStatusRequest())

            except (TypeError, rospy.ROSInterruptException, rospy.ServiceException,
                    rospy.ServiceException, rospy.ROSSerializationException) as e:
                logger.error('homing state check failed: {0}'.format(e))
                return False

            description = response.status.split(':')

            state = description[0]
            sub_state = '' if len(description) < 1 else description[1]
            
            print description
            print state
            print sub_state
            print '****************'

            if state != 'RUNNING':
                return False

            if sub_state == 'HOMING_DONE':
                return True

            time.sleep(0.5)

    def call_service(self, value):

        switch = rospy.ServiceProxy('xbotcore/HomingExample_switch', SetBool)

        try:
            switch(SetBoolRequest(value))
            rospy.sleep(0.5)

        except (TypeError, rospy.ROSInterruptException, rospy.ServiceException,
                rospy.ServiceException, rospy.ROSSerializationException) as e:
            logger.error('homing cmd {0} failed: {1}'.format(value, e))
            return False

        logger.debug('homing cmd {0} succeeded'.format(value))

        return True

    def start(self):

        logger.info("starting Homing Plugin...")

        if not self.call_service(True):
            logger.error('error during homing')
            self.state = SwModuleState.Error
            return self.state

        self.wait_for_homing_end()

        if not self.call_service(False):
            logger.error('error during homing closure')
            self.state = SwModuleState.Error
            return self.state

        logger.info("homing completed!")
        self.state = SwModuleState.On

        return self.state

    def terminate(self, sig=SIGINT):

        switch = rospy.ServiceProxy('xbotcore/HomingExample_switch', SetBool)

        try:
            switch(SetBoolRequest(False))
            rospy.sleep(0.5)

        except (TypeError, rospy.ROSInterruptException, rospy.ServiceException,
                rospy.ServiceException, rospy.ROSSerializationException) as e:
            logger.error('homing stop failed: {0}'.format(e))
            self.state = SwModuleState.Error
            return self.state

        self.state = SwModuleState.Off
        return self.state

    def update_status(self):

        if self.state is SwModuleState.Error:
            logger.warning('Homing module is in error state')
            return self.state

        status = rospy.ServiceProxy('xbotcore/HomingExample_status', PluginStatus)

        try:
            response = status(PluginStatusRequest())

        except (TypeError, rospy.ROSInterruptException, rospy.ServiceException,
                rospy.ServiceException, rospy.ROSSerializationException) as e:
            logger.error('homing state check failed: {0}'.format(e))
            self.state = SwModuleState.Off
            return self.state

        description = response.status.split(':')

        self.state = SwModuleState.Off if description[0] != 'RUNNING' else SwModuleState.On

        logger.debug('homing pinged, state {0}'.format(self.state.name))
        return self.state


class Controller(RunSomething, object):

    def __init__(self, service_name, **kwargs):

        wait_timeout = 2
        command_string = 'rosservice' + ' ' + 'call' + ' ' + service_name
        RunSomething.__init__(self, command_string, wait_timeout, **kwargs)
        self.base_command = command_string

    def call_service(self, value):

        switch = rospy.ServiceProxy('xbotcore/CartesianImpedancePlugin_switch', SetBool)

        try:
            switch(SetBoolRequest(value))
            rospy.sleep(0.5)

        except (TypeError, rospy.ROSInterruptException, rospy.ServiceException,
                rospy.ServiceException, rospy.ROSSerializationException) as e:
            logger.error('controller cmd {0} failed: {1}'.format(value, e))
            return False

        logger.debug('controller cmd {0} succeeded'.format(value))

        return True

    def start(self):

        logger.info("starting Controller Plugin...")

        if not self.call_service(True):
            logger.error('error during controller activation')
            self.state = SwModuleState.Error
            return self.state

        logger.info("controller activated!")
        self.state = SwModuleState.On

        return self.state

    def terminate(self, sig=SIGINT):

        logger.warning('controller terminate called')
        self.state = SwModuleState.Off
        return self.state

    def update_status(self):

        if self.state is SwModuleState.Error:
            logger.warning('Controller module is in error state')
            return self.state

        status = rospy.ServiceProxy('xbotcore/CartesianImpedancePlugin_status', PluginStatus)

        try:
            response = status(PluginStatusRequest())

        except (TypeError, rospy.ROSInterruptException, rospy.ServiceException,
                rospy.ServiceException, rospy.ROSSerializationException) as e:
            logger.warning('controller state check failed: {0}'.format(e))
            self.state = SwModuleState.Off
            return self.state

        description = response.status.split(':')

        self.state = SwModuleState.Off if description[0] != 'RUNNING' else SwModuleState.On

        logger.debug('homing pinged, state {0}'.format(self.state.name))
        return self.state
