from cartesian_interface.pyci_all import *
import pickle_utilities
import rospy
import rbdl
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import threading
import time
import tf
import math

import rospy
import roslaunch
import timeit
from sensor_msgs.msg import JointState

from urdf_parser_py.urdf import URDF
# from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics, kdl_to_mat
import PyKDL as kdl

pickle_path = '/home/edoardo/Documents/Papers/ICRA2021_task_based_optimization/pickles'

def tf_subscriber(ci, caller_id=""):
    poses = []
    while True:
        poses.append(ci.getPoseFromTf("/ci/ee_A", "/ci/ee_B"))
        rospy.sleep(0.01)

    return poses


# def listen(ci, poses):
#     t = threading.currentThread()
#     while getattr(t, "do_run", True):
#         poses.append(ci.getPoseFromTf("/ci/ee_A", "/ci/ee_B"))
#         time.sleep(0.01)
#     print("Stopping as you wish.")

def listen(ci, poses):
    t = threading.currentThread()
    listener = tf.TransformListener()
    rate = rospy.Rate(100.0)
    while getattr(t, "do_run", True):
        print "Listening"
        try:
            (trans, rot) = listener.lookupTransform("/ci/ee_A", "/ci/ee_B", rospy.Time(0))
            poses.append(((trans, rot)))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
    print("Stopping as you wish.")

def playback():
    ci = pyci.CartesianInterfaceRos()

    goal_time = 4.0
    ee_A_waypoints = []
    ee_B_waypoints = []

    task_list = ci.getTaskList()
    print task_list
    # get 1st task
    task_A = task_list[0]
    task_B = task_list[1]
    print task_A, task_B

    poses_tuple_list = pickle_utilities.load_pickle(pickle_path + '/ee_A_poses_5DOF.pkl')
    for p in poses_tuple_list:
        target_pose = Affine3(pos=p[0], rot=p[1])
        # w = pyci.WayPoint()
        # w.frame = target_pose
        # w.time = 4.0
        ee_A_waypoints.append(target_pose)

    poses_tuple_list = pickle_utilities.load_pickle(pickle_path + '/ee_B_poses_5DOF.pkl')
    for p in poses_tuple_list:
        target_pose = Affine3(pos=p[0], rot=p[1])
        # w = pyci.WayPoint()
        # w.frame = target_pose
        # w.time = 4.0
        ee_B_waypoints.append(target_pose)

    print ee_A_waypoints, ee_B_waypoints
    # ci.setWaypoints(task_A, ee_A_waypoints, False)
    # ci.setWaypoints(task_B, ee_B_waypoints, False)
    ee_A_waypoints[0].translation += [0.05, 0.05, 0.05]
    ci.setTargetPose(task_A, ee_A_waypoints[0], goal_time, False)
    ci.setTargetPose(task_B, ee_B_waypoints[0], goal_time, False)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task_A)
    toc = timeit.default_timer()
    print 'Task completed'


    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task_B)
    toc = timeit.default_timer()
    print 'Task completed'

    ci.setTargetPose(task_A, ee_A_waypoints[1], goal_time, False)
    ci.setTargetPose(task_B, ee_B_waypoints[1], goal_time, False)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task_A)
    toc = timeit.default_timer()
    print 'Task completed'


    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task_B)
    toc = timeit.default_timer()
    print 'Task completed'

    ci.setTargetPose(task_A, ee_A_waypoints[2], goal_time, False)
    ci.setTargetPose(task_B, ee_B_waypoints[2], goal_time, False)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task_A)
    toc = timeit.default_timer()
    print 'Task completed'


    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task_B)
    toc = timeit.default_timer()
    print 'Task completed'

    ci.setTargetPose(task_A, ee_A_waypoints[3], goal_time, False)
    ci.setTargetPose(task_B, ee_B_waypoints[3], goal_time, False)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task_A)
    toc = timeit.default_timer()
    print 'Task completed'

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task_B)
    toc = timeit.default_timer()
    print 'Task completed'

    ci.setTargetPose(task_A, ee_A_waypoints[4], goal_time, False)
    ci.setTargetPose(task_B, ee_B_waypoints[4], goal_time, False)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task_A)
    toc = timeit.default_timer()
    print 'Task completed'

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task_B)
    toc = timeit.default_timer()
    print 'Task completed'

    ci.setTargetPose(task_A, ee_A_waypoints[5], goal_time, False)
    ci.setTargetPose(task_B, ee_B_waypoints[5], goal_time, False)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task_A)
    toc = timeit.default_timer()
    print 'Task completed'

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task_B)
    toc = timeit.default_timer()
    print 'Task completed'


def cartesio_ik2(ci):
    task_list = ci.getTaskList()
    # get 1st task
    task = task_list[0]
    print task_list

    ci.setBaseLink(task, "ee_B")
    # ci.setBaseLink(task, "L_2_B")
    ee_pose = ci.getPoseFromTf("/ci/ee_A", "/ci/ee_B")
    # ee_pose = ci.getPoseFromTf("/ci/L_3_A", "/ci/L_2_B")
    print ee_pose

    ci.setVelocityLimits(task, 10.0, 20.0)
    ci.setAccelerationLimits(task, 50.0, 100.0)
    goal_time = 4.0  # 2.0  #

    position_incremental = -ee_pose.translation / 2.0  #  4.0*3.0 - [0.1, 0.1, 0.0]
    rot = slerp([0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], np.arange(0, 1, 0.5))
    target_pose_incremental = Affine3(pos=position_incremental, rot=rot[1])  # [0.0, 0.7, 0.0, 0.7]
    # target_pose_incremental = Affine3(pos=[0.0, 0.0, 0.0], rot=[0.0, -1.0, 0.0, 0.0])
    print target_pose_incremental

    ci.setTargetPose(task, target_pose_incremental, goal_time, True)
    # ci.setTargetPose(task, target_pose, goal_time)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task)
    toc = timeit.default_timer()
    print 'Task completed'

    print '\nReach completed in ', toc - tic
    print 'Expected time was ', goal_time
    print 'Possible difference is due to velocity/acceleration limits!'

    solution_msg = rospy.wait_for_message("/cartesian/solution", JointState, 0.5)
    print 'Solution:' + str(solution_msg.position)

    ee_pose = ci.getPoseFromTf("/ci/ee_A", "/ci/ee_B")
    print ee_pose

    # record global poses
    ee_A_poses = []
    ee_A_pose = ci.getPoseFromTf("/ci/ee_A", "/ci/base_link")
    print ee_A_pose
    ee_A_poses.append((ee_A_pose.translation, ee_A_pose.quaternion))
    ee_B_poses = []
    ee_B_pose = ci.getPoseFromTf("/ci/ee_B", "/ci/base_link")
    print ee_B_pose
    ee_B_poses.append((ee_B_pose.translation, ee_B_pose.quaternion))

    # waypoint 1
    target_pose_1 = Affine3(pos=[0.055, 0.055, 0.002], rot=[1.0, 0.0, 0.0, 0.0])
    # target_pose_1 = Affine3(pos=[0.0, 0.0, 0.05], rot=[0.995, 0.0, 0.1, 0.0])

    ci.setTargetPose(task, target_pose_1, goal_time)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task)
    toc = timeit.default_timer()
    print 'Task completed'

    print '\nReach completed in ', toc - tic
    print 'Expected time was ', goal_time
    print 'Possible difference is due to velocity/acceleration limits!'

    solution_msg = rospy.wait_for_message("/cartesian/solution", JointState, 0.5)
    print 'Solution:' + str(solution_msg.position)
    q_1 = np.array(solution_msg.position)
    tau_1 = q_1 * 0

    ee_pose_1 = ci.getPoseFromTf("/ci/ee_A", "/ci/ee_B")
    print ee_pose_1

    # record global poses
    ee_A_pose = ci.getPoseFromTf("/ci/ee_A", "/ci/base_link")
    print ee_A_pose
    ee_A_poses.append((ee_A_pose.translation, ee_A_pose.quaternion))
    ee_B_pose = ci.getPoseFromTf("/ci/ee_B", "/ci/base_link")
    print ee_B_pose
    ee_B_poses.append((ee_B_pose.translation, ee_B_pose.quaternion))

    # waypoint 2
    target_pose_2 = Affine3(pos=[0.055, -0.055, 0.002], rot=[1.0, 0.0, 0.0, 0.0])
    # target_pose_1 = Affine3(pos=[0.0, 0.0, 0.05], rot=[0.995, 0.0, 0.1, 0.0])

    ci.setTargetPose(task, target_pose_2, goal_time)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task)
    toc = timeit.default_timer()
    print 'Task completed'

    print '\nReach completed in ', toc - tic
    print 'Expected time was ', goal_time
    print 'Possible difference is due to velocity/acceleration limits!'

    solution_msg = rospy.wait_for_message("/cartesian/solution", JointState, 0.5)
    print 'Solution:' + str(solution_msg.position)
    q_2 = np.array(solution_msg.position)
    tau_2 = q_2 * 0

    ee_pose_2 = ci.getPoseFromTf("/ci/ee_A", "/ci/ee_B")
    print ee_pose_2

    # record global poses
    ee_A_pose = ci.getPoseFromTf("/ci/ee_A", "/ci/base_link")
    print ee_A_pose
    ee_A_poses.append((ee_A_pose.translation, ee_A_pose.quaternion))
    ee_B_pose = ci.getPoseFromTf("/ci/ee_B", "/ci/base_link")
    print ee_B_pose
    ee_B_poses.append((ee_B_pose.translation, ee_B_pose.quaternion))

    # waypoint 3
    target_pose_3 = Affine3(pos=[-0.055, -0.055, 0.002], rot=[1.0, 0.0, 0.0, 0.0])
    # target_pose_1 = Affine3(pos=[0.0, 0.0, 0.05], rot=[0.995, 0.0, 0.1, 0.0])

    ci.setTargetPose(task, target_pose_3, goal_time)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task)
    toc = timeit.default_timer()
    print 'Task completed'

    print '\nReach completed in ', toc - tic
    print 'Expected time was ', goal_time
    print 'Possible difference is due to velocity/acceleration limits!'

    solution_msg = rospy.wait_for_message("/cartesian/solution", JointState, 0.5)
    print 'Solution:' + str(solution_msg.position)
    q_3 = np.array(solution_msg.position)
    tau_3 = q_3 * 0

    ee_pose_3 = ci.getPoseFromTf("/ci/ee_A", "/ci/ee_B")
    print ee_pose_3

    # record global poses
    ee_A_pose = ci.getPoseFromTf("/ci/ee_A", "/ci/base_link")
    print ee_A_pose
    ee_A_poses.append((ee_A_pose.translation, ee_A_pose.quaternion))
    ee_B_pose = ci.getPoseFromTf("/ci/ee_B", "/ci/base_link")
    print ee_B_pose
    ee_B_poses.append((ee_B_pose.translation, ee_B_pose.quaternion))

    # waypoint 4
    target_pose_4 = Affine3(pos=[-0.055, 0.055, 0.002], rot=[1.0, 0.0, 0.0, 0.0])
    # target_pose_1 = Affine3(pos=[0.0, 0.0, 0.05], rot=[0.995, 0.0, 0.1, 0.0])

    ci.setTargetPose(task, target_pose_4, goal_time)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task)
    toc = timeit.default_timer()
    print 'Task completed'

    print '\nReach completed in ', toc - tic
    print 'Expected time was ', goal_time
    print 'Possible difference is due to velocity/acceleration limits!'

    solution_msg = rospy.wait_for_message("/cartesian/solution", JointState, 0.5)
    print 'Solution:' + str(solution_msg.position)
    q_4 = np.array(solution_msg.position)
    tau_4 = q_4 * 0

    ee_pose_4 = ci.getPoseFromTf("/ci/ee_A", "/ci/ee_B")
    print ee_pose_4

    # record global poses
    ee_A_pose = ci.getPoseFromTf("/ci/ee_A", "/ci/base_link")
    print ee_A_pose
    ee_A_poses.append((ee_A_pose.translation, ee_A_pose.quaternion))
    ee_B_pose = ci.getPoseFromTf("/ci/ee_B", "/ci/base_link")
    print ee_B_pose
    ee_B_poses.append((ee_B_pose.translation, ee_B_pose.quaternion))

    # waypoint 5
    target_pose_5 = Affine3(pos=[0.055, 0.055, 0.002], rot=[1.0, 0.0, 0.0, 0.0])
    # target_pose_1 = Affine3(pos=[0.0, 0.0, 0.05], rot=[0.995, 0.0, 0.1, 0.0])

    ci.setTargetPose(task, target_pose_5, goal_time)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task)
    toc = timeit.default_timer()
    print 'Task completed'

    print '\nReach completed in ', toc - tic
    print 'Expected time was ', goal_time
    print 'Possible difference is due to velocity/acceleration limits!'

    solution_msg = rospy.wait_for_message("/cartesian/solution", JointState, 0.5)
    print 'Solution:' + str(solution_msg.position)
    q_5 = np.array(solution_msg.position)
    tau_5 = q_5 * 0

    ee_pose_5 = ci.getPoseFromTf("/ci/ee_A", "/ci/ee_B")
    print ee_pose_5

    # record global poses
    ee_A_pose = ci.getPoseFromTf("/ci/ee_A", "/ci/base_link")
    print ee_A_pose
    ee_A_poses.append((ee_A_pose.translation, ee_A_pose.quaternion))
    ee_B_pose = ci.getPoseFromTf("/ci/ee_B", "/ci/base_link")
    print ee_B_pose
    ee_B_poses.append((ee_B_pose.translation, ee_B_pose.quaternion))

    pickle_utilities.dump_pickle(pickle_path + 'ee_A_poses_2nd_bis.pkl', ee_A_poses)
    pickle_utilities.dump_pickle(pickle_path + 'ee_B_poses_2nd_bis.pkl', ee_B_poses)

    # Compute error norm. Only translation , no rotation!
    tr_norm_1, rot_norm_1 = compute_error_norm(ee_pose_1, target_pose_1)
    tr_norm_2, rot_norm_2 = compute_error_norm(ee_pose_2, target_pose_2)
    tr_norm_3, rot_norm_3 = compute_error_norm(ee_pose_3, target_pose_3)
    tr_norm_4, rot_norm_4 = compute_error_norm(ee_pose_4, target_pose_4)
    tr_norm_5, rot_norm_5 = compute_error_norm(ee_pose_5, target_pose_5)
    tr_norms = [tr_norm_1, tr_norm_2, tr_norm_3, tr_norm_4, tr_norm_5]
    rot_norms = [rot_norm_1, rot_norm_2, rot_norm_3, rot_norm_4, rot_norm_5]

    J = [None] * len(tr_norms)
    m = [None] * len(tr_norms)
    fr = [None] * len(tr_norms)

    # Check accuracy of goals reaching
    if all(norm < 0.001 for norm in tr_norms) and all(norm < 0.75 for norm in rot_norms):
        success = True
        model = rbdl.loadModel('/home/edoardo/MultiDoF-superbuild/external/modular/ModularBot/urdf/ModularBot.urdf')
        rbdl.InverseDynamics(model, q_1, q_1 * 0, q_1 * 0, tau_1)
        rbdl.InverseDynamics(model, q_2, q_2 * 0, q_2 * 0, tau_2)
        rbdl.InverseDynamics(model, q_3, q_3 * 0, q_3 * 0, tau_3)
        rbdl.InverseDynamics(model, q_4, q_4 * 0, q_4 * 0, tau_4)
        rbdl.InverseDynamics(model, q_5, q_5 * 0, q_5 * 0, tau_5)
        J[0], m[0], fr[0] = get_jacobian(q_1, 3)
        J[1], m[1], fr[1] = get_jacobian(q_2, 3)
        J[2], m[2], fr[2] = get_jacobian(q_3, 3)
        J[3], m[3], fr[3] = get_jacobian(q_4, 3)
        J[4], m[4], fr[4] = get_jacobian(q_5, 3)
        for manip in m:
            if manip < 0.02 or math.isnan(manip):
                success = False
    else:
        success = False

    result = {
        'success': success,
        'tr_norm': tr_norms,
        'q': [q_1, q_2, q_3, q_4, q_5],
        'tau': [tau_1, tau_2, tau_3, tau_4, tau_5],
        'J': J,
        'manip': m,
        'force_ratios': fr
    }

    return result


def cartesio_ik(ci):
    # # initialize ROS node
    # rospy.init_node('cartesIO_client', anonymous=True)
    #
    # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # cartesio_launch = "/home/edoardo/MultiDoF-superbuild/robots/ModularBot_opt/launch/ModularBot_ik.launch"
    # roslaunch.configure_logging(uuid)
    # launch = roslaunch.parent.ROSLaunchParent(uuid, [cartesio_launch])
    # launch.start()
    # rospy.loginfo("started")
    #
    # ci = pyci.CartesianInterfaceRos()
    task_list = ci.getTaskList()
    # get 1st task
    task = task_list[0]
    print task_list

    ci.setBaseLink(task, "ee_B")
    # ci.setBaseLink(task, "L_2_B")
    ee_pose = ci.getPoseFromTf("/ci/ee_A", "/ci/ee_B")
    # ee_pose = ci.getPoseFromTf("/ci/L_3_A", "/ci/L_2_B")
    print ee_pose

    ci.setVelocityLimits(task, 10.0, 20.0)
    ci.setAccelerationLimits(task, 50.0, 100.0)
    goal_time = 1.0  #  2.0  #

    # launch tf listener
    # listened_poses = []
    # package = 'simple_package'
    # executable = 'tf_listener_node.py'
    # node = roslaunch.core.Node(package, executable)
    # launch = roslaunch.scriptapi.ROSLaunch()
    # launch.start()
    # process = launch.launch(node)
    # print process.is_alive()

    # t = threading.Thread(target=listen, args=(ci, listened_poses,))
    # t.start()

    # waypoint

    # target_pose.translation = ee_pose.translation + np.array([0.2, 0.0, -0.2])
    # rotation = Affine3(pos=[0.3, 0.3, -0.1], rot=[0.0, 0.7, 0.0, 0.7])
    # target_pose = ee_pose * rotation
    # print target_pose
    position_incremental = -ee_pose.translation/4.0 * 3.0 - [0.1, 0.1, 0.0]
    rot = slerp([0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], np.arange(0, 1, 0.25))
    target_pose_incremental = Affine3(pos=position_incremental, rot=rot[1])  # [0.0, 0.7, 0.0, 0.7]
    # target_pose_incremental = Affine3(pos=[0.0, 0.0, 0.0], rot=[0.0, -1.0, 0.0, 0.0])
    print target_pose_incremental

    ci.setTargetPose(task, target_pose_incremental, goal_time, True)
    # ci.setTargetPose(task, target_pose, goal_time)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task)
    toc = timeit.default_timer()
    print 'Task completed'

    print '\nReach completed in ', toc - tic
    print 'Expected time was ', goal_time
    print 'Possible difference is due to velocity/acceleration limits!'

    solution_msg = rospy.wait_for_message("/cartesian/solution", JointState, 0.5)
    print 'Solution:' + str(solution_msg.position)

    ee_pose = ci.getPoseFromTf("/ci/ee_A", "/ci/ee_B")
    print ee_pose

    # record global poses
    ee_A_poses = []
    ee_A_pose = ci.getPoseFromTf("/ci/ee_A", "/ci/base_link")
    print ee_A_pose
    ee_A_poses.append((ee_A_pose.translation, ee_A_pose.quaternion))
    ee_B_poses = []
    ee_B_pose = ci.getPoseFromTf("/ci/ee_B", "/ci/base_link")
    print ee_B_pose
    ee_B_poses.append((ee_B_pose.translation, ee_B_pose.quaternion))

    # pre-insertion
    target_pose_1 = Affine3(pos=[0.0, 0.0, 0.05], rot=[1.0, 0.0, 0.0, 0.0])
    # target_pose_1 = Affine3(pos=[0.0, 0.0, 0.05], rot=[0.995, 0.0, 0.1, 0.0])

    ci.setTargetPose(task, target_pose_1, goal_time)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task)
    toc = timeit.default_timer()
    print 'Task completed'

    print '\nReach completed in ', toc - tic
    print 'Expected time was ', goal_time
    print 'Possible difference is due to velocity/acceleration limits!'

    solution_msg = rospy.wait_for_message("/cartesian/solution", JointState, 0.5)
    print 'Solution:' + str(solution_msg.position)
    q_1 = np.array(solution_msg.position)
    tau_1 = q_1 * 0

    ee_pose_1 = ci.getPoseFromTf("/ci/ee_A", "/ci/ee_B")
    print ee_pose_1

    # record global poses
    ee_A_pose = ci.getPoseFromTf("/ci/ee_A", "/ci/base_link")
    print ee_A_pose
    ee_A_poses.append((ee_A_pose.translation, ee_A_pose.quaternion))
    ee_B_pose = ci.getPoseFromTf("/ci/ee_B", "/ci/base_link")
    print ee_B_pose
    ee_B_poses.append((ee_B_pose.translation, ee_B_pose.quaternion))

    # final goal
    target_pose_2 = Affine3(pos=[0.0, 0.0, -0.05], rot=[1.0, 0.0, 0.0, 0.0])

    ci.setTargetPose(task, target_pose_2, goal_time)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted(task)
    toc = timeit.default_timer()
    print 'Task completed'

    print '\nReach completed in ', toc - tic
    print 'Expected time was ', goal_time
    print 'Possible difference is due to velocity/acceleration limits!'

    solution_msg = rospy.wait_for_message("/cartesian/solution", JointState, 0.5)
    print 'Solution:' + str(solution_msg.position)
    q_2 = np.array(solution_msg.position)
    tau_2 = q_2 * 0

    ee_pose_2 = ci.getPoseFromTf("/ci/ee_A", "/ci/ee_B")
    print ee_pose_2

    # record global poses
    ee_A_pose = ci.getPoseFromTf("/ci/ee_A", "/ci/base_link")
    print ee_A_pose
    ee_A_poses.append((ee_A_pose.translation, ee_A_pose.quaternion))
    ee_B_pose = ci.getPoseFromTf("/ci/ee_B", "/ci/base_link")
    print ee_B_pose
    ee_B_poses.append((ee_B_pose.translation, ee_B_pose.quaternion))

    # stop listener thread
    # t.do_run = False
    # t.join()
    # process.stop()
    # print "Poses size: ", len(listened_poses)

    pickle_utilities.dump_pickle(pickle_path + 'ee_A_poses_5DOF.pkl', ee_A_poses)
    pickle_utilities.dump_pickle(pickle_path + 'ee_B_poses_5DOF.pkl', ee_B_poses)

    tr_norm_1, rot_norm_1 = compute_error_norm(ee_pose_1, target_pose_1)
    tr_norm_2, rot_norm_2 = compute_error_norm(ee_pose_2, target_pose_2)

    J1 = None
    J2 = None
    m1 = None
    m2 = None
    fr1 = None
    fr2 = None

    # Check accuracy of goals reaching
    if tr_norm_1 < 0.001 and tr_norm_2 < 0.001 and rot_norm_1 < 0.005 and rot_norm_2 < 0.005:
        success = True
        model = rbdl.loadModel('/home/edoardo/MultiDoF-superbuild/external/modular/ModularBot/urdf/ModularBot.urdf')
        rbdl.InverseDynamics(model, q_1, q_1 * 0, q_1 * 0, tau_1)
        rbdl.InverseDynamics(model, q_2, q_2 * 0, q_2 * 0, tau_2)
        J1, m1, fr1 = get_jacobian(q_1, 1)
        J2, m2, fr2 = get_jacobian(q_2, 1)
    else:
        success = False

    result = {
        'success': success,
        'tr_norm_1': tr_norm_1,
        'rot_norm_1': rot_norm_1,
        'tr_norm_2': tr_norm_2,
        'rot_norm_2': rot_norm_2,
        'q_1': q_1,
        'q_2': q_2,
        'tau_1': tau_1,
        'tau_2': tau_2,
        'J1': J1,
        'J2': J2,
        'manip1': m1,
        'manip2': m2,
        'force_ratios1': fr1,
        'force_ratios2': fr2
    }

    return result


def get_jacobian(q, indexes_to_reduce):
    robot = URDF.from_xml_file('/home/edoardo/MultiDoF-superbuild/external/modular/ModularBot/urdf/ModularBot.urdf')
    kdl_kin_A = KDLKinematics(robot, 'base_link', 'ee_A')
    kdl_kin_B = KDLKinematics(robot, 'base_link', 'ee_B')

    n = len(q)
    n_A = len(kdl_kin_A.get_joint_names())
    n_B = len(kdl_kin_B.get_joint_names())
    print 'n', n, ', n_A: ', n_A, ', n_B: ', n_B
    q_A = q[:n_A]
    q_B = q[n_A:]

    w_T_A = kdl_kin_A.forward(q_A)
    w_T_B = kdl_kin_B.forward(q_B)

    # Compute the vector origin_target-origin_base w.r.t. world
    w_p_A = w_T_A[:3, -1]
    w_p_B = w_T_B[:3, -1]
    B_p_A = w_p_A - w_p_B

    # Compute orientation of base link w.r.t world
    w_R_B = w_T_B[:3, :3]

    # Compute base e target link jacobians
    J_A = kdl_kin_A.jacobian(q_A)
    J_B = kdl_kin_B.jacobian(q_B, B_p_A)  # change reference point of the Jacobian

    J_r = J_A
    J_r.resize(n)

    # Compute relative jacobian in worldframe.
    # TODO: this is valid only if there is no common joint between the two chains
    i = 0
    while i < n_B:
        J_r.setColumn(n_A + i, - J_B.getColumn(i))
        i += 1

    w_R_B_kdl = kdl.Rotation(w_R_B[0, 0], w_R_B[0, 1], w_R_B[0, 2],
                             w_R_B[1, 0], w_R_B[1, 1], w_R_B[1, 2],
                             w_R_B[2, 0], w_R_B[2, 1], w_R_B[2, 2])

    # Rotate to base frame
    J_r.changeBase(w_R_B_kdl.Inverse())

    J_r = kdl_to_mat(J_r)

    manip, force_transmission_ratios = get_manipulability(J_r, indexes_to_reduce)

    return J_r, manip, force_transmission_ratios


def get_manipulability(J_r, indexes_to_reduce):
    # Consider the reuced jacobian.
    # Z rotation is not meaningful: indexes_to_reduce = 1
    # Rotation not meaningful: indexes_to_reduce = 3
    j = J_r[:-indexes_to_reduce, :]
    jT = j.T
    jjT = np.matmul(j, j.T)
    det = np.linalg.det(jjT)
    manip = np.sqrt(det)

    # Cartesian vector
    vector = [1, 0, 0, 0, 0, 0]
    # Consider the (indexes_to_reduce)-dof reduced vector
    vector = vector[:-indexes_to_reduce]

    # minimising this index correspond to maximize force tranmissibility
    # We want to minimise the one on z direction and maximize the others
    force_transmission_ratios = []
    for i in vector:  # range(0, len(vector)-1):
        u = np.array(vector)
        ft_ratio = u.dot(jjT).dot(u)
        force_transmission_ratios.append(np.asscalar(ft_ratio))
        vector.insert(0, vector.pop())

    return manip, force_transmission_ratios


# TODO: add indexes to specify which direction are not relevant for erro computation
def compute_error_norm(ee_pose, target_pose):
    # Check norms
    tr_norm = np.linalg.norm(ee_pose.translation - target_pose.translation)
    goal_rot = R.from_quat(target_pose.quaternion)
    goal_rot_inv = goal_rot.inv()
    r = R.from_quat(ee_pose.quaternion)
    e_r = goal_rot_inv * r
    alpha = e_r.as_rotvec()
    rot_norm = np.linalg.norm(alpha[:2])

    return tr_norm, rot_norm


def slerp(v0, v1, t_array):
    """Spherical linear interpolation."""
    # >>> slerp([1,0,0,0], [0,0,0,1], np.arange(0, 1, 0.001))
    t_array = np.array(t_array)
    v0 = np.array(v0)
    v1 = np.array(v1)
    dot = np.sum(v0 * v1)

    if dot < 0.0:
        v1 = -v1
        dot = -dot

    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        result = v0[np.newaxis, :] + t_array[:, np.newaxis] * (v1 - v0)[np.newaxis, :]
        return (result.T / np.linalg.norm(result, axis=1)).T

    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)

    theta = theta_0 * t_array
    sin_theta = np.sin(theta)

    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0[:, np.newaxis] * v0[np.newaxis, :]) + (s1[:, np.newaxis] * v1[np.newaxis, :])


def manual_ik():
    model = rbdl.loadModel('/home/edoardo/tree_ws/src/ModularBot/urdf/ModularBot.urdf')

    print(len(model.mBodies))
    i = 0
    while i < len(model.mBodies):
        print(model.GetBodyName(i))
        i += 1

    last_id = len(model.mBodies) - 1
    print(model.GetBodyName(last_id))

    last_body = model.GetBody(last_id)
    print last_body

    n = len(model.mBodies) - 1
    q = np.zeros(n)
    offset = np.array([0., 0.131999958147, 0.0720448848742]) # use transform for distal link [0., 0.131999958147, 0.0720448848742]
    R = rbdl.CalcBodyWorldOrientation(model, q, last_id)
    print R, np.linalg.det(R)
    offset_body = np.matmul(np.transpose(R),offset)
    print offset_body
    G = np.zeros((6,n))
    rbdl.CalcPointJacobian6D(model, q, last_id, offset_body, G)
    G[[0,1,2,3,4,5], :] = G[[3,4,5,0,1,2], :]
    print G
    J_trans = G[[0, 1, 2]]
    J_rot = G[[3, 4, 5]]

    ee_pose = rbdl.CalcBaseToBodyCoordinates(model, q, last_id, offset_body)
    print ee_pose

    target_pose = ee_pose + np.array([0.2, -0.2, 0.0])
    print target_pose

    error_pose = target_pose - ee_pose
    print error_pose

    new_q = q

    for i in range(400):
        R = rbdl.CalcBodyWorldOrientation(model, new_q, last_id)
        offset_body = np.matmul(np.transpose(R), offset)
        rbdl.CalcPointJacobian6D(model, new_q, last_id, offset_body, G)
        G[[0, 1, 2, 3, 4, 5], :] = G[[3, 4, 5, 0, 1, 2], :]
        J_trans = G[[0, 1, 2]]
        J_rot = G[[3, 4, 5]]

        ee_pose = rbdl.CalcBaseToBodyCoordinates(model, new_q, last_id, offset_body)
        error_pose = target_pose - ee_pose

        new_direction = np.zeros(n)
        for k in range(3):
            gradient = J_trans[[k]].reshape((J_trans.shape[1],))
            new_direction += 0.2 * error_pose[k] * gradient
            # print new_direction

        new_q += new_direction
        # print new_q
        # print ee_pose
        plt.scatter(i, new_q[0], color='red')
        plt.scatter(i, new_q[1], color='blue')
        plt.scatter(i, new_q[2], color='green')
        plt.scatter(i, new_q[3], color='yellow')
        plt.pause(0.001)

    # plt.grid(True)
    plt.show()
    # print new_q
    print ee_pose, target_pose

if __name__ == "__main__":
    # manual_ik()
    # cartesio_ik()
    playback()

    # target_pose_incremental = Affine3(pos=[-0.4, 0.0, -0.1], rot=[0.0, 0.7, 0.0, 0.7])
    # # rotation = Affine3(pos=[0.3, 0.0, -0.1], rot=[0.0, 0.7, 0.0, 0.7])
    # # target_pose_2 = target_pose * rotation
    #
    # ci.setTargetPose(task, target_pose_incremental, goal_time, True)
    #
    # print 'Waiting for task to complete..'
    # tic = timeit.default_timer()
    # ci.waitReachCompleted(task)
    # toc = timeit.default_timer()
    # print 'Task completed'
    #
    # print '\nReach completed in ', toc - tic
    # print 'Expected time was ', goal_time
    # print 'Possible difference is due to velocity/acceleration limits!'
    #
    # solution_msg = rospy.wait_for_message("/cartesian/solution", JointState, 0.5)
    # print 'Solution:' + str(solution_msg.position)
