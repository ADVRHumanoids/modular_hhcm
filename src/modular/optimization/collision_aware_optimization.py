from itertools import product
import numpy
import rospy
import roslaunch
import rosnode
#from cartesian_interface.pyci_all import *
from std_msgs.msg import Empty
import time
import numpy as np

import sys
import copy
import rospy
import moveit_commander
from moveit_commander.exception import MoveItCommanderException
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib_msgs.msg
import tf2_msgs.msg 
import sensor_msgs.msg


from trac_ik_python.trac_ik import IK
from numpy.random import random
import time

from modular.optimization.pickle_utilities import load_pickle, dump_pickle
from modular.optimization.generator import generate_robot_set
# from modular.optimization.ik import cartesio_ik, cartesio_ik2, get_manipulability, compute_error_norm

from modular.URDF_writer import UrdfWriter

# from modular.ros_objects import RosNode

list_of_candidates = []

pickle_path = '/home/tree/pickles/'

class Solution(object):
    def __init__(self, sol_robot, sol_xy, sol_angle_offset, sol_pose_1, sol_pose_2, sol_q, sol_tau):
        self.robot = sol_robot
        self.xy = sol_xy
        self.angle_offset = sol_angle_offset
        self.pose_1 = sol_pose_1
        self.pose_2 = sol_pose_2
        self.q = sol_q
        self.tau = sol_tau

class Candidate(object):
    def __init__(self, sol_robot, sol_xy, sol_angle_offset, result_dict):
        self.robot = sol_robot
        self.xy = sol_xy
        self.angle_offset = sol_angle_offset
        self.result = result_dict
        # self.err_trasl_1 = err_trasl_1
        # self.err_rot_1 = err_rot_1
        # self.err_trasl_2 = err_trasl_2
        # self.err_rot_2 = err_rot_2
        # self.q1 = sol_q1
        # self.q2 = sol_q2
        # self.tau1 = sol_tau1
        # self.tau2 = sol_tau2
        # self.J1 = sol_J1
        # self.J2 = sol_J2

    # def print(self):
    #     print "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
    #     print self.robot
    #     print self.xy
    #     print self.err_trasl_1, self.err_rot_1, self.err_trasl_2, self.err_rot_2
    #     print self.tau1, self.tau2
    #     print self.q1, self.q2

def evaluate(candidates):
    tau_list = []
    dist_list = []
    manip_list = []
    ratios_list = []
    for c in candidates:
        skip = False
        for chain in c.robot:
            for i in range(len(chain)):
                try:
                    if chain[i] == 0 and chain[i] == chain[i + 1]:
                        skip = True
                        break
                except IndexError:
                    pass
            else:
                continue  # only executed i[f the inner loop did NOT break
            break  # only executed if the inner loop DID break

        print "%%%%%%%%%%%%%%%%%%%%%%%%%%"
        print "Robot:", c.robot
        print "X-Y 2nd base position:", c.xy
        print "Angle offset:", c.angle_offset
        print "Trasl. err. norm. 1:", c.result['tr_norm_1']
        print "Rot. err. norm. 1:", c.result['rot_norm_1']
        print "Trasl. err. norm. 2:", c.result['tr_norm_2']
        print "Rot. err. norm. 2:", c.result['rot_norm_2']
        if not skip:
            tau_norm = np.linalg.norm(c.result['tau_1']) + np.linalg.norm(c.result['tau_2'])
            distance = np.linalg.norm(c.xy)
            m1, fr1 = get_manipulability(c.result['J1'], 1)
            m2, fr2 = get_manipulability(c.result['J2'], 1)
            manipulability = [m1, m2]
            force_ratios = [fr1, fr2]
        else:
            tau_norm = 1000
            distance = 1000
            manipulability = [0.0, 0.0]
            force_ratios = [[], []]

        setattr(c, "tau_norm", tau_norm)
        setattr(c, "distance", distance)
        setattr(c, "manipulability", manipulability)
        setattr(c, "force_ratios", force_ratios)

        tau_list.append(tau_norm)
        dist_list.append(distance)
        manip_list.append(manipulability)
        ratios_list.append(force_ratios)
        print "Tau norm: ", tau_norm

    return tau_list, dist_list, manip_list, ratios_list


def filter_manipulability(candidates, manip_list):
    # consider nan as zero
    nan = 0.0
    idx_to_rmv = []
    # for c in candidates:
    #     if c.result['manip1'] < 0.02 or c.result['manip1'] < 0.02:
    #         idx_to_rmv.append(c)
    # for el in idx_to_rmv:
    #     candidates.remove(el)
    for i in xrange(len(manip_list)):
        if manip_list[i][0] < 0.02 or manip_list[i][1] < 0.02:
            idx_to_rmv.append(i)

    idx_to_rmv.reverse()
    return idx_to_rmv


def evaluate_robot(robot_structure, coordinates, angle_offsets):
    # Create joint map to store homing values
    homing_joint_map = {}
    # homing_value = float(builder_joint_map[joint_module.name]['angle'])

    urdf_writer = UrdfWriter(speedup=True)

    # Add a table
    urdf_writer.add_table()

    # Add 1st chain base
    # urdf_writer.add_socket(0.150, 0.225, 0.0, 0.0)
    urdf_writer.add_socket(0.0, 0.0, 0.0, 0.0)

    # Add 1st chain
    for module in robot_structure[0]:
        if module == 1:
            data = urdf_writer.add_module('module_joint_double_elbow_ORANGE_B.yaml', 0, False)
            homing_joint_map[data['lastModule_name']] = {'angle': 0.0}
        elif module == 0:
            data = urdf_writer.add_module('module_joint_yaw_ORANGE_B.yaml', 0, False)
            # set an homing value for the joint
            homing_joint_map[data['lastModule_name']] = {'angle': 0.0}
        elif module == 2:
            data = urdf_writer.add_module('module_link_elbow_90_B.yaml', 0, False)
        elif module == 3:
            data = urdf_writer.add_module('module_link_straight_140_B.yaml', 0, False)

    # Add a simple virtual end-effector
    urdf_writer.add_simple_ee(0.0, 0.0, 0.189, 0.0)
    # urdf_writer.select_module_from_name(parent)
    # gripper
    # data = urdf_writer.add_module('module_gripper_B.yaml', 0, False)
    
    # urdf_writer.access_module("table")

    # # Add 2nd chain base. We place it at (0.0, 0.0) for now
    # urdf_writer.add_socket(0.0, 0.0, 0.0, 0.0)
    # # Add 2nd chain
    # for module in robot[1]:
    #     if module:
    #         data = urdf_writer.add_module('module_joint_double_elbow_ORANGE_B.yaml', 0, False)
    #     else:
    #         data = urdf_writer.add_module('module_joint_yaw_ORANGE_B.yaml', 0, False)
    #     homing_joint_map[data['lastModule_name']] = {'angle': 0.1}

    # # Add a simple virtual end-effector
    # # urdf_writer.add_simple_ee(0.0, 0.0, 0.075, 0.0)
    # # gripper
    # data = urdf_writer.add_module('module_gripper_B.yaml', 0, False)

    # # Modify position and orientation of the 2nd base
    # urdf_writer.move_socket("L_0_B", xy[0], xy[1], 0.0, angle_offset)
    # print "Moving socket!!!!!!!!!!!!", xy[0], xy[1]

    # string = urdf_writer.process_urdf()

    urdf = urdf_writer.write_urdf()
    srdf = urdf_writer.write_srdf(homing_joint_map)
    joint_map = urdf_writer.write_joint_map()
    lowlevel_config = urdf_writer.write_lowlevel_config()
    # probdesc = urdf_writer.write_hardcodedproblem_description("ee_B", "ee_A")

    urdf_writer.deploy_robot("pino_moveit")

    # rosnode.kill_nodes(['/moveit_client'])
    # rospy.init_node('moveit_client', anonymous=True)
    
    # Run MoveIt collision updater
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    # find launch filename
    update_collisions_launch = "/home/tree/moveit_ws/src/pino_moveit/launch/update_collision.launch"
    launch = roslaunch.parent.ROSLaunchParent(uuid, [update_collisions_launch])
    launch.start()
    rospy.loginfo("started")
    launch.spin()

    # TODO: To be fixed
    urdf_path =  '/home/tree/moveit_ws/src/pino_moveit/urdf/ModularBot.urdf'
    srdf_path = '/home/tree/moveit_ws/src/pino_moveit/srdf/ModularBot_collisions_disabled.srdf'
    robot_description = open(urdf_path).read()
    robot_description_semantic = open(srdf_path).read()
    rospy.set_param("robot_description", robot_description)
    rospy.set_param("robot_description_semantic", robot_description_semantic)

    # rosnode.kill_nodes(['/joint_state_desired_publisher'])
    # rosnode.kill_nodes(['/robot_state_publisher'])

    # # Launch MoveIt simulation
    # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)
    # # find launch filename
    # demo_launch = "/home/tree/moveit_ws/src/pino_moveit/launch/demo.launch"
    # launch = roslaunch.parent.ROSLaunchParent(uuid, [demo_launch])
    # launch.start()
    # rospy.loginfo("started")
    # # launch.spin()
    # # rospy.sleep(2.0)
    # rospy.wait_for_message("/move_group/status", actionlib_msgs.msg.GoalStatusArray)

    # print "============ Starting tutorial setup"
    # moveit_commander.roscpp_initialize(sys.argv)
    # plane_pose = geometry_msgs.msg.PoseStamped()
    # plane_pose.header.frame_id = robot.get_planning_frame()
    # plane_pose.pose.orientation.x = 1.0
    # plane_pose.pose.orientation.w = 1.0
    # plane_pose.pose.position.y = -0.01
    # scene.add_plane('vertical_plane', plane_pose)       

    # Modify position and orientation of the 2nd base
    for xy in coordinates: # [(0.4, 0.0), (0.6, 0.0)]:
        for angle_offset in angle_offsets:  # [0.0, 1.57, 3.14, -1.57]
            # This robot breaks Opensot for some reason. skip it
            if angle_offset == -1.57 and robot_structure == ((0, 1, 0), (0, 1)):
                continue
            
            urdf_writer.move_socket("L_0_A", xy[0], xy[1], 0.0, angle_offset)
            print("Moving socket!!!!!!!!!!!!", xy[0], xy[1])
            urdf = urdf_writer.write_urdf()
            #We need to deploy because of recent changes to the deploy logic. TODO: fix this
            urdf_writer.deploy_robot("pino_moveit")
            robot_description = open(urdf_path).read()
            rospy.set_param("robot_description", robot_description)
            
            rosnode.kill_nodes(['/joint_state_publisher'])  # Node will be re-spawned
            rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)
            # joint_state_desired_publisher = RosNode(package='topic_tools',
            #         node='relay',
            #         arg_string='args="joint_states joint_states_desired"',
            #         wait_timeout=1.0) #,
                    # shell=shell,
                    # stdin=std_io, stdout=std_io, stderr=std_io,
                    # preexec_fn=os.setsid)
            rosnode.kill_nodes(['/joint_state_desired_publisher'])  # Node will be re-spawned
            rospy.wait_for_message("/joint_states_desired", sensor_msgs.msg.JointState)
            rosnode.kill_nodes(['/robot_state_publisher'])  # Node will be re-spawned 
            rospy.wait_for_message("/tf", tf2_msgs.msg.TFMessage)
            
            # Instance of RobotCommander and MoveGroupCommander
            robot = moveit_commander.RobotCommander()
            scene = moveit_commander.PlanningSceneInterface()
            group_A = moveit_commander.MoveGroupCommander("arm_A")
            rospy.sleep(0.5)

            print("============ Reference frame: %s" % group_A.get_planning_frame())
            print("============ Reference frame: %s" % group_A.get_end_effector_link())
            print("============ Robot Groups:")
            print(robot.get_group_names())
            print("============ Group current Pose:")
            print(group_A.get_current_pose())

            group_A.set_max_velocity_scaling_factor(1.0)
            group_A.set_max_acceleration_scaling_factor(1.0)

            result = run_mission(robot, group_A)

            del robot, scene, group_A

            if result is not None:
                list_of_candidates.append(Candidate(robot_structure, xy, angle_offset, result))

    return


def run_mission(robot, group_A):
    # Trac IK solver
    ik_solver = IK("base_link",
                   "ee_A")

    print("IK solver uses link chain:")
    print(ik_solver.link_names)
    print("IK solver base frame:")
    print(ik_solver.base_link)
    print("IK solver tip link:")
    print(ik_solver.tip_link)
    print("IK solver for joints:")
    print(ik_solver.joint_names)
    print("IK solver using joint limits:")
    lb, up = ik_solver.get_joint_limits()
    print("Lower bound: " + str(lb))
    print("Upper bound: " + str(up))

    # q_init = group_A.get_current_joint_values()
    q_init = [0.3] * ik_solver.number_of_joints
    evaluated_poses = []
    planned_trajectories = []
    success = True
    
    ########################
    # 1st pre-insertion pose
    p1_pre = geometry_msgs.msg.PoseStamped()
    p1_pre.header.frame_id = group_A.get_pose_reference_frame()
    p1_pre.pose.position.x = 0.45
    p1_pre.pose.position.y = 0.6
    p1_pre.pose.position.z = 0.1
    p1_pre.pose.orientation.x = p1_pre.pose.orientation.w = 0.0
    p1_pre.pose.orientation.y = p1_pre.pose.orientation.z = 0.707
    
    # Set tolerance bounds!
    bx = by = bz = 0.001
    bry = brz = 0.05
    brx = 9999
    bounds = [bx, by, bz, brx, bry, brz]

    # Solve Ik for p1_pre
    q1_pre = trac_ik_solve(p1_pre.pose, bounds, q_init, ik_solver)
    if q1_pre is None:
        success = False
        # launch.shutdown()
        # moveit_commander.roscpp_shutdown()
        return None
    else:
        # Move to q1_pre
        q1_pre_traj = moveit_trajectory_plan(q1_pre, group_A)
        p1_pre_actual = group_A.get_current_pose()
        evaluated_poses.append((p1_pre_actual, p1_pre))

        # 1st insertion pose
        p1_ins = copy.deepcopy(p1_pre)
        p1_ins.pose.position.y += 0.1

        # Solve Ik for p1_ins
        q1_ins = trac_ik_solve(p1_ins.pose, bounds, q1_pre, ik_solver)
        if q1_ins is None:
            success = False

        # Move to q1_ins
        q1_ins_traj = moveit_trajectory_plan(q1_ins, group_A)
        p1_ins_actual = group_A.get_current_pose()
        evaluated_poses.append((p1_ins_actual, p1_ins))

        # Come back to to q1_pre
        q1_back_traj = moveit_trajectory_plan(q1_pre, group_A)

        # Store trajectories
        planned_trajectories.append((q1_pre_traj,q1_ins_traj,q1_back_traj))

    ###############################
    # 2nd pre-insertion pose
    p2_pre = copy.deepcopy(p1_pre)
    p2_pre.pose.position.x += 0.12

    # Solve Ik for p2_pre
    q2_pre = trac_ik_solve(p2_pre.pose, bounds, q1_pre, ik_solver)
    if q2_pre is None:
        success = False
        # launch.shutdown()
        # moveit_commander.roscpp_shutdown()
        return None
    else:
        # Move to q2_pre
        q2_pre_traj = moveit_trajectory_plan(q2_pre, group_A)
        p2_pre_actual = group_A.get_current_pose()
        evaluated_poses.append((p2_pre_actual, p2_pre))

        # 2nd insertion pose
        p2_ins = copy.deepcopy(p2_pre)
        p2_ins.pose.position.y += 0.1

        # Solve Ik for p2_ins
        q2_ins = trac_ik_solve(p2_ins.pose, bounds, q2_pre, ik_solver)
        if q2_ins is None:
            success = False

        # Move to q2_ins
        q2_ins_traj = moveit_trajectory_plan(q2_ins, group_A)
        p2_ins_actual = group_A.get_current_pose()
        evaluated_poses.append((p2_ins_actual, p2_ins))

        # Come back to to q2_pre
        q2_back_traj = moveit_trajectory_plan(q2_pre, group_A)

        # Store trajectories
        planned_trajectories.append((q2_pre_traj,q2_ins_traj,q2_back_traj))

    ###############################
    # 3rd pre-insertion pose
    p3_pre = copy.deepcopy(p2_pre)
    p3_pre.pose.position.z += 0.12

    # Solve Ik for p3_pre
    q3_pre = trac_ik_solve(p3_pre.pose, bounds, q2_pre, ik_solver)
    if q3_pre is None:
        success = False
        # launch.shutdown()
        # moveit_commander.roscpp_shutdown()
        return None
    else:
        # Move to q3_pre
        q3_pre_traj = moveit_trajectory_plan(q3_pre, group_A)
        p3_pre_actual = group_A.get_current_pose()
        evaluated_poses.append((p3_pre_actual, p3_pre))

        # 3rd insertion pose
        p3_ins = copy.deepcopy(p3_pre)
        p3_ins.pose.position.y += 0.1

        # Solve Ik for p3_ins
        q3_ins = trac_ik_solve(p3_ins.pose, bounds, q3_pre, ik_solver)
        if q3_ins is None:
            success = False

        # Move to q1_ins
        q3_ins_traj = moveit_trajectory_plan(q3_ins, group_A)
        p3_ins_actual = group_A.get_current_pose()
        evaluated_poses.append((p3_ins_actual, p3_ins))

        # Come back to to q1_pre
        q3_back_traj = moveit_trajectory_plan(q3_pre, group_A)

        # Store trajectories
        planned_trajectories.append((q3_pre_traj,q3_ins_traj,q3_back_traj))

    ###############################
    # 4th pre-insertion pose
    p4_pre = copy.deepcopy(p3_pre)
    p4_pre.pose.position.x -= 0.12

    # Solve Ik for p4_pre
    q4_pre = trac_ik_solve(p4_pre.pose, bounds, q3_pre, ik_solver)
    if q4_pre is None:
        success = False
        # launch.shutdown()
        # moveit_commander.roscpp_shutdown()
        return None
    else:
        # Move to q4_pre
        q4_pre_traj = moveit_trajectory_plan(q4_pre, group_A)
        p4_pre_actual = group_A.get_current_pose()
        evaluated_poses.append((p4_pre_actual, p4_pre))

        # 4th insertion pose
        p4_ins = copy.deepcopy(p4_pre)
        p4_ins.pose.position.y += 0.1

        # Solve Ik for p4_ins
        q4_ins = trac_ik_solve(p4_ins.pose, bounds, q4_pre, ik_solver)
        if q4_ins is None:
            success = False

        # Move to q1_ins
        q4_ins_traj = moveit_trajectory_plan(q4_ins, group_A)
        p4_ins_actual = group_A.get_current_pose()
        evaluated_poses.append((p4_ins_actual, p4_ins))

        # Come back to to q1_pre
        q4_back_traj = moveit_trajectory_plan(q4_pre, group_A)

        # Store trajectories
        planned_trajectories.append((q4_pre_traj,q4_ins_traj,q4_back_traj))

    # # compute norms
    # norms = [compute_error_norm(p[0], p[1]) for p in evaluated_poses]
    # # transpose it using zip
    # tr_norms, rot_norms = zip(*norms)

    result = {
        'success': success,
        'poses': evaluated_poses,
        'trajectories': planned_trajectories
        # 'tr_norm': tr_norms,
        # 'rot_norm': rot_norms,
        # 'q': [q_1, q_2, q_3, q_4, q_5],
        # 'tau': [tau_1, tau_2, tau_3, tau_4, tau_5],
        # 'J': J,
        # 'manip': m,
        # 'force_ratios': fr
    }

    return result


def trac_ik_solve(pose, bounds, qinit, ik_solver):
    x =  pose.position.x
    y = pose.position.y
    z = pose.position.z
    rx = pose.orientation.x
    ry = pose.orientation.y
    rz = pose.orientation.z
    rw = pose.orientation.w
    bx = bounds[0]
    by = bounds[1]
    bz = bounds[2]
    brx = bounds[3]
    bry = bounds[4]
    brz = bounds[5]
    q = None
    for attempt in range(3):
        q = ik_solver.get_ik(qinit,
                               x, y, z,
                                rx, ry, rz, rw,
                                bx, by, bz,
                                brx, bry, brz)
        if q is not None:
            break
    else:
        print("IK failed to find a solution for {}".format(([x,y,z], [rx,ry,rz,rw])))
        
    return q


def moveit_trajectory_plan(q_goal, group):
    try:
        group.set_joint_value_target(q_goal)
        plan = group.plan()
        group.execute(plan)
        return plan  # trajectory_msgs.msg._JointTrajectory.JointTrajectory()
    except MoveItCommanderException as exc:
        print("MoveIt failed with the following exception: {}".format(exc))
        print("Exiting!")
        return None


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


def optimize():
    rosnode.kill_nodes(['/moveit_client'])
    rospy.init_node('moveit_client', anonymous=True)
    
    # # Run MoveIt collision updater
    # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)
    # # find launch filename
    # update_collisions_launch = "/home/tree/moveit_ws/src/pino_moveit/launch/update_collision.launch"
    # launch = roslaunch.parent.ROSLaunchParent(uuid, [update_collisions_launch])
    # launch.start()
    # rospy.loginfo("started")
    # launch.spin()

    # Launch MoveIt simulation
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    # find launch filename
    demo_launch = "/home/tree/moveit_ws/src/pino_moveit/launch/demo.launch"
    launch = roslaunch.parent.ROSLaunchParent(uuid, [demo_launch])
    launch.start()
    rospy.loginfo("started")
    # launch.spin()
    # rospy.sleep(2.0)
    rospy.wait_for_message("/move_group/status", actionlib_msgs.msg.GoalStatusArray)

    print("============ Starting tutorial setup")
    moveit_commander.roscpp_initialize(sys.argv)

    # Instance of UrdfWriter class
    # urdf_writer = UrdfWriter(speedup=True)

    required_dofs = 5
    robot_set = generate_robot_set(required_dofs + 1)
    # second_set = generate_robot_set(required_dofs + 2)
    # robot_set = first_set.union(second_set)

    # Create grid
    x = list(numpy.linspace(0.0, 1.4, 8))
    y = list(numpy.linspace(0.0, 0.6, 4))
    coordinates = list(product(x, y))
    # coordinates.remove((0.0, 0.0))
    # coordinates.remove((0.6, 0.6))
    # coordinates = [xy for xy in coordinates if xy[1] <= xy[0]]
    print(coordinates)
    print(len(coordinates))
    angle_offsets = [0.0]#, 1.57, -1.57, 3.14]
    coordinates = [(0.2, 0.2)]

    for robot in robot_set:  # [((1, 0, 1, 0, 1), ()), ((0, 1, 1, 1, 0), ())]
        print("Evaluating", robot)
        # if not robot[1]:
        #     print "Skipping 1-arm robot"
        #     continue

        # Skip robots with two consecutives Yaw modules
        skip = False
        for chain in robot:
            for i in range(len(chain)):
                try:
                    if chain[i] == 0 and chain[i] == chain[i+1]:
                        skip = True
                        break
                except IndexError:
                    pass
            else:
                continue  # only executed if the inner loop did NOT break
            break  # only executed if the inner loop DID break
        if skip:
            print "Skipping robot with two consecutive Yaw joints"
            continue  # Skip to next robot

        # 1st chain should be the one with higher number of modules (optional)
        if len(robot[1]) > len(robot[0]):
            robot = (robot[1], robot[0])


        # Modify position and orientation of the 2nd base
        # for xy in coordinates: # [(0.4, 0.0), (0.6, 0.0)]:
        #     for angle_offset in [0.0, 1.57]:  # [0.0, 1.57, 3.14, -1.57]
        #         # This robot breaks Opensot for some reason. skip it
        #         if angle_offset == -1.57 and robot == ((0, 1, 0), (0, 1)):
        #             continue
                # urdf_writer.move_socket("L_0_A", xy[0], xy[1], 0.0, angle_offset)
                # print("Moving socket!!!!!!!!!!!!", xy[0], xy[1])

        evaluate_robot(robot, coordinates, angle_offsets)
                # string = urdf_writer.process_urdf()

                # urdf = urdf_writer.write_urdf()
                # srdf = urdf_writer.write_srdf(homing_joint_map)
                # # joint_map = urdf_writer.write_joint_map()
                # # lowlevel_config = urdf_writer.write_lowlevel_config()
                # # probdesc = urdf_writer.write_problem_description_dual_arm()

                # # data = urdf_writer.deploy_robot("ModularBot_opt")

                # rospy.set_param("robot_description", urdf)
                # rospy.set_param("robot_description_semantic", srdf)
                # rosnode.kill_nodes(['/ros_server_node'])  # Node will be re-spawned
                # rospy.wait_for_message("/cartesian/heartbeat", Empty)
                # # # Instance of Cartesian Interface
                # ci = pyci.CartesianInterfaceRos()
                # result = cartesio_ik(ci)
                # if result['success']:
                #     c = Candidate(robot, xy, angle_offset, result)
                #     list_of_candidates.append(c)

    launch.shutdown()
    moveit_commander.roscpp_shutdown()

    dump_pickle(pickle_path + time.strftime("%Y%m%d-%H%M%S") + '.pkl', list_of_candidates)
    # launch.shutdown()
    return

def optimize2ndtask():
    rospy.init_node('cartesIO_client', anonymous=True)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    cartesio_launch = "/home/edoardo/MultiDoF-superbuild/robots/ModularBot_opt/launch/ModularBot_ik.launch"
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [cartesio_launch])
    launch.start()
    rospy.loginfo("started")

    # Instance of UrdfWriter class
    urdf_writer = UrdfWriter(speedup=True)

    first_task_solutions = load_pickle(pickle_path + '/20201028-011803_peginhole.pkl')

    taus, dists, manips, ratios = evaluate(first_task_solutions)
    idx_to_rmv = filter_manipulability(first_task_solutions, manips)
    for idx in idx_to_rmv:
        del first_task_solutions[idx]
        del taus[idx]
        del dists[idx]
        del manips[idx]
        del ratios[idx]
    print'Number of candidates: ', len(first_task_solutions)

    for sol in first_task_solutions:
        print "Evaluating", sol.robot, sol.xy, sol.angle_offset

        if not sol.robot[1]:
            print "Skipping 1-arm robot"
            continue

        # Create joint map to store homing values
        homing_joint_map = {}
        # homing_value = float(builder_joint_map[joint_module.name]['angle'])

        urdf_writer.__init__(speedup=True)  #TODO: implement reset method for urdf_writer

        # Add a table
        urdf_writer.add_table()

        # Add 1st chain base
        urdf_writer.add_socket(0.0, 0.0, 0.0, 0.0)

        # Add 1st chain
        for module in sol.robot[0]:
            if module:
                data = urdf_writer.add_module('module_joint_double_elbow_ORANGE_B.yaml', 0, False)
            else:
                data = urdf_writer.add_module('module_joint_yaw_ORANGE_B.yaml', 0, False)
            # set an homing value for the joint
            homing_joint_map[data['lastModule_name']] = {'angle': 0.1}

        # Add a simple virtual end-effector
        urdf_writer.add_simple_ee(0.0, 0.0, 0.189, 0.0)
        # urdf_writer.select_mo   dule_from_name(parent)
        urdf_writer.access_module("table")

        # Add 2nd chain base. We place it at (0.0, 0.0) for now
        urdf_writer.add_socket(0.0, 0.0, 0.0, 0.0)
        # Add 2nd chain
        for module in sol.robot[1]:
            if module:
                data = urdf_writer.add_module('module_joint_double_elbow_ORANGE_B.yaml', 0, False)
            else:
                data = urdf_writer.add_module('module_joint_yaw_ORANGE_B.yaml', 0, False)
            homing_joint_map[data['lastModule_name']] = {'angle': 0.1}

        # Add a simple virtual end-effector
        urdf_writer.add_simple_ee(0.0, 0.0, 0.075, 0.0)

        # This robot breaks Opensot for some reason. skip it
        if sol.angle_offset == -1.57 and sol.robot == ((0, 1, 0), (0, 1)):
            continue
        urdf_writer.move_socket("L_0_B", sol.xy[0], sol.xy[1], 0.0, sol.angle_offset)
        print "Moving socket!!!!!!!!!!!!", sol.xy[0], sol.xy[1]
        string = urdf_writer.process_urdf()

        urdf = urdf_writer.write_urdf()
        srdf = urdf_writer.write_srdf(homing_joint_map)
        # joint_map = urdf_writer.write_joint_map()
        # lowlevel_config = urdf_writer.write_lowlevel_config()
        # probdesc = urdf_writer.write_problem_description_dual_arm()

        # data = urdf_writer.deploy_robot("ModularBot_opt")

        rospy.set_param("robot_description", urdf)
        rospy.set_param("robot_description_semantic", srdf)
        rosnode.kill_nodes(['/ros_server_node'])  # Node will be re-spawned
        rospy.wait_for_message("/cartesian/heartbeat", Empty)
        # # Instance of Cartesian Interface
        ci = pyci.CartesianInterfaceRos()
        result = cartesio_ik2(ci)
        if result['success']:
            c = Candidate(sol.robot, sol.xy, sol.angle_offset, result)
            list_of_candidates.append(c)

    dump_pickle(pickle_path + time.strftime("%Y%m%d-%H%M%S") + '.pkl', list_of_candidates)
    launch.shutdown()


if __name__ == "__main__":
    optimize()
    # optimize2ndtask()
    # evaluate_robot(((1, 0, 1, 0, 1), ()), (0.0, 0.0), 1.57)
    # evaluate_robot(((0, 1, 1, 1, 0), ()), (0.150, 0.225), 3.14)