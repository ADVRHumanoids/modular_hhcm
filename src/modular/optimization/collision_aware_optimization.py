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
import moveit_msgs.msg
import geometry_msgs.msg

from modular.optimization.pickle_utilities import load_pickle, dump_pickle
from modular.optimization.generator import generate_robot_set
#from modular.optimization.ik import cartesio_ik, cartesio_ik2, get_manipulability

from modular.URDF_writer import UrdfWriter

list_of_candidates = []

pickle_path = '/home/tree/pickles'

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


def evaluate_robot(robot, xy, angle_offset):
    # This robot breaks Opensot for some reason. skip it
    if angle_offset == -1.57 and robot == ((0, 1, 0), (0, 1)):
        print 'Skipping'
        return

    # Create joint map to store homing values
    homing_joint_map = {}
    # homing_value = float(builder_joint_map[joint_module.name]['angle'])

    urdf_writer = UrdfWriter(speedup=True)

    # Add a table
    urdf_writer.add_table()

    # Add 1st chain base
    urdf_writer.add_socket(0.0, 0.0, 0.0, 0.0)

    # Add 1st chain
    for module in robot[0]:
        if module:
            data = urdf_writer.add_module('module_joint_double_elbow_ORANGE_B.yaml', 0, False)
        else:
            data = urdf_writer.add_module('module_joint_yaw_ORANGE_B.yaml', 0, False)
        # set an homing value for the joint
        homing_joint_map[data['lastModule_name']] = {'angle': 0.1}

    # Add a simple virtual end-effector
    # urdf_writer.add_simple_ee(0.0, 0.0, 0.189, 0.0)
    # urdf_writer.add_simple_ee(0.0, 0.0, 0.189, 0.0)
    # urdf_writer.select_module_from_name(parent)
    # gripper
    data = urdf_writer.add_module('module_gripper.yaml', 0, False)
    
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
    # data = urdf_writer.add_module('module_gripper.yaml', 0, False)

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

    
    rospy.init_node('moveit_planning', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    # find launch filename
    update_collisions_launch = "/home/tree/moveit_ws/src/pino_moveit/launch/update_collision.launch"
    launch = roslaunch.parent.ROSLaunchParent(uuid, [update_collisions_launch])
    launch.start()
    rospy.loginfo("started")
    launch.spin()

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    # find launch filename
    update_collisions_launch = "/home/tree/moveit_ws/src/pino_moveit/launch/demo.launch"
    launch = roslaunch.parent.ROSLaunchParent(uuid, [update_collisions_launch])
    launch.start()
    rospy.loginfo("started")
    # launch.spin()
    rospy.sleep(10)

    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm_A")
    
    print "============ Reference frame: %s" % group.get_planning_frame()
    print "============ Reference frame: %s" % group.get_end_effector_link()
    print "============ Robot Groups:"
    print robot.get_group_names()
    print "============ Group current Pose:"
    print group.get_current_pose

    waypoints = []

    # start with the current pose
    waypoints.append(group.get_current_pose().pose)

    wpose = group.get_current_pose().pose
    wpose.position.x= 0.753716170788
    wpose.position.y= 0.479139626026
    wpose.position.z= 0.534746289253
    wpose.orientation.x= -0.0496546439826
    wpose.orientation.y= 0.710804343224
    wpose.orientation.z= 0.699813246727
    wpose.orientation.w= 0.050527382642
    waypoints.append(copy.deepcopy(wpose))

    (plan3, fraction) = group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)         # jump_threshold

    print "============ Waiting while RVIZ displays plan3..."

    if fraction >=0.9:
        print("SUCCESS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    launch.spin()

    return


    # if result['success']:
    #     return Candidate(robot, xy, angle_offset, result)
    # else:
    #     return None


def optimize():
    rospy.init_node('cartesIO_client', anonymous=True)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    cartesio_launch = "/home/edoardo/MultiDoF-superbuild/robots/ModularBot_opt/launch/ModularBot_ik.launch"
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [cartesio_launch])
    launch.start()
    rospy.loginfo("started")

    # Instance of UrdfWriter class
    urdf_writer = UrdfWriter(speedup=True)

    required_dofs = 5
    robot_set = generate_robot_set(required_dofs)

    # Create grid
    x = list(numpy.linspace(0.0, 0.6, 4))
    y = list(numpy.linspace(0.0, 0.6, 4))
    coordinates = list(product(x, y))
    coordinates.remove((0.0, 0.0))
    coordinates.remove((0.6, 0.6))
    coordinates = [xy for xy in coordinates if xy[1] <= xy[0]]
    print coordinates
    print len(coordinates)

    for robot in robot_set:  # [((0, 1, 0, 1), (1,)), ((0, 1, 0, 1, 1), ()), ((0, 1, 1), (0, 1))]
        print "Evaluating", robot

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

        angle_offset = 3.14

        # Create joint map to store homing values
        homing_joint_map = {}
        # homing_value = float(builder_joint_map[joint_module.name]['angle'])

        urdf_writer.__init__(speedup=True)  #TODO: implement reset method for urdf_writer

        # Add a table
        urdf_writer.add_table()

        # Add 1st chain base
        urdf_writer.add_socket(0.0, 0.0, 0.0, 0.0)

        # Add 1st chain
        for module in robot[0]:
            if module:
                data = urdf_writer.add_module('module_joint_double_elbow_ORANGE_B.yaml', 0, False)
            else:
                data = urdf_writer.add_module('module_joint_yaw_ORANGE_B.yaml', 0, False)
            # set an homing value for the joint
            homing_joint_map[data['lastModule_name']] = {'angle': 0.1}

        # Add a simple virtual end-effector
        urdf_writer.add_simple_ee(0.0, 0.0, 0.189, 0.0)
        # urdf_writer.select_module_from_name(parent)
        urdf_writer.access_module("table")

        # Add 2nd chain base. We place it at (0.0, 0.0) for now
        urdf_writer.add_socket(0.0, 0.0, 0.0, 0.0)
        # Add 2nd chain
        for module in robot[1]:
            if module:
                data = urdf_writer.add_module('module_joint_double_elbow_ORANGE_B.yaml', 0, False)
            else:
                data = urdf_writer.add_module('module_joint_yaw_ORANGE_B.yaml', 0, False)
            homing_joint_map[data['lastModule_name']] = {'angle': 0.1}

        # Add a simple virtual end-effector
        urdf_writer.add_simple_ee(0.0, 0.0, 0.075, 0.0)
        # Modify position and orientation of the 2nd base
        for xy in [(0.4, 0.0), (0.6, 0.0)]: # coordinates:
            for angle_offset in [3.14]:  # [0.0, 1.57, 3.14, -1.57]
                # This robot breaks Opensot for some reason. skip it
                if angle_offset == -1.57 and robot == ((0, 1, 0), (0, 1)):
                    continue
                urdf_writer.move_socket("L_0_B", xy[0], xy[1], 0.0, angle_offset)
                print "Moving socket!!!!!!!!!!!!", xy[0], xy[1],
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
                result = cartesio_ik(ci)
                if result['success']:
                    c = Candidate(robot, xy, angle_offset, result)
                    list_of_candidates.append(c)

    dump_pickle(pickle_path + time.strftime("%Y%m%d-%H%M%S") + '.pkl', list_of_candidates)
    launch.shutdown()

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
    # optimize()
    # optimize2ndtask()
    #evaluate_robot(((0, 1, 1, 1), (0, 1)), (0.4, 0.4), 3.14)
    evaluate_robot(((0, 1, 0, 1, 0, 1), ()), (0.6, 0.0), 3.14)