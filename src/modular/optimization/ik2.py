from cartesian_interface.pyci_all import *
import rbdl
import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
import timeit
from sensor_msgs.msg import JointState

from urdf_parser_py.urdf import URDF
# from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics, kdl_to_mat
import PyKDL as kdl


class cartesio_ik2(object):
    def __init__(self):
        self.ci = pyci.CartesianInterfaceRos()
        self.task_list = self.ci.getTaskList()
        # get 1st task
        self.task = self.task_list[0]
        print self.task_list

        self.ci.setBaseLink(self.task, "ee_B")

        self.ci.setVelocityLimits(self.task, 10.0, 20.0)
        self.ci.setAccelerationLimits(self.task, 50.0, 100.0)
        self.goal_time = 1.0  # 2.0  #

        self.ee_A_poses = []
        self.ee_B_poses = []

        self.target_poses = []
        self.ee_poses = []
        self.q = []
        self.tau = []
        self.J = []
        self.manipulability = []
        self.force_transmission_ratios = []

        self.trasl_error_norm = []
        self.rot_error_norm = []

    def record_poses(self):
        # record global poses
        ee_A_pose = self.ci.getPoseFromTf("/ci/ee_A", "/ci/base_link")
        print ee_A_pose
        self.ee_A_poses.append((ee_A_pose.translation, ee_A_pose.quaternion))
        ee_B_pose = self.ci.getPoseFromTf("/ci/ee_B", "/ci/base_link")
        print ee_B_pose
        self.ee_B_poses.append((ee_B_pose.translation, ee_B_pose.quaternion))

    def reach_pose(self, pose, incremental=False):
        self.ci.setTargetPose(self.task, pose, self.goal_time, incremental)
        # ci.setTargetPose(task, target_pose, goal_time)

        print 'Waiting for task to complete..'
        tic = timeit.default_timer()
        self.ci.waitReachCompleted(self.task)
        toc = timeit.default_timer()
        print 'Task completed'

        print '\nReach completed in ', toc - tic
        print 'Expected time was ', self.goal_time
        print 'Possible difference is due to velocity/acceleration limits!'

        solution_msg = rospy.wait_for_message("/cartesian/solution", JointState, 0.5)
        print 'Solution:' + str(solution_msg.position)

        ee_pose = self.ci.getPoseFromTf("/ci/ee_A", "/ci/ee_B")
        print 'B_pose_A: ', ee_pose

        return ee_pose, solution_msg

    def execute(self):
        ee_pose = self.ci.getPoseFromTf("/ci/ee_A", "/ci/ee_B")
        print ee_pose

        # starting pose

        # target_pose.translation = ee_pose.translation + np.array([0.2, 0.0, -0.2])
        # rotation = Affine3(pos=[0.3, 0.3, -0.1], rot=[0.0, 0.7, 0.0, 0.7])
        # target_pose = ee_pose * rotation
        # print target_pose
        position_incremental = -ee_pose.translation / 2.0
        rot = slerp([0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], np.arange(0, 1, 0.5))
        target_pose_incremental = Affine3(pos=position_incremental, rot=rot[1])  # [0.0, 0.7, 0.0, 0.7]
        # target_pose_incremental = Affine3(pos=[0.0, 0.0, 0.0], rot=[0.0, -1.0, 0.0, 0.0])
        print target_pose_incremental

        B_pose_A, sol_msg = self.reach_pose(target_pose_incremental, True)

        # record global poses
        self.record_poses()

        # 1st waypoint
        target_pose_1 = Affine3(pos=[0.055, 0.055, 0.01], rot=[1.0, 0.0, 0.0, 0.0])
        self.target_poses.append(target_pose_1)
        # 2nd waypoint
        target_pose_2 = Affine3(pos=[0.055, -0.055, 0.01], rot=[1.0, 0.0, 0.0, 0.0])
        self.target_poses.append(target_pose_2)
        # 3rd waypoint
        target_pose_3 = Affine3(pos=[-0.055, -0.055, 0.01], rot=[1.0, 0.0, 0.0, 0.0])
        self.target_poses.append(target_pose_3)
        # 4th waypoint
        target_pose_4 = Affine3(pos=[-0.055, 0.055, 0.01], rot=[1.0, 0.0, 0.0, 0.0])
        self.target_poses.append(target_pose_4)
        # 5th waypoint
        target_pose_5 = Affine3(pos=[0.055, 0.055, 0.01], rot=[1.0, 0.0, 0.0, 0.0])
        self.target_poses.append(target_pose_5)

        for target_pose in self.target_poses:
            B_pose_A, sol_msg = self.reach_pose(target_pose, False)

            self.ee_poses.append(B_pose_A)
            self.q.append(np.array(sol_msg.position))
            self.tau.append(None)
            self.J.append(None)
            self.manipulability.append(None)
            self.force_transmission_ratios.append(None)

            # record global poses
            self.record_poses()

        # pickle_utilities.dump_pickle('ee_A_poses.pkl', ee_A_poses)
        # pickle_utilities.dump_pickle('ee_B_poses.pkl', ee_B_poses)

        success = True
        for i in range(0, len(self.ee_poses)):
            tr_norm, rot_norm = compute_error_norm(self.ee_poses[i], self.target_poses[i])
            self.trasl_error_norm.append(tr_norm)
            self.rot_error_norm.append(rot_norm)

            # Check accuracy of goals reaching
            if tr_norm < 0.001:  # and rot_norm < 0.005:
                model = rbdl.loadModel('/ModularBot/urdf/ModularBot.urdf')
                rbdl.InverseDynamics(model, self.q[i], self.q[i] * 0, self.q[i] * 0, self.tau[i])
                self.J[i], self.manipulability[i], self.force_transmission_ratios[i] = get_jacobian(self.q[i])
            else:
                success = False

        result = {
            'success': success,
            'tr_norm': self.trasl_error_norm,
            'rot_norm': self.rot_error_norm,
            'q': self.q,
            'tau': self.tau,
            'J': self.J,
            'manipulability': self.manipulability,
            'force_ratios': self.force_transmission_ratios
        }

        return result


def get_jacobian(q):
    robot = URDF.from_xml_file('/ModularBot/urdf/ModularBot.urdf')
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

    # Consider the reuced jacobian. Z rotation is not meaningful
    j = J_r[:-1, :]
    jT = j.T
    jjT = np.matmul(j, j.T)
    det = np.linalg.det(jjT)
    manip = np.sqrt(det)

    # Consider the 5-dof reduced vector
    vector = [1, 0, 0, 0, 0]

    # minimising this index correspond to maximize force tranmissibility
    # We want to minimise the one on z direction and maximize the others
    force_transmission_ratios = []
    for i in vector:  # range(0, len(vector)-1):
        u = np.array(vector)
        ft_ratio = u.dot(jjT).dot(u)
        force_transmission_ratios.append(ft_ratio)
        vector.insert(0, vector.pop())

    return J_r, manip, force_transmission_ratios


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
