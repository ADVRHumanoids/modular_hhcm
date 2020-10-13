from cartesian_interface.pyci_all import *
import rospy
import rbdl
import numpy
import matplotlib.pyplot as plt

import rospy
import roslaunch
import timeit
from sensor_msgs.msg import JointState

def cartesio_ik():
    # initialize ROS node
    rospy.init_node('cartesIO_client', anonymous=True)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    cartesio_launch = "/home/edoardo/MultiDoF-superbuild/robots/ModularBot/launch/ModularBot_ik.launch"
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [cartesio_launch])
    launch.start()
    rospy.loginfo("started")

    ci = pyci.CartesianInterfaceRos()

    # ee_name = model.GetBodyName(last_id)
    ee_pose = ci.getPoseFromTf("/ci/L_4_A", "/ci/world")
    target_pose = ee_pose.copy()
    target_pose.translation = ee_pose.translation + numpy.array([0.2, 0.0, -0.2])
    print ee_pose
    print target_pose
    target_pose_incremental = Affine3(pos=[0.2, 0.0, -0.4], rot=[0.0, .0, 0.7, 0.7])

    task_list = ci.getTaskList()
    # get 1st task
    task = task_list[0]
    print task

    ci.setVelocityLimits(task, 10.0, 20.0)

    ci.setAccelerationLimits(task, 50.0, 100.0)
    goal_time = 0.2
    ci.setTargetPose(task, target_pose_incremental, goal_time, True)

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

    rospy.sleep(5000)
    print 'Exiting...'


def manual_ik():
    model = rbdl.loadModel('/home/edoardo/tree_ws/src/ModularBot/urdf/ModularBot.urdf')

    print(len(model.mBodies))
    i = 0
    while i < len(model.mBodies):
        print(model.GetBodyName(i))
        i += 1

    last_id = len(model.mBodies) - 1
    print(model.GetBodyName(last_id))

    last_body = model.GetBody(last_id);
    print last_body

    n = len(model.mBodies) - 1
    q = numpy.zeros(n)
    offset = numpy.array([0., 0.131999958147, 0.0720448848742]) # use transform for distal link [0., 0.131999958147, 0.0720448848742]
    R = rbdl.CalcBodyWorldOrientation(model, q, last_id)
    print R, numpy.linalg.det(R)
    offset_body = numpy.matmul(numpy.transpose(R),offset)
    print offset_body
    G = numpy.zeros((6,n))
    rbdl.CalcPointJacobian6D(model, q, last_id, offset_body, G)
    G[[0,1,2,3,4,5], :] = G[[3,4,5,0,1,2], :]
    print G
    J_trans = G[[0, 1, 2]]
    J_rot = G[[3, 4, 5]]

    ee_pose = rbdl.CalcBaseToBodyCoordinates(model, q, last_id, offset_body)
    print ee_pose

    target_pose = ee_pose + numpy.array([0.2, -0.2, 0.0])
    print target_pose

    error_pose = target_pose - ee_pose
    print error_pose

    new_q = q

    for i in range(400):
        R = rbdl.CalcBodyWorldOrientation(model, new_q, last_id)
        offset_body = numpy.matmul(numpy.transpose(R), offset)
        rbdl.CalcPointJacobian6D(model, new_q, last_id, offset_body, G)
        G[[0, 1, 2, 3, 4, 5], :] = G[[3, 4, 5, 0, 1, 2], :]
        J_trans = G[[0, 1, 2]]
        J_rot = G[[3, 4, 5]]

        ee_pose = rbdl.CalcBaseToBodyCoordinates(model, new_q, last_id, offset_body)
        error_pose = target_pose - ee_pose

        new_direction = numpy.zeros(n)
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
    cartesio_ik()