# !/usr/bin/env python
from cartesian_interface.pyci_all import *

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

import timeit


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    ci = pyci.CartesianInterfaceRos()

    target_pose = Affine3(pos=[0.3, 0.4, 0], rot=[1.0, .0, .0, .0])

    ci.setVelocityLimits('L_6_A', 1000.0, 1000.0)

    ci.setAccelerationLimits('L_6_A', 1000.0, 1000.0)

    # w1 = pyci.WayPoint()
    # w1.frame = Affine3(pos=[0.3, 0.3, 0], rot=[1.0, .0, .0, .0])
    # # w1.frame.linear = pose.linear.copy()
    # w1.time = 1.0
    #
    # w2 = pyci.WayPoint()
    # w2.frame = Affine3(pos=[0.3, 0.6, 0], rot=[1.0, .0, .0, .0])
    # # w1.frame.linear = pose.linear.copy()
    # w2.time = 2.0
    #
    # w3 = pyci.WayPoint()
    # w3.frame = Affine3(pos=[-0.3, 0.6, 0], rot=[1.0, .0, .0, .0])
    # # w1.frame.linear = pose.linear.copy()
    # w3.time = 3.0
    #
    # w4 = pyci.WayPoint()
    # w4.frame = Affine3(pos=[-0.3, 0.3, 0], rot=[1.0, .0, .0, .0])
    # # w1.frame.linear = pose.linear.copy()
    # w4.time = 4.0
    #
    # # w5 = pyci.WayPoint()
    # # w5.frame = Affine3(pos=[0.3, 0.3, 0], rot=[1.0, .0, .0, .0])
    # # # w1.frame.linear = pose.linear.copy()
    # # w5.time = 10.0
    #
    # ci.setWaypoints('L_6_A', [w1, w2, w3, w4], False)

    ci.setTargetPose('L_6_A', target_pose, 0.5)

    print 'Waiting for task to complete..'
    tic = timeit.default_timer()
    ci.waitReachCompleted('L_6_A')
    toc = timeit.default_timer()
    print 'Task completed'

    print '\nReach completed in ', toc - tic
    print 'Expected time was ', w4.time
    print 'Possible difference is due to velocity/acceleration limits!'

    solution_msg = rospy.wait_for_message("/cartesian/solution", JointState, 0.5)
    print 'Solution:' + str(solution_msg.position)

    print 'Exiting...'