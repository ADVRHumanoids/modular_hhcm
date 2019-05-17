#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$


import rospy
from geometry_msgs.msg import Pose, PoseStamped
from cartesian_interface.msg import ReachPoseActionGoal


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def talker():
    pub = rospy.Publisher('/cartesian/L_6_A/reach/goal', ReachPoseActionGoal, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        action_goal = ReachPoseActionGoal()
        ref_pose = Pose()

        ref_pose.position.x = 0.5
        ref_pose.position.y = 0.5
        ref_pose.position.z = 0.0

        ref_pose.orientation.x = 0.0
        ref_pose.orientation.y = 0.0
        ref_pose.orientation.z = 0.0
        ref_pose.orientation.w = 1.0

        action_goal.goal.frames.append(ref_pose)
        action_goal.goal.time.append(2.0)

        action_goal.header.seq = 1
        action_goal.header.stamp = rospy.Time.now()
        action_goal.header.frame_id = "world"

        # ref_pose = PoseStamped()
        #
        # ref_pose.header.seq = 1
        # ref_pose.header.stamp = rospy.Time.now()
        # ref_pose.header.frame_id = "world"
        #
        # ref_pose.pose.position.x = 0.0
        # ref_pose.pose.position.y = 0.0
        # ref_pose.pose.position.z = 0.0
        #
        # ref_pose.pose.orientation.x = 0.0
        # ref_pose.pose.orientation.y = 0.0
        # ref_pose.pose.orientation.z = 0.0
        # ref_pose.pose.orientation.w = 1.0

        rospy.loginfo(action_goal)
        pub.publish(action_goal)
        # rospy.loginfo(ref_pose)
        # pub.publish(ref_pose)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
