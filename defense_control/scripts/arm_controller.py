#!/usr/bin/env python

import sys
import rospy
import math
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from trac_ik_python.trac_ik import IK
from geometry_msgs.msg import PoseStamped

ik_solver = None
pub = None
seed_state = [0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0]

limits_lower = [-1.5, -0.34, -0.01, -2.09, -3.14,  0.0, -3.14]
limits_upper = [ 1.5,  1.57,  0.01,   0.0,  3.14, 2.09,  3.14]

def controller_callback(msg):
    global ik_solver, pub, seed_state

    pos = msg.pose.position
    ori = msg.pose.orientation
    ik = ik_solver.get_ik(seed_state, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)

    if ik:
        jt = JointTrajectory()
        jt.joint_names = list(ik_solver.joint_names)

        tp = JointTrajectoryPoint()
        tp.positions = list(ik)
        tp.time_from_start = rospy.Duration(nsecs=500000000)
        jt.points = [tp]

        jt.header.stamp = rospy.Time.now()
        pub.publish(jt)
    else:
        rospy.logwarn("IK_ERROR")

def main():
    global ik_solver, pub
    rospy.init_node('joint_driver', anonymous=True)

    ik_solver = IK("iiwa_link_0", "just_the_tip", timeout=0.05, solve_type="Distance")
    ik_solver.set_joint_limits(limits_lower, limits_upper)

    pub = rospy.Publisher('out', JointTrajectory, queue_size=10)

    ##########################################
    jt = JointTrajectory()
    jt.joint_names = list(ik_solver.joint_names)

    tp = JointTrajectoryPoint()
    tp.positions = seed_state
    tp.time_from_start = rospy.Duration(secs=5)
    jt.points = [tp]

    jt.header.stamp = rospy.Time.now()
    pub.publish(jt)
    rospy.sleep(5.0)
    ##########################################

    rospy.Subscriber('in', PoseStamped, controller_callback)

    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
