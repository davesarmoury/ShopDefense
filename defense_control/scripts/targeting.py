#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf.transformations import *
import tf
import numpy
from zed_interfaces.msg import ObjectsStamped
import math

DEBUG = False
HEAD_DISTANCE = 0.8

def det_callback(msg):
    global broadcaster, cam_mtx, pub
    rospy.loginfo("Found " + str(len(msg.objects)) + " attacker(s)")

    for o in msg.objects:
        o_mtx = compose_matrix(translate=[o.position[0], o.position[1], o.position[2]])
        o_mtx = numpy.matmul(cam_mtx, o_mtx)
        scale, shear, angles, translate, perspective = decompose_matrix(o_mtx)
        if DEBUG:
            broadcaster.sendTransform(translate,
                         [0,0,0,1],
                         rospy.Time.now(),
                         "object_" + str(o.label_id),
                         "world")

        horizontal_distance = math.sqrt(translate[0]*translate[0] + translate[1]*translate[1])
        yaw = math.atan2(translate[1], translate[0])
        pitch = math.atan2(translate[2],horizontal_distance) * -1.0
        h_mtx = compose_matrix(translate=[HEAD_DISTANCE, 0, 0])
        t_mtx = compose_matrix(angles=[0,pitch,yaw])
        t_mtx = numpy.matmul(t_mtx, h_mtx)
        scale, shear, angles2, translate2, perspective = decompose_matrix(t_mtx)
        quat = quaternion_from_euler(angles2[0], angles2[1], angles2[2])
        if DEBUG:
            broadcaster.sendTransform(translate2,
                         quat,
                         rospy.Time.now(),
                         "target_" + str(o.label_id),
                         "world")

        ps = PoseStamped()
        ps.pose.position.x = translate2[0]
        ps.pose.position.y = translate2[1]
        ps.pose.position.z = translate2[2]
        ps.pose.orientation.x = quat[0]
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "world"
        pub.publish(ps)

if __name__ == '__main__':
    global broadcaster, cam_mtx, pub
    rospy.init_node('defense_targeting')
    if DEBUG:
        broadcaster = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    listener.waitForTransform('world', 'zed2_left_camera_frame', rospy.Time(0), rospy.Duration(0.5))
    trans, rot = listener.lookupTransform('world', 'zed2_left_camera_frame', rospy.Time(0))

    cam_mtx = compose_matrix(translate=trans, angles=euler_from_quaternion(rot))

    rospy.Subscriber("/zed2/zed_node/obj_det/objects", ObjectsStamped, det_callback)
    pub = rospy.Publisher('/in', PoseStamped, queue_size=10)

    rospy.spin()
