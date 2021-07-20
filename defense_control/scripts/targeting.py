#!/usr/bin/env python

import rospy
from std_msgs.msg import String, UInt8
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf.transformations import *
import tf
import numpy
from zed_interfaces.msg import ObjectsStamped
import math

DEBUG = True
HEAD_DISTANCE = 0.6

def det_callback(msg):
    global broadcaster, cam_mtx, world_mtx, pub, pub2
    if len(msg.objects) > 0:
        rospy.loginfo("Found " + str(len(msg.objects)) + " attacker(s)")

    min_distance = 99999.0
    target_mtx = None
    for o in msg.objects:
        o_mtx = compose_matrix(translate=[o.position[0], o.position[1], o.position[2]])
        o_mtx = numpy.matmul(cam_mtx, o_mtx)
        scale, shear, angles, translate, perspective = decompose_matrix(o_mtx)
        if DEBUG:
            broadcaster.sendTransform(translate,
                         [0,0,0,1],
                         rospy.Time.now(),
                         "object_" + str(o.label_id),
                         "spray_origin_link")

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
                         "spray_origin_link")

        if horizontal_distance < min_distance:
            min_distance = horizontal_distance
            target_mtx = t_mtx

    if min_distance < 20.0:
        t_w_mtx = numpy.matmul(world_mtx, target_mtx)
        scale, shear, angles2, translate2, perspective = decompose_matrix(t_w_mtx)
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

    if min_distance < 5.0 and min_distance > 1.0:
        pub2.publish(255)
    else:
        pub2.publish(0)

if __name__ == '__main__':
    global broadcaster, cam_mtx, pub, pub2
    rospy.init_node('defense_targeting')
    if DEBUG:
        broadcaster = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    listener.waitForTransform('spray_origin_link', 'zed2_left_camera_frame', rospy.Time(0), rospy.Duration(0.5))
    trans, rot = listener.lookupTransform('spray_origin_link', 'zed2_left_camera_frame', rospy.Time(0))

    cam_mtx = compose_matrix(translate=trans, angles=euler_from_quaternion(rot))

    listener.waitForTransform('world', 'spray_origin_link', rospy.Time(0), rospy.Duration(0.5))
    trans, rot = listener.lookupTransform('world', 'spray_origin_link', rospy.Time(0))

    world_mtx = compose_matrix(translate=trans, angles=euler_from_quaternion(rot))

    rospy.Subscriber("/zed_node/obj_det/objects", ObjectsStamped, det_callback)
    pub = rospy.Publisher('/arm_pos', PoseStamped, queue_size=10)
    pub2 = rospy.Publisher('/spray', UInt8, queue_size=10)

    rospy.spin()
