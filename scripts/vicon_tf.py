#!/usr/bin/env python3
# coding=utf8
import rospy
import math
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, Point, Quaternion
from tf2_msgs.msg import TFMessage
import tf, tf2_ros
import tf.transformations
#system
import copy
import _thread
import time
    

cur_odom_to_baselink = None
cur_map_to_odom = None

class VT:
    def __init__(self, args):
        ## about ROS
        rospy.init_node("vicon_ros_monitor")
        ##odom listener
        # self.sub_odom = rospy.Subscriber(vins_topic, Odometry, self.odometryCb)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0))
        self.transformer =  tf.TransformListener(self.tf_buffer)

        ## custom tf
        # self.rgb_to_parallel_tf_broadcaster = tf.TransformBroadcaster()

    def odometryCb(self, msg1):
        # print (msg1.pose.pose)
        self.agent_odom = msg1
    
    def pose_to_mat(pose_msg):
        return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
        )
    
    def handle_tf(self, msg1):
        global cur_odom_to_baselink, cur_vicon_to_odom
        msg_time = msg1.header.stamp
        try:
            self.transformer.waitForTransform(pose_frame, base_frame, msg_time, rospy.Duration(0.1))
            trans, rot = self.transformer.lookupTransform(pose_frame, base_frame, msg_time) #rot is quaternion, trans is xyz vector ([x, y, z], [x, y, z, w])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e: #exception case
            print("Exception occurred : ", e)
            return

        br = tf.TransformBroadcaster()
        br.sendTransform(trans,rot, rospy.Time.now(), "map","sudo")



if __name__ == '__main__':
    # arguments
    base_frame = rospy.get_param('~base_frame')
    pose_frame = rospy.get_param('~pose_frame')
    sudo_frame = rospy.get_param('~sudo_frame')
    map_frame = rospy.get_param('~map_frame')
        
    ## node open
    vicon_node = VT()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shut Down")
