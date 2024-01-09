#!/usr/bin/env python3
# coding=utf8
import rospy
import math
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, Point, Quaternion
from tf2_msgs.msg import TFMessage
import tf, tf2_ros
#SVD
from rigid_transform_3D import rigid_transform_3D
#system
import _thread
import time
    
cur_slam_odom = None
cur_slam_odom_list = []
t_matrix_A = np.empty((3,0))
t_matrix_B = np.empty((3,0))
stack = 0

class VT:
    def __init__(self):
        self.x_t = None
        self.y_t  = None
        self.z_t = None
        self.x_r = None
        self.y_r = None
        self.z_r = None
        self.w_r = None
        ##odom listener
        # self.sub_odom = rospy.Subscriber(vins_topic, Odometry, self.odometryCb)

        self.tf_buffer = tf2_ros.Buffer()
        self.transformer = tf2_ros.TransformListener(self.tf_buffer)

        # arguments
        self.base_frame = rospy.get_param('~base_frame')
        self.pose_frame = rospy.get_param('~pose_frame')
        self.sudo_frame = rospy.get_param('~sudo_frame')
        self.map_frame = rospy.get_param('~map_frame')
        self.slam_topic = rospy.get_param('~slam_topic')
        self.calibration = rospy.get_param('~calibration', True)
        #ros subscribers
        self.slam_sub = rospy.Subscriber(self.slam_topic, Odometry, self.cb_save_slam_odom, queue_size=1)
        #ros publishers
        self.pose_pub = rospy.Publisher('~pose', PoseStamped, queue_size = 1)
        self.odom_pub = rospy.Publisher('~Odometry', Odometry, queue_size = 1)
        self.vicon_path_pub = rospy.Publisher('~vicon_path', Path, queue_size=1)
        self.custom_tf = tf.TransformBroadcaster()
        self.vicon_path = Path()
        #log info for launch param
        rospy.loginfo('Base_frame as...: ' + self.base_frame)
        rospy.loginfo('Pose_frame as...: ' + self.pose_frame)
    
    def handle_tf(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                # self.transformer.waitForTransform(self.base_frame,self.pose_frame, rospy.Time.now(), rospy.Duration(0.1))
                transform = self.tf_buffer.lookup_transform(self.base_frame,self.pose_frame,rospy.Time.now(), rospy.Duration(0.1)) #rot is quaternion, trans is xyz vector ([x, y, z], [x, y, z, w])
                #pose
                x_t = transform.transform.translation.x
                y_t = transform.transform.translation.y
                z_t = transform.transform.translation.z
                #rot
                x_r = transform.transform.rotation.x
                y_r = transform.transform.rotation.y  
                z_r = transform.transform.rotation.z
                w_r = transform.transform.rotation.w 
                
            except(tf2_ros.LookupException,
                   tf2_ros.ConnectivityException,
                   tf2_ros.ExtrapolationException,
                   tf2_ros.TransformException) as e:
                print("Exception occurred : ", e)
                continue
        
            #publish pose
            pose = PoseStamped()
            pose.header.frame_id = self.base_frame
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            self.pose_pub.publish(pose)
            
            localization = Odometry()
            localization.pose.pose = Pose(transform.transform.translation, transform.transform.rotation)
            localization.header.stamp = transform.header.stamp
            localization.header.frame_id = self.base_frame
            localization.child_frame_id = self.map_frame
            self.odom_pub.publish(localization)

            #vicon path
            self.vicon_path.header.frame_id=self.base_frame
            self.vicon_path.poses.append(pose)
            self.vicon_path.header.stamp = rospy.Time.now()
            self.vicon_path_pub.publish(self.vicon_path)
            
            if (self.calibration == False):
                #frame publisher
                self.custom_tf.sendTransform(
                    translation=[x_t,y_t,z_t],
                    rotation=[x_r,y_r,z_r,w_r],
                    time=rospy.Time.now(),
                    child=self.map_frame, #define it as frame that you want to hold as world frame
                    parent=self.base_frame)
            else :
                if cur_slam_odom is not None:
                    if stack < 10:
                        T_map_to_odom = self.pose_to_mat(cur_slam_odom) #gets 4x4 SE(3) matrix
                        T_vicon_pose = self.pose_to_mat(localization)
                        A_t = T_map_to_odom[3,:2] #(x y z) -> t
                        B_t = T_vicon_pose[3,:2] #(x y z) -> t
                        t_matrix_A = np.hstack((t_matrix_A, A_t.reshape(3, 1)))
                        t_matrix_B = np.hstack((t_matrix_B, B_t.reshape(3, 1)))
                        stack+=1
                    if stack == 10:
                        print(t_matrix_A + " A poses")
                        print(t_matrix_B + " B poses")
                else:
                    T_map_to_odom = np.eye(4) #no odom go eyemat
                    
                
                if len(cur_slam_odom_list) >= 10:
                    R,t = rigid_transform_3D(t_matrix_A, t_matrix_B)
                    print(R + " :SVD Rotation")
                    print(t + " :SVD Translation")
                    
                #frame publisher
                self.custom_tf.sendTransform(
                    translation=[x_t,y_t,z_t],
                    rotation=[x_r,y_r,z_r,w_r],
                    time=rospy.Time.now(),
                    child=self.map_frame, #define it as frame that you want to hold as world frame
                    parent=self.base_frame)
            #spin rate
            rate.sleep()
    
    #SE(3) conversion module 
    def pose_to_mat(pose_msg):
        return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation))
    
    def cb_save_slam_odom(odom_msg):
        global cur_slam_odom
        cur_slam_odom = odom_msg
    
if __name__ == '__main__':
    ## about ROS
    rospy.init_node("vicon_ros_monitor")
    ## node open
    vicon_node = VT()
    try:
        #run thread
        _thread.start_new_thread(vicon_node.handle_tf, ())
        rospy.spin()
    except KeyboardInterrupt:
        print("Shut Down")
