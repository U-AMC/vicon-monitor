#!/usr/bin/env python3

#python glob module
import numpy as np
import math
#ROS module
import rospy
from sensor_msgs.msg import Imu #temporarily deprecated
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped #temporarily deprecated
from tf2_msgs.msg import TFMessage #temporarily deprecated
import tf
#custom moudle
from rigid_transform_3D import rigid_transform_3D 
from func import vicon_cb, quaternion_to_euler_angle

#set variables
sync_cnt = 0
pose = PoseStamped()
counter = 0
x = 0.
y = 0.
dt = 1./50.

#ROS node init
rospy.init_node('pose_to_odom')

base_frame = rospy.get_param('~base_frame')
pose_frame = rospy.get_param('~pose_frame')
sudo_frame = rospy.get_param('~sudo_frame')
map_frame = rospy.get_param('~map_frame')

#pose topic strcat
tf_sub_name = rospy.get_param('~pose_from_tf') + ('/pose')

# rospy.get_param('_pose_sub')
vicon_sub = rospy.Subscriber(tf_sub_name, PoseStamped, vicon_cb, queue_size=10)
odom_pub_0 = rospy.Publisher('/odometry_dummy_0', Odometry, queue_size=10)
rate = rospy.Rate(100.0) #pub rate, follow realtime
#====================================================#
# add more for additional tf from vicon
# odom_pub_1 = rospy.Publisher('/odometry_dummy_1', Odometry, queue_size=100)
# odom_pub_2 = rospy.Publisher('/odom', Odometry, queue_size=100)
#====================================================#

while not rospy.is_shutdown():                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
    (v_roll,v_pitch,v_yaw) = quaternion_to_euler_angle(pose.pose.orientation.w, pose.pose.orientation.x , pose.pose.orientation.y, pose.pose.orientation.z)
    v_phi = float((v_roll))
    v_theta = float((v_pitch))
    v_psi = float((v_yaw))
    
    x = pose.pose.position.x
    y = pose.pose.position.y
    z = pose.pose.position.z

    yaw = math.radians(v_psi)      
    
    if counter > 0:
        vel_x_world = (x - x_prev) / dt
        vel_y_world = (y - y_prev) / dt

        x_prev = x
        y_prev = y


        twist_x = math.cos(yaw) * vel_x_world + math.sin(yaw) * vel_y_world
        twist_y = math.cos(yaw) * vel_y_world - math.sin(yaw) * vel_x_world


        odom = Odometry()
        odom.header.frame_id = sudo_frame
        odom.child_frame_id = map_frame
        odom.header.stamp = rospy.Time.now()

        #odom pose&orientation
        odom.pose.pose.position.x = pose.pose.position.x
        odom.pose.pose.position.y = pose.pose.position.y
        odom.pose.pose.position.z = pose.pose.position.z

        odom.pose.pose.orientation.x = pose.pose.orientation.x
        odom.pose.pose.orientation.y = pose.pose.orientation.y
        odom.pose.pose.orientation.z = pose.pose.orientation.z
        odom.pose.pose.orientation.w = pose.pose.orientation.w

        odom.twist.twist.linear.x = twist_x
        odom.twist.twist.linear.y = twist_y
        odom.twist.twist.linear.z = (z - z_prev) / dt
        z_prev = z

        odom.twist.twist.angular.x = 0.
        odom.twist.twist.angular.y = 0.
        odom.twist.twist.angular.z = 0.

        odom_pub_0.publish(odom)

        #pose,orientaion backup
        if sync_cnt < 100:
            init_pose_x = pose.pose.position.x 
            init_pose_y = pose.pose.position.y
            init_pose_z = pose.pose.position.z 
 
            init_or_x = pose.pose.orientation.x #+ float(0.7071068)
            init_or_y = pose.pose.orientation.y
            init_or_z = pose.pose.orientation.z
            init_or_w = pose.pose.orientation.w #- float(0.7071068)

            # init_or_x = v_phi
            # init_or_y = v_theta
            # init_or_z = v_psi
            # init_or_w = pose.pose.orientation.w

        br = tf.TransformBroadcaster()
        if sync_cnt < 100:
            br.sendTransform((x,y,z),[pose.pose.orientation.x, pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w],rospy.Time.now(), "map","sudo")
            sync_cnt += 1
        elif sync_cnt >= 100:
            #fix that frame shit
            br.sendTransform((init_pose_x, init_pose_y,init_pose_z),[init_or_x,init_or_y,init_or_z,init_or_w],rospy.Time.now(), "map","sudo")
             
    else:
        x_prev = x
        y_prev = y
        z_prev = z
        counter += 1

    rate.sleep()