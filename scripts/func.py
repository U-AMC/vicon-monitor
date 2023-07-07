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

def vicon_cb(data):
    global pose
    pose = data


def quaternion_to_euler_angle(w, x, y, z):
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))

	return X, Y, Z