# vicon-monitor
This package is an extension utilizing vicon bridge to snatch its ROS Tf data to use Vicon Motion Capture system as GT for inddor Spatial Data Processing and evaluation. 

You can collaborate this package to record bag, or monitor Robot / Agent 's odometry in real-time
This package publishes the following ROS topics
  1. Odometry (nav_msgs)
  2. Pose (nav_msgs)
  3. Path (nav_msgs)
  4. Custom TF (to connect your SLAM/base_link frame for real-time monitoring)

# How to use
--------------------------
TODO

**Vicon Monitor requires Vicon bridge together in ROS workspace. Check the Acknowledgement** 



#Acknowledgement
---------------------------
https://github.com/ethz-asl/vicon_bridge

**Vicon Bridge is a driver providing data from VICON motion capture systems. It is based on the vicon_mocap package from the starmac stacks.**
