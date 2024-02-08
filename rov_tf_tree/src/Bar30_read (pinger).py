#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import Point,Pose,Quaternion,Twist
from brping import Ping1D

#preparing publisher for tf
odom_broadcaster=tf2_ros.TransformBroadcaster()
tf_point=Point()
tf_pose=Pose()
tf_quat=Quaternion()
tf_quat.w=1
tf_twist=Twist()
total_depth=1.15

publisher_name='depth_publisher'
publisher_topic='/BlueRov2/odom/depth'


pub=rospy.Publisher(publisher_topic,Odometry,queue_size=100)
###Odometry
odom_msg=Odometry()
depth=0
z_covarince=[0]*36
z_covarince[14]=0.3
seq=1



#starting publisher

rospy.init_node(publisher_name)
rate=rospy.Rate(5)
rospy.loginfo("started publishing depth from Pinger")

myPing=Ping1D()
myPing.connect_udp("192.168.2.2",9090)
if myPing.initialize() is False:
    print("Failed to initialize Ping!")
    exit(1)

while not rospy.is_shutdown():
    #detting pinger distance
    ping_dist=myPing.get_distance()
    ##Construct message
    odom_msg.header.frame_id="world"
    odom_msg.header.stamp=rospy.Time.now()
    odom_msg.header.seq=seq
    seq+=1
    odom_msg.child_frame_id="Bar30_frame"
    depth=total_depth-ping_dist["distance"]
    odom_msg.pose.covariance=z_covarince
    pub.publish(odom_msg)
    rate.sleep()





