#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import *
from math import pi
from geometry_msgs.msg import Quaternion,Vector3,Transform,Vector3Stamped


publish_topic="/BlueRov2/imu"
publisher=rospy.Publisher(publish_topic,Imu,queue_size=100)
#138 degrees from east
degree2rad=138*pi/180
yaw_rotation=quaternion_from_euler(0,0,degree2rad)

def repostImuData(data):
    global yaw_rotation
    data.header.frame_id="pixhawk"

    #changing imu orientation to point 0 yaw towards camera OX
    
    old_orientation=[data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w]
    yaw_rotation_inv=quaternion_inverse(yaw_rotation)
    new_orient=quaternion_multiply(quaternion_conjugate(yaw_rotation_inv),old_orientation)
    new_orient=quaternion_multiply(old_orientation,yaw_rotation_inv)
    data.orientation.x=new_orient[0]
    data.orientation.y=new_orient[1]
    data.orientation.z=new_orient[2]
    data.orientation.w=new_orient[3]

    publisher.publish(data)

if __name__=="__main__":
    rospy.init_node('republish_imu_frameid', anonymous = True)

    rospy.Subscriber('/mavros/imu/data',Imu,repostImuData,queue_size=1000)
    rospy.spin()