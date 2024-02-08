#!/usr/bin/env python

import rospy
from struct import pack, unpack
from mavros_msgs.msg import Mavlink
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import Point,Pose,Quaternion,Twist

#preparing publisher for tf
odom_broadcaster=tf2_ros.TransformBroadcaster()
tf_point=Point()
tf_pose=Pose()
tf_quat=Quaternion()
tf_quat.w=1
tf_twist=Twist()


publisher_name='depth_publisher'
publisher_topic='/BlueRov2/odom/depth'
flag=False#to loginfo that publisher started publishing

subscriber_name='Bar30_primary_listener'
#/mavros/imu/diff_pressure
subscriber_topic='/mavlink/from'
pub=rospy.Publisher(publisher_topic,Odometry,queue_size=100)
###Odometry
odom_msg=Odometry()
depth=0
water_density=1000#kg/m^3 for fresh water
g=9.80675#[m/s^2] in Cluj-Napoca
z_covarince=[0]*36
z_covarince[14]=0.3
seq=1
atm_press=970

def decode_info(data):
    global seq,odom_msg,atm_press
    #id 137 for scaled_pressure2 which is bar30 sensor 137
    if data.msgid==137:
        #pack=convert from C struct to Python values->QQ unsigned long long
        p=pack("QQ",*data.payload64)
        #unpack value as unsigned int, floar, float and short
        ##Construct header
        time_boot_ms,pres_abs,press_diff,temperature=unpack("Iffhxx",p)
        #if seq==1 and pres_abs<1000:
            #atm_press=pres_abs
        odom_msg.header.frame_id="world"
        odom_msg.header.stamp=data.header.stamp
        odom_msg.header.seq=seq
        seq+=1
        odom_msg.child_frame_id="Bar30_frame"
        #odom_msg.child_frame_id="Rov_frame"
        #rospy.loginfo(pres_abs-atm_press)
        #rospy.loginfo(pres_abs)
        #rospy.loginfo(temperature)
        #rospy.loginfo("/n")
        #press_diff is in hPa and we need to convert to Pa=> 1 hPa=>100 Pa
        odom_msg.pose.pose.position.z=-(pres_abs-atm_press)*100/water_density/g
        odom_msg.pose.covariance=z_covarince

        #publish the transform over tf
        #odom_broadcaster((0,0,odom_msg.pose.pose.position.z),tf_quat,odom_msg.header.stamp,odom_msg.child_frame_id,odom_msg.header.frame_id)

        pub.publish(odom_msg)

def listener():
    rospy.init_node(subscriber_name,anonymous=True)
    rospy.Subscriber(subscriber_topic,Mavlink,decode_info)
    rospy.loginfo("%s started listening to mavlink Bar30....",subscriber_name)
    rospy.spin()

if __name__=="__main__":
    listener()
