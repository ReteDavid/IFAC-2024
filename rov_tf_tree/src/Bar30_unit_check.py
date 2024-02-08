#!/usr/bin/env python

import rospy
from struct import pack, unpack
from mavros_msgs.msg import Mavlink
from std_msgs.msg import Float32


publisher_name='ext_bar30_diff_press'
publisher_topic='/external/pressure'

subscriber_name='Bar30_primary_listener'
subscriber_topic='/mavlink/from'
pub=rospy.Publisher(publisher_topic,Float32,queue_size=100)

odom_msg=Float32()



def decode_info(data):
    global seq,odom_msg
    #id 137 for scaled_pressure2 which is bar30 sensor
    if data.msgid==137:
        #pack=convert from C struct to Python values->QQ unsigned long long
        p=pack("QQ",*data.payload64)
        #unpack value as unsigned int, floar, float and short
        time_boot_ms,pres_abs,press_diff,temperature=unpack("Iffhxx",p)
        
        odom_msg.data=press_diff

        pub.publish(odom_msg)

def listener():
    rospy.init_node(subscriber_name,anonymous=True)
    rospy.Subscriber(subscriber_topic,Mavlink,decode_info)
    rospy.loginfo("%s started listening to mavlink Bar30....",subscriber_name)
    rospy.spin()

if __name__=="__main__":
    listener()