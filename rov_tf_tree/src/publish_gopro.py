#!/usr/bin/env python2

import rospy
from sensor_msgs import msg
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



def takes_data_from_camera():
     
    rate = rospy.Rate(10)  
    vid = cv2.VideoCapture('udp://0.0.0.0:8554',cv2.CAP_FFMPEG)
    
    while not rospy.is_shutdown():
       
        ret, frame = vid.read()
        bridge = CvBridge()
        video_bridge = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub.publish(video_bridge)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            vid.release()
            cv2.destroyAllWindows()
            break
    
  


if __name__ == '__main__':
    
    rospy.init_node('Server',anonymous = True)
    pub = rospy.Publisher('TestOps/Camera', Image, queue_size=10)
    takes_data_from_camera()
