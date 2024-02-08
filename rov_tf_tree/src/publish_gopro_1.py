#!/usr/bin/env python3

import rospy
from sensor_msgs import msg
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



def takes_data_from_camera():
     
    rate = rospy.Rate(10)  
    vid = cv2.VideoCapture('udp://:8554',cv2.CAP_FFMPEG)

    width = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))
    time=rospy.Time.now()

    
    writer = cv2.VideoWriter('goprorov.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 20, (width, height))
    
    while not rospy.is_shutdown():
       
        ret, frame = vid.read()
        bridge = CvBridge()
        video_bridge = bridge.cv2_to_imgmsg(frame, "bgr8")
        video_bridge.header.stamp=rospy.Time.now()
        #rospy.loginfo(video_bridge.header.stamp)
        pub.publish(video_bridge)
        writer.write(frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            vid.release()
            writer.release()
            cv2.destroyAllWindows()
            break
    
  


if __name__ == '__main__':
    
    rospy.init_node('Server',anonymous = True)
    pub = rospy.Publisher('TestOps/Camera', Image, queue_size=10)
    takes_data_from_camera()
