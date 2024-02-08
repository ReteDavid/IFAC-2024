#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float32MultiArray
from mavros_msgs.msg import DebugValue
from litterdetection import objPose
import tf2_ros
from  tf.transformations import *
from math import pi
from matplotlib import pyplot as plt

from sensor_msgs.msg import Imu
from math import *
x_last = -100
y_last = -100
obj_list_x = []
obj_list_y = []
class SelectObject:

    def __init__(self):
        self.bridge=CvBridge()
        self.img=None
        self.tilt_deg = None
        self.x_cam = None
        self.y_cam = None
        self.z_world = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.trans=None
        self.detections=None
        
    def tilt_callback(self, tilt):
        self.tilt = float(tilt.data)*pi/180

    def pose_callback(self, coord):
        self.x_cam = coord.pose.pose.position.x
        self.y_cam = coord.pose.pose.position.y

    def depth_callback(self, depth):
        self.depth = -depth.pose.pose.position.z

    def orientation_callback(self, msg):
        #change to euler, msg is quaternion
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion)

    def ekf_callback(self,msg):
        #switch coord for plotting
        self.x_cam = msg.pose.pose.position.x
        self.y_cam = msg.pose.pose.position.y
        self.depth= -msg.pose.pose.position.z
        #considering only yaw for orientation
        quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion) 
        #yaw=-euler[2]+90 #rov yaw+90

        self.roll=euler[0]
        self.pitch=euler[1]
        self.yaw=euler[2]
    
    def yolo_callback(self,detections):
        global x_last, y_last, obj_list_x, obj_list_y
        x=int((detections.data[0]+detections.data[2])/2)
        y=int((detections.data[1]+detections.data[3])/2)

        newx=x*int(floor(1920/1080))
        newy=y*int(floor(1080/1080))
        print(f"{newx} and {newy}")

        x_obj, y_obj, x_cam, y_cam, z_cam = objPose(x,y, self.depth, self.x_cam, self.y_cam,self.roll, self.pitch, self.yaw, self.tilt)
        
        print('x obiect is: ', x_obj/1000)
        print('y obiect is: ', y_obj/1000)
        print('x_cam, y_cam, z_cam', x_cam, y_cam, z_cam/1000)
        obj_coord=Odometry()
        obj_coord.pose.pose.position.y=x_obj/1000
        obj_coord.pose.pose.position.x=y_obj/1000
        max_distance_jump=0.2
        x_current=obj_coord.pose.pose.position.x
        y_current=obj_coord.pose.pose.position.y
        test = True
        for i in range(len(obj_list_x)):

            if sqrt((obj_list_x[i]-x_current)**2+(obj_list_y[i]-y_current)**2)<max_distance_jump:
                test=False
                break
        if test == True:
            pub.publish(obj_coord)
            obj_list_x.append(x_current)
            obj_list_y.append(y_current)


    def select_obj(self,ros_image):
        cv_image=self.bridge.compressed_imgmsg_to_cv2(ros_image,'bgr8')
        #cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")

        #print(cv_image.size)


        scale=50
        #width=int(cv_image.shape[1])
        #height=int(cv_image.shape[0])
        width = 1920
        height = 1080
        self.img = cv2.resize(cv_image, (width,height),interpolation=cv2.INTER_AREA)
        self.img = cv_image
        cv2.circle(self.img, (960, 540), 8, (255,102,255), -1)
        cv2.imshow('Feed',self.img)
        cv2.setMouseCallback('Feed',self.Mouse_Event)
        cv2.waitKey(1)

    def Mouse_Event(self,event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # read colours at left clicked point
            b = self.img[y,x,0]
            g = self.img[y,x,1]
            r = self.img[y,x,2]

            try:
                self.trans=tf_buffer.lookup_transform('Rov_frame','gimball',rospy.Time(0))
            except:
                pass
            self.x_cam_off=self.trans.transform.translation.x*10
            self.y_cam_off=self.trans.transform.translation.y*10
            #self.depth=-self.trans.transform.translation.z

            #quaternion=[self.trans.transform.rotation.x,self.trans.transform.rotation.y,self.trans.transform.rotation.z,self.trans.transform.rotation.w]
            #(self.roll,self.pitch,self.yaw)=euler_from_quaternion(quaternion)
            
            print(f'{self.x_cam_off} and {self.y_cam_off}')

            #bluerov2 cam

            x_obj, y_obj, x_cam, y_cam, z_cam = objPose(x, y, self.depth, self.x_cam-self.x_cam_off, self.y_cam-self.y_cam_off, self.roll, self.pitch, self.yaw, self.tilt)

            #overhead camera
            #x_obj, y_obj, x_cam, y_cam, z_cam, y_dir = objPose(x, y, 0, 0, 0, 0, 0,0,0)

            print('x obiect: ', x_obj/1000)
            print('y obiect: ', y_obj/1000)
            print('x_cam, y_cam, y_dir', x_cam, y_cam)
            obj_coord=Odometry()
            obj_coord.pose.pose.position.y=x_obj/1000
            obj_coord.pose.pose.position.x=y_obj/1000
            pub.publish(obj_coord)
            """
            plt.plot(x_cam, y_cam, 'g^', x_obj/1000, y_obj/1000, 'ro')
            
            plt.axis("equal")
            axes=plt.gca()
            axes.set_xlim([-1.5,1.5])
            axes.set_ylim([-1.5,1.5])
            plt.draw()
            plt.show()

            cv2.circle(self.img, (x, y), 5, (255, 255, 255), -1)"""
            # chage the colour of a portion of image
            #rospy.loginfo((b,g,r))
    
pub=rospy.Publisher('/Object/coord',Odometry,queue_size=10)

if __name__ == '__main__':

    counter=0
    rospy.init_node('object_detection_listener', anonymous = True)
    tf_buffer=tf2_ros.Buffer()
    listener=tf2_ros.TransformListener(tf_buffer)

    obj_click_GUI = SelectObject()

    #rospy.Subscriber('/bluerov2/camera/image_raw/compressed', CompressedImage, obj_click_GUI.select_obj)
    #rospy.Subscriber('/TestOps/Camera', Image, obj_click_GUI.select_obj)
    
    rospy.Subscriber('/BlueRov2/odom/depth', Odometry, obj_click_GUI.depth_callback)
    rospy.Subscriber('/BlueRov2/real_coord', Odometry, obj_click_GUI.pose_callback)
    rospy.Subscriber('/odometry/filtered', Odometry, obj_click_GUI.ekf_callback)
    #rospy.Subscriber('/mavros/imu/data', Imu, obj_click_GUI.orientation_callback)
    rospy.Subscriber('/BlueRov2/tilt' , Float64, obj_click_GUI.tilt_callback)
    rospy.Subscriber('/bbox',Float32MultiArray,obj_click_GUI.yolo_callback)


    rospy.spin()