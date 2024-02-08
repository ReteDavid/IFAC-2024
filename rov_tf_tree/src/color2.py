#!/usr/bin/env python

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as Img
from math import cos, sin
from nav_msgs.msg import Odometry
import message_filters
import sys
import math
from cmath import sqrt

seq=1
last_point=Odometry()
bridge = CvBridge()

class Task:
    
    def __init__(self):
        self.x_world = None
        self.y_world = None
        self.z_world = None

    def depth_callback(self, depth):
        self.z_world = -depth.pose.pose.position.z

    def image_callback(self, msg):

        image = bridge.imgmsg_to_cv2(msg, "bgr8")
        camera_matrix = np.array(((927.091270, 0.000000, 957.570804), (0.000000,919.995427, 533.540912), (0.000000, 0.000000, 1.000000)))
        K = np.array(((927.091270, 0.000000, 957.570804, 0.000000), (0.000000,919.995427, 533.540912, 0.000000), (0.000000, 0.000000, 1.000000, 0.000000)))

        dist = np.array((0.05, 0.07, -0.11, 0.05, 0.000000))

        w = 1920
        h = 1080
        newcammtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist, (w,h), 1, (w,h))
        undistort = cv2.undistort(image, camera_matrix, dist, None, newcammtx)

        frame = image
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        #get hue value of the center pixel
        height, width,_ = frame.shape

        #cx = int(width/2)
	#cy = int(height/2)
        cx=1450
        cy=90
	
        center_pixel = hsv[cy,cx]
        #print(center_pixel)

        cv2.circle(frame, (cx,cy), 5, (255,0,0), 3)

        #SEGMENTATION
        #yellow interval
        lower_yellow = np.array([20, 98, 115])
        upper_yellow = np.array([50, 255, 255])

        # thresholding
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_yellow = cv2.GaussianBlur(mask_yellow, (5, 5), 0)

        yellow_patch = cv2.bitwise_and(frame, frame, mask=mask_yellow)

        image = yellow_patch

	# create contours
        contours,_ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	#get max contour
        c = max(contours, key = cv2.contourArea)
        
        #get centroid
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        #center
        cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
        cv2.putText(frame, "center", (cX-15, cY-25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


	# cam center
        cv2.circle(frame, (957, 533), 8, (255,102,255), -1)

        contur = cv2.drawContours(frame, c, -1, (0, 255, 0), 3)

        contur_bridge = bridge.cv2_to_imgmsg(contur, "bgr8")
        pub.publish(contur_bridge)

	#POSE ESTIMATION

        inv_K = np.linalg.pinv(K)
        inv_cam_matrix = np.linalg.inv(camera_matrix)
	
	#centrul obiectului detectat (converted to homogenous coordinates)
	
        centru = np.array([cX, cY, 1])

        centru = np.vstack(centru)

	#center point of camera
        C = np.array([0,0,0]) #x,y,z of camera
        C = np.vstack(C)

	# line on which the projected point lies in camera reference frame

        M_int = np.dot(inv_cam_matrix, centru)
        d = M_int-C
        
        roll =0
        pitch=0
        yaw=0

	# rotation matrix
        R_roll = np.matrix([[1, 0, 0], [0, cos(roll), sin(roll)], [0, -sin(roll), cos(roll)]])
        R_pitch = np.matrix([[cos(pitch), 0, -sin(pitch)], [0, 1, 0], [sin(pitch), 0, cos(pitch)]])
        R_yaw = np.matrix([[cos(yaw), sin(yaw), 0], [-sin(yaw), cos(yaw), 0], [0, 0, 1]])
        R = R_yaw * R_pitch * R_roll #Rz*Ry*Rx
        
        #camera position in world reference frame
        inv_R = np.linalg.inv(R)
        
        #image coordinates:
        u, v = cX, cY
        image_coord = np.array([u,v,1]).reshape(-1,1)

        #derived camera coord'
        camera_coord = np.dot(inv_cam_matrix,image_coord) #aka M_int
        #direction cam
        dirr = camera_coord-C
        #dir world 
        dwor = np.dot(inv_R, dirr)

        #vehicle coord
        camera_coord = np.append(camera_coord,1)

        homo_points = image_coord

        depth = self.z_world

        zet = 2100 - depth #mm
        homo_points = homo_points*zet
        world_coord = np.linalg.inv(camera_matrix)@homo_points
        world_coord = np.squeeze(np.asarray(world_coord/1000))
        print('Rov position: \n', world_coord)
        
        global seq, last_point
        cov=[0.0]*36
        cov[0]=6
        cov[7]=6
        odom=Odometry()
        odom.pose.covariance=cov
        odom.header.frame_id="world"
        odom.header.stamp=msg.header.stamp
        odom.header.seq=seq
        seq+=1
        odom.child_frame_id="yellow_patch"
        self.x_world = world_coord[0]
        self.y_world = world_coord[1]
        #odom.pose.pose.position.x=(2.1+depth)*world_coord[0,0]
        #odom.pose.pose.position.y=(2.1+depth)*world_coord[0,1]
        odom.pose.pose.position.x=world_coord[0]
        odom.pose.pose.position.y=world_coord[1]
        
        #change coords so that ox points forward and oy points right
        old_x=odom.pose.pose.position.x
        old_y=odom.pose.pose.position.y
        new_x=-old_y
        new_y=old_x
        odom.pose.pose.position.x=new_x
        odom.pose.pose.position.y=new_y

        #check for outliers when loosing color detection
        x_last=last_point.pose.pose.position.x
        y_last=last_point.pose.pose.position.y
        x_current=odom.pose.pose.position.x
        y_current=odom.pose.pose.position.y
        max_distance_jump=1.5
        if(seq==1):
            last_point=odom#presuming the robot's yellow patch is seen by camera
        if math.sqrt((x_current-x_last)**2+(y_current-y_last)**2)>max_distance_jump:
            odom.pose.pose.position.x=last_point.pose.pose.position.x
            odom.pose.pose.position.y=last_point.pose.pose.position.y
        else:
            last_point=odom

        real.publish(odom)
        odomz = Odometry()
        #odomz.pose.pose.position.z = self.z_world*1.0
        
        """
        theta1 = math.atan(2.15/math.sqrt(world_coord[0,0]**2+world_coord[0,1]**2))
        n1=1
        n2=1.33
        S = -depth/math.cos(theta1)
        R = S*(n1/n2)
        theta2 = math.asin((n1/n2)*math.sin(theta1))
        P = math.sqrt(R**2 + S**2 - 2*R*S*math.cos(theta1-theta2))
        beta = math.pi-theta1-math.asin(R*math.sin(theta1-theta2)/P)
        delta = P*math.cos(beta)
        alpha = math.atan(world_coord[0,0]/world_coord[0,1])
        deltaX = delta*math.cos(alpha)
        deltaY=  delta*math.sin(alpha)
        odom.pose.pose.position.x+=deltaX
        odom.pose.pose.position.y+=deltaY
        odom.pose.pose.position.z = 2.15-self.z_world
        world.publish(odom) #with refraction correction
        
        odomreal = Odometry()
        odomreal.pose.pose.position.x =(2.1)* world_coord[0,0] #for some reason the coordinates are cut in half => see calibration etc.
        odomreal.pose.pose.position.y =(2.1)* world_coord[0,1]
        #odomreal.pose.pose.position.z = self.z_world*1.0
        real.publish(odomreal) # real coordinates without refraction correction
        """


pub = rospy.Publisher('colordetection', Image, queue_size=10)
world = rospy.Publisher('/BlueRov2/plane', Odometry, queue_size=10)
real = rospy.Publisher('/BlueRov2/real_coord', Odometry, queue_size=10)

if __name__ == '__main__':

    rospy.init_node('listener',anonymous = True)
    
    task = Task()

    
    rospy.Subscriber('/TestOps/Camera', Image, task.image_callback)
    rospy.Subscriber('/BlueRov2/odom/depth', Odometry, task.depth_callback)

    rospy.spin()
