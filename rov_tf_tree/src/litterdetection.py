#!/usr/bin/env python

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as Img
from math import cos, sin, pi
from nav_msgs.msg import Odometry
from sympy import symbols, solve, Matrix, Array
from std_msgs.msg import Float64

bridge = CvBridge()


def objPose(x, y, depth, x_cam, y_cam, roll, pitch, yaw, tilt):

        
        # for BlueROV2 camera
        camera_matrix = np.array(
                [[1075.166267, 0.000000, 948.291239],
                [0.000000, 1070.497776, 547.863205],
                [0.000000, 0.000000, 1.000000]])
        K = np.array(
                [[1075.166267, 0.000000, 948.291239, 0.000000],
                [0.000000, 1070.497776, 547.863205, 0.000000],
                [0.000000, 0.000000, 1.000000, 0.000000]])
        
        # overhead camera gopro
        '''camera_matrix = np.array(((927.091270, 0.000000, 957.570804), (0.000000,919.995427, 533.540912), (0.000000, 0.000000, 1.000000)))
        K = np.array(((927.091270, 0.000000, 957.570804, 0.000000), (0.000000,919.995427, 533.540912, 0.000000), (0.000000, 0.000000, 1.000000, 0.000000)))'''

        dist = np.array((0.041068, -0.005382, 0.002886, -0.006773, 0.000000))

        #POSE ESTIMATION
        #inverse of cam matrix

        inv_K = np.linalg.pinv(K)
        inv_cam_matrix = np.linalg.inv(camera_matrix)
        # rotation matrix
        R_roll = np.matrix([[1, 0, 0], [0, cos(roll), sin(roll)], [0, -sin(roll), cos(roll)]])
        R_pitch = np.matrix([[cos(pitch), 0, -sin(pitch)], [0, 1, 0], [sin(pitch), 0, cos(pitch)]])
        R_yaw = np.matrix([[cos(yaw), sin(yaw), 0], [-sin(yaw), cos(yaw), 0], [0, 0, 1]])
        R = R_yaw * R_pitch * R_roll
        #R=np.matrix([[cos(kp), sin(kp), 0], [-sin(kp), cos(kp), 0], [0, 0, 1]])

        #multiply R by tilt+++++++++++++++++++++++++++++++++
        R_tilt = np.matrix([[cos(tilt), 0, -sin(tilt)], [0, 1, 0], [sin(tilt), 0, cos(tilt)]])

        R=R*R_tilt #do it with mavros/imu/data
        
        #camera position in world reference frame
        inv_R = np.linalg.inv(R)
        
        #image coordinates:

        u, v= x,y
        image_coord = np.array([u,v,1]).reshape(-1,1) #aka center

        #derived camera coord'
        camera_coord = np.dot(inv_cam_matrix,image_coord) #aka M_int

        #direction cam
        #center point of camera

        z_obiect = 0
        z_cam = z_obiect + 1.2 - depth*1000
        #Overheadcam
        #z_cam=0

        C = np.array([0,0,0])
        C = np.vstack(C)
        dirr = camera_coord-C
        dirr = np.squeeze(np.asarray(dirr))

        dwor = np.matmul(inv_R, dirr)
        dwor = np.squeeze(np.asarray(dwor))

        x_dir = dwor[0]
        y_dir = dwor[1]
        z_dir = dwor[2]
        s = (z_cam - z_obiect)/z_dir
        x_obiect = x_cam + s*x_dir
        y_obiect = y_cam + s*y_dir

        '''aux = x
        x_obiect=-y_obiect
        y_obiect=aux'''

        return x_obiect, y_obiect, x_cam, y_cam, z_cam
