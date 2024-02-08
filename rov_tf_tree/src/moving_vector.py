#!/usr/bin/env python

import cv2
import rospy
import math
import numpy as np
from tf.transformations import *
from nav_msgs.msg import Odometry



class MovingGraph:

    def __init__(self):
        self.height=960
        self.width=960
        self.cv_image = np.uint8(np.full((self.height,self.width,3), (255,255,255)))

        #scale=50
        #width=int(cv_image.shape[1]*scale/100)
        #height=int(cv_image.shape[0]*scale/100)
        #self.img = cv2.resize(cv_image, (width,height),interpolation=cv2.INTER_AREA)
        cv2.rectangle(self.cv_image,(330,130),(630,830),(255,178,102),-1)
        self.draw_grid((20,20),(192,192,192))
        
        #cv2.imshow('Seaclear Graph',self.cv_image)
        #cv2.setMouseCallback('Feed',self.Mouse_Event)
        
        #cv2.startWindowThread()
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
    
    def put_axes(self,color=(0,0,0)):
        pass

    def draw_grid(self, grid_shape, color=(0, 255, 0), thickness=1):
        h, w, _ = self.cv_image.shape
        rows, cols = grid_shape
        dy, dx = h / rows, w / cols

        # draw vertical lines
        for x in np.linspace(start=dx, stop=w-dx, num=cols-1):
            x = int(round(x))
            cv2.line(self.cv_image, (x, 0), (x, h), color=color, thickness=thickness)

        # draw horizontal lines
        for y in np.linspace(start=dy, stop=h-dy, num=rows-1):
            y = int(round(y))
            cv2.line(self.cv_image, (0, y), (w, y), color=color, thickness=thickness)

    def move_graph(self,coords):
        shape=self.cv_image.shape
        y_start=shape[0]-130 #start of blue rectangle y
        x_start=330 #start of blue rectangle x

        y_end=shape[0]-830 #end of blue rectangle y
        x_end=630 #end of blue rectangle x

        center_x=(x_start+x_end)/2
        center_y=(y_start+y_end)/2

        quaternion=(coords.pose.pose.orientation.x,coords.pose.pose.orientation.y,coords.pose.pose.orientation.z,coords.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion) 
        yaw=-euler[2] #rov yaw

        #shift x and y coords
        x=coords.pose.pose.position.y
        y=coords.pose.pose.position.x
        #scale coords around center
        x*=center_x
        y*=center_y
        x+=center_x
        y+=center_y
        #ending arrow
        arrow_length=10 #pixels
        shift_x=math.sin(yaw)*arrow_length
        shift_y=math.cos(yaw)*arrow_length
        x_end=x+shift_x
        y_end=y+shift_y
        cv2.arrowedLine(self.cv_image,(int(x),int(y)),(int(x_end),int(y_end)),(0,0,255),2)
        cv2.imshow('Seaclear Graph',self.cv_image)
        cv2.waitKey(1)


if __name__ == '__main__':

    rospy.init_node('listener', anonymous = True)

    graph = MovingGraph()

    rospy.Subscriber('/odometry/filtered', Odometry, graph.move_graph)

    rospy.spin()