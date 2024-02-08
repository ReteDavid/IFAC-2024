#!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt
import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import *

class PlotPoints:
    
    def __init__(self):
        self.arrow=None
        self.obj_x=[]
        self.obj_y=[]
        self.rovx=[]
        self.rovy=[]
        pass

    def obj_coord(self,obj_coords):
        self.obj_x.append(obj_coords.pose.pose.position.x)
        self.obj_y.append(obj_coords.pose.pose.position.y)

    def plot_x(self,msg):
        global counter
        if len(self.rovx)>=30:
            self.rovx=self.rovx[1:]+self.rovx[:1]
            self.rovy=self.rovy[1:]+self.rovy[:1]
            self.rovx[-1]=msg.pose.pose.position.y
            self.rovy[-1]=msg.pose.pose.position.x
        else:
            self.rovx.append(msg.pose.pose.position.y)
            self.rovy.append(msg.pose.pose.position.x)
        #print(len(self.rovx))
        '''
        print(len(self.obj_x))
        self.rovx.append(msg.pose.pose.position.y)
        self.rovy.append(msg.pose.pose.position.x)
        '''
        if counter % 10 == 0:
            stamp = msg.header.stamp
            time = stamp.secs + stamp.nsecs * 1e-9
            plt.plot(0, 0, "ro")            
            plt.plot(self.rovx, self.rovy, 'ko')
            plt.plot(self.obj_x,self.obj_y,"g^")
            plt.legend(["Origin","ROV","Objects"])
            
            plt.draw()
            plt.axis("equal")
            axes=plt.gca()
            axes.set_xlim([-1.5,1.5])
            axes.set_ylim([-1.5,1.5])
            self.draw_arrow(msg)
            plt.draw()
            plt.pause(0.00000000001)
            self.arrow.remove()

        counter += 1
    
    def draw_arrow(self,msg):
        quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion) 
        yaw=-euler[2]+90 #rov yaw+90

        x=msg.pose.pose.position.y
        y=msg.pose.pose.position.x

        arrow_length=0.15 #pixels
        shift_x=math.sin(yaw)*arrow_length
        shift_y=math.cos(yaw)*arrow_length

        self.arrow=plt.arrow(x,y,shift_x,shift_y,width=0.015,ec="black")



if __name__ == '__main__':
    counter = 0

    graph=PlotPoints()

    rospy.init_node("plotter")
    rospy.Subscriber("/odometry/filtered", Odometry, graph.plot_x)
    rospy.Subscriber("/Object/coord",Odometry,graph.obj_coord)

    rospy.spin()