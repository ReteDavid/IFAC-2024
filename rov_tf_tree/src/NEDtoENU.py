#!/usr/bin/env python


import rosbag
import numpy as np
import rospy
import math

from tf import transformations

def NED2ENU(measurement):
    measurement.linear_acceleration.y*=-1
    measurement.linear_acceleration.z*=-1

    measurement.orientation.y*=-1
    measurement.orientation.z*=-1

    measurement.angular_velocity.y*=-1
    measurement.angular_velocity.z*=-1

    return measurement

def NED2ENU_second(measurement):
    acc_x_aux=measurement.linear_acceleration.x
    measurement.linear_acceleration.x=measurement.linear_acceleration.y
    measurement.linear_acceleration.y=acc_x_aux
    measurement.linear_acceleration.z*=-1

    ang_vel_x=measurement.angular_velocity.x
    measurement.angular_velocity.x=measurement.angular_velocity.y
    measurement.angular_velocity.y=ang_vel_x
    measurement.angular_velocity.z=-measurement.angular_velocity.z
    
    
    aux_orient_x= measurement.orientation.x
    measurement.orientation.x=measurement.orientation.y
    measurement.orientation.y=aux_orient_x
    measurement.orientation.z*=-1

    return measurement

def quat_multiply(q1,q2):
    a=q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2]
    b=q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1]
    c=q1[0]*q2[3]+q1[1]*q2[2]-q1[2]*q2[1]+q1[3]*q2[0]
    w=q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3]
    return (a,b,c,w)
def quat_normalize(q1):
    q1_aux=list(q1)
    q1_len=0
    for axis in q1_aux:
        q1_len+=axis**2
    q1_len=math.sqrt(q1_len)
    for i in range(len(q1_aux)):
        q1_aux[i]/=q1_len      
    return tuple(q1_aux)

def NED2ENU_third(measurement):
    
    acc_x_aux=measurement.linear_acceleration.x
    measurement.linear_acceleration.x=measurement.linear_acceleration.y
    measurement.linear_acceleration.y=acc_x_aux
    measurement.linear_acceleration.z*=-1
    
    ang_vel_x=measurement.angular_velocity.x
    measurement.angular_velocity.x=measurement.angular_velocity.y
    measurement.angular_velocity.y=ang_vel_x
    measurement.angular_velocity.z=-measurement.angular_velocity.z
    

    #q1*q2 means rotation with q2 and then q1: we want to rotate by q1 and then q2=>q2*q1
    q2=(0.7071068, 0.7071068,0,0)#q2
    #q1
    a1=measurement.orientation.x
    b1=measurement.orientation.y
    c1=measurement.orientation.z
    d1=measurement.orientation.w
    q1=(a1,b1,c1,d1)
    #end of q1
    q3=quat_multiply(q2,q1)#default was q2,q1
    q3=quat_normalize(q3)
    measurement.orientation.x=q3[0]
    measurement.orientation.y=q3[1]
    measurement.orientation.z=q3[2]
    measurement.orientation.w=q3[3]

    return measurement

def NED2ENU_fourth(measurement):
    acc_x_aux=measurement.linear_acceleration.x
    measurement.linear_acceleration.x=measurement.linear_acceleration.y
    measurement.linear_acceleration.y=acc_x_aux
    measurement.linear_acceleration.z*=-1

    ang_vel_x=measurement.angular_velocity.x
    measurement.angular_velocity.x=measurement.angular_velocity.y
    measurement.angular_velocity.y=ang_vel_x
    measurement.angular_velocity.z=-measurement.angular_velocity.z
    
    aux_orient_z= measurement.orientation.z
    measurement.orientation.z=measurement.orientation.y
    measurement.orientation.y=aux_orient_z
    

    return measurement


    

bagInName="/home/alineitudor/Licenta/BagFiles/Square.bag"
bagOutName="/home/alineitudor/Licenta/BagFiles/Square2Cov.bag"
bagin=rosbag.Bag(bagInName)
bagout=rosbag.Bag(bagOutName,'w')



n=0
bias=[]
for topic, msg, t in bagin:
    n+=1
    if n==1:
        bias=[msg.linear_acceleration.x, msg.linear_acceleration.y]
        #print(f"first measurement\n {msg}")
        #print(topic)
        break
bagin.close()


variance_acc=1
variance_ang_vel=0.4
variance_orient=0.3
linear_acc_cov=[variance_acc, 0, 0,0,variance_acc,0,0,0,variance_acc]
ang_vel_cov=[variance_ang_vel,0,0,0,variance_ang_vel,0,0,0,variance_ang_vel]
orient_cov=[variance_orient,0,0,0,variance_orient,0,0,0,variance_orient]
bagin=rosbag.Bag(bagInName)
i=0
#print(f"bias {bias}")

file_name="/home/alineitudor/Licenta/BagFiles/IMU and depth/norm.txt"
f=open(file_name,'w')
for topic,msg,t in  bagin:
    #i+=1
    
    #if(i==(n-1)/2 or i==n/2 or i==(n+1)/2):
        #print(f"measured acc while moving{msg.linear_acceleration}")
    msg.header.frame_id="pixhawk"
    msg.linear_acceleration.x-=bias[0]
    msg.linear_acceleration.y-=bias[1]
    
    #msg=NED2ENU(msg)
    msg=NED2ENU_second(msg)
    #msg=NED2ENU_third(msg)
    #msg=NED2ENU_fourth(msg)
    norm=math.sqrt(msg.orientation.x**2+msg.orientation.y**2+msg.orientation.z**2+msg.orientation.w**2)
    eul_ang=transformations.euler_from_quaternion([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
    f.write(str(norm)+"\t"+str(eul_ang)+"\n")

    #if(i==(n-1)/2 or i==n/2 or i==(n+1)/2):
        #print(f"measured acc while moving {msg.linear_acceleration}")
    msg.linear_acceleration_covariance=linear_acc_cov
    msg.angular_velocity_covariance=ang_vel_cov
    msg.orientation_covariance=orient_cov
    bagout.write(topic,msg,t)

f.close()
print(msg)
bagout.close()
bagin.close()

rospy.init_node('da')
rospy.spin()


