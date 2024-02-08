#!/usr/bin/env python


import rosbag
    

bagInName="/home/catkin_ws/src/bagFiles/Data_2023-11-17-perfect-square.bag"
bagOutName="/home/catkin_ws/src/bagFiles/rov_cam_feed_square2.bag"
bagin=rosbag.Bag(bagInName)
bagout=rosbag.Bag(bagOutName,'w')

for topic,msg,t in  bagin:

    if topic=="/bluerov2/camera/image_raw/compressed":
        bagout.write(topic,msg,t)

print("done")
bagout.close()
bagin.close()



