# IFAC-2024
A BlueROV2-based platform for underwater mapping experiments


To work with BlueROV2 and ROS+QGroundControl we used the following package found at:
https://github.com/patrickelectric/bluerov_ros_playground.git

Starting GoPRO stream:

	python publish_gopro.py

To launch filter and color detection use (it runs the filter using our bag files, but it can run directly with the ROV):

	roslaunch rov_tf_tree filter.launch
 
To start YOLO detection:

	rosrun recognition ros_recognition_yolo.py
 
Starting mapping graph:

	rosrun rov_tf_tree object_click_test.py
