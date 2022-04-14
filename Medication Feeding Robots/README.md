# Niryo One Implementation

# Steps to run this code
1. git clone this directory into a workspace and catkin build
2. source the file devel/setup.bash of the workspace
3. in 3 separate terminals, type in commands:
	- roslaunch niryo_one_moveit_config medication.launch
	- rosrun images face_detect
	- roslaunch move_group motion.launch

for command (b), you need to go into the folder where the face_detect.cpp
     file is located ('cd src/images/src')



# Note
install pip if you don't have it already

pip install torch

(if above doesn't work, try pip install --no-cache-dir torch)

pip install future
