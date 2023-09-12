# LineFollower
This is line follower code using ROS Noetic and OpenCV. I am using Gazebo to run the simulation.

Package "techno_x" contains URDF model of the robot, several test worlds and their launch files.
(The robot model is stored in /techno_x/urdf/robo.xacro)

Package "follower_line" consists of the OpenCV code (follower3.py), test world and a launch file to open the world (black.launch)
("follower_line/scripts/follower3.py      and      "follower_line/launch/black.launch")




https://github.com/shagnikguha/LineFollower/assets/125911642/67073324-4328-45e5-97c3-1c6937732f6b



*Update: follower2.py is the updated code using a PID class to calculate angle*         





*Update: Added code to make webcam or any other usb camera publish to a topic using Open_CV*
