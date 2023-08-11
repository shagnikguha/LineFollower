# LineFollower
This is line follower code using ROS Noetic and OpenCV. I am using Gazebo to run the simulation.

Package "techno_x" contains URDF model of the robot, several test worlds and their launch files.(The robot model is stored in /techno_x/urdf/robo.xacro)

Package "follower_line" consists of the OpenCV code (follower2.py), test world and a launch file to ope n the world (black.launch)
          this package also contains "follower3.py" which combines the functionality of "follower.py" and endpoint detection(detects yellow in the endpoint) and stops
