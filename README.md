# LineFollower
Line Follower code using ROS Noetic and OpenCV. Simulation done in Gazebo

Package "techno_x" contains URDF model of the robot, several test worlds and their launch files.

Package "follower_line" consists of the OpenCV code (follower.py), test world and a launch file to ope n the world (black.launch)
          this package also contains "follower3.py" which combines the functionality of "follower.py" and endpoint detection(detects yellow in the endpoint) and stops
