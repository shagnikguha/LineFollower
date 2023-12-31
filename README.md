# Line Follower with ROS Noetic and OpenCV

This project implements a Line Follower robot simulation using ROS Noetic and OpenCV, with Gazebo as the simulation environment.

## Project Structure

### techno_x Package
- **URDF Model**: The robot model is defined in `/techno_x/urdf/robo.xacro`.
- **Test Worlds and Launch Files**: Several test worlds and their corresponding launch files are included in the package.

### follower_line Package
- **OpenCV Code**: The line following logic is implemented in the Python script `follower2.py`, located in `follower_line/scripts/`.
- **Test World and Launch File**: The package includes a test world and a launch file (`black.launch`) to initiate the simulation. 

## Getting Started

1. Clone this repository into your ROS workspace:

    ```bash
    git clone https://github.com/shagnikguha/LineFollower.git
    ```

2. Build the packages:

    ```bash
    cd <your_ros_workspace>
    catkin_make
    ```

## Running the Simulation

1. Launch the Gazebo simulation with the line follower robot:

    ```bash
    roslaunch follower_line black.launch
    rosrun follower_line follower2.py 
    ```

2. [Optional] To utilize an external camera, use the launch file that utilizes the "usb_cam" package: 

    ```bash
    roslaunch follower_line my_cam_launch.launch
    ```
    Make sure to add the proper topic in the `follower2.py` code

## Simulation:

https://github.com/shagnikguha/LineFollower/assets/125911642/56e0bae2-bf9b-4712-aa48-ff421f2fad69

