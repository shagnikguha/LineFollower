# Line Follower with ROS Noetic and OpenCV

This project implements a Line Follower robot simulation using ROS Noetic and OpenCV, with Gazebo as the simulation environment.

## Project Structure

### techno_x Package
- **URDF Model**: The robot model is defined in `/techno_x/urdf/robo.xacro`.
- **Test Worlds and Launch Files**: Several test worlds and their corresponding launch files are included in the package.

### follower_line Package
- **OpenCV Code**: The line following logic is implemented in the Python script `follower2.py`, located in `follower_line/scripts/`.
- **Test World and Launch File**: The package includes a test world and a launch file (`black.launch`) to initiate the simulation. Introduced a new launch file that leverages the "usb_cam" package to access external cameras, providing flexibility for different camera setups.

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
    ```

2. [Optional] To utilize an external camera, launch the new launch file:

    ```bash
    roslaunch follower_line external_camera.launch
    ```

## Additional Notes

- Carefully review and modify launch files and parameters to suit your specific setup.
- Explore and customize the code (`follower2.py` and `follower3.py`) to meet your project requirements.

## Contribution

Feel free to contribute to this project by creating issues or submitting pull requests.

## License

This project is licensed under the [MIT License](LICENSE).

For a visual overview, visit the [Line Follower Project](https://github.com/shagnikguha/LineFollower) on GitHub.
