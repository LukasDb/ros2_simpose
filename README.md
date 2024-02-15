# ROS2 6IMPOSE
## Installation using Robostack (micromamba)
Otherwise follow, regular ROS2 installation instructions and install this package
- Setting up environment (or run setup_env.sh)
    - `mamba create -n ros2 -c conda-forge -c robostack-staging ros-humble-desktop-full`
    - `mamba activate ros2`
    - `mamba install ros-humble-desktop-full`
    - `mamba install compilers cmake pkg-config make ninja colcon-common-extensions`
    - navigate to root of this repo
    - `micromamba install ros-humble-realsense2-camera`

    - `colcon build --symlink-install`
    - `source install/setup.bash`


## Testing
- Run `ros2 launch simpose test_simpose.launch.py` to start the siximpose node and rviz2


### Notes
- if you get 'pybind11 rclpy not present on your system' make sure you are using colcon of your env (not the system one)