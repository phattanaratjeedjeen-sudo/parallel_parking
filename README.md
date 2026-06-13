# Geometry Base Path Planning for Parallel Parking
>Note: This package was build and test in Ubuntu 24.04 with ROS2 Jazzy

## Table of Contents
- [Use This Package](#use-package)
- [Project Overview](#project-overview)
- [Vehicle Model](#vehicle-model)
- [Core Strategy](#core-strategy)
- [Simulation Method](#simulation-method)
- [Result](#result)
- [Reference](#reference)


## Use This Package

1. Install CARLA 0.9.16

2. Install CARLA ROS bridge

3. Clone this package
    ```bash
    cd ~
    git clone -b SINGLE https://github.com/phattanaratjeedjeen-sudo/parallel_parking.git
    ```

4. Set up environment
    ```bash
    cd ~/park_ws
    colcon build && source install/setup.bash
    ```

    ```bash
    echo "source ~/park_ws/install/setup.bash" >> ~/.bashrc  
    source ~/.bashrc
    ```

5. Launch
    ```bash
    cd ~/carla
    ./CarlaUE4.sh -windowed -ResX=800 -ResY=600 -prefernvidia -quality-level=Low
    ```

    Open new terminal
    ```bash
    cd ~/park_ws
    ros2 launch lka_bringup bring_up_carla.launch.py
    ```

    Open new terminal
    ```bash
    cd ~/park_ws
    parking_planning.launch.py
    ```


## Project Overview


## Vehicle Model


## Core Strategy


## Simulation Method


## Result


## Reference
1. 
