# Geometry Base Path Planning for Parallel Parking
>Note: This package was build and test in Ubuntu 24.04 with ROS2 Jazzy

## Table of Contents
- [Use This Package](#use-package)
- [Project Overview](#project-overview)
- [Vehicle Model](#vehicle-model)
- [Core Strategy](#core-strategy)
- [Simulation Method](#simulation-method)
- [Result](#result)
- [Discussion](#discussion)
- [Conclusion](#conclusion)
- [Reference](#reference)


## Use This Package

### Environment
- OS: ubuntu 24.04
- ROS2: Jazzy
- Carla: 0.9.16
- Carla-ROS bridge

### Steps
1. Clone this package
    ```bash
    cd ~
    git clone -b SINGLE https://github.com/phattanaratjeedjeen-sudo/parallel_parking.git
    ```

2. Set up environment
    ```bash
    cd ~/park_ws
    colcon build && source install/setup.bash
    ```

    ```bash
    echo "source ~/park_ws/install/setup.bash" >> ~/.bashrc  
    source ~/.bashrc
    ```

3. Launch
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


## Discussion


## Conclusion


## Reference
1. 
