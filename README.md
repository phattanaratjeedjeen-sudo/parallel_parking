# Geometry Base Path Planning for Parallel Parking

## Table of Contents
- [File Structure](#file-structure)
- [Project Overview](#project-overview)
- [Use This Package](#use-package)
- [Vehicle Model](#vehicle-model)
- [Core Strategy](#core-strategy)
- [Simulation Method](#simulation-method)
- [Result](#result)
- [Discussion](#discussion)
- [Conclusion](#conclusion)
- [Reference](#reference)


## File Structure

### Carla-Ros
```text
src/lka_bringup/
├── config/
│   └── objects.json                -> spawn configuration
├── launch/
│   ├── bring_up_carla.launch.py    -> manage carla-ros connection
│   └── parking_control.launch.py   -> main launch file
└── scripts/
    ├── log_data.py                 -> logger output      
    ├── park_planning.py            -> main script
    ├── plot_results.py             -> plot indivedual result
    └── update_spawn_config.py      -> change front obstrucle spawn point
```

### Pure Carla
( add here )


## Project Overview
This project focuses on developing, simulating, and evaluating geometric path planning algorithms for autonomous parallel parking. It is designed to handle varying parking space constraints and compute mathematically optimal trajectories using a kinematic **bicycle model**.

The core of the system calculates feasible paths using the vehicle's minimum turning radius, wheelbase, and overhang constraints. Depending on the parking spot length and maximum steering angle, the planner dynamically switches between different parking strategies:

- **Single-Trial Parking** 
    Utilized when the parking space is large enough. The algorithm computes a continuous two-arc reverse maneuver, finding the exact tangent point to smoothly transition between opposing steering angles.

- **Crab-Like (Shunt) Parking**
    Initiated when the space is tight but strictly larger than the vehicle footprint. The algorithm calculates the required lateral offset and executes a sequence of forward-backward shunting loops, alternating steering extremes to laterally shift the vehicle deeply into the spot without collisions.

- **Human-Like Parking**
    ( add here )

### Limitations
This project focuses on the core kinematics of parallel parking between two longitudinal obstacles. The simulated environment assumes the following constraints:
- The vehicle must maneuver between a defined front and rear vehicle (represented by static bounding boxes).
- The simulation currently assumes infinite lateral space opposite the parking spot (i.e., no oncoming traffic or opposite curbs to restrict the turning radius).
- The system does not currently include perception-based detection or collision avoidance for lateral boundaries such as sidewalks, curbs, or walls.

### Key Performance Indicators 
To evaluate the efficiency and accuracy of each parking strategy, the system tracks specific KPIs during simulation:

- **Number of Gear Changes**
    Measures the maneuver's efficiency. Fewer gear changes more optimal path.

- **Final Position Accuracy**
    Evaluates the vehicle's final resting pose (X, Y, and Yaw) compared to the ideal target center of the parking spot to ensure safe alignment.

### Simulation Approach
The algorithms are rigorously tested within the CARLA simulator integrated with ROS 2. The simulation method involves systematically varying key physical constraints specifically the parking spot length ($L$) and the vehicle's maximum steering angle ($\delta_{max}$) to observe how the planners adapt. Automated data logging records the vehicle's odometry, steering commands, and KPIs across these varying scenarios to evaluate the robustness, mathematical limits, and overall success rate of each parking method.


## Use This Package
This package is devinded to 2 version 
- Carla-ROS: implement single trial and crab-like parking method 
- Pure Carla: implement human-like parking method

### Carla-Ros

#### Environment
- OS: ubuntu 24.04
- ROS2: Jazzy
- Carla: 0.9.16
- Carla-ROS bridge

#### Steps
1. Clone this package
    ```bash
    cd ~
    git clone -b SINGL https://github.com/phattanaratjeedjeen-sudo/parallel_parking.git
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

3. Set up parking spot
    ```bash
    # set park length to 6.2m
    python3 ~/park_ws/src/lka_bringup/scripts/update_spawn_config.py 6.2
    ```

4. Launch
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
    # L must match with step3
    ros2 launch lka_bringup parking_control.launch.py L:=6.2 max_steer:=35.0
    ```

### Pure Carla

#### Environment
( add here )

#### Steps
( add here )


## Vehicle Model
This project utilizes a standard **kinematic bicycle model** to plan and evaluate parking trajectories. The model simplifies the vehicle's kinematics by representing the two front wheels as a single steerable front wheel, and the two rear wheels as a single fixed rear wheel. 

To guarantee collision-free maneuvers, path planning is calculated using the center of the rear axle as the primary reference frame. The vehicle's footprint is expanded using dimensions like overhangs and mirror offsets to ensure the bounding box does not intersect with obstacles.

![vehecle parameters](/material/images/vihecle_parm.png)

### Vehicle Parameters
Below are the core vehicle(Tesla Model 3) parameters:

| Notation | Description | Default Value |
| :------------------ | :---------- | :------------ |
| $\delta_{max}$  | Maximum steering angle limit                               | `35.0` deg |
| $a$             | Wheelbase (distance from rear axle to front axle)          | `3.005` m  |
| $b$             | Half track width                                           | `0.835` m  |
| $d_{front}$     | Front overhang (distance from front axle to front bumper)  | `0.81` m   |
| $d_{rear}$      | Rear overhang (distance from rear axle to rear bumper)     | `0.98` m   |
| $d_{side}$      | Side overhang (distance to the outer edge of side mirrors) | `0.25` m   |
| $L_{car}$       | Total vehicle length ($a + d_{front} + d_{rear}$)          | `4.795` m  |
| $offset_{rear}$ | Distance from the vehicle's origin to the rear axle        | `1.42` m   |

Using these parameters, the system dynamically calculates essential pathing limits, such as the minimum turning radius of the rear axle ($R_{E, min}$) based on Ackermann steering geometry:  

$$
R_{E, min} = \frac{a}{\tan(\delta_{max})}
$$

## Core Strategy

### Single Trial
This maneuver is triggered when the parking space is large enough ($L \ge L_{min}$) to allow continuous reverse entry using two circular arcs. Let the starting rear axle pose be $(x_s, y_s, \psi_s)$ and the target rear axle pose be $(x_t, y_t, \psi_t)$.

![single trial](/material/images/single.png)

1. **Minimum Spot Length Calculation**

The minimum required parking length $L_{min}$ to park without shunting is determined by the vehicle's turning radius and spatial footprint:

$$
R_{Bt, min} = \sqrt{(R_{E, min} + b + d_{side})^2 + (a + d_{front})^2}
$$

$$
L_{min} = d_{rear} + \sqrt{R_{Bt, min}^2 - (R_{E, min} - b - d_{side})^2}
$$

2. **Target Turning Center ($C_t$)**
    
The final arc always uses the minimum turning radius ($R_{E, min}$). For a right-side parking maneuver, its center $C_t$ is:

$$
C_{t,x} = x_t + R_{E,min} \cos\left(\psi_t + \frac{\pi}{2}\right)
$$

$$
C_{t,y} = y_t + R_{E,min} \sin\left(\psi_t + \frac{\pi}{2}\right)
$$

3. **Feasibility Check (Condition to Begin)**
    
The distance from the start position to the target turning center $d_{Ct,Einit} = \sqrt{(C_{t,x} - x_s)^2 + (C_{t,y} - y_s)^2}$ must satisfy a minimum geometric bound. Where $\alpha$ is the angle between the starting lateral vector and the vector to $C_t$:
    
$$
d_{Ct,Einit,min} = R_{E,min} \cos\alpha + \sqrt{(R_{E,min} \cos\alpha)^2 + 3 R_{E,min}^2}
$$

If $d_{Ct,Einit} < 1.05 \cdot d_{Ct,Einit,min}$ (for safety) the path is unfeasible, and the vehicle must adjust forward to create more space.

4. **First Arc Radius and Steering Angle**
    
If feasible, the turning radius for the first arc ($R_{E,init}$) is calculated to ensure the two circles perfectly touch:

$$
R_{E,init} = \frac{d_{Ct,Einit}^2 - R_{E,min}^2}{2 R_{E,min} + 2 d_{Ct,Einit} \cos\alpha}
$$

The initial steering angle ($\delta_{init}$) is computed via Ackermann geometry:

$$
\delta_{init} = \arctan\left(\frac{a}{R_{E,init}}\right)
$$

5. **Initial Turning Center ($C_i$) & Tangent Point ($T_e$)**
The center for the first arc ($C_i$) is located at distance $R_{E,init}$ perpendicular to the starting orientation:
$$
C_{i,x} = x_s + R_{E,init} \cos\left(\psi_s - \frac{\pi}{2}\right)
$$
$$
C_{i,y} = y_s + R_{E,init} \sin\left(\psi_s - \frac{\pi}{2}\right)
$$
    
The transition tangent point ($T_e$) is where the vehicle switches steering. It lies on the line segment connecting $C_i$ and $C_t$, weighted by their radii:
$$
T_{e,x} = C_{i,x} + \frac{R_{E,init}}{R_{E,init} + R_{E,min}} (C_{t,x} - C_{i,x})
$$
$$
T_{e,y} = C_{i,y} + \frac{R_{E,init}}{R_{E,init} + R_{E,min}} (C_{t,y} - C_{i,y})
$$

| Notation | Description | unit |
| :--- | :--- | :--- |
| $L_{min}$          | Minimum required parking length to park without shunting            | m   |
| $R_{Bt, min}$      | Minimum turning radius of the vehicle's bounding box                | m   |
| $x_s, y_s, \psi_s$ | Starting pose of the vehicle's rear axle (X, Y, Yaw)                | m   |
| $x_t, y_t, \psi_t$ | Target pose of the vehicle's rear axle (X, Y, Yaw)                  | m   |
| $C_t$              | Turning center of the target (final) arc                            | m   |
| $\alpha$           | Angle between the starting lateral vector and the vector to $C_t$   | rad |
| $d_{Ct,Einit}$     | Distance from the start position to the target turning center $C_t$ | m   |
| $d_{Ct,Einit,min}$ | Minimum required distance to $C_t$ for a feasible single-trial path | m   |
| $R_{E,init}$       | Turning radius of the first arc                                     | m   |
| $R_{E,min}$        | Minimum turning radius of the rear axle                             | m   |
| $\delta_{init}$    | Steering angle required for the first arc                           | rad |
| $C_i$              | Turning center of the initial (first) arc                           |  |
| $T_e$              | Tangency point where the vehicle transitions between the two arcs   |  |

### Crab-Like Parking
When the parking spot is too tight for a single maneuver ($L_{car} < L < L_{min}$), the system employs a shunting strategy modeled after crab-walking. 

![multi trial](/material/images/multi.png)

1. **Lateral Offset**
The algorithm calculates the maximum depth the vehicle *can* safely reach in a single initial reverse trial. It sets a temporary parallel target pose outside the spot by a lateral distance of $d_{offset}$.

2. **Shunting Loops**
Once aligned at this offset, the vehicle shifts laterally into the spot by performing $N$ forward-backward loops. During each longitudinal stroke, the steering smoothly alternates between maximum left and right angles ($\pm\delta_{max}$) to maximize lateral movement.

3. **Displacement Math**
The lateral displacement per stroke ($\Delta y$) relies on the available longitudinal clearance ($l_{stroke}$):
$$
l_{stroke} = (L - L_{car}) - 2 \cdot offset_{limit}
$$
$$
\Delta y = 2 \cdot \left( R_{E,min} - \sqrt{ \max\left(0, R_{E,min}^2 - \left(\frac{l_{stroke}}{2}\right)^2\right) } \right)
$$
   
The required number of total shunting loops ($N_{trials}$) is computed dynamically to cover the total $d_{offset}$:
$$
N_{trials} = \left\lfloor \frac{d_{offset}}{2 \cdot \Delta y} + 0.5 \right\rfloor
$$

| Notation | Description | unit |
| :--- | :--- | :--- |
| $L_{car}$        | Total length of the vehicle                                      |
| $L$              | Length of the parking spot                                       |
| $d_{offset}$     | Total lateral distance needed to shift into the parking spot     |
| $l_{stroke}$     | Available longitudinal clearance for each shunting stroke        |
| $offset_{limit}$ | Minimum safety clearance maintained from obstacles during shunts |
| $\Delta y$       | Lateral displacement gained per single longitudinal stroke       |
| $N_{trials}$     | Total number of required shunting loops (1 forward + 1 backward) |

### Human-Like Parking
    ( add here )


## Simulation Method
To test all planning method (single, crab-like, human-like). There are 2 parameter to vary
1. the parking spot length
2. the maximum steering angle
While vary these parameters. Drived car's spawn point, obstrucles size, park offset and target parking position are fixed.

| Notation(unit) | Description | Value(start/step/stop) | type |
| --------  | -------- | -------- | -------- |
| $\delta_{max}$(deg) | maximum steering angle               | 30/5/40     | vary |
| $L$(m)              | park spot length                     | 5.2/0.5/7.2 | vary |
| $c$(m)              | rear-front park offset               | 0.1         | fix  |
| $p$(m)              | front obstrucle-car side distance    | 0.3         | fix  |
| $x_s, y_s$ (m)      | spawn position                       |             | fix  |
| $x_f, y_f$ (m)      | target position                      |             | fix  |

![start and desired condition](/material/images/sim_con.png)
Log parameters

| Notation(unit) | Description | 
| -------- | -------- | 
| n(times) | number of times the gear was changed | 
| x, y (m) | car position                         | 
 
## Result


## Discussion


## Conclusion


## Reference
1. H. Vorobieva, S. Glaser, N. Minoiu-Enache, and S. Mammar, "Geometric path planning for automatic parallel parking in tiny spots," IFAC Proceedings Volumes, vol. 45, no. 24, pp. 36–42, 2012, doi: 10.3182/20120912-3-BG-2031.00008.