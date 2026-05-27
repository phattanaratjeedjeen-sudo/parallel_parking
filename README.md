# CARLA Autonomous Parallel Parking

A fully autonomous parallel parking system built on the [CARLA](https://carla.org/) simulator. An Audi e-tron ego vehicle detects a parking gap between two stationary cars and executes a multi-phase manoeuvre using a state-machine controller with PID speed regulation, obstacle sensors, IMU feedback, and real-time trajectory logging.

---

## Features

- **7-state parking state machine** — pull-up → reverse arc → steer unwind → shunt loop → parked
- **PID speed controller** (State 1) for smooth forward pull-up
- **Dynamic geometry** — wheelbase, turning radius, and reference path are derived at runtime from the vehicle's physics model
- **6 obstacle sensors** (front, rear, FL, FR, RL, RR) with colour-coded HUD alerts
- **Real-time Pygame display** — external/cockpit view toggle (`T`), reference path toggle (`C`)
- **Automatic data logging**
  - Pose log (x, y, elapsed time, velocity, acceleration, jerk, direction) → timestamped `.txt`
  - Steering angle log (elapsed time, wheel angle °) → timestamped `.txt`
  - Vehicle model + parking constants → timestamped `.csv`
- **Graph plotter** — one command renders 6 analysis plots from any saved run

---

## Project Structure

```
.
├── spawn_car.py      # Main autonomous parking simulation
├── graphplotter.py   # Plot trajectory & dynamics from saved logs
├── pathtest.py       # Standalone geometry / turning-radius calculator
└── README.md
```

---

## Prerequisites

| Dependency | Notes |
|---|---|
| [CARLA Simulator](https://carla.org/) | Tested on CARLA 0.9.x — must be running before launching the script |
| Python 3.7+ | |
| `carla` Python package | Provided with the CARLA installation (`PythonAPI/`) |
| `pygame` | `pip install pygame` |
| `numpy` | `pip install numpy` |
| `matplotlib` | `pip install matplotlib` (for `graphplotter.py`) |
| `keyboard` | `pip install keyboard` |

---

## Getting Started

### 1. Start the CARLA server

```bash
# Windows
CarlaUE4.exe

# Linux
./CarlaUE4.sh
```

### 2. Run the parking simulation

```bash
python spawn_car.py
```

The script will:
1. Connect to `localhost:2000`
2. Spawn two parked Audi e-tron vehicles and one ego vehicle
3. Attach RGB camera, LiDAR, IMU, and 6 obstacle sensors to the ego car
4. Execute the autonomous parking sequence autonomously
5. Save logs on exit

### 3. Plot the results

After a run, two log files are created in the working directory:

```
New_Audi_etron_x=16_yaw=35_pose.txt    # or the auto-timestamped green_path_*.txt
New_Audi_etron_x=16_yaw=35_steer.txt   # or steer_log_*.txt
```

Edit the filenames at the top of `graphplotter.py` to match, then run:

```bash
python graphplotter.py
```

This opens a 2×3 figure with:

| Graph | Content |
|---|---|
| 1 | Trajectory — X vs Y position |
| 2 | Velocity over time (m/s) |
| 3 | X and Y position over time |
| 4 | Acceleration over time (m/s²) |
| 5 | Jerk over time (m/s³) |
| 6 | Steering wheel angle over time (°) |

---

## Controls (during simulation)

| Key | Action |
|---|---|
| `T` | Toggle external ↔ cockpit camera view |
| `C` | Toggle reference path visibility |
| Close window | End simulation and save logs |

---

## Parking State Machine

```
State 1  FWD pull-up      Drive forward to pull-up X using PID control
State 2  Arc 1 rev-right  Reverse with full right lock until yaw ≥ 45°
State 3  Steer unwind      Hold brake, gradually unwind steering to 0°
State 5  Shunt loop        Alternate fwd/rev shunts (full-lock each way)
                           until |yaw| ≤ 2° and speed < 0.02 m/s
State 6  Parked            Apply hand-brake; log total shunting actions
```

> State 4 (straight reverse) is available in the codebase for alternative manoeuvre strategies.

---

## Output Files

| File | Description |
|---|---|
| `green_path_<timestamp>.txt` | JSON array — `[x, y, elapsed_s, velocity_ms, accel_ms2, jerk_ms3, direction]` per frame |
| `steer_log_<timestamp>.txt` | JSON array — `[elapsed_s, wheel_angle_deg]` per frame |
| `parking_model_<timestamp>.csv` | Vehicle specs, wheel parameters, gear ratios, and all parking constants |

---

## Configuration

Key constants at the top of `spawn_car.py`:

```python
MAX_STEER_ANGLE   = 30.0   # Physical wheel lock (degrees)
THROTTLE_FWD      = 0.20   # Forward throttle in shunt loop
THROTTLE_REV      = 0.32   # Reverse throttle
PULL_UP_X         = 14.5   # X coordinate to stop before reversing
ARC1_YAW_DEG      = 45     # Yaw target for the first reversing arc
```

PID gains (State 1):

```python
Kp, Ki, Kd = 0.7, 0.0, 0.7
```

---

## License

This project is provided for research and educational use. CARLA is licensed under the MIT License.
