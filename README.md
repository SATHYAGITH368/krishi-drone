# 🌾 Krishi Drone (KD)

Krishi Drone (KD) is an autonomous farming drone stack built on ROS 2 for:

1. Detecting infected plants using computer vision.
2. Stabilizing and navigating the Swift Pico drone in Gazebo and hardware.
3. Executing mini-theme mission logic (pickup, hoop traversal, hover/spray, return).

---

## 1) Infected Plant Detection

### Aim
Detect infected plants from tray/crop imagery and publish/report infected locations for downstream navigation and spraying.

### Why it matters
- Early disease detection prevents spread.
- Enables targeted spraying (lower chemical usage).
- Reduces manual inspection effort.

### Pipeline (implemented)
1. ArUco marker detection
2. Perspective correction
3. Tray cropping and border removal
4. Block segmentation
5. Plant-wise HSV analysis
6. Most infected plant selection

### Files (Software vs Hardware)

#### Software (simulation/offline)
- `software/offline_infected_plant_detection.py`
- `software/simulation_workflow.png`

#### Hardware (real-time ROS)
- `hardware/realtime_infected_plant_detection/realtime_infected_plant_detection.py`
- `hardware/realtime_infected_plant_detection/hardware_workflow.png`
- `hardware/realtime_infected_plant_detection/camera_calibration.yaml`

---

## 2) Control System — Task 1C (Hovering PID/LQR)

### Aim 🎯
Develop a controller to stabilize Swift Pico at setpoint `[-7, 0, 20]` in Gazebo.

### Prerequisites 📋
- Task 0, Task 1A, Task 1B completed
- ROS 2 basics (pub/sub, nodes)
- Python or C++
- Control basics (PID/LQR)

### Controller choice
- PID or LQR are both valid.
- Our implementation used PID for primary simulation control.

### What we implemented (important code points)

#### Main hover controller (simulation)
- File: `software/hovering-pid gazebo sim/KD_1002_pico_controller.py`
- Node: `pico_controller`
- Setpoint: `[-7, 0, 20]`
- Topics:
  - Sub: `/whycon/poses`
  - Sub: `/throttle_pid`, `/pitch_pid`, `/roll_pid`
  - Pub: `/drone_command`, `/pid_error`
- Safety:
  - Arm/disarm sequence implemented
  - RC saturation in `[1000, 2000]`
- Strategy:
  - Throttle first
  - Pitch enabled after altitude settles
  - Roll enabled after pitch settles

### Installation / setup for new users

```bash
cd ~/pico_ws
colcon build
source install/setup.bash
sudo apt install ros-humble-plotjuggler-ros
```

If build errors occur:

```bash
sudo apt install ros-humble-camera-info-manager
sudo apt install ros-humble-image-view
cd ~/pico_ws
colcon build
```

Set Gazebo model path:

```bash
echo 'export GZ_SIM_RESOURCE_PATH="$HOME/pico_ws/src/swift_pico_description/models"' >> ~/.bashrc
source ~/.bashrc
```

### Run Task 1C (simulation)

Terminal 1:
```bash
ros2 launch swift_pico task_1c.launch.py
```

Terminal 2 (PID):
```bash
ros2 run swift_pico pico_controller_PID.py
```

or (LQR):
```bash
ros2 run swift_pico pico_controller_LQR.py
```

Optional tuning GUI:
```bash
ros2 launch pid_tune pid_tune_drone.launch.py node_name:=button_ui
```

Optional plotting:
```bash
ros2 run plotjuggler plotjuggler
```

### Scoring targets
- Reach setpoint `[-7, 0, 20]`
- Maintain error within `±0.4` (x,y,z)
- Hold for at least 10 seconds
- Finish inside 60 seconds (faster is better)

### Submission artifacts (Task 1C)
- Controller file renamed: `KD_<team_id>_pico_controller.py/cpp`
- ROS bag files:
  - `task_1c_0.db3`
  - `metadata.yaml`

Zip format:
```bash
zip -r KD_<team_id>_task_1c.zip KD_<team_id>_pico_controller.py task_1c_0.db3 metadata.yaml
```

---

## 3) Waypoint Navigation — Task 2A

### Aim 🎯
Navigate Swift Pico through predefined waypoints in Gazebo using ROS 2 Action + Service architecture.

### Core requirement
- Reach all specified waypoints in sequence
- Hold each waypoint within `±0.4` for at least 2 seconds
- Complete mission within 180 seconds

### Architecture implemented

1. **Waypoint Service** provides waypoint list.
2. **Action Client** requests waypoints and sends goals sequentially.
3. **Action Server** executes navigation + PID control and publishes feedback/result.

### Files (simulation)
- `software/waypoint navigation-simulation/KD_1002_waypoint_service.py`
- `software/waypoint navigation-simulation/KD_1002_pico_client.py`
- `software/waypoint navigation-simulation/KD_1002_pico_server.py`
- `software/waypoint navigation-simulation/KD_1002_rqt_graph.png`

### Files (hardware)
- `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_waypoint_service.py`
- `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_pico_client.py`
- `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_pico_server.py`

### Important implementation notes from code
- Action server receives `NavToWaypoint` goals.
- Service provides waypoint array (`GetWaypoints`).
- Yaw orientation taken from `/rotors/odometry`.
- PID loop runs with bounded RC output.
- Stabilization timer at each waypoint before goal success.

### Why we used filtering for WhyCon jitter
In waypoint server code, we observed noisy/jittery WhyCon pose, causing unstable derivatives and command spikes. To improve control smoothness:

- Implemented **Exponential Moving Average** filter:
  - `filtered = alpha * raw + (1 - alpha) * filtered_prev`
  - `alpha = 0.25`
- This reduced noise while preserving responsiveness.
- Implemented in:
  - `software/waypoint navigation-simulation/KD_1002_pico_server.py`
  - `software/mini task run -waypoint navigation/KD_1002_pico_server.py`
  - `hardware/.../KD_1002_pico_server.py`

### Run Task 2A

Terminal 1:
```bash
ros2 launch swift_pico task_2a.launch.py
```

Terminal 2:
```bash
ros2 run swift_pico waypoint_service.py
```

Terminal 3:
```bash
ros2 run swift_pico pico_server.py
```

Terminal 4:
```bash
ros2 run swift_pico pico_client.py
```

Record bag:
```bash
ros2 bag record -o task_2a /whycon/poses
```

### Submission artifacts (Task 2A)
- `KD_<team_id>_pico_client.py/cpp`
- `KD_<team_id>_pico_server.py/cpp`
- `KD_<team_id>_waypoint_service.py/cpp`
- `KD_<team_id>_rqt_graph.png`
- `task_2a_0.db3`
- `metadata.yaml`

---

## 4) Mini Theme Run — Task 2B

### Aim 🎯
Run full mission: detect infected plants, collect pesticide, pass hoops, hover/spray infected plants, and return home.

### Mission blocks implemented
1. Ground → pesticide station
2. Hoops traversal transitions
3. Block-wise infected plant hover
4. Spray (descent + hold)
5. Return to ground station

### Files (simulation mini run)
- `software/mini task run -waypoint navigation/KD_1002_waypoint_service.py`
- `software/mini task run -waypoint navigation/KD_1002_pico_server.py`
- `software/mini task run -waypoint navigation/KD_1002_pico_client.py`
- `software/mini task run -waypoint navigation/KD_1002_rqt_graph.png`

### Files (hardware mini run)
- `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_waypoint_service.py`
- `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_pico_server.py`
- `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_pico_client.py`

### Run Task 2B

Terminal 1:
```bash
ros2 launch swift_pico task_2b.launch.py
```

Terminal 2:
```bash
ros2 run swift_pico waypoint_service.py
```

Terminal 3:
```bash
ros2 run swift_pico pico_server.py
```

Terminal 4:
```bash
ros2 run swift_pico pico_client.py
```

Record bag:
```bash
ros2 bag record -o task_2b /whycon/poses
```

### Submission artifacts (Task 2B)
- `KD_<team_id>_pico_client.py/cpp`
- `KD_<team_id>_pico_server.py/cpp`
- `KD_<team_id>_waypoint_service.py/cpp`
- `KD_<team_id>_rqt_graph.png`
- `task_2b_0.db3`
- `metadata.yaml`

---

## 5) Repository Map

- `software/`
  - `offline_infected_plant_detection.py`
  - `hovering-pid gazebo sim/`
  - `waypoint navigation-simulation/`
  - `mini task run -waypoint navigation/`
- `hardware/realtime_infected_plant_detection/`
  - `realtime_infected_plant_detection.py`
  - `hardware waypoint,hovering,mini theme/`
  - arena and circuit references

---

## 6) What a New User Should Do First

1. Complete ROS 2 + Swift Pico workspace setup (Task 1A baseline).
2. Build workspace:
   ```bash
   cd ~/pico_ws
   colcon build
   source install/setup.bash
   ```
3. Run Task 1C first and stabilize hover.
4. Move to Task 2A (service + action flow).
5. Move to Task 2B (full mini-theme mission).
6. Record rosbag + rqt graph + unlisted YouTube demo.

---

## 7) Team

- Team ID: `1002`
- Authors: `sathyagith`, `rudra`, `shavya`, `nagendra`

