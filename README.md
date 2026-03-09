# 🌾 Krishi Drone (KD)

Krishi Drone (KD) is a ROS 2 based autonomous farming drone project focused on:

1. Infected plant detection using vision.
2. Drone stabilization and waypoint navigation in simulation and hardware.
3. Mini-theme mission execution (pickup, traversal, hover/spray, return).

---

## Key Components

1. **Infection Detection**
   - Identifies abnormal plant health patterns from tray images/camera feeds.
   - Implemented in both simulation and hardware pipelines.

2. **Control System**
   - PID-based hovering and waypoint tracking.
   - Action + Service architecture for sequential waypoint missions.

---

## Why Infected Plant Detection Matters

Early infection detection is important for:

- Preventing disease spread to healthy plants.
- Reducing unnecessary pesticide usage.
- Improving intervention speed and consistency.
- Lowering manual monitoring effort.

### Common Indicators of Infection

- **Yellowing leaves**: nutrient stress or disease.
- **Browning/necrosis**: cell damage and progressive infection.
- **Uneven growth**: stress, overwatering, or localized damage.

---

## Detection Methodology (Restored + Implemented)

### 1. ArUco Marker Detection
- Purpose: spatial reference and tray alignment.
- Approach: detect marker corners and standardize ordering.

### 2. Perspective Transformation
- Purpose: convert skewed camera view to top-down aligned view.
- Approach: `cv2.getPerspectiveTransform` + `cv2.warpPerspective`.

### 3. Tray Cropping and Border Removal
- Purpose: isolate plant region and remove irrelevant frame artifacts.
- Approach: HSV thresholding + morphology cleanup.

### 4. Block Segmentation
- Purpose: split tray into analyzable plant cells.
- Approach: geometric segmentation and orientation normalization.

### 5. Plant-wise HSV Analysis
- Purpose: quantify infection ratio per plant region.
- Approach: color-space thresholding and infected-area ratio measurement.

### 6. Most Infected Plant Selection
- Purpose: prioritize actionable targets for spraying.
- Output:
  - Simulation: file output / structured text.
  - Hardware: ROS message/JSON publish for mission stack.

---

## Code Map (Software vs Hardware)

### Infection Detection

**Software (simulation/offline)**
- `software/offline_infected_plant_detection.py`
- `software/simulation_workflow.png`

**Hardware (real-time)**
- `hardware/realtime_infected_plant_detection/realtime_infected_plant_detection.py`
- `hardware/realtime_infected_plant_detection/hardware_workflow.png`
- `hardware/realtime_infected_plant_detection/camera_calibration.yaml`

---

## Task 1C — Control System (Hovering PID)

### Aim 🎯
Stabilize Swift Pico at setpoint `[-7, 0, 20]` in Gazebo with bounded error.

### What we implemented in code

**Main file (simulation)**
- `software/hovering-pid gazebo sim/KD_1002_pico_controller.py`

**Main file (hardware pipeline copy)**
- `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_pico_controller.py`

### Controller approach

- ROS2 node publishes `/drone_command` and `/pid_error`.
- Subscribes to WhyCon pose and PID tuning topics.
- Uses staged enable logic to improve stability:
  - throttle control active first,
  - pitch activated after altitude error reduces,
  - roll activated after pitch axis gets closer.
- RC command saturation applied (`1000` to `2000`) for safe output limits.
- Arm/disarm sequence included for safer startup.

### Setup for new users

```bash
cd ~/pico_ws
colcon build
source install/setup.bash
sudo apt install ros-humble-plotjuggler-ros
```

If dependency issues appear:

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

### Run (Task 1C)

Terminal 1:
```bash
ros2 launch swift_pico task_1c.launch.py
```

Terminal 2:
```bash
ros2 run swift_pico pico_controller_PID.py
```

Optional tuning GUI:
```bash
ros2 launch pid_tune pid_tune_drone.launch.py node_name:=button_ui
```

Optional plots:
```bash
ros2 run plotjuggler plotjuggler
```

---

## Task 2A — Waypoint Navigation (Action + Service)

### Aim 🎯
Navigate Swift Pico through task waypoints using custom ROS 2 Action and Service workflow.

### What we implemented in code

**Simulation files**
- `software/waypoint navigation-simulation/KD_1002_waypoint_service.py`
- `software/waypoint navigation-simulation/KD_1002_pico_server.py`
- `software/waypoint navigation-simulation/KD_1002_pico_client.py`

**Hardware files**
- `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_waypoint_service.py`
- `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_pico_server.py`
- `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_pico_client.py`

### Architecture used

1. **Waypoint Service** returns predefined waypoint sequence.
2. **Action Client** requests waypoints and sends goals one-by-one.
3. **Action Server** executes each goal, publishes feedback, and reports completion after stabilization.

### Important control improvements from our code

- **Exponential moving average filter for WhyCon jitter**:
  - `filtered = alpha*raw + (1-alpha)*prev_filtered`
  - `alpha = 0.25`
- Why used:
  - raw WhyCon positions were jittery,
  - unfiltered derivative caused spikes,
  - filter made control smoother near stabilization zone.
- Added **derivative smoothing** in mini-task server.
- Added **ramp-to-target behavior** for smoother transitions.
- Added integral clamping on lateral axes to reduce windup.

### Run (Task 2A)

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

---

## Task 2B — Mini Theme Run (Simulation Mission)

### Aim 🎯
Execute mission pipeline:

1. Detect infected plants.
2. Navigate to pesticide station and pickup point.
3. Traverse through hoop routes.
4. Hover and spray at infected plant points.
5. Return to ground station.

### What we implemented in code

**Simulation files**
- `software/mini task run -waypoint navigation/KD_1002_waypoint_service.py`
- `software/mini task run -waypoint navigation/KD_1002_pico_server.py`
- `software/mini task run -waypoint navigation/KD_1002_pico_client.py`

**Hardware-mapped files**
- `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_waypoint_service.py`
- `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_pico_server.py`
- `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_pico_client.py`

### Execution flow used

1. Waypoint service provides mission waypoint set.
2. Client triggers sequential goals.
3. Server stabilizes at each target and sends result.
4. Mission logic handles pickup-hover-spray-return segments.

### Run (Task 2B)

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


## Hardware Implementation (Camera Calibration, Arena Setup, Latching Circuit, Arming/Disarming)

### What We Did in Hardware
1. Calibrated the camera and validated projection stability.
2. Built and verified arena setup with all mission props.
3. Implemented a latching circuit (Arduino-based control side) for reliable actuator state holding.
4. Followed strict arming/disarming safety workflow on real drone.
5. Ran hardware mission scripts mapped from simulation architecture.

### 1) Camera Calibration
We calibrated the camera first and saved parameters to:

- `hardware/realtime_infected_plant_detection/camera_calibration.yaml`

Why this was required:
- Raw lens distortion shifts tray and plant coordinates.
- Distorted coordinates break detection-to-navigation mapping.
- Calibration improves geometric consistency for real-time detection and waypoint targeting.

What we validated after calibration:
- Better tray boundary alignment.
- More stable plant cell segmentation.
- Reduced coordinate drift between frames.

### 2) Arena Setup
Arena setup references:

- `hardware/realtime_infected_plant_detection/arena setup,with latching circuit/KD_task_3a_1002_arena_only.png`
- `hardware/realtime_infected_plant_detection/arena setup,with latching circuit/KD_task_3a_1002_arena_with_props.png`
- `hardware/realtime_infected_plant_detection/arena setup,with latching circuit/KD_task_3a_1002_healthy_plant.png`
- `hardware/realtime_infected_plant_detection/arena setup,with latching circuit/KD_task_3a_1002_infected_plant.png`
- `hardware/realtime_infected_plant_detection/arena setup,with latching circuit/KD_task_3a_1002_pesticide_prop.png`

What this means in practice:
- We fixed prop placement so waypoint trajectories remain reproducible.
- We ensured marker/plant visibility in camera FOV before flight.
- We verified that mission-critical points (pickup, hover, spray) are reachable safely with configured path clearances.

### 3) Why We Used a Latching Circuit (Arduino Side)
Circuit reference:

- `hardware/realtime_infected_plant_detection/arena setup,with latching circuit/KD_task_3a_1002_circuit_diagram.png`

Reason:
- In flight, control pulses can be brief/noisy.
- A simple momentary trigger can reset unintentionally.
- We needed actuator state retention (ON/OFF hold) until explicit reset.

Meaning in our mission:
- Payload/spray trigger remains stable after activation.
- Prevents accidental toggles during hover/spray.
- Makes pickup and spray operations deterministic and repeatable.

### 4) Arming/Disarming on Hardware Drone
Hardware validation references:

- `hardware/realtime_infected_plant_detection/arming and disarming drone/KD_task_3c_1002_checklist_front.png`
- `hardware/realtime_infected_plant_detection/arming and disarming drone/KD_task_3c_1002_swift_pico.png`

Safety flow we followed:
1. Start in disarmed state.
2. Verify checklist (clear area, links, power, props safety).
3. Arm only after checks pass.
4. Keep RC outputs bounded during run.
5. Force disarm on stop/fault.

Why this matters:
- Prevents unintended motor start.
- Protects hardware/team during setup and payload handling.
- Ensures repeatable and safe test cycles.

### 5) Hardware Scripts Used
- Hover/controller:
  - `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_pico_controller.py`
- Waypoint service:
  - `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_waypoint_service.py`
- Action server:
  - `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_pico_server.py`
- Action client:
  - `hardware/realtime_infected_plant_detection/hardware waypoint,hovering,mini theme/KD_1002_pico_client.py`
- Hardware real-time detection:
  - `hardware/realtime_infected_plant_detection/realtime_infected_plant_detection.py`

---
