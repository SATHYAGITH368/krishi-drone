# krishi-drone
Krishi Drone (KD) is a small autonomous farming drone that helps protect crops by spotting pests and plant diseases using computer vision. Guided by an overhead camera, it flies precisely to problem areas and sprays biopesticides only where needed, using ROS 2 and control system with simulation and real hardware support

There are two main important parts Detection of infection/abnormalities in plant growth then is the pid/lqr control system implementation for hovering and also full theme run,waypoint navigation .

## Infected Plant Detection

This repository implements infected plant detection using computer vision and ROS integration for real-time monitoring. It has two implementations:
Simulation (software) – works on static images
Real plant detection (hardware) – works on live camera streams using ROS
The goal is to enable precision agriculture by identifying plants showing early signs of infection or stress.

### Why Infected Plant Detection Matters
Early detection of plant infection is critical in agriculture to:
Prevent the spread of disease to healthy plants
Optimize water and fertilizer usage
Automate visual inspection to save labor
Improve research accuracy in controlled agriculture experiments
Infected plants often show:
Yellowing leaves (nutrient deficiency or fungal infection)
Browning or necrosis (cell death or disease)
Uneven growth patterns (environmental stress or overwatering)

### Methodology Overview
The detection pipeline involves multiple steps from tray identification to most infected plant selection. The process has been visualized in the repository as workflow images:
Simulation workflow: software/simulation_workflow.png
Hardware workflow: hardware/hardware_workflow.png

#### ArUco Marker Detection
Purpose: Detect reference markers placed at tray corners to ensure accurate alignment.
Implementation:
Multiple ArUco dictionaries are tried to detect at least four markers.
Corner positions are sorted to maintain consistent order (top-left, top-right, bottom-right, bottom-left).
Why: Helps correct for camera tilt and skew.
Learning Resources: OpenCV ArUco Markers

#### Perspective Transformation
Purpose: Converts skewed images into a top-down view of the tray.
Implementation:
Detected ArUco corners are used as source points.
Destination points define a perfect rectangle.
cv2.getPerspectiveTransform and cv2.warpPerspective generate the warped image.
Why: Ensures consistent orientation for all tray blocks.
Learning Resources: Perspective Transform in OpenCV

#### Tray Cropping & Border Removal
Purpose: Remove non-plant areas (tray edges, soil, background) for cleaner analysis.
Implementation:
HSV color thresholding identifies white tray regions.
Morphological operations remove noise.
Borders are removed to focus on the plant-containing area only.

#### Block Segmentation
Purpose: Divide the tray into two bottom blocks and segment each block into individual plant cells (2 columns × 3 rows).
Implementation:
Bottom half of the tray is extracted.
Each block is cropped and rotated for uniform orientation.
Each block is further divided into individual plant cells for analysis.

#### Plant-wise HSV Analysis
Purpose: Identify infected regions based on leaf color changes (yellow/brown).
Implementation:
Each plant cell is converted to HSV.
HSV thresholds detect stressed or infected pixels.
Infection ratio = (# infected pixels) / (total pixels)

#### Most Infected Plant Identification
Purpose: Determine which plant in each block has the highest infection ratio.
Outputs:
Simulation: output_task1a.txt containing most infected plants.
Hardware (ROS): Published as a JSON string on /detected_plants.

### Folder Structure
Simulation (Software)
software/
├── simulation_workflow.png      # Workflow image for simulation
├── offline_infected_plant_detection.py        # Main script for static image processing
└── output_task1a.txt            # Example output

Purpose: Processes static images and detects infected plants in trays.

Real Plant Detection (Hardware)
hardware/
├── hardware_workflow.png        # Workflow image for real plant detection
├── infected_plants_node.py      # ROS node subscribing to camera images
         
Purpose: Processes live ROS camera frames to detect infected plants in real-time.
Visualization: Saves blocks and plant cells for verification.
ROS Integration: Publishes results to /detected_plants topic using std_msgs/String.

### Agricultural Relevance
Provides early warning of infection or stress
Enables targeted intervention on most affected plants
Reduces manual labor and inspection time
Ensures consistent, reproducible results for research and greenhouse studies

### Notes:
Workflow images provide step-by-step visualization:
Simulation workflow: software/simulation_workflow.png
Hardware workflow: hardware/hardware_workflow.png
All code is modular, allowing easy extension for larger trays or different plant arrangements.
