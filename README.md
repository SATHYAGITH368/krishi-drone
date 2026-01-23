# 🌾 Krishi Drone (KD)

Krishi Drone (KD) is a small autonomous farming drone designed to protect crops by detecting pests and plant diseases using computer vision. Guided by an overhead camera, it precisely targets problem areas and applies biopesticides only where needed. The system is built on ROS 2 and supports both simulation and real hardware.

## Key Components

1. **Infection Detection**
   - Identifies abnormalities in plant growth using computer vision and ROS for real-time monitoring.
   - Implemented in two modes:
     - **Simulation (Software)**: Works on static images.
     - **Real Plant Detection (Hardware)**: Operates on live camera streams using ROS.

2. **Control System**
   - Implements PID/LQR control for hovering, waypoint navigation, and full mission execution.

## Why Infected Plant Detection Matters

Early detection is crucial for:

- Preventing disease spread to healthy plants.
- Optimizing water and fertilizer usage.
- Automating visual inspections to save labor.
- Improving research accuracy in controlled experiments.

### Common Indicators of Infection

- **Yellowing Leaves**: Nutrient deficiency or fungal infection.
- **Browning/Necrosis**: Cell death or disease.
- **Uneven Growth**: Environmental stress or overwatering.

## Methodology Overview

The detection pipeline involves several steps, visualized in workflow images:

- **Simulation Workflow**: `software/simulation_workflow.png`
- **Hardware Workflow**: `hardware/hardware_workflow.png`

### Detection Steps

#### ArUco Marker Detection

- **Purpose**: Align trays accurately using reference markers.
- **Implementation**: 
  - Uses multiple ArUco dictionaries to detect markers.
  - Sorts corners to maintain consistent order.
- **Why**: Corrects camera tilt and skew.

#### Perspective Transformation

- **Purpose**: Converts images to a top-down view.
- **Implementation**: 
  - Uses detected corners as source points.
  - Applies `cv2.getPerspectiveTransform` and `cv2.warpPerspective`.
- **Why**: Ensures consistent orientation.

#### Tray Cropping & Border Removal

- **Purpose**: Focuses analysis on plant areas.
- **Implementation**: 
  - Uses HSV thresholding to identify tray regions.
  - Removes noise with morphological operations.

#### Block Segmentation

- **Purpose**: Divides trays into analyzable units.
- **Implementation**: 
  - Extracts and rotates blocks for uniform orientation.
  - Segments into individual plant cells.

#### Plant-wise HSV Analysis

- **Purpose**: Detects infected regions by color changes.
- **Implementation**: 
  - Converts cells to HSV.
  - Calculates infection ratio.

#### Most Infected Plant Identification

- **Purpose**: Identifies plants with the highest infection ratio.
- **Outputs**:
  - **Simulation**: `output_task1a.txt`
  - **Hardware (ROS)**: Published as JSON on `/detected_plants`.

## ROS Integration

- **CV Bridge**: Used to convert ROS image messages to OpenCV images.
- **Subscription**: Listens to the `/image_raw` topic for real-time image data.

## Folder Structure

### Simulation (Software)

- `software/`
  - `simulation_workflow.png`: Workflow image.
  - `offline_infected_plant_detection.py`: Main script.
  - `output_task1a.txt`: Example output.

### Real Plant Detection (Hardware)

- `hardware/`
  - `hardware_workflow.png`: Workflow image.
  - `realtime_infected_plant_detection.py`: ROS node script.

## Differences Between Software and Hardware Implementations

- **Software (Simulation)**:
  - Processes static images.
  - Useful for algorithm testing and validation.
  - Outputs results to a text file.

- **Hardware (Real-Time Detection)**:
  - Processes live camera feeds.
  - Runs as a ROS node.
  - Publishes detection results in real-time.

## Agricultural Relevance

- Provides early warnings of infection or stress.
- Enables targeted interventions.
- Reduces manual labor.
- Ensures consistent results for research.

### Notes

- Workflow images provide step-by-step visualization.
- Code is modular for easy extension.
