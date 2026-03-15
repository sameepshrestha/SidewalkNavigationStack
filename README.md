# FrodoBots Navigation Stack Competition

This repository contains the navigation stack for the FrodoBots competition.

## Overview

The stack implements a **Direction Guided MPPI (Model Predictive Path Integral)** controller.

### Key Features
- **Reactive Control:** The MPPI controller acts as a reactive controller, determining immediate velocity commands to avoid obstacles while following a target direction.
- **Directional Guidance:** The robot follows geographic headings derived from GPS waypoints.
- **Perception-Driven:** Navigability is determined by a vision-based segmentation model (Depth Anything 3) producing a local traversability map.

## Installation & Dependencies

### Depth Anything 3 (DA3)
The Depth Anything 3 model is integrated within the `frodobot_perception_models` package.
Location: `frodobot_perception_models/model/src/depth_anything_3`

### Requirements
- ROS 2 Humble
- PyTorch
- Nav2 (specifically `nav2_mppi_controller`)
- `robot_localization` (for basic TF transforms)

## Current Status

- **Tuning Phase:** The MPPI controller is currently in the tuning phase to balance obstacle avoidance and goal progress.
- **Localization:** Current pose and location estimates from the hardware can be variable. 
- **Future Improvements:** We plan to integrate **OrienterNet** or **OSMLoc** to improve orientation/heading accuracy, especially for initial global alignment.

## Project Structure
- `frodobot_bridge`: SDK bridge between robot hardware and ROS 2.
- `frodobot_localization`: Custom nodes for heading fusion (Magnetometer + GPS).
- `frodobot_perception_models`: AI model definitions (Segmentation + Depth).
- `frodobot_perception_ros`: ROS 2 wrapper for the perception models.
- `frodobot_mapping`: Generates local traversability costmaps from perception data.
- `frodobot_navigation`: Nav2 configuration and waypoint following logic.
- `frodobot_mppi_critics`: Custom C++ critics for the MPPI controller.
- `sidewalk_localization`: Supplementary EKF-based localization.
