# frodobot_mapping

Real-time traversability mapping for the FrodoBots Earth Rover competition stack.

---

## What It Does

This package takes the **traversable-path point cloud** produced by `frodobot_perception_ros` and converts it into a **2D rolling OccupancyGrid** centred on the robot. This grid is the bridge between perception and planning — it tells the motion planner (MPPI) exactly which cells on the ground are safe to drive through.

```
frodobot_perception_ros           frodobot_mapping             Nav2 / MPPI
 /perception/traversable_cloud ──► traversability_map_node ──► /mapping/traversability
       (PointCloud2)                                              (OccupancyGrid)
```

---

## Why OccupancyGrid and Not GridMap?

`nav_msgs/OccupancyGrid` is the **universal ROS 2 standard**. Every Nav2 plugin, RViz, and most motion planners consume it natively. `grid_map_msgs/GridMap` is a powerful multi-layer format but requires extra ROS packages (C++ only) and is overkill for a single-layer traversability map.

---

## Grid Cell Values (Nav2 convention)

| Value | Meaning |
|-------|---------|
| `-1`  | **Unknown** — never observed by the robot |
| `0`   | **Free / Traversable** — perception confirmed this is drivable |
| `100` | **Lethal obstacle** — used by the slope filter (disabled by default) |

The grid is always centred on `base_link` (the robot body). As the robot moves, the grid follows it — old parts of the world fall off the edges and new areas come in.

---

## How the Node Works (Step by Step)

### 1. Receive Traversable Cloud
Subscribes to `/perception/traversable_cloud` — a `PointCloud2` already in the `base_link` frame. These are 3D points backprojected from pixels the **segmentation model** identified as the navigable path.

### 2. Map to Grid
Each 3D point `(X, Y, Z)` is projected down to 2D `(X, Y)` and converted to a grid cell index:
```python
ix = floor((x - origin_x) / resolution)
iy = floor((y - origin_y) / resolution)
```
The grid origin (`bottom-left corner`) is at `(-grid_width/2, -grid_height/2)` in `base_link`.

### 3. Mark Cells as Free
Each cell that receives a traversable point is marked `0` (free). The timestamp is stored for decay.

### 4. Publish OccupancyGrid
At `publish_rate_hz` (default 5 Hz), the full grid is wrapped into a `nav_msgs/OccupancyGrid` message and published on `/mapping/traversability`.

---

## Configuration (`config/mapping.yaml`)

```yaml
traversability_map_node:
  ros__parameters:
    resolution: 0.05          # 5 cm/cell — balance between detail and memory
    grid_width_m: 20.0        # 20 m total width centred on robot
    grid_height_m: 20.0       # 20 m total height centred on robot
    cloud_topic: "/perception/traversable_cloud"
    output_topic: "/mapping/traversability"
    frame_id: "base_link"
    publish_rate_hz: 5.0
    decay_s: 0.0              # 0 = cells stay marked; set to ~2.0 for live robot
    slope_filter_enable: false
    slope_height_thresh_m: 0.15
```

### Key Tuning Knobs

| Parameter | Effect |
|-----------|--------|
| `resolution` | Smaller = more detail, more memory. 5 cm is a good default. |
| `grid_width_m` / `grid_height_m` | How much of the world around the robot is tracked. 20×20 m covers about 2–3 seconds of driving at 2 m/s. |
| `decay_s` | How long cells stay marked before fading to unknown. Set to `0` for testing on a single video frame; set to `2.0`–`5.0` on live robot. |
| `slope_filter_enable` | When true, cells where the **height range** of all received points exceeds `slope_height_thresh_m` are marked lethal (obstacle). Useful for detecting curbs and bumps. Keep disabled until the map looks correct. |

---

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/perception/traversable_cloud` | `sensor_msgs/PointCloud2` | **In** | Traversable path points from the perception node |
| `/mapping/traversability` | `nav_msgs/OccupancyGrid` | **Out** | 2D rolling traversability map |

---

## How to Run

### With the video test launch (standalone, no robot needed)

```bash
# Terminal 1 — perception node replaying a video
cd frodobot_perception_ros
source install/setup.bash
ros2 run frodobot_perception_ros perception_node --ros-args \
  -p video_file:=/path/to/video.mp4

# Terminal 2 — mapping node
cd frodobot_mapping
source install/setup.bash
ros2 launch frodobot_mapping mapping.launch.py

# Terminal 3 — static TF so RViz can find base_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
```

### Visualise in RViz2
1. Add → By Topic → `/mapping/traversability` → **Map**
2. Set **Fixed Frame** → `base_link`
3. The traversable path will appear as a coloured 2D strip in front of the robot.

---

## Architecture in the Full Stack

```
[Camera]
   │
   ▼
frodobot_perception_ros          (depth + segmentation → traversable PointCloud2)
   │
   ▼
frodobot_mapping                 (PointCloud2 → OccupancyGrid)
   │
   ▼
Nav2 MPPI Controller             (OccupancyGrid → cmd_vel, avoiding lethal cells)
   │
   ▼
frodobot_bridge                  (cmd_vel → robot wheel commands)
```

The Reasoning Head (future) will inject additional cost layers into the OccupancyGrid — for example, adding high cost to regions where the VLM identifies hazards like puddles or construction cones — without modifying the core mapping or planning nodes.

---

## Future Extensions

- **Slope filter** — already wired in, just set `slope_filter_enable: true` in config
- **Obstacle layer** — merge a second cloud of non-traversable obstacles (detected from depth) as lethal cells
- **Map decay** — tune `decay_s` to make the map reflect only the robot's current view
- **Static map fusion** — optionally fuse with a prior BEV map from OSMLoc for long-range awareness
