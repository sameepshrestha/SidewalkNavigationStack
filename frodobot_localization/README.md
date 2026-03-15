# heading_fusion

A small ROS 2 package that fuses:
- GPS course-over-ground from a sliding window of NavSatFix points
- IMU gyro yaw-rate integration
- an optional second global yaw source

and publishes a clean heading odometry topic for outdoor robots.

## Why this package exists

For robots like FrodoBot, the hard problem is often not absolute GPS position, but
stable **global heading**. Raw magnetometer heading is often unreliable outdoors.
This package gives you a practical heading source you can feed into `robot_localization`
and `navsat_transform_node`.

## Topics

### Subscribed
- `/imu/data` (`sensor_msgs/Imu`)
- `/gps/fix` (`sensor_msgs/NavSatFix`)
- `/global_yaw/odom` (`nav_msgs/Odometry`, optional)

### Published
- `/heading/odom` (`nav_msgs/Odometry`)
- `/heading/pose` (`geometry_msgs/PoseWithCovarianceStamped`)

## Key idea

1. Integrate the IMU yaw-rate for smooth short-term heading.
2. Estimate GPS heading from a short sliding window of positions using PCA.
3. Correct the heading estimate toward GPS course-over-ground only when motion is
   large enough to be trustworthy.
4. Optionally pull the estimate toward a second global yaw source.

## Build

```bash
cd ~/your_ws/src
cp -r heading_fusion ~/your_ws/src/
cd ~/your_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select heading_fusion
source install/setup.bash
```

## Run

```bash
ros2 launch heading_fusion heading_fusion.launch.py
```

## Recommended integration with robot_localization

1. Run this node and publish `/heading/odom`.
2. Use `/heading/odom` as the heading/odometry source in `robot_localization`.
3. Set `use_odometry_yaw: true` in `navsat_transform_node` if you want navsat_transform
   to use this fused heading instead of raw IMU yaw.

## Notes

- This package is intentionally simple and readable.
- It is a heading estimator, not a full SLAM/VIO system.
- Exact IMU translation is not modeled because for small slow rovers that is usually
  much less important than correct yaw alignment and bias handling.
