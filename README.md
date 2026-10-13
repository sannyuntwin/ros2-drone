# ros2-drone

ROS 2 workspace for autonomous drone control with PX4 and OpenCV-based object tracking.

## Packages

| Package | Type | Description |
|---|---|---|
| `drone_controllers` | ament_python | Offboard control + tracking nodes |
| `drone_bringup` | ament_cmake | Launch files and parameters |

## Nodes

### `offboard_node`
Arms the drone, engages PX4 offboard mode, and commands it to a fixed hover altitude via position setpoints.

**Topics published:**
- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

**Topics subscribed:**
- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_status`

**Parameters:**
- `target_altitude` (default `-5.0` m, NED frame)
- `takeoff_counter_threshold` (default `10` ticks)

---

### `tracking_node`
Tracks a target in a camera feed using an OpenCV tracker (CSRT/KCF/MOSSE) and publishes velocity setpoints to keep the target centred in frame.

**Topics published:**
- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `~/debug_image` — annotated camera frame

**Topics subscribed:**
- `/camera/image_raw` (configurable)
- `~/enable` — publish `std_msgs/Bool True` to start tracking

**Parameters:**
- `tracker_type` (default `CSRT`)
- `image_topic` (default `/camera/image_raw`)
- `gain_lateral`, `gain_vertical`, `max_velocity`

---

## Dependencies

- ROS 2 Humble or newer
- [px4_msgs](https://github.com/PX4/px4_msgs)
- `cv_bridge`, `sensor_msgs`, `geometry_msgs`, `std_msgs`
- OpenCV (`python3-opencv`)

## Build

```bash
# From workspace root (d:\ros2-drone on Windows, or wherever you cloned this)
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# Launch both nodes with default parameters
ros2 launch drone_bringup drone.launch.py

# Enable tracking at runtime
ros2 topic pub --once /tracking_node/enable std_msgs/msg/Bool "data: true"
```

## Architecture

```
offboard_node ──────────────────────────────────────► PX4 FMU
  (position hold / arm / takeoff)

tracking_node ─── /camera/image_raw ◄── camera driver
  (OpenCV tracker) ──────────────────────────────────► PX4 FMU
                                                        (velocity setpoints)
```
