# ros2-drone

ROS 2 workspace for autonomous drone control with PX4 offboard mode and OpenCV-based object tracking.

## Tech Stack

| Component | Version |
|---|---|
| OS | Ubuntu 24.04 LTS (Noble Numbat) |
| ROS 2 | Jazzy Jalopy |
| PX4-Autopilot | main branch (Oct 2026) |
| Micro XRCE-DDS Agent | built from source (eProsima) |
| Gazebo | Harmonic 8.x |
| Python | 3.12 |
| OpenCV | 4.x (`python3-opencv`) |

## Packages

| Package | Type | Description |
|---|---|---|
| `drone_controllers` | ament_python | Offboard control + OpenCV tracking nodes |
| `drone_bringup` | ament_cmake | Launch files and parameters |

---

## Prerequisites (Ubuntu 24.04)

### 1. Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep
```

### 2. Build px4_msgs

```bash
mkdir -p ~/ros2/px4_ws/src && cd ~/ros2/px4_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ~/ros2/px4_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

### 3. Build Micro XRCE-DDS Agent from source

```bash
sudo apt install -y cmake g++ pip git python3-vcstool
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo cp ~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent /usr/local/bin/
```

### 4. Clone and build PX4-Autopilot

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash Tools/setup/ubuntu.sh   # installs Gazebo + toolchain (~10 min)
```

### 5. Install extra ROS 2 dependencies

```bash
sudo apt install -y ros-jazzy-cv-bridge python3-opencv
```

---

## Setup: Clone and Build This Repo

```bash
cd ~/ros2
git clone https://github.com/sannyuntwin/ros2-drone.git
cd ros2-drone

source /opt/ros/jazzy/setup.bash
source ~/ros2/px4_ws/install/setup.bash

colcon build --symlink-install
source install/setup.bash
```

---

## Running the SITL Simulation

You need **3 terminals**. Source ROS 2 in each one before running:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2/px4_ws/install/setup.bash
source ~/ros2/ros2-drone/install/setup.bash
```

### Terminal 1 — PX4 SITL + Gazebo

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Wait for Gazebo to open and the `pxh>` prompt to appear. Then type these **once** at the `pxh>` prompt to bypass GCS/RC preflight checks:

```
param set COM_RCL_EXCEPT 4
param set NAV_DLL_ACT 0
param save
```

> These params persist across restarts (saved to BSON). You only need to set them once per SITL build.

### Terminal 2 — Micro XRCE-DDS Bridge

```bash
MicroXRCEAgent udp4 -p 8888
```

Wait for `Successful connection` messages from PX4.

### Terminal 3 — ROS 2 Offboard Node

```bash
cd ~/ros2/ros2-drone
source /opt/ros/jazzy/setup.bash
source ~/ros2/px4_ws/install/setup.bash
source install/setup.bash
ros2 run drone_controllers offboard_node
```

Expected output:
```
[INFO] [offboard_node]: OffboardNode started
[INFO] [offboard_node]: Offboard mode engaged
[INFO] [offboard_node]: Arm command sent
```

Terminal 1 (`pxh>`) should then show:
```
INFO  [commander] Armed by external command
INFO  [commander] Takeoff detected
```

The drone will lift off and hover at **5 metres** in Gazebo.

---

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

**Parameters** (in `config/params.yaml`):
- `target_altitude` — hover altitude in metres, NED frame (default `-5.0`, negative = up)
- `takeoff_counter_threshold` — timer ticks before arm + engage offboard (default `10`)

---

### `tracking_node`

Tracks a target in a camera feed using an OpenCV tracker and publishes velocity setpoints to keep the target centred in frame.

**Topics published:**
- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `~/debug_image` — annotated camera frame

**Topics subscribed:**
- `/camera/image_raw` (configurable)
- `~/enable` — publish `std_msgs/Bool True` to start tracking

**Parameters:**
- `tracker_type` — `CSRT` | `KCF` | `MOSSE` (default `CSRT`)
- `image_topic` — camera topic (default `/camera/image_raw`)
- `gain_lateral`, `gain_vertical` — pixels → velocity gain
- `max_velocity` — velocity clamp in m/s

Enable tracking at runtime:
```bash
ros2 topic pub --once /tracking_node/enable std_msgs/msg/Bool "data: true"
```

---

## Launch (both nodes together)

```bash
ros2 launch drone_bringup drone.launch.py
```

---

## Architecture

```
offboard_node ──────────────────────────────────────► PX4 FMU
  (arm / offboard engage / position hold)

tracking_node ◄── /camera/image_raw ◄── camera driver
  (OpenCV CSRT tracker) ───────────────────────────► PX4 FMU
                                                       (velocity setpoints)

ROS 2  ◄──── Micro XRCE-DDS Agent (udp4:8888) ──────► PX4 uORB topics
```

## Coordinate Frame

PX4 uses **NED (North-East-Down)**. Altitude setpoints must be **negative** for upward motion (e.g., `-5.0` = 5 metres above ground).
