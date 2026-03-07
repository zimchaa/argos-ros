# argos-ros

ROS2 Humble workspace for the ARGOS robot — a Raspberry Pi 4 tracked chassis with a 4-joint OWI-535 arm, onboard sensors, and a camera.

This workspace wraps the hardware drivers from [argos-robot](https://github.com/zimchaa/argos-robot) as ROS2 nodes, exposing motors and sensors via standard topics and services.

## Packages

| Package | Type | Purpose |
|---|---|---|
| `argos_msgs` | CMake (ament) | Custom message definitions |
| `argos_hardware` | Python (ament) | Hardware driver nodes + vendored core |
| `argos_bringup` | Python (ament) | Launch files |

## Nodes

### Motor control

**`hardware_bridge`** — single node owning the SafetyMonitor (tracks + arm)

| Interface | Type | Description |
|---|---|---|
| `/cmd_vel` sub | `geometry_msgs/Twist` | Base motion. `linear.x` [-1,1] = speed, `angular.z` = turn (CCW+) |
| `/arm/joint_speeds` sub | `argos_msgs/JointSpeeds` | Arm joint speeds (-100 to 100 each) |
| `/emergency_stop` srv | `std_srvs/Trigger` | Immediate halt of all motors |

### Sensors

| Node | Topic | Message type | Rate | Hardware |
|---|---|---|---|---|
| `imu_node` | `/imu/raw` | `sensor_msgs/Imu` | 100 Hz | MPU-6050 (I2C 0x68) |
| `flotilla_node` | `/flotilla` | `argos_msgs/FlotillaData` | 50 Hz | Flotilla dock (USB serial) |
| `ahrs_node` | `/imu/data`, `/ahrs` | `sensor_msgs/Imu`, `argos_msgs/AhrsData` | 50 Hz | Madgwick 9DOF fusion |
| `sonar_node` | `/sonar/range` | `sensor_msgs/Range` | 10 Hz | HC-SR04 (BOARD 29/31) |
| `ir_node` | `/ir/proximity` | `argos_msgs/IrProximity` | 20 Hz | 2x IR (BOARD 7, 12) |
| `camera_node` | `/camera/image_raw` | `sensor_msgs/Image` | 30 Hz | USB webcam (640x480) |

## Custom messages

- **`AhrsData`** — roll, pitch, yaw, heading (degrees) + orientation quaternion
- **`JointSpeeds`** — shoulder, elbow, wrist, gripper speeds (-100 to 100)
- **`IrProximity`** — ir1, ir2 booleans (obstacle detected)
- **`FlotillaData`** — weather (temp/pressure), body + arm motion (accel/mag), validity flags

## Target platform

- **Robot**: Raspberry Pi 4 (`argos-ros.local`), Ubuntu 22.04, ROS2 Humble
- **Development**: Raspberry Pi 400 or remote workstation

## Setup

### Prerequisites

```bash
# ROS2 Humble (already installed)
sudo apt install ros-dev-tools

# Hardware Python deps (system-wide, required for ros2 run)
sudo apt install python3-rpi.gpio python3-smbus2 python3-serial python3-opencv
```

### Build

```bash
cd ~/Projects/Argos/argos-ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Run

```bash
# Source both setups
source /opt/ros/humble/setup.bash
source ~/Projects/Argos/argos-ros/install/setup.bash

# Full bringup (motors + all sensors)
ros2 launch argos_bringup argos.launch.py

# Or individual launch groups
ros2 launch argos_bringup motors.launch.py
ros2 launch argos_bringup sensors.launch.py

# Or single nodes
ros2 run argos_hardware ir_node
ros2 run argos_hardware imu_node
```

### Development workflow

Code is edited on the development machine, pushed via GitHub, and pulled on the robot:

```bash
# Dev machine
git push

# Robot
cd ~/Projects/Argos/argos-ros
git pull
# Python changes take effect immediately (symlink install)
# Rebuild only needed for new executables, package.xml, or message changes:
colcon build --symlink-install
```

## Architecture

```
                    +-------------------+
  /cmd_vel -------->|                   |----> Waveshare HAT (I2C 0x40)
                    | hardware_bridge   |        Left track (motor 0)
  /arm/joint ------>|  (SafetyMonitor)  |        Right track (motor 1)
  speeds            |                   |----> MotorShield (GPIO)
                    +-------------------+        shoulder/elbow/wrist/gripper
  /emergency_stop ------^

  +----------+    /imu/raw     +----------+    /imu/data
  | imu_node |--------------->| ahrs_node |------------->
  +----------+                |           |    /ahrs
                  /flotilla   |  Madgwick |------------->
  +--------------+----------->|  9DOF     |
  | flotilla_node|            +----------+
  +--------------+---> /flotilla

  +------------+    /sonar/range        +----------+    /ir/proximity
  | sonar_node |----------->            | ir_node  |----------->
  +------------+                        +----------+

  +-------------+   /camera/image_raw
  | camera_node |----------->
  +-------------+
```

## Vendored core

The `argos_hardware` package includes a vendored copy of the `argos-robot` hardware drivers at `argos_hardware/core/`. Import paths are updated from `argos.*` to `argos_hardware.core.*`. The vendored modules are:

- `core/config.py` — motor configs, sensor constants, axis remaps
- `core/drivers/` — PCA9685 (I2C) and GPIOMotor drivers
- `core/base/tracks.py` — TrackedBase differential drive
- `core/arm/joints.py` — RobotArm 4-joint controller
- `core/safety/monitor.py` — SafetyMonitor (speed clamping + watchdog)
- `core/sensorium/` — MPU-6050, HC-SR04, IR, Flotilla, MadgwickAHRS
- `core/vision/camera.py` — USB webcam wrapper

## Related

- [argos-robot](https://github.com/zimchaa/argos-robot) — standalone Python hardware library (no ROS dependency)
