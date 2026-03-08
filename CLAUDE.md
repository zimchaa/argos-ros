# ARGOS-ROS — Claude Context

## Project overview

ROS2 Humble workspace for the ARGOS robot. Wraps the standalone
[argos-robot](https://github.com/zimchaa/argos-robot) Python hardware library
as ROS2 nodes with standard topics and services.

The robot is a Raspberry Pi 4 tracked chassis with a 4-joint OWI-535 arm,
multiple sensors, and a USB camera. See the argos-robot CLAUDE.md for full
hardware details, wiring, calibration data, and driver internals.

---

## Repository layout

```
src/
  argos_msgs/                    # CMake ament — custom message types
    CMakeLists.txt
    package.xml
    msg/
      AhrsData.msg               # roll/pitch/yaw/heading + quaternion
      JointSpeeds.msg            # shoulder/elbow/wrist/gripper (-100..100)
      IrProximity.msg            # ir1/ir2 booleans
      FlotillaData.msg           # weather + body/arm motion + validity flags
  argos_hardware/                # Python ament — hardware driver nodes
    setup.py / setup.cfg / package.xml
    argos_hardware/
      hardware_bridge_node.py    # cmd_vel + arm + emergency stop (owns SafetyMonitor)
      imu_node.py                # MPU-6050 → /imu/raw (sensor_msgs/Imu, 100 Hz)
      flotilla_node.py           # Flotilla dock → /flotilla (FlotillaData, 50 Hz)
      ahrs_node.py               # Madgwick fusion → /imu/data + /ahrs (50 Hz)
      sonar_node.py              # HC-SR04 → /sonar/range (sensor_msgs/Range, 10 Hz)
      ir_node.py                 # IR pair → /ir/proximity (IrProximity, 20 Hz)
      camera_node.py             # Webcam → /camera/image_raw (sensor_msgs/Image, 30 Hz)
      core/                      # Vendored from argos-robot (import paths updated)
        config.py                # Motor configs, sensor constants, axis remaps
        drivers/pca9685.py       # PCA9685 I2C driver + I2CMotor (tracks)
        drivers/gpio_motor.py    # GPIOMotor (arm joints via MotorShield)
        base/tracks.py           # TrackedBase differential drive
        arm/joints.py            # RobotArm 4-joint controller
        safety/monitor.py        # SafetyMonitor (speed clamping + watchdog)
        sensorium/imu.py         # MPU-6050 driver
        sensorium/sonar.py       # HC-SR04 driver
        sensorium/ir.py          # IR proximity driver
        sensorium/flotilla.py    # Flotilla dock USB serial reader
        sensorium/ahrs.py        # MadgwickAHRS 9DOF filter
        vision/camera.py         # USB webcam wrapper
  argos_bringup/                 # Python ament — launch files
    launch/
      argos.launch.py            # Full bringup (motors + sensors)
      motors.launch.py           # hardware_bridge only
      sensors.launch.py          # All sensor nodes
```

---

## Key design decisions

### Vendored core
The argos-robot hardware drivers are vendored into `argos_hardware/core/` rather
than installed as a separate Python package. Imports changed from `argos.*` to
`argos_hardware.core.*`. This avoids PYTHONPATH issues on the robot and keeps
everything in one colcon workspace.

### Single hardware bridge
One `hardware_bridge` node owns the SafetyMonitor, which wraps both TrackedBase
(I2C) and RobotArm (GPIO). These cannot be split across processes because they
share hardware resources (GPIO.setmode, PCA9685 instance cache). All motor
commands flow through this node.

### Sensor nodes are independent
Each sensor gets its own node. The AHRS node subscribes to `/imu/raw` and
`/flotilla` topics rather than accessing hardware directly, avoiding I2C
conflicts with the IMU node.

### No cv_bridge dependency
The camera node converts OpenCV frames to `sensor_msgs/Image` manually to avoid
requiring the `ros-humble-cv-bridge` package.

### System Python for hardware deps
`ros2 run` uses system Python, not a venv. Hardware Python deps must be installed
system-wide via apt: `python3-rpi.gpio`, `python3-smbus2`, `python3-serial`,
`python3-opencv`.

---

## Environments

| Machine | Hostname | Role | OS | ROS2 |
|---|---|---|---|---|
| Raspberry Pi 4 | `argos-ros.local` | Robot (ROS2 target) | Ubuntu 22.04 | Humble (native) |
| Dev workstation | `thirtythr33` | Development + git push | Ubuntu 24.04 | Jazzy (native), Humble (Docker) |

SSH key auth configured: `~/.ssh/argos_ros` → `zimchaa@argos-ros.local`.

### Docker dev environment (ROS2 Humble)

The dev machine runs Ubuntu 24.04 with ROS2 Jazzy natively. Since Jazzy and
Humble use incompatible DDS wire protocols, a Docker container provides ROS2
Humble for cross-machine communication with the robot.

Files in repo root: `Dockerfile`, `docker-compose.yml`, `.dockerignore`.

- **Base image**: `ros:humble-ros-base-jammy` (Ubuntu 22.04)
- **Network**: `network_mode: host` for DDS multicast discovery
- **Volumes**: `./src` bind-mounted; named volumes for `build/`, `install/`, `log/`
- **Deps**: colcon, smbus2, serial, opencv (apt); RPi.GPIO stub for import compat

```bash
# Build packages inside the container
docker compose run --rm humble bash -c \
  "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Interactive shell (build artifacts persist across runs)
docker compose run --rm humble
source install/setup.bash
ros2 topic list                          # verify robot topics visible
ros2 run argos_hardware control_panel    # launch control panel
```

---

## Development workflow

1. Edit code locally in this repo
2. `git push` to GitHub
3. On robot: `git pull` (Python changes take effect immediately via symlink install)
4. Rebuild on robot only when needed: `colcon build --symlink-install`

Rebuild required for: new executables in setup.py, package.xml changes, new/modified .msg files.

---

## Topic map

```
/cmd_vel                geometry_msgs/Twist       → hardware_bridge (base motion)
/arm/joint_speeds       argos_msgs/JointSpeeds    → hardware_bridge (arm control)
/emergency_stop         std_srvs/Trigger          → hardware_bridge (service)

/imu/raw                sensor_msgs/Imu           ← imu_node (no orientation)
/imu/data               sensor_msgs/Imu           ← ahrs_node (with orientation)
/ahrs                   argos_msgs/AhrsData       ← ahrs_node (Euler angles)
/flotilla               argos_msgs/FlotillaData   ← flotilla_node
/sonar/range            sensor_msgs/Range         ← sonar_node
/ir/proximity           argos_msgs/IrProximity    ← ir_node
/camera/image_raw       sensor_msgs/Image         ← camera_node
```

---

## Conventions

- **Speed range**: -100..100 for all motors (same as argos-robot)
- **cmd_vel mapping**: `linear.x` [-1,1] → speed, `angular.z` CCW+ → differential
- **Frame IDs**: `base_link`, `imu_link`, `sonar_link`, `camera_link`
- **Axis remaps**: applied in ahrs_node using constants from `core/config.py`
- **Safety**: all motor commands pass through SafetyMonitor (speed clamping + watchdog)
- **Node resilience**: each node catches hardware init failures and logs errors rather than crashing

---

## Hardware reference

See the [argos-robot CLAUDE.md](https://github.com/zimchaa/argos-robot) for:
- Complete hardware mapping (I2C addresses, GPIO pin tables, motor IDs)
- MotorConfig limits (max_speed, min_speed, max_duration per joint)
- Sensor axis remap derivation and probe results
- Magnetometer calibration data
- Arm kinematics reference dimensions
- Sensorium design and sensor fusion architecture

---

## Status

Session 1 complete (2026-03-07).

**Working:**
- All 3 packages build cleanly on robot (`colcon build --symlink-install`)
- All 7 executables registered and launchable
- IR node tested end-to-end: `/ir/proximity` publishing correctly
- GitHub sync workflow confirmed (push → pull → immediate effect)

**Not yet tested on this robot:**
- IMU, flotilla, AHRS, sonar, camera nodes (sensors may not be connected yet)
- Motor control via hardware_bridge (tracks + arm)
- Full bringup launch
- Multi-node operation

**Planned:**
- Remote control interface for testing actuators and sensors from dev machine
- URDF robot description package
- Navigation integration
