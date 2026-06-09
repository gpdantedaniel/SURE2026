# GELOTRON 9000

Built by Dante Garcia and Clayton Dame

JMK Laboratories 2026. All rights reserved.

---

## Installation

## Usage

Starting up the arm's driver:

```bash
ros2 launch annin_ar4_driver driver.launch.py calibrate:=True include_gripper:=False ar_model:=mk3 serial_port:=/dev/ttyACM0
```

Starting up MoveIt path planning software:

```bash
ros2 launch annin_ar4_moveit_config moveit.launch.py ar_model:=mk3 include_gripper:=False
```

## Features

## Future Directions
