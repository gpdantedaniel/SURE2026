# GELOTRON 9000

Converting the AR4 MK3 into a surgery room-ready bioprinter for thermally sensitive hydrogels.

JMK Laboratories 2026. All rights reserved.

---

## Notable Features

* Custom PrusaSlicer-to-bioprinting GCODE parser

## Installation

This project was developed on [Ubuntu LTS 26.04 LTS](https://documentation.ubuntu.com/release-notes/26.04/) with [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html#installation). To work with this project, setup Ubuntu and ROS 2 Jazzy Jalisco following their instructions.

Having installed Ubuntu and ROS 2, in a separate directory, clone [Yifei Cheng's ROS 2 driver](https://github.com/ycheng517/ar4_ros_driver) for the AR4 MK3 robot arm. Follow the repository's installation instructions

## Usage


At any point throughout these instructions, please consult the ROS 2 documentation if you are unfamiliar with certain concepts (*e.g.*, overlay vs. underlay, nodes and services). 


First, activate Yifei Cheng's AR4 driver as your **underlay**.


```bash
source install/setup.bash
```

Following this, launch the driver and await calibration to finish.


```bash
ros2 launch annin_ar4_driver driver.launch.py calibrate:=True include_gripper:=False ar_model:=mk3 serial_port:=/dev/ttyACM0
```

Once the robot's been calibrated, start up MoveIt:

```bash
ros2 launch annin_ar4_moveit_config moveit.launch.py ar_model:=mk3 include_gripper:=False
```

Then, in a new terminal window, `cd` into our project's `ros2_ws` and activate the workspace as your **overlay**.

```bash
source install/local_setup.bash
```

After this, god help you.

## Software Architecture

For now, the application executes 3D printing trajectories by translating GCODE instructions produced by the Prusa Slicer (2.9.5) and generating waypoints in cartesian space. Based on these, the **/compute_cartesian_path** service is then queried to generate a trajectory, which gets subsequently passed to the **/execute_trajectory** action. 

```mermaid
graph LR;
    A[START]
    B[GCODE Parser]
    C["/compute_cartesian_path"]
    D["/execute_trajectory"]
    E[END]

    A -->|GCODE| B
    B -->|waypoints| C
    C -->|trajectory| D
    D -->|execution| E
```



## Future Directions