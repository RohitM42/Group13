# Reverse-Logistics Robot Simulation (Webots)

This repository contains a Webots simulation of an autonomous warehouse robot based on the KUKA YouBot.
The robot detects packages on a conveyor, reads QR/ArUco markers, picks them up, navigates autonomously, and places them in the correct drop-off chute.

# Features
Modified KUKA YouBot with omnidirectional drive and extended gripper

* QR/ArUco marker decoding using the Quirc library
* Customisable Box textures via textureUrl field
* Supervisor-based odometry and pose estimation
* Custom GOTO controller for smooth omnidirectional motion
* Waypoint navigation with modular support for A* path planning
* RobotWindow UI for manual GOTO commands and live robot pose
* Modular architecture (arm, base, gripper, navigation, perception)

# How It Works

1. Overhead camera detects a package and decodes its QR marker.
2. Robot picks up the package using predefined arm poses.
3. Navigation system issues a sequence of GOTO commands or custom paths.
4. Robot drops the package in the chute corresponding to its classification.
5. Robot returns to the conveyor and waits for the next package.

# Set Up (Windows)

Install Webots https://github.com/cyberbotics/webots

```
cd "your_project_path"\libraries
make clean
make

cp youbot_control\youbot_control.dll ..\controllers\youbot\

cd "your_project_path"\controllers\youbot\
make clean
make
```

This is a Windows set up. To make it work on other OS you need to change makefiles

# Pre-Programmed Packages / External Libraries Used

We used the following external components:

- **Quirc** — a lightweight third-party QR decoder.  
- **Webots built-in APIs**, which provide the core simulation interfaces:
  - Supervisor API (pose access, field manipulation, customData)
  - Robot API (motors, sensors, communication)
  - Camera API (image capture)
  - Physics engine, kinematics, and PROTO definitions

Although Webots provides basic controller examples for the KUKA YouBot, **all behaviour modules in this project were either rewritten, extended, or substantially modified** to support autonomous sorting.  

This includes the youbot, arm control, base movement logic, navigation system, obstacle handling, and overall task workflow.

# Project Structure

```
Project
├─ controllers
│  ├─ camera_controller
│  │  ├─ camera_controller.c
│  │  ├─ Makefile
│  │  └─ quirc
│  │     ├─ ...
│  ├─ conveyor_controller
│  │  ├─ conveyor_controller.c
│  │  └─ Makefile
│  └─ youbot
│     ├─ Makefile
│     └─ youbot.c
├─ libraries
│  ├─ Makefile
│  └─ youbot_control
│     ├─ include
│     │  ├─ ...
│     ├─ Makefile
│     └─ src
│        ├─ arm.c
│        ├─ base.c
│        ├─ gripper.c
│        ├─ navigation.c
│        ├─ obstacles.c
│        └─ tiny_math.c
├─ plugins
│  └─ robot_windows
│     └─ youbot_window
│        └─ youbot_window.html
├─ protos
│  ├─ CardboardBox_A.proto
│  ├─ CardboardBox_B.proto
│  ├─ ConveyorBelt.proto
│  ├─ icons
│  │  ├─ ...
│  ├─ KukaBox.proto
│  ├─ Parquetry.proto
│  ├─ RoCKInShelf.proto
│  ├─ WoodenPalletStack.proto
│  └─ Youbot.proto
├─ README.md
└─ worlds
   ├─ .youbot.jpg
   ├─ .youbot.wbproj
   └─ youbot.wbt
```

# Authors

Ardrit Kucana,
Rohit Mamtora,
Ivan Korniichuk
