# Controling manipulator from camera interface

This project allows a user to **click on an object in a camera feed**, and automatically control a **robot manipulator** to grab the object using **inverse kinematics**. It combines **3D object tracking**, **robot control**, **shared memory**, and **real-time interaction**.

# Project Overview

This system enables a robot to:
1. Track objects in 3D using a depth camera.
2. Accept a user click on the camera feed.
3. Convert the clicked object's position into (x, y, z) coordinates.
4. Send the coordinates via shared memory.
5. Command the robot to move and grab the object.

# Necessary component

| Component       | Description |
|----------------|-------------|
| **DepthAI Camera** | Captures RGB-D data and tracks objects using YOLO |
| **Python + OpenCV** | For live object tracking and click interface |
| **Shared Memory + TCP** | Used to send position data to manipulator PC (LAN) |
| **Manipulator (Robot Arm)** | Controlled using inverse kinematics |
| **Arduino (Gripper)** | Operates gripper through serial commands |
| **INTime RTOS (Real-Time OS)** | Used on manipulator PC to control timing-critical operations and receive TCP data |

# Process Flow

```text
[User Click] 
    ↓
[Python Script] → detect object → get (x, y, z)
    ↓
[Send via TCP]
    ↓
[INTime .exe on Manipulator PC]
    ↓
[Write to Shared Memory]
    ↓
[Robot reads data]
    ↓
[Move to (x, y, z) → Grip → Move to drop zone → Release]

```
# 
