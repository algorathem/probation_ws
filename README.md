# GateLocator ROS2 Node

## Overview

**GateLocator** is a ROS2 node designed to autonomously locate and pass through a gate using camera detections and depth control. The main goals of this project are:

1. **Autonomous Depth Control:** Quickly descend to a target depth using proportional control.
2. **Gate Detection and Tracking:** Use bounding box data from a vision node to detect the gate.
3. **Adaptive Navigation:** Steer and move forward through the gate, allowing small lateral adjustments if the robot is offset.
---

## Thought Process

1. **Depth Control:**  
   - The robot should reach a target depth quickly. A proportional gain (`k_depth`) controls descent speed, clamped to avoid excessive velocity.  
   - The target depth was obtained by monitoring altitude data published in ```/mavros/global_position/rel_alt```, with the help of a foxglove simulation.

2. **Gate Detection:**  
   - A vision node provides bounding boxes (`BoundingBoxArray`).  
   - Only boxes labeled `"gate"` are considered.  
   - The last time the gate was seen along with a gate timeout variable was used to prevent erratic behavior when the gate temporarily disappears from published detections (when other objects are published instead).

3. **Navigation Logic:**  
   - **Searching:** If the gate is not detected, rotate in place.  
   - **Centering:** If the gate is detected but off-center, apply angular and lateral corrections.  
   - **Forward Motion:** Move forward if sufficiently centered. Forward speed scales slightly based on lateral error to reduce overshoot.  

4. **Parameters:**  
   - `target_alt`: Desired depth (-1.5 m).  
   - `forward_speed`: Maximum forward speed (1 m/s).  
   - `k_p`, `k_lat`: Proportional gains for angular and lateral corrections.  
   - `center_threshold`: Maximum x-error to consider “centered.”  (0.2)
   - `gate_timeout`: Duration to consider the gate still visible after last detection. (1.5 s)

---

## Setup Steps for ROS2 Gate Locator Simulation

1. **Build ROS2 Workspace**

```bash
cd probation_ws
colcon build
source install/setup.bash
```
2. **Start ROS TCP Endpoint**
```ros2 run ros_tcp_endpoint default_server_endpoint```
The endpoint will start on 0.0.0.0:10000 by default.

3. **Launch Unity Simulation**
Open the Unity simulation project.

If you haven’t installed the simulation yet, download it from the workshop [Notion page](https://mecatron.notion.site/ros2).

4. **Enter Guided Mode to allow the node to control the vehicle autonomously**
```ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"```

5. **Enter Object Detection Mode**
In the Unity simulation project, press ```Tab``` to activate the main camera for object detection.

6. **Run the gate locator node**
```ros2 run gate_autonomy gate_locator```

## Expected Behaviour

1. The robot first descends to the target depth (gate-level).
2. Once at depth, it rotates in place to locate the gate.  
3. After detecting the gate, it moves forward while adjusting its lateral position to stay centered on the gate.  
4. If lateral correction is not needed, it proceeds straight through the gate without unnecessary rotation.

