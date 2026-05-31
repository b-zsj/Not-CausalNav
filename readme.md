# Not CausalNav - Embodied AI Navigation Pipeline

## Overview

This repository contains a ROS 2 implementation inspired by the **CausalNav**. It is an Embodied AI perception-to-action pipeline designed to allow an autonomous robot to navigate a large open/semi-open environment using high-level natural language commands, spatial semantic memory, and dynamic obstacle avoidance.

The system is split across two packages for performance:

1. **`causalnav_cpp`**: C++ data plane for low-latency point cloud segmentation and dynamic entity filtering.
2. **`causalnav_py`**: Python-based control plane handling topological routing via an LLM, short-term spatial memory (Embodied Graph), and local Artificial Potential Field (APF) control.

---

## Limitations

This implementation is **under development**. It currently has several difference from the original:

* **Local Controller:** This repository currently uses a APF controller. Instead of predicting the movement of dynamic objects, It just keeps them in distance. The NMPC controller similar to the original paper will be implemented.
* **Macro-Routing:** The `semantic_planner_node` currently uses a hardcoded NetworkX topological graph. You must manually configure the nodes and edges for your specific simulation/real-life map, or implement something like OSMnx when running in real-life.
* **3D Perception:** This pipeline currently uses a geometric median to project 2D YOLO bounding boxes (instead of the pixel-level masks) into 3D LiDAR space. It will be updated soon.

---

## System Architecture

| Node | Language | Role |
| --- | --- | --- |
| `pc_segmenter_node` | C++ | Fuses 3D point clouds with 2D YOLO tracks to calculate 3D semantic centroids. |
| `dynamic_remover_node` | C++ | Filters dynamic objects (people, cars) out of the raw point cloud to prevent SLAM map corruption. |
| `embodied_graph_node` | Python | Maintains a spatial-temporal graph, tracking objects to determine if they are static landmarks or dynamic agents. |
| `semantic_planner_node` | Python | Queries a local LLM (like Qwen) to interpret user commands and calculate shortest paths over a NetworkX graph. |
| `controller_node` | Python | Drives the robot using attractive vectors (to waypoints) and repulsive vectors (away from pedestrians). |

---

## Dependencies

Build these in your workspace wrt instructions on the respective repositories:

1. **[yolo_ros](https://github.com/mgonzs13/yolo_ros)**: Provides 2D bounding boxes and tracking IDs
2. **[FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)**: 3D Slam algorithm used in the original. 
3. **[isaac_sim_pointcloud_full_publisher](https://github.com/REGATTE/isaac_sim_pointcloud_full_publisher)**: Converts raw Isaac Sim LiDAR data into the structured format required by Fast-lio2. No required if you are not using Isaac-sim.

Additionally, you need to have a local LLM server running (e.g., `llama-server` hosting Qwen 3.6) exposed at `http://127.0.0.1:8080/v1/chat/completions`.

---

## Installation & Build

**1. Clone the packages**

```bash
cd ~/ros2_ws/src
git clone https://github.com/b-zsj/Not-CausalNav.git

```

**2. Install ROS Dependencies**

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

```

**3. Build the Workspace**
*Crucial: You must build the C++ packages in `Release` mode, or the point cloud processing will be too slow.*

```bash
colcon build --packages-select causalnav_cpp --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select causalnav_py --symlink-install
source install/setup.bash

```

---

## How to Run the System

To bring up the entire pipeline, open multiple terminals and launch the components in the following sequence:

### Step 1: Start the External Data Sources

1. Start your local **LLM API Server**.
2. Launch **Isaac Sim** and start your simulation environment (ensure ROS 2 Bridge is enabled).

### Step 2: Isaac Sim LiDAR Pre-Processing

Launch the full point cloud publisher to attach rings and timestamps to the Isaac Sim LiDAR.
*Note: We remap the output to `/point_cloud` which CausalNav and FAST-LIO expect.*

```bash
ros2 launch isaac_sim_pointcloud_full_publisher full_pcd_pub.launch.py \
  robot_namespace:=jetbot \
  config_file:=velodyne_vls_128.yaml \
  --ros-args -r /jetbot/scan3D_with_rings:=/point_cloud

```

### Step 3: Localization (FAST-LIO)

Launch FAST-LIO to generate odometry from the processed point cloud. (Configure `avia.yaml` or your specific LiDAR config to listen to `/point_cloud`).

```bash
ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml

```

### Step 4: Semantic Perception (yolo_ros)

Launch the YOLO tracking node. Ensure tracking is enabled so objects are assigned persistent IDs.

```bash
ros2 launch yolo_bringup yolo.launch.py \
  model:=yolov8m.pt \
  use_tracking:=True \
  input_image_topic:=/camera/rgb/image_raw

```

### Step 5: CausalNav Data Plane (C++)

Run the fast C++ sensor-fusion and filtering nodes.

```bash
ros2 run causalnav_cpp pc_segmenter_node
ros2 run causalnav_cpp dynamic_remover_node

```

### Step 6: CausalNav Control Plane (Python)

Run the memory, planning, and control nodes.

```bash
ros2 run causalnav_py embodied_graph_node
ros2 run causalnav_py semantic_planner_node
ros2 run causalnav_py apf_controller_node

```

### Step 7: Send a Command

With the pipeline running, issue a natural language command to the semantic planner:

```bash
ros2 topic pub --once /user_command std_msgs/msg/String "{data: 'Navigate to the cafe patio'}"

```

The LLM will parse this, find the shortest path on the NetworkX graph, and the APF controller will output `/cmd_vel` instructions to the Isaac Sim Action Graph to drive the robot.
