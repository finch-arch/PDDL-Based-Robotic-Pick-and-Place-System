
=======
# pyTAMP-MoveIt Bridge

A **Task and Motion Planning (TAMP)** system for the Franka Emika Panda robot. This project provides a ROS 2 bridge between high-level symbolic planning and low-level motion execution with MoveIt 2, enabling autonomous pick-and-place operations.

> **Note:** This package was originally designed as a bridge for the [pyTAMP](https://github.com/IJRLab/pyTAMP) framework (using MCTS-based planning). However, the current implementation uses **PDDL-based symbolic planning with pyperplan** instead of pyTAMP's MCTS planner. The pyTAMP integration (`scene_interface.py`) is stubbed but can be activated by uncommenting the imports and installing pyTAMP/pykin.

## Overview

This system bridges high-level symbolic planning (PDDL) with low-level motion execution (MoveIt 2) to enable the Panda robot to autonomously pick up objects detected by YOLOv8 and place them at target locations.

### Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              TAMP System                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────┐    ┌───────────────────┐    ┌────────────────────────┐   │
│  │   YOLOv8     │    │ Object State      │    │ PDDL Planner Node      │   │
│  │   Publisher  │───▶│ Manager           │───▶│ (pyperplan backend)    │   │
│  │              │    │                   │    │                        │   │
│  │ /image_raw   │    │ /Yolov8_Inference │    │ /world_state           │   │
│  │              │    │ /camera/depth/    │    │ /planning_goal         │   │
│  └──────────────┘    │ image_raw         │    │                        │   │
│                      └───────────────────┘    │ → PDDL Problem        │   │
│                                               │ → BFS Search          │   │
│                                               │ → TaskAction[]        │   │
│                                               └───────────┬────────────┘   │
│                                                           │                │
│                                                           ▼                │
│                                               ┌────────────────────────┐   │
│                                               │ Action Executor        │   │
│                                               │                        │   │
│                                               │ • MoveIt Action Client │   │
│                                               │ • Gripper Controller   │   │
│                                               │ • Pick/Place Actions   │   │
│                                               └────────────────────────┘   │
│                                                           │                │
│                                                           ▼                │
│                                               ┌────────────────────────┐   │
│                                               │ MoveIt 2 + Gazebo       │   │
│                                               │ (Panda Robot)          │   │
│                                               └────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Project Structure

```
TAMP-1/
├── src/
│   ├── pytamp_moveit_bridge/          # Core TAMP bridge package
│   │   ├── src/
│   │   │   ├── planner_node.py        # PDDL-based task planner
│   │   │   ├── action_executor.py     # MoveIt action execution
│   │   │   ├── object_state_manager.py # Perception → WorldState bridge
│   │   │   ├── scene_interface.py     # Scene management interface
│   │   │   └── republish_joint_states.py
│   │   ├── scripts/
│   │   │   └── mock_perception.py     # Mock perception for testing
│   │   ├── launch/
│   │   │   ├── integration.launch.py  # Full system launch (Gazebo)
│   │   │   └── fake_integration.launch.py # Mock perception launch
│   │   ├── msg/
│   │   │   ├── ObjectState.msg        # Single object state
│   │   │   ├── WorldState.msg         # All detected objects
│   │   │   └── TaskAction.msg         # Task-level action
│   │   └── pddl/
│   │       ├── domain.pddl            # Pick-and-place domain
│   │       └── problem.pddl           # Problem template
│   │
│   ├── yolov8_obb/                    # YOLOv8 OBB detection
│   │   ├── scripts/
│   │   │   ├── yolov8_obb_publisher.py # YOLO inference publisher
│   │   │   ├── yolov8_obb_subscriber.py
│   │   │   └── best.pt                # YOLOv8 trained model
│   │   └── launch/
│   │       └── yolov8_obb.launch.py
│   │
│   ├── yolov8_obb_msgs/               # Custom YOLOv8 messages
│   │   └── msg/
│   │       ├── Yolov8Inference.msg    # Detection array
│   │       └── InferenceResult.msg    # Single detection
│   │
│   ├── panda_moveit_config/           # MoveIt configuration
│   │   ├── config/
│   │   │   ├── panda.urdf.xacro       # Robot URDF with camera
│   │   │   ├── panda.srdf             # Semantic robot description
│   │   │   ├── kinematics.yaml        # IK solver config
│   │   │   ├── ompl_planning.yaml     # OMPL planners config
│   │   │   ├── ros2_controllers.yaml  # Controller config
│   │   │   └── moveit_controllers.yaml
│   │   └── launch/
│   │       ├── moveit_gazebo_obb.py   # Gazebo + MoveIt launch
│   │       └── moveit_rviz.launch.py
│   │
│   └── robot_description/             # Robot meshes and URDF
│       ├── urdf/
│       │   └── panda.urdf             # Panda robot URDF
│       └── meshes/                    # STL meshes for Panda
│
└── README.md
```

## Key Components

### 1. PDDL Planner Node (`planner_node.py`)

The task-level planner that generates symbolic action sequences:

- **Subscribes to**: `/world_state` (perceived objects), `/planning_goal` (trigger)
- **Publishes**: `/task_plan` (sequence of TaskAction messages)
- **Backend**: Uses `pyperplan` with BFS (Breadth-First Search)
- **Domain**: Pick-and-place with actions: `pick(object, location)` and `place(object, location)`

```pddl
;; domain.pddl
(define (domain pick-and-place)
  (:predicates
    (on ?o ?l)          ;; object ?o is at location ?l
    (holding ?o)        ;; robot is holding object ?o
    (gripper-empty)     ;; gripper is empty
    (at-goal ?o)        ;; object has reached target
  )
  
  (:action pick
    :parameters (?o ?l)
    :precondition (and (on ?o ?l) (gripper-empty))
    :effect (and (holding ?o) (not (on ?o ?l)) (not (gripper-empty)))
  )
  
  (:action place
    :parameters (?o ?l)
    :precondition (holding ?o)
    :effect (and (on ?o ?l) (at-goal ?o) (not (holding ?o)) (gripper-empty))
  )
)
```

### 2. Action Executor (`action_executor.py`)

Executes task-level actions via MoveIt 2:

- **Subscribes to**: `/task_plan` (TaskAction messages), `/world_state`
- **Action Clients**: `MoveGroup` (arm motion), `GripperCommand` (gripper)
- **Pick Sequence**:
  1. Open gripper
  2. Move to pre-grasp pose (above object)
  3. Descend to grasp pose
  4. Close gripper on object
  5. Lift object
- **Place Sequence**:
  1. Move to pre-place pose
  2. Descend to place location
  3. Open gripper
  4. Retreat

### 3. Object State Manager (`object_state_manager.py`)

Bridges perception output to world state:

- **Subscribes to**: `/Yolov8_Inference` (YOLO detections), `/camera/depth/image_raw`, `/camera/camera_info`
- **Publishes**: `/world_state` (WorldState message)
- **Features**:
  - 2D → 3D projection using depth image and camera intrinsics
  - TF transformation from camera frame to `panda_link0` (robot base)
  - Object orientation (yaw) extraction from OBB corners
  - Calibration offsets for systematic errors

### 4. YOLOv8 OBB Publisher (`yolov8_obb_publisher.py`)

Real-time object detection with Oriented Bounding Boxes:

- **Input**: `/image_raw` (RGB camera feed)
- **Output**: `/Yolov8_Inference` (InferenceResult array with 4-corner coordinates)
- **Model**: Custom trained YOLOv8 OBB model (`best.pt`)
- **Confidence Threshold**: 0.90

## ROS 2 Topics and Services

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/world_state` | `WorldState` | Pub | Current detected objects with poses |
| `/task_plan` | `TaskAction` | Pub | Task-level action commands |
| `/planning_goal` | `std_msgs/String` | Sub | Trigger planning (e.g., "bolt") |
| `/Yolov8_Inference` | `Yolov8Inference` | Pub | YOLOv8 OBB detection results |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Pub | Depth image from camera |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Pub | Camera intrinsics |
| `/move_action` | `moveit_msgs/MoveGroup` | Action | MoveIt motion planning |

## Custom Messages

### WorldState.msg
```
std_msgs/Header header
ObjectState[] objects
```

### ObjectState.msg
```
string object_id
string class_name
geometry_msgs/Pose pose
```

### TaskAction.msg
```
string action_type      # PICK or PLACE
string object_id
geometry_msgs/Pose target_pose
```

### Yolov8Inference.msg
```
std_msgs/Header header
InferenceResult[] yolov8_inference
```

### InferenceResult.msg
```
string class_name
float64[] coordinates    # 8 values: [x1,y1, x2,y2, x3,y3, x4,y4]
```

## Installation

### Prerequisites

- **ROS 2** (Humble or later)
- **Python 3.8+**
- **MoveIt 2**
- **Gazebo** (for simulation)

### Dependencies

```bash
# ROS 2 packages
sudo apt install ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-gazebo-ros2-control

# Python dependencies
pip install ultralytics pyperplan opencv-python numpy

# Additional ROS packages
sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-tf2-ros
```

### Build

```bash
# Navigate to workspace
cd /path/to/TAMP-1

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

## Usage

### Full Integration with Gazebo

```bash
# Launch the complete system (Gazebo + MoveIt + Perception + Planner + Executor)
ros2 launch pytamp_moveit_bridge integration.launch.py
```

### Mock Perception (No Camera)

```bash
# Launch with mock perception for testing without real camera
ros2 launch pytamp_moveit_bridge fake_integration.launch.py
```

### Individual Components

```bash
# Launch just YOLOv8 perception
ros2 launch yolov8_obb yolov8_obb.launch.py

# Launch MoveIt + Gazebo
ros2 launch panda_moveit_config moveit_gazebo_obb.py
```

### Triggering a Pick-and-Place Task

```bash
# Publish a planning goal
ros2 topic pub /planning_goal std_msgs/String "data: 'bolt'" --once
```

## Configuration

### Robot Configuration

The Panda robot is configured with:
- **7-DOF arm** (panda_joint1 through panda_joint7)
- **2-finger gripper** (panda_finger_joint1, panda_finger_joint2)
- **Mounted camera** at `panda_link0` frame

### Planning Groups

| Group | Joints |
|-------|--------|
| `panda_arm` | panda_joint1-7 |
| `hand` | panda_finger_joint1, panda_finger_joint2 |
| `panda_arm_hand` | All joints |

### Named States

- `ready`: Standard picking position
- `extended`: Arm fully extended
- `transport`: Safe transport configuration

### OMPL Planners Available

RRTConnect, RRTstar, PRM, FMT, BFMT, LBKPIECE, and more (see `ompl_planning.yaml`)

## Calibration

The object state manager includes systematic calibration offsets:

```python
# Calibration correction: camera systematic errors
CALIB_Y_OFFSET = -0.137  # Y-axis over-estimation
CALIB_Z_OFFSET = -0.027  # Z-axis over-estimation
```

These compensate for systematic errors in the depth camera projection.

## Gripper Approach Strategy

The action executor uses a multi-step approach:

```python
# TCP offset for Panda end-effector
self.tcp_offset = 0.103        # Tool center point offset
self.pre_grasp_offset = 0.05   # Height above object for approach
self.lift_height = 0.10        # Lift height after grasp

# Grasp sequence:
# 1. Open gripper (0.04m)
# 2. Move to pre-grasp (object + pre_grasp_offset + tcp_offset)
# 3. Descend to grasp pose
# 4. Close gripper (0.009m for bolt shaft)
# 5. Lift
```

## PDDL Problem Generation

The planner dynamically generates PDDL problems based on detected objects:

```python
def _generate_problem_pddl(self, obj_id: str) -> str:
    safe_id = re.sub(r'[^a-z0-9_-]', '_', obj_id.lower())
    return f"""
    (define (problem pick-and-place-{safe_id})
      (:domain pick-and-place)
      (:objects {safe_id} table target)
      (:init (on {safe_id} table) (gripper-empty))
      (:goal (at-goal {safe_id}))
    )
    """
```

## Development

### Adding New Object Classes

1. Train a YOLOv8 OBB model on your objects
2. Replace `best.pt` in `yolov8_obb/scripts/`
3. Update the PDDL domain if new actions are needed

### Extending the Planner

The PDDL domain can be extended with additional predicates and actions:

```pddl
;; Example: Adding stacking capability
(:predicates
  (stackable ?o1 ?o2)
  (on-top ?o1 ?o2)
)

(:action stack
  :parameters (?o1 ?o2 ?l)
  :precondition (and (holding ?o1) (on ?o2 ?l) (stackable ?o1 ?o2))
  :effect (and (on-top ?o1 ?o2) (not (holding ?o1)) (gripper-empty))
)
```

## Troubleshooting

### Common Issues

1. **No objects detected**: Check YOLO model path and camera topics
2. **TF transform errors**: Ensure `robot_state_publisher` is running
3. **Motion planning failures**: Check collision scene and joint limits
4. **Gripper timeout**: Increase `stall_timeout` in controller config

### Debug Topics

```bash
# View world state
ros2 topic echo /world_state

# View YOLO detections
ros2 topic echo /Yolov8_Inference

# View planning goals
ros2 topic echo /planning_goal
```

## License

See individual package files for licensing information.

## Acknowledgments

- **MoveIt 2** - Motion planning framework
- **pyperplan** - PDDL planner backend
- **YOLOv8 (Ultralytics)** - Object detection
- **Franka Emika** - Panda robot platform
 9fe5bb3 (changed readme file)
