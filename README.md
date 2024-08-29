# Spot ROS2 FlexBE Behaviors and States

This repository contains a collection of FlexBE states, which are designed as service calls to interact with Boston Dynamics Spot's APIs. These states are modular pieces of behavior logic used to build complex robot behaviors. FlexBE is a behavior engine that enables the easy construction of robot behaviors using state machines, allowing developers to define and sequence robotic actions in a visual and intuitive manner. Each state script in this repository represents a specific action that can be part of a larger FlexBE behavior. 

The states in this repository can also be combined with those in the [spot_nav_behaviors repository](https://github.com/uml-robotics/spot_nav_behaviors) to enable multiple autonomous executions of behaviors. This is possible due to [the counter state](https://github.com/uml-robotics/spot_nav_behaviors/blob/master/spot_nav_flexbe_states/src/spot_nav_flexbe_states/counter.py) available in the spot_nav_behaviors repository, which allows for iterative execution of tasks.

----

## Description of States in `spot_ros2_flexbe_states`

* `arm_joint_move.py`: Calls the service for moving Spot's arm joints according to the 6 joint values provided as parameters. This state allows precise control of each arm joint, enabling complex manipulation tasks.

  **Parameters:**
  - **Name:** `joint_targets`
    - **Type:** `float32[]`
    - **Description:** The elements in this array should be in the following order, where each value corresponds to the target position for a specific joint: the first value is for joint `sh0`, the second is for `sh1`, the third is for `el0`, the fourth is for `el1`, the fifth is for `wr0`, and the sixth is for `wr1`.

* `arm_stow.py`: Calls the service to move Spot’s arm to the stow position, safely securing it in a compact state.

  **Parameters:** None

* `arm_unstow.py`: Calls the service to move Spot’s arm to the ready or unstow position, preparing it for manipulation tasks.

  **Parameters:** None

* `capture_image.py`: Calls the service to capture images from Spot's cameras. This state allows specifying the camera source and the local path where the image should be saved.

  **Parameters:**
  - **Name:** `source`
    - **Type:** `string`
    - **Description:** The camera source from which the image should be captured. The available sources are: `back_fisheye_image`, `frontleft_fisheye_image`, `frontright_fisheye_image`, `hand_color_image`, `left_fisheye_image`, and `right_fisheye_image`.

  - **Name:** `image_save_path`
    - **Type:** `string`
    - **Description:** The file path where the captured image should be saved locally.

* `dock.py`: Calls the service to command Spot to dock at a specified docking station, utilizing its onboard capabilities for autonomous docking.

  **Parameters:**
  - **Name:** `dock_id`
    - **Type:** `integer`
    - **Description:** The ID number of the docking station where Spot should dock.

* `grasp.py`: Calls the service for commanding Spot to autonomously grasp a specified object from a given frame.

  **Parameters:**
  - **Name:** `frame`
    - **Type:** `String`
    - **Description:** The frame of reference from which the object’s coordinates are provided.

  - **Name:** `object_rt_frame`
    - **Type:** `float32[]`
    - **Description:** The x, y, z coordinates of the object to be grasped, relative to the specified frame. Note that the array elements should be ordered as follows:
      - First element: x
      - Second element: y
      - Third element: z

* `gripper_close.py`: Calls the service to close Spot's gripper, either to secure an object that is being held or to close it completely. This state is typically used immediately after the grasp state to continue holding the object.

  **Parameters:** None

* `gripper_open.py`: Calls the service to open the Spot’s gripper. This state is typically used at the end of a manipulation sequence.

  **Parameters:** None

* `localize.py`: Calls the service to localize Spot to a specific waypoint. It is typically used at the beginning of a navigation sequence to orient Spot at a designated waypoint.

  **Parameters:**
  - **Name:** `waypoint_id`
    - **Type:** `String`
    - **Description:** The full ID of the waypoint where Spot should localize itself. Use the complete waypoint ID (e.g., `maiden-bowfin-iR+9MA8N4SeW6mFwYh7uNQ==`) rather than short forms (e.g., `mb`).

* `navigate.py`: Calls the service to navigate the Spot robot to a specified waypoint ID. Note that this navigation state uses waypoint IDs, not fiducials, as the fiducial option is not enabled within this state. This state can also localize the robot to a waypoint and upload a map if needed.

  **Parameters:**
  - **Name:** `goal_waypoint_id`
    - **Type:** `String`
    - **Description:** The full waypoint ID of the destination waypoint where Spot should navigate. Use the complete waypoint ID (e.g., `maiden-bowfin-iR+9MA8N4SeW6mFwYh7uNQ==`) rather than short forms (e.g., `mb`).

  - **Name:** `init_waypoint_id`
    - **Type:** `String`
    - **Description:** The full waypoint ID of the initial waypoint where Spot should localize itself. Use the complete waypoint ID (e.g., `maiden-bowfin-iR+9MA8N4SeW6mFwYh7uNQ==`) rather than short forms (e.g., `mb`).

  - **Name:** `upload_path`
    - **Type:** `String`
    - **Description:** Path to the directory containing the graph/map to be uploaded to the robot.

* `sit.py`: Calls the service to command Spot to sit.

  **Parameters:** None

* `stand.py`: Calls the service to command Spot to stand up.

  **Parameters:** None

* `undock.py`: Calls the service to command Spot to undock from a docking station.

  **Parameters:** None

* `upload_graph.py`: Calls the service to upload a graph to the Spot, which would be required for navigation.

  **Parameters:**
  - **Name:** `path_to_graph`
    - **Type:** `String`
    - **Description:** Path to the directory containing the graph/map to be uploaded to the robot.


## Dependencies and Installation
### Dependencies
- **FlexBE**
- **spot_ros2**

To use these ROS2 FlexBE states with Boston Dynamics Spot, ensure you have a compatible ROS2 setup and the necessary dependencies mentioned above installed.

### 1. Install FlexBE Behavior Engine
For detailed installation instructions, visit: [FlexBE Behavior Engine Installation](https://github.com/FlexBE/flexbe_behavior_engine/tree/humble).

**Commands:**
```bash
# Navigate to the ros_workspace/src directory
cd ros_workspace/src

# Clone the FlexBE Behavior Engine repository
git clone https://github.com/FlexBE/flexbe_behavior_engine.git
```
### 2. Install FlexBE App
For detailed installation instructions, visit: [FlexBE App Installation](https://github.com/FlexBE/flexbe_app/tree/humble).

**Commands:**
```bash
# Navigate to the ros_workspace/src directory
cd ros_workspace/src

# Clone the FlexBE App repository
git clone https://github.com/FlexBE/flexbe_app.git

# Update rosdep
rosdep update

# Install dependencies
rosdep install --from-paths src --ignore-src
```

### 3. Clone and Install spot_ros2
For detailed installation instructions, visit: [spot_ros2 Installation](https://github.com/bdaiinstitute/spot_ros2?tab=readme-ov-file#installation)

**Commands:**
```bash
# Clone the spot_ros2 repository
# NOTE: Replace the link below with the link to the repository with the newly added services
git clone https://github.com/bdaiinstitute/spot_ros2.git

# Navigate to the spot_ros2 directory
cd spot_ros2

# Initialize and update git submodules
git submodule init
git submodule update

# Install dependencies for spot_ros2
./install_spot_ros2.sh
```

### 4. Clone and install this repository
```
git clone https://github.com/yashpatel1392/spot_ros2_behaviors
```

### 5. Build the workspace
```
colcon build
source install/setup.bash
```

## Usage
### 1. Run Spot ROS2 Driver
The Spot ROS2 Driver 
```
ros2 launch spot_driver spot_driver.launch.py [config_file:=<path/to/config.yaml>] [spot_name:=<Robot Name>] [publish_point_clouds:=<True|False>] [launch_rviz:=<True|False>] [uncompress_images:=<True|False>] [publish_compressed_images:=<True|False>]
```
### 2. Run Flexbe Application
```
ros2 launch flexbe_app flexbe_full.launch.py
```

### 3. Create or Edit a Behavior (Assuming the FlexBE App is running)
1. **Create a New Behavior:** Click on the "Create New Behavior" button. Name your behavior and define its properties.
2. **Add States:** Use the drag-and-drop interface to add states to your behavior. You can incorporate the provided states (e.g., sit, `stand`, `undock`, `upload_graph`) by dragging them from the list of available states.
3. **Configure States:** Click on each state to configure its parameters, such as service calls or action goals. For instance, set parameters for the `upload_graph` state to include the `path_to_graph`.
4. **Define Transitions:** Set up transitions between states by connecting them with arrows. Define the conditions under which the behavior transitions from one state to another.
5. **Save the Behavior:** Once you have configured your states and transitions, save your behavior.

### 4. Execute the Behavior
1. **Load the Behavior:** In the FlexBE Behavior Designer interface, load the behavior you created or edited.
2. **Execute the Behavior:** Click on the "Start" button to execute the behavior. Monitor the execution and ensure the states are processed as expected.
