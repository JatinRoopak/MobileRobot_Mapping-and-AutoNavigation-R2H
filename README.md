# Autonomous Mobile Robot Navigation

This package implements a differential drive mobile robot in ROS 2. It features a complete navigation stack including Gazebo simulation, SLAM (Simultaneous Localization and Mapping), and autonomous path planning using the Navigation 2 (Nav2) stack.

## 🎥 Demo
[Watch the simulation in action](https://github.com/user-attachments/assets/22191b87-0ffd-4239-b749-aca34fe9d9a0)

## 🛠️ Prerequisites
Ensure you have ROS 2 (Humble or Iron) installed. Install the required navigation and simulation packages:

```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-twist-mux
```

## 📦 Installation
1. **Clone the repository** into your workspace `src` folder:
```bash
cd ~/ros2_ws/src
git clone <YOUR_REPO_LINK_HERE>
```

2. **Build the package**:
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 🚀 Usage Guide (Running Order)

To run the full autonomous simulation, open **5 separate terminals**. 
*Note: Make sure to source your workspace in every terminal:* `source install/setup.bash`

### 1. Launch Simulation
This starts the Gazebo environment, spawns the robot, and publishes the robot state.
```bash
ros2 launch mobile_robot viz.launch.py use_sim_time:=true
```

### 2. Start Twist Multiplexer
This handles velocity commands. *(Note: Install `twist_mux` if you don't have it using the prerequisites above)*.
```bash
ros2 run twist_mux twist_mux --ros-args --params-file ./src/mobile_robot/config/twist_mux.yaml -r cmd_vel_out:=/diff_cont/cmd_vel_unstamped -p use_sim_time:=true
```

### 3. Launch SLAM (Mapping)
This starts `slam_toolbox` to generate the map in real-time.
```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/mobile_robot/config/mapper_params_online_async.yaml use_sim_time:=true
```

### 4. Launch Navigation Stack (Nav2)
This starts the path planning and costmaps.
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

### 5. Visualization
Open RViz to visualize the robot and set goals.
```bash
rviz2
```

---

## 🗺️ How to Navigate & Configure RViz

When you first open RViz, you may need to configure the displays manually.

### 1. Set Global Options
* In the **Displays** panel (left side), find **Global Options**.
* Set **Fixed Frame** to `map`.

### 2. Add Displays
Click the **Add** button in the bottom-left corner and add the following:

* **RobotModel**:
  * This visualizes your robot's URDF mesh.
  * **Description Topic**: Ensure it is set to `/robot_description`.
  
* **Map** (For SLAM):
  * **Topic**: Select `/map`.
  * **Durability Policy**: **Crucial!** Change this to `Transient Local`. If you leave it as 'Volatile', the map will not appear.

* **TF** (Transforms):
  * This shows the coordinate frames (base_link, odom, etc.) to verify connections.

* **Map** (For Costmaps - Optional):
  * Add another Map display and set the topic to `/global_costmap/costmap` to see what the navigation stack sees.

### 3. Initialize Pose
* Click the **2D Pose Estimate** button in the top toolbar.
* Click and drag on the map where your robot is currently standing to align the internal logic with the visual simulation.

### 4. Send a Goal
* Click the **Nav2 Goal** button in the top toolbar.
* Click any point on the map and drag to set the orientation.
* The robot will plan a path (green line) and drive there autonomously.

## ⚠️ Troubleshooting
If you see errors like `Timed out waiting for transform` or the map never loads, it is usually a time synchronization issue. 
* **Fix:** Ensure you included `use_sim_time:=true` in **ALL** commands (Steps 1-4). This forces all nodes to use the simulation clock instead of the system wall clock.