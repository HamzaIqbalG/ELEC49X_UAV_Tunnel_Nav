# Development Guide for Beginners

This guide will help you get started with developing on the ELEC49X UAV Tunnel Navigation project.

## Table of Contents
1. [Initial Setup](#initial-setup)
2. [Daily Workflow](#daily-workflow)
3. [Understanding the Project Structure](#understanding-the-project-structure)
4. [Running Components](#running-components)
5. [Adding New Code](#adding-new-code)
6. [Common Tasks](#common-tasks)
7. [Troubleshooting](#troubleshooting)

---

## Initial Setup

### First Time Setup (One-time)

1. **Clone the repository** (if you haven't already):
   ```bash
   git clone <repository-url>
   cd ELEC49X_UAV_Tunnel_Nav
   ```

2. **Source ROS 2 Humble**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
   > **Note:** You'll need to do this every time you open a new terminal, or add it to your `~/.bashrc`

3. **Build the workspace**:
   ```bash
   ./setup_workspace.sh
   ```
   This will:
   - Install dependencies
   - Build all packages
   - Set up the workspace

4. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```
   > **Note:** You'll need to do this every time you open a new terminal to use the packages

5. **Verify setup**:
   ```bash
   ros2 launch uav_bringup test_workspace.launch.py
   ```
   In another terminal:
   ```bash
   source install/setup.bash
   ros2 topic echo /test_topic
   ```
   You should see messages being published. Press `Ctrl+C` to stop.

---

## Daily Workflow

### Starting Your Work Session

Every time you start working, open a terminal and run:

```bash
# 1. Navigate to workspace
cd ~/ELEC49X_UAV_Tunnel_Nav

# 2. Source ROS 2
source /opt/ros/humble/setup.bash

# 3. Source your workspace
source install/setup.bash
```

**Pro Tip:** Add these to your `~/.bashrc` to auto-source:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ELEC49X_UAV_Tunnel_Nav/install/setup.bash" >> ~/.bashrc
```

### Making Changes and Rebuilding

1. **Edit your code** in `src/<package_name>/`

2. **Rebuild the package**:
   ```bash
   colcon build --symlink-install --packages-select <package_name>
   ```
   Example:
   ```bash
   colcon build --symlink-install --packages-select uav_bringup
   ```

3. **Rebuild everything** (if needed):
   ```bash
   colcon build --symlink-install
   ```

4. **Re-source** (if you're in the same terminal):
   ```bash
   source install/setup.bash
   ```

---

## Understanding the Project Structure

```
ELEC49X_UAV_Tunnel_Nav/
├── src/
│   ├── uav_simulation/          # Gazebo simulation package
│   │   ├── models/              # 3D models (drone, etc.)
│   │   ├── worlds/              # Gazebo world files
│   │   ├── launch/              # Launch files for Gazebo
│   │   ├── package.xml          # Package dependencies
│   │   └── CMakeLists.txt       # Build configuration
│   │
│   ├── uav_bringup/             # Main ROS 2 package
│   │   ├── src/                 # C++ source files
│   │   │   ├── test_node.cpp    # Test node (currently used)
│   │   │   └── basic_node.cpp   # UAV node (for future)
│   │   ├── launch/              # Launch files
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   ├── tunnel_controller/       # (To be created) Control algorithms
│   ├── tunnel_perception/       # (To be created) Perception/localization
│   └── msgs/                    # (To be created) Custom messages
│
├── install/                     # Built/installed files (don't edit)
├── build/                       # Build artifacts (don't edit)
├── log/                         # Build logs (don't edit)
├── setup_workspace.sh           # Setup script
└── README.md                    # Project overview
```

---

## Running Components

### 1. Test ROS 2 Integration

**Purpose:** Verify that ROS 2 workspace is working correctly.

**Terminal 1:**
```bash
source install/setup.bash
ros2 launch uav_bringup test_workspace.launch.py
```

**Terminal 2:**
```bash
source install/setup.bash
ros2 topic list              # See all topics
ros2 topic echo /test_topic   # See messages
```

**What you should see:**
- Terminal 1: Messages like "Publishing: 'Hello from test_node! Count: X'"
- Terminal 2: Messages being echoed

**Stop:** Press `Ctrl+C` in Terminal 1

---

### 2. Launch Gazebo Simulation

**Purpose:** Run the Gazebo simulator with the tunnel world.

```bash
source install/setup.bash
ros2 launch uav_simulation simulate.launch.py
```

**What you should see:**
- Gazebo GUI opens
- Tunnel world loads (gray rectangular tunnel)
- Some ALSA/audio warnings (harmless, can be ignored)

**Stop:** Close Gazebo window or press `Ctrl+C`

---

### 3. Run Individual Nodes

**Purpose:** Run a specific node without a launch file.

```bash
source install/setup.bash
ros2 run uav_bringup test_node
```

**Stop:** Press `Ctrl+C`

---

## Adding New Code

### Creating a New ROS 2 Node

1. **Create the source file** in `src/<package_name>/src/`:
   ```cpp
   // src/uav_bringup/src/my_new_node.cpp
   #include <rclcpp/rclcpp.hpp>
   
   class MyNewNode : public rclcpp::Node
   {
   public:
     MyNewNode() : Node("my_new_node")
     {
       RCLCPP_INFO(this->get_logger(), "My new node started!");
     }
   };
   
   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<MyNewNode>());
     rclcpp::shutdown();
     return 0;
   }
   ```

2. **Add to CMakeLists.txt**:
   ```cmake
   add_executable(my_new_node src/my_new_node.cpp)
   ament_target_dependencies(my_new_node rclcpp)
   
   install(TARGETS
     my_new_node
     DESTINATION lib/${PROJECT_NAME}
   )
   ```

3. **Build**:
   ```bash
   colcon build --symlink-install --packages-select uav_bringup
   source install/setup.bash
   ```

4. **Run**:
   ```bash
   ros2 run uav_bringup my_new_node
   ```

---

### Creating a New Package

1. **Navigate to src/**:
   ```bash
   cd src
   ```

2. **Create package** (example for a controller):
   ```bash
   ros2 pkg create --build-type ament_cmake tunnel_controller \
     --dependencies rclcpp std_msgs geometry_msgs
   ```

3. **Edit package.xml** - Update description, maintainer, etc.

4. **Add your code** in `tunnel_controller/src/`

5. **Build**:
   ```bash
   cd ..
   colcon build --symlink-install --packages-select tunnel_controller
   source install/setup.bash
   ```

---

## Common Tasks

### Viewing ROS 2 Topics

```bash
# List all topics
ros2 topic list

# See messages on a topic
ros2 topic echo /topic_name

# See topic info
ros2 topic info /topic_name

# See topic type
ros2 topic type /topic_name
```

### Viewing ROS 2 Nodes

```bash
# List all running nodes
ros2 node list

# Get node info
ros2 node info /node_name
```

### Viewing ROS 2 Services

```bash
# List all services
ros2 service list

# Call a service
ros2 service call /service_name service_type arguments
```

### Debugging

**Check if package built correctly:**
```bash
colcon build --symlink-install --packages-select <package_name> --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

**View build errors:**
```bash
# Build with verbose output
colcon build --symlink-install --event-handlers console_direct+
```

**Check if node is running:**
```bash
ros2 node list
```

**Check if topics are publishing:**
```bash
ros2 topic hz /topic_name  # See publishing rate
ros2 topic bw /topic_name  # See bandwidth
```

---

## Troubleshooting

### "Package not found" error

**Problem:** `ros2 run` or `ros2 launch` can't find your package

**Solution:**
```bash
# Make sure you've sourced the workspace
source install/setup.bash

# Verify package is built
colcon list

# Rebuild if needed
colcon build --symlink-install
```

### "No executable found" error

**Problem:** Node executable doesn't exist

**Solution:**
1. Check CMakeLists.txt has the executable defined
2. Rebuild: `colcon build --symlink-install --packages-select <package>`
3. Re-source: `source install/setup.bash`

### Gazebo not launching

**Problem:** Gazebo fails to start

**Solution:**
```bash
# Check if Gazebo is installed
gazebo --version

# Try launching Gazebo directly
gazebo

# Check for port conflicts (Gazebo uses port 11345)
# Kill any existing Gazebo processes
pkill -f gazebo
```

### Build errors

**Problem:** Code doesn't compile

**Solution:**
1. Check for syntax errors in your code
2. Verify all dependencies are in `package.xml`
3. Check CMakeLists.txt is correct
4. Look at build output for specific error messages
5. Clean and rebuild:
   ```bash
   rm -rf build/ install/
   colcon build --symlink-install
   ```

### Topics not appearing

**Problem:** Can't see expected topics

**Solution:**
1. Make sure the node is actually running: `ros2 node list`
2. Check the node is publishing: `ros2 node info /node_name`
3. Verify topic names match (check for typos)
4. Make sure you've sourced the workspace

---

## Quick Reference Commands

```bash
# Setup (first time)
source /opt/ros/humble/setup.bash
./setup_workspace.sh
source install/setup.bash

# Daily workflow
source /opt/ros/humble/setup.bash
source install/setup.bash

# Build
colcon build --symlink-install                    # Build all
colcon build --symlink-install --packages-select <pkg>  # Build one package

# Run
ros2 launch <package> <launch_file>              # Launch file
ros2 run <package> <executable>                  # Single node

# Debug
ros2 topic list                                  # List topics
ros2 topic echo /topic_name                      # Echo topic
ros2 node list                                   # List nodes
ros2 node info /node_name                        # Node info
```

---

## Next Steps

1. **Familiarize yourself** with the test node code in `src/uav_bringup/src/test_node.cpp`
2. **Try modifying** the test node to publish different messages
3. **Create your first node** following the "Adding New Code" section
4. **Explore ROS 2** - check out the [ROS 2 documentation](https://docs.ros.org/en/humble/)

Happy coding! 🚀

