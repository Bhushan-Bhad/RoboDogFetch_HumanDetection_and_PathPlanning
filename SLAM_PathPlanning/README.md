### **SLAM_PathPlanning Workspace**

This workspace is designed to handle SLAM, receive ROS topics (e.g., fisheye camera point clouds) from the internal Raspberry Pi (`192.168.123.161`), and perform path planning on the Jetson board (`192.168.123.15`). The Jetson board acts as the ROS master and performs the primary computational tasks.

#### **Setup and Usage**

1. **Connect to the Jetson Board**  
   - SSH into the Jetson board:  
     ```bash
     ssh unitree@192.168.123.15
     password: 123
     ```

2. **Build the Code**  
   - Navigate to the `SLAM_PathPlanning` workspace and build it:  
     ```bash
     cd ~/SLAM_PathPlanning
     catkin_make
     source devel/setup.bash
     ```

3. **Set Up ROS Environment**  
   - Configure the Jetson board as the ROS master:  
     ```bash
     export ROS_MASTER_URI=http://192.168.123.15:11311
     export ROS_IP=192.168.123.15
     ```

4. **Launch All Nodes**  
   - Use the `master.launch` file to start SLAM, path planning, and communication with other boards:  
     ```bash
     roslaunch SLAM_PathPlanning master.launch
     ```

#### **How It Works**
- **SLAM**:  
  The Jetson board performs dynamic mapping using a point cloud from the internal Raspberry Pi. Due to hardware limitations, traditional visual SLAM methods (e.g., ORB-SLAM) are replaced by a custom approach using point cloud data and a dynamic window-based mapping algorithm.
  
- **Receiving ROS Topics**:  
  The `master.launch` file subscribes to point cloud data published by the internal Raspberry Pi over ROS. This data is processed to create a map of the environment for navigation.

- **Path Planning**:  
  A* algorithm is used for real-time path planning. It dynamically computes the optimal path to the target while avoiding obstacles. The calculated waypoints are sent to the robot control node for execution.

#### **Notes**
- Ensure the Jetson board is configured as the ROS master, and all slave devices (internal Raspberry Pi and external Raspberry Pi) point to it using their respective `ROS_MASTER_URI` and `ROS_IP` configurations.  
- The Jetson board must have internet access to install any missing dependencies.  
- Launch the `master.launch` file only after ensuring that the internal Raspberry Pi is publishing the required ROS topics and the external Raspberry Pi is set up for object detection and hand gesture recognition.
