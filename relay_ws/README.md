### **relay_ws Repository**  

This repository handles the transfer of ROS topics related to fisheye camera point cloud data from the internal Raspberry Pi to the master Jetson board for further processing.  

#### **Setup and Usage**  

1. **SSH into the Internal Raspberry Pi**  
   - Use the following command to access the internal Raspberry Pi (IP: `192.168.123.161`):  
     ```bash
     ssh pi@192.168.123.161
     password: 00000000
     ```

2. **Build the Code**  
   - Navigate to the `relay_ws` directory and build the ROS workspace:  
     ```bash
     cd ~/relay_ws
     catkin_make
     source devel/setup.bash
     ```

3. **Set Up ROS Environment**  
   - Configure the ROS environment variables to set the Jetson board as the master and this Raspberry Pi as the slave:  
     ```bash
     export ROS_MASTER_URI=http://192.168.123.15:11311
     export ROS_IP=192.168.123.161
     ```

4. **Launch the Node**  
   - Run the ROS node to publish the fisheye camera point cloud data:  
     ```bash
     rosrun relay_pkg relay_node.py
     ```

#### **How It Works**  
- **ROS_MASTER_URI**: Specifies the Jetson board (`192.168.123.15`) as the ROS master for the system.  
- **ROS_IP**: Sets the IP address of the internal Raspberry Pi (`192.168.123.161`) as its communication endpoint.  
- Once launched, this node subscribes to the fisheye camera topics available on the internal Raspberry Pi and publishes them to the master Jetson board for further processing, such as mapping and path planning.  

#### **Notes**  
- Ensure both the internal Raspberry Pi and the Jetson board are connected to the same network.  
- Run the same `ROS_MASTER_URI` and `ROS_IP` commands on every new terminal session before launching the ROS nodes.  
