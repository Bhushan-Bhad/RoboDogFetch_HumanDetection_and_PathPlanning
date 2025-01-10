### **ObjectDetection Workspace**  

This workspace handles object detection, hand gesture recognition, and robot control using the external Raspberry Pi (`192.168.123.17`). The Raspberry Pi runs Ubuntu 20.04 with ROS Noetic and is connected to the Jetson board via an Ethernet cable. It also powers an external webcam for precise human detection and gesture recognition.

#### **Setup and Usage**  

1. **Connect to the External Raspberry Pi**  
   - SSH into the Raspberry Pi using the following command:  
     ```bash
     ssh pi@192.168.123.17
     password: [your-password]
     ```

2. **Build the Code**  
   - Navigate to the `ObjectDetection` workspace and build the ROS workspace:  
     ```bash
     cd ~/ObjectDetection
     catkin_make
     source devel/setup.bash
     ```

3. **Set Up ROS Environment**  
   - Configure the ROS environment variables to set the Jetson board as the ROS master and this Raspberry Pi as a slave:  
     ```bash
     export ROS_MASTER_URI=http://192.168.123.15:11311
     export ROS_IP=192.168.123.17
     ```

4. **Launch All Nodes**  
   - Use the `all_node.launch` file to start object detection, hand gesture recognition, and robot control:  
     ```bash
     roslaunch ObjectDetection all_node.launch
     ```

#### **How It Works**  
- **Object Detection**: Detects objects (e.g., human detection ) using YOLOv8.  
- **Hand Gesture Recognition**: Recognizes specific hand gestures (e.g., the "Hi" gesture) using Mediapipe.  
- **Robot Control**: Sends commands to the Unitree robot dog using the Unitree Legged SDK for precise movements.  

The `all_node.launch` file is configured to:  
- Initialize object detection and gesture recognition nodes.  
- Set up communication with the Jetson board for target identification.  
- Control the robot's movements based on detected gestures and objects.

#### **Notes**  
- Ensure the Raspberry Pi and the Jetson board are connected via an Ethernet cable.  
- Attach the external webcam to this Raspberry Pi before launching the nodes.  
- Run the `ROS_MASTER_URI` and `ROS_IP` commands in every new terminal session before launching the `all_node.launch` file.
