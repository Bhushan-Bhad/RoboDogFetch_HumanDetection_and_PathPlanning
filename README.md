# RoboDogFetch_HumanDetection_and_PathPlanning

This repository contains the code and documentation for the RoboDogFetch project, a robotic system built using the Unitree Go1 Edu quadruped robot. The project integrates object detection, hand gesture recognition, indoor navigation, and path planning to enable the robot to autonomously fetch and return a ball based on hand gestures.  

## **Project Overview**  

The project consists of three distinct repositories, each corresponding to one of the three computational units (boards) involved in the system:  

1. **Internal Raspberry Pi (192.168.123.161):**  
   Runs ROS Noetic and publishes fisheye camera point clouds via ROS topics. These topics are sent to the Jetson board for navigation and mapping tasks.  

2. **External Raspberry Pi (192.168.123.17):**  
   Runs Ubuntu 20.04 and ROS Melodic. Equipped with a webcam and updated Python 3.8, this Raspberry Pi is responsible for:  
   - Object detection and hand gesture recognition (using YOLOv8 and Mediapipe).  
   - Robot control via the Unitree Legged SDK.  

3. **Jetson Board (192.168.123.15):**  
   Acts as the master node. Handles:  
   - SLAM and path planning using point cloud data received from the internal Raspberry Pi.  
   - Integration with the external Raspberry Pi for target identification and gesture detection.  

## **System Requirements**  

### **Hardware**  
- **Unitree Go1 Edu:** Equipped with fisheye cameras and powered by internal hardware.  
- **Jetson Board:** For SLAM and path planning computations.  
- **Two Raspberry Pi Boards:**  
  - Internal (pre-installed in the robot).  
  - External (with a connected webcam).  

### **Software**  
- **Internal Raspberry Pi:**  
  - Ubuntu 18.04, ROS Noetic.  
- **Jetson Board:**  
  - Ubuntu 18.04, ROS Melodic.  
  - Python 3.6.  
- **External Raspberry Pi:**  
  - Ubuntu 20.04, ROS Melodic.  
  - Python 3.8.  
  - Mediapipe, YOLOv8 for object and gesture detection.  

## **Setup Instructions**  

### **1. Connecting to the Unitree Robot**  
1. Connect your local machine to the Unitree WiFi network:  
   - SSID: `UnitreeRoboticsGO1-000`  
   - Password: `00000000`  
2. SSH into the robot:  
   ```bash
   ssh -X unitree@192.168.123.161
   password: 123
   ```  

### **2. Accessing Fisheye Camera Data**  
To obtain fisheye camera data:  
1. Access the robot's dashboard by navigating to `192.168.12.1` in a browser.  
2. Under the "Vision" tab, explore the camera feeds and system diagnostics.  

### **3. Communication Setup**  
- The internal Raspberry Pi publishes point cloud data to the Jetson board using ROS.  
- The external Raspberry Pi communicates with the Jetson board via an Ethernet cable for gesture detection and robot control.  

### **4. Path Planning and Navigation**  
1. The Jetson board subscribes to point cloud topics for dynamic mapping.  
2. The path planning algorithm (A*) computes an optimal trajectory based on a dynamic map.  
3. Robot control commands are executed using the Unitree Legged SDK.  

## **Dynamic Map Generation and Path Planning**  

- **Dynamic Mapping:**  
   Implemented using a Sliding Window Aggregation (SWA) algorithm to process fisheye camera point clouds into real-time maps.  
- **Path Planning:**  
   Uses the A* algorithm to compute efficient paths while dynamically avoiding obstacles.  

## **Challenges and Solutions**  

1. **Fisheye Camera Limitations:**  
   - The fisheye cameras were unsuitable for precise gesture recognition due to distortion.  
   - Solution: Integrated an external webcam with higher clarity.  

2. **SLAM Challenges on Jetson:**  
   - Dependency conflicts between OpenCV versions (3.2 and 4.1).  
   - Limited RAM (4 GB) caused memory overflows during SLAM library builds.  
   - Solution: Switched to dynamic mapping with point clouds instead of SLAM.  

3. **External Raspberry Pi Overhead:**  
   - Object detection added latency, and the Raspberry Pi occasionally crashed.  
   - Solution: Optimized object detection algorithms for reduced computational load.  

4. **Jetson Python Compatibility:**  
   - Python 3.8 required for YOLOv8, but Jetson supports only Python 3.6.  
   - Solution: Used the external Raspberry Pi for Python 3.8-dependent tasks.  

## **Results and Evaluation**  

- **Dynamic Mapping:** Successfully generated maps using fisheye camera data and avoided obstacles in real-time.  
- **Gesture Recognition:** Achieved robust detection of "Hi" gestures in dynamic environments.  
- **Path Planning:** A* algorithm computed paths with minimal deviation (1°) and dynamically re-planned in response to obstacles.  

## **Future Improvements**  
- Upgrade Jetson board to support newer libraries and Python versions.  
- Explore lightweight SLAM libraries compatible with limited hardware.  
- Investigate advanced camera systems (e.g., depth cameras) to enhance gesture detection.  

## **Known Issues**  
- Occasional crashes of the external Raspberry Pi during prolonged operation.  
- Limited computational power for SLAM integration.  

## **Conclusion**  
The RoboDogFetch system demonstrates the feasibility of integrating camera-based navigation, gesture recognition, and dynamic path planning into a robotic platform. This research thesis provides insights into overcoming hardware and software limitations in autonomous mobile robotics.  
