# ROS Task Solution - Edge Detection

Welcome to the ROS Task Solution for Edge Detection! This project demonstrates how to process camera images with an edge detection algorithm while also visualizing pointcloud data in RViz. Whether you're an experienced ROS developer or just starting out, this guide will walk you through the process.

## How It Works

1. **Image Acquisition:**  
   The node subscribes to a ROS topic streaming image data from your robot's camera.

2. **Image Preprocessing:**  
   Using `cv_bridge`, incoming ROS image messages are converted into OpenCV images, and a Gaussian blur is applied to reduce noise.

3. **Edge Detection:**  
   The Canny edge detection algorithm processes the preprocessed images, identifying edges based on rapid changes in brightness.

4. **Data Visualization & Transformation:**  
   Along with edge detection, pointcloud data is visualized in RViz after applying necessary transformations. This provides a comprehensive view of both the image and spatial data.

## What You Need

Before you begin, make sure you have the following installed:

- **ROS:**  
  Tested with [ROS Noetic](http://wiki.ros.org/noetic), though it should work with other versions with minimal adjustments.
  
- **OpenCV:**  
  Required for image processing. Install the version that matches your ROS distribution.

- **cv_bridge:**  
  Necessary for converting between ROS image messages and OpenCV images.

- **Python:**  
  Our nodes are written in Python 3 (as used in ROS Noetic). For older ROS versions, ensure compatibility with Python 2 if needed.
- **To Run**
  To make sure that you have the necesssary bag files and Robot URDF folder with correct gripper attached to it.
  For Basic Task: Run Canny_edge_detector.py
  For Vision_ROS: Run edge_detection_client.py and server
  For Robot_ROS: Run edge_detection_with_transformation.py








## Getting Started

1. **Clone the Repository into Your ROS Workspace:**

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/arian-dias/ROS_TASK_SOL.git
