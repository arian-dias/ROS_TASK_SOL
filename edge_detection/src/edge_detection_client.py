#!/usr/bin/env python3

import rospy
import os
from edge_detection.srv import EdgeDetection
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Initialize CV Bridge
bridge = CvBridge()

def edge_detection_client(directory_path):
    print("Starting Edge Detection Client...")
    rospy.init_node('edge_detection_client', anonymous=True)
    
    print("Waiting for service...")
    rospy.wait_for_service('edge_detection')
    print("Service is available!")

    try:
        edge_detect = rospy.ServiceProxy('edge_detection', EdgeDetection)

        # Process all images in the directory
        for filename in sorted(os.listdir(directory_path)):
            if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff')):
                image_path = os.path.join(directory_path, filename)
                rospy.loginfo(f"Processing: {image_path}")
                print(f"Calling service with: {image_path}")

                # Call the service
                response = edge_detect(image_path)
                print("Service response received!")

                # Convert ROS Image message to OpenCV format
                edge_image = bridge.imgmsg_to_cv2(response.edge_image, "mono8")
                print(f"Image size: {edge_image.shape}")

                # Ensure edge_image is not empty
                if edge_image is None or edge_image.size == 0:
                    rospy.logerr(f"Received an empty image for {image_path}")
                    continue

                # Display the detected edges
                cv2.imshow("Edge Detection", edge_image)
                cv2.waitKey(500)  # Display each image for 500ms
                
        cv2.destroyAllWindows()
    
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    directory_path = os.path.expanduser("~/catkin_ws/src/edge_detection_ros_challenge/edge_detection_ros/edge_detection/data")
    print(f"Using directory: {directory_path}")
    edge_detection_client(directory_path)
