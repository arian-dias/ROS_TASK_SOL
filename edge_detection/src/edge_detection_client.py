#!/usr/bin/env python3
"""
Edge Detection Client

This script acts as a client for the ROS edge detection service.
It reads images from a designated data folder, sends each image
to the edge detection service, and then displays the resulting
edge-detected images.
"""

import rospy
import os
from edge_detection.srv import EdgeDetection
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Create a CV Bridge instance for converting between ROS and OpenCV images.
bridge = CvBridge()

def edge_detection_client(data_directory):
    """
    For each image in the specified directory, call the edge detection service
    and display the processed (edge-detected) image.
    """
    print("Starting Edge Detection Client...")
    
    # Initialize the ROS node for this client.
    rospy.init_node('edge_detection_client', anonymous=True)
    
    # Wait for the edge detection service to be available.
    print("Waiting for the edge_detection service to become available...")
    rospy.wait_for_service('edge_detection')
    print("Edge Detection service is now available!")

    try:
        # Create a proxy to call the edge detection service.
        detect_edges = rospy.ServiceProxy('edge_detection', EdgeDetection)

        # Iterate through all files in the data directory.
        for image_file in sorted(os.listdir(data_directory)):
            # Only process files that are likely images.
            if image_file.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff')):
                full_image_path = os.path.join(data_directory, image_file)
                rospy.loginfo(f"Processing image: {full_image_path}")
                print(f"Sending image to service: {full_image_path}")

                # Call the service with the image file path.
                response = detect_edges(full_image_path)
                print("Received response from edge detection service!")

                # Convert the received ROS Image message into an OpenCV image (grayscale).
                edge_detected_image = bridge.imgmsg_to_cv2(response.edge_image, "mono8")
                print(f"Processed image dimensions: {edge_detected_image.shape}")

                # Validate the resulting image.
                if edge_detected_image is None or edge_detected_image.size == 0:
                    rospy.logerr(f"Empty image returned for {full_image_path}. Skipping...")
                    continue

                # Display the edge-detected image.
                cv2.imshow("Edge Detection", edge_detected_image)
                cv2.waitKey(500)  # Display each image for 500 milliseconds
                
        # Close all the OpenCV windows after processing.
        cv2.destroyAllWindows()

    except rospy.ServiceException as error:
        rospy.logerr(f"Failed to call edge detection service: {error}")

if __name__ == "__main__":
    # Determine the absolute path of this script.
    current_script_path = os.path.abspath(__file__)
    # Assume this script is in the 'src' folder; get that directory.
    source_directory = os.path.dirname(current_script_path)
    # The 'data' folder is assumed to be a sibling of the 'src' folder.
    data_directory = os.path.join(source_directory, '..', 'data')
    data_directory = os.path.normpath(data_directory)

    print(f"Using image data from: {data_directory}")
    edge_detection_client(data_directory)
