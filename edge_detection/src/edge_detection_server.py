#!/usr/bin/env python3
"""
Edge Detection Server

This script implements a ROS service for performing edge detection using OpenCV.
When the service receives a request containing an image file path, it loads the image,
applies a Gaussian blur to reduce noise, uses the Canny method for edge detection,
and returns the processed edge-detected image as a ROS Image message.
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from edge_detection.srv import EdgeDetection, EdgeDetectionResponse

# Create a CV Bridge instance to handle conversions between ROS and OpenCV images.
bridge = CvBridge()

def handle_edge_detection(req):
    """
    Handle the edge detection service request.
    
    Reads the image from the specified file path, processes it to detect edges,
    and returns the processed image wrapped in a ROS Image message.

    :param req: The service request containing 'image_path'.
    :return: A response containing the edge-detected image.
    """
    # Load the image in grayscale for edge detection.
    image = cv2.imread(req.image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        rospy.logerr(f"Failed to load image: {req.image_path}")
        return EdgeDetectionResponse()

    # Apply Gaussian Blur to smooth the image and reduce noise.
    blurred = cv2.GaussianBlur(image, (5, 5), 1.4)

    # Apply Canny edge detection to find edges in the blurred image.
    edges = cv2.Canny(blurred, 50, 150)

    # Convert the processed image back to a ROS Image message with mono8 encoding.
    edge_image_msg = bridge.cv2_to_imgmsg(edges, encoding="mono8")

    return EdgeDetectionResponse(edge_image_msg)

def edge_detection_server():
    """
    Initializes the ROS node and starts the edge detection service.
    The service listens for incoming requests and processes them using the
    handle_edge_detection callback.
    """
    rospy.init_node('edge_detection_server')
    # Create a service named 'edge_detection' with the specified callback.
    service = rospy.Service('edge_detection', EdgeDetection, handle_edge_detection)
    rospy.loginfo("Edge Detection Service is now ready.")
    rospy.spin()

if __name__ == "__main__":
    edge_detection_server()
