#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from edge_detection.srv import EdgeDetection, EdgeDetectionResponse

# Initialize CV Bridge
bridge = CvBridge()

def handle_edge_detection(req):
    # Read the input image
    image = cv2.imread(req.image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        rospy.logerr(f"Failed to load image: {req.image_path}")
        return EdgeDetectionResponse()

    # Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(image, (5, 5), 1.4)

    # Apply Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)

    # Convert the processed image to ROS Image format
    edge_image_msg = bridge.cv2_to_imgmsg(edges, encoding="mono8")

    return EdgeDetectionResponse(edge_image_msg)

def edge_detection_server():
    rospy.init_node('edge_detection_server')
    service = rospy.Service('edge_detection', EdgeDetection, handle_edge_detection)
    rospy.loginfo("Edge Detection Service Ready")
    rospy.spin()

if __name__ == "__main__":
    edge_detection_server()
