#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Init CV Bridge
bridge = CvBridge()

#  Init the Publishers
image_pub = None
edge_pub = None

# Callback for RGB image
def image_callback(msg):
    """
    Callback for processing the raw RGB image.
    
    Steps:
      - Convert the ROS image message to an OpenCV image.
      - Convert the image to grayscale.
      - Apply Gaussian blur to reduce noise.
      - Use the Canny algorithm to detect edges.
      - Publish the original image and the processed edge image.
    """
    global image_pub, edge_pub

    rospy.loginfo("Received an image from ROS bag.")

    # Convert ROS Image message to OpenCV format in a try-catch loop
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logerr(f"Failed to convert image: {e}")
        return

    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    #gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRGB)
    #gray = cv2.cvtColor(cv_image,cv2.COLOR_RGB2GRAY)
    

    blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)
    edges = cv2.Canny(blurred, 50, 150)

    edge_image_msg = bridge.cv2_to_imgmsg(edges, encoding="mono8")

    # Publishing both original image and edge image
    image_pub.publish(msg)
    edge_pub.publish(edge_image_msg)

    rospy.loginfo("Published edge-detected image to RViz.")

# Main function
def main():
    """
    Main function to initialize the ROS node, set up subscribers and publishers,
    and keep the node running.
    """
    global image_pub, edge_pub

    rospy.init_node('edge_detection_subscriber')

    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)

    # Publishers for visualization in RViz
    image_pub = rospy.Publisher('/edge_detection/input_image', Image, queue_size=1)
    edge_pub = rospy.Publisher('/edge_detection/edge_image', Image, queue_size=1)

    rospy.loginfo("Edge Detection Subscriber Node Started.")
    rospy.spin()

if __name__ == "__main__":
    main()
