#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo, PointCloud
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge

bridge = CvBridge()

# Publishers
image_pub = None
edge_pub = None
pointcloud_pub = None  
camera_info = None


def image_callback(msg):
    global image_pub, edge_pub

    rospy.loginfo("Received RGB image.")

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logerr("Failed to convert RGB image: %s", e)
        return

    # Converting the image to grayscale 
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # blurring the graysca=led image to reduce noise 
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)

    # Detecting edges using Canny-edge detecting algorithm from opencv
    edges = cv2.Canny(blurred, 50, 150)

    # Converting the edges back to a ROS Image message
    edge_image_msg = bridge.cv2_to_imgmsg(edges, encoding="mono8")

    # Publishing the original image and the edge image
    image_pub.publish(msg)
    edge_pub.publish(edge_image_msg)

    rospy.loginfo("Published edge-detected image.")


def depth_callback(msg):
    global camera_info, pointcloud_pub

    if camera_info is None:
        rospy.logwarn("Camera info not received yet.")
        return

    rospy.loginfo("Received depth image.")

    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception as e:
        rospy.logerr("Failed to convert depth image: %s", e)
        return

    # Get camera intrinsics parameters from camera_info
    fx, fy = camera_info.K[0], camera_info.K[4]
    cx, cy = camera_info.K[2], camera_info.K[5]

    # For Canny, convert depth image to 8-bit 
    edges = cv2.Canny(depth_image.astype(np.uint8), 50, 150)

    # Create a PointCloud message
    cloud = PointCloud()
    cloud.header.stamp = rospy.Time.now()
    # Using the camera frame since we're not transforming the points.
    cloud.header.frame_id = "camera_color_optical_frame"

    points = []

    # Loop through all pixels in the edge image
    for v in range(edges.shape[0]):
        for u in range(edges.shape[1]):
            if edges[v, u] > 0:  # If this pixel is an edge
                # Get the depth value in meters (assuming depth_image is in millimeters)
                z = depth_image[v, u] / 1000.0
                if z <= 0:
                    continue

                # Compute the x and y coordinates in the camera frame using the pinhole camera model
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                # Create a Point32 and add it to the list
                point = Point32(x=x, y=y, z=z)
                points.append(point)

    cloud.points = points

    rospy.loginfo("Generated %d 3D points.", len(points))
    pointcloud_pub.publish(cloud)


def camera_info_callback(msg):
    global camera_info
    camera_info = msg
    rospy.loginfo("Camera info received.")


#!/usr/bin/env python3
"""
Edge Detection and Point Cloud Generation Node

This ROS node subscribes to:
  - A raw RGB image topic
  - A raw depth image topic
  - A camera info topic

It processes the images to:
  - Detect edges in the RGB image (using Canny edge detection)
  - Detect edges in the depth image and generate a 3D point cloud based on these edges

It publishes:
  - The original RGB image (for reference)
  - The edge-detected image
  - A PointCloud message containing 3D points corresponding to edge pixels
"""

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo, PointCloud
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge

# Create an instance of CvBridge to convert between ROS and OpenCV images
bridge = CvBridge()

# Global publishers (will be initialized in main)
image_pub = None       # Publisher for the original image
edge_pub = None        # Publisher for the edge-detected image
pointcloud_pub = None  # Publisher for the 3D point cloud

# Global variable to store camera calibration info
camera_info = None


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

    rospy.loginfo("Received RGB image.")

    try:
        # Convert the ROS image to a BGR OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logerr("Failed to convert RGB image: %s", e)
        return

    \
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)
    edges = cv2.Canny(blurred, 50, 150)
    
    edge_image_msg = bridge.cv2_to_imgmsg(edges, encoding="mono8")
    
    # Publish both the original image and the edge-detected image
    image_pub.publish(msg)
    edge_pub.publish(edge_image_msg)
    
    rospy.loginfo("Published edge-detected image.")


def depth_callback(msg):
    """
    Callback for processing the depth image.
    
    Steps:
      - Ensure camera calibration data is available.
      - Convert the ROS depth image to an OpenCV image.
      - Detect edges in the depth image.
      - For each edge pixel, compute the corresponding 3D coordinates using the camera intrinsics.
      - Publish a PointCloud containing all the computed 3D points.
    """
    global camera_info, pointcloud_pub

    if camera_info is None:
        rospy.logwarn("Camera info not received yet.")
        return

    rospy.loginfo("Received depth image.")

    try:
        # Convert the ROS depth image to an OpenCV image (keeping its original encoding) in a try-catch loop.
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception as e:
        rospy.logerr("Failed to convert depth image: %s", e)
        return

    # Retrieving intrinsic parameters from the camera info
    fx = camera_info.K[0]  # Focal length in x
    fy = camera_info.K[4]  # Focal length in y
    cx = camera_info.K[2]  # Optical center x
    cy = camera_info.K[5]  # Optical center y

    # Convert the depth image to 8-bit and perform Canny edge detection
    edges = cv2.Canny(depth_image.astype(np.uint8), 50, 150)

    # Initializing a new PointCloud message
    cloud = PointCloud()
    cloud.header.stamp = rospy.Time.now()
    cloud.header.frame_id = "camera_color_optical_frame"  # Frame of reference for the points

    points = []
    
    # Loop over each pixel in the edge-detected image
    for v in range(edges.shape[0]):
        for u in range(edges.shape[1]):
            if edges[v, u] > 0:  # If this pixel is an edge
                # Convert the depth from millimeters to meters
                z = depth_image[v, u] / 1000.0
                if z <= 0:
                    continue

                # Using the pinhole camera model formulae to compute the 3D coordinates
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                # Create a 3D point and add it to the list
                point = Point32(x=x, y=y, z=z)
                points.append(point)

    cloud.points = points

    rospy.loginfo("Generated %d 3D points.", len(points))
    pointcloud_pub.publish(cloud)


def camera_info_callback(msg):
    """
    Callback to receive and store the camera's intrinsic calibration parameters.
    """
    global camera_info
    camera_info = msg
    rospy.loginfo("Camera info received.")


def main():
    """
    Main function to initialize the ROS node, set up subscribers and publishers,
    and keep the node running.
    """
    global image_pub, edge_pub, pointcloud_pub

    rospy.init_node('edge_detection_pointcloud_node')

    # Subscribe to the necessary topics
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_callback)
    rospy.Subscriber('/camera/depth/camera_info', CameraInfo, camera_info_callback)

    # Set up publishers for the processed images and point cloud
    image_pub = rospy.Publisher('/edge_detection/input_image', Image, queue_size=1)
    edge_pub = rospy.Publisher('/edge_detection/edge_image', Image, queue_size=1)
    pointcloud_pub = rospy.Publisher('edge_points', PointCloud, queue_size=1)

    rospy.loginfo("Edge Detection PointCloud Node Started.")
    
    # Keep the node running until shutdown
    rospy.spin()


if __name__ == "__main__":
    main()


def main():
    """
    Main function to initialize the ROS node, set up subscribers and publishers,
    and keep the node running.
    """
    global image_pub, edge_pub, pointcloud_pub

    rospy.init_node('edge_detection_pointcloud_node')

    # Subscribers
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_callback)
    rospy.Subscriber('/camera/depth/camera_info', CameraInfo, camera_info_callback)

    # Publishers
    image_pub = rospy.Publisher('/edge_detection/input_image', Image, queue_size=1)
    edge_pub = rospy.Publisher('/edge_detection/edge_image', Image, queue_size=1)
    # Publisher for sensor_msgs/PointCloud on the topic 'edge_points'
    pointcloud_pub = rospy.Publisher('edge_points', PointCloud, queue_size=1)

    rospy.loginfo("Edge Detection PointCloud Node Started.")
    rospy.spin()


if __name__ == "__main__":
    main()
