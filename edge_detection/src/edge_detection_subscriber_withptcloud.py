#!/usr/bin/env python3
"""
Edge Detection and 3D Point Cloud Publisher Node

This node subscribes to:
  - A raw RGB image topic
  - A raw depth image topic
  - A camera info topic
  - A pre-existing point cloud topic

It performs the following tasks:
  - Converts the RGB image to grayscale, blurs it, and applies Canny edge detection.
  - Processes the depth image to detect edges and computes 3D points from edge pixels using camera intrinsics.
  - Publishes the original RGB image and the edge-detected image.
  - Publishes a new PointCloud2 message created from the 3D points computed from the depth image.
  - Optionally republishes the pre-existing point cloud.
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import struct

# Create an instance of CvBridge for image conversion between ROS and OpenCV.
bridge = CvBridge()

# Global publisher variables (will be initialized in the main function).
image_pub = None               # Publishes the original RGB image.
edge_pub = None                # Publishes the edge-detected image.
pointcloud_pub = None          # Publishes the generated 3D point cloud.
preexisting_pointcloud_pub = None  # Publishes the pre-existing point cloud.

# Global variable to store the camera's intrinsic parameters.
camera_info = None


def image_callback(msg):
    """
    Callback for processing the RGB image.
    
    Converts the incoming ROS Image message to an OpenCV BGR image,
    then converts it to grayscale, applies Gaussian blur, and uses
    the Canny algorithm to detect edges. Finally, it publishes both
    the original and the edge-detected images.
    """
    global image_pub, edge_pub

    rospy.loginfo("Received RGB image.")

    try:
        # Convert the ROS image to an OpenCV image (BGR format).
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logerr(f"Failed to convert RGB image: {e}")
        return

    # Convert the image to grayscale.
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # Apply Gaussian blur to reduce noise.
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)
    # Apply Canny edge detection.
    edges = cv2.Canny(blurred, 50, 150)

    # Convert the edge image back to a ROS Image message.
    edge_image_msg = bridge.cv2_to_imgmsg(edges, encoding="mono8")

    # Publish both the original image and the edge-detected image.
    image_pub.publish(msg)
    edge_pub.publish(edge_image_msg)

    rospy.loginfo("Published edge-detected image.")


def depth_callback(msg):
    """
    Callback for processing the depth image.
    
    Converts the incoming depth image to an OpenCV image and uses
    the camera intrinsics to convert edge pixels into 3D points. It
    then publishes these points as a PointCloud2 message.
    """
    global camera_info, pointcloud_pub

    if camera_info is None:
        rospy.logwarn("Camera info not received yet.")
        return

    rospy.loginfo("Received depth image.")

    try:
        # Convert the ROS depth image to an OpenCV image.
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        rospy.loginfo(f"Depth image size: {depth_image.shape}")
    except Exception as e:
        rospy.logerr(f"Failed to convert depth image: {e}")
        return

    # Extract camera intrinsic parameters.
    fx, fy = camera_info.K[0], camera_info.K[4]
    cx, cy = camera_info.K[2], camera_info.K[5]

    # Detect edges in the depth image by converting it to 8-bit.
    edges = cv2.Canny(depth_image.astype(np.uint8), 50, 150)
    points = []

    # Loop through each pixel in the edge image.
    for v in range(edges.shape[0]):
        for u in range(edges.shape[1]):
            if edges[v, u] > 0:  # If this pixel is an edge.
                # Convert the depth value from millimeters to meters.
                z = depth_image[v, u] / 1000.0
                if z > 0:  # Only consider valid depth values.
                    # Compute the 3D coordinates using the pinhole camera model.
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append((x, y, z))

    rospy.loginfo(f"Generated {len(points)} 3D points.")

    # Publish the computed 3D points as a PointCloud2 message.
    publish_point_cloud(points, msg.header)


def camera_info_callback(msg):
    """
    Callback to receive and store camera calibration information.
    
    This callback updates the global camera_info variable with the latest
    intrinsic parameters, which are later used for 3D point calculation.
    """
    global camera_info
    camera_info = msg
    rospy.loginfo("Camera info received.")


def preexisting_pointcloud_callback(msg):
    """
    Callback for processing an already existing point cloud.
    
    This function simply republishes the received PointCloud2 message
    to another topic.
    """
    global preexisting_pointcloud_pub

    rospy.loginfo("Received pre-existing point cloud.")
    preexisting_pointcloud_pub.publish(msg)


def publish_point_cloud(points, header):
    """
    Converts a list of 3D points into a PointCloud2 message and publishes it.
    
    Args:
        points (list of tuple): A list where each tuple contains (x, y, z) coordinates.
        header (std_msgs/Header): The header from the original depth image message,
                                  used to timestamp and frame the point cloud.
    """
    global pointcloud_pub

    rospy.loginfo(f"Publishing {len(points)} points as PointCloud2.")

    # Define the fields for the point cloud (x, y, and z coordinates).
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]

    # Pack the 3D point data into binary format.
    pointcloud_data = []
    for p in points:
        pointcloud_data.append(struct.pack('fff', *p))

    # Create and populate the PointCloud2 message.
    pointcloud_msg = PointCloud2()
    pointcloud_msg.header = header
    pointcloud_msg.height = 1
    pointcloud_msg.width = len(points)
    pointcloud_msg.fields = fields
    pointcloud_msg.is_bigendian = False
    pointcloud_msg.point_step = 12  # Each point consists of three float32 values.
    pointcloud_msg.row_step = pointcloud_msg.point_step * len(points)
    pointcloud_msg.is_dense = True
    pointcloud_msg.data = b''.join(pointcloud_data)

    # Publish the point cloud message.
    pointcloud_pub.publish(pointcloud_msg)


def main():
    """
    Main function to initialize the ROS node, set up subscribers and publishers,
    and keep the node running.
    """
    global image_pub, edge_pub, pointcloud_pub, preexisting_pointcloud_pub

    rospy.init_node('edge_detection_subscriber')

    # Subscribers for RGB image, depth image, camera info, and pre-existing point cloud.
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_callback)
    rospy.Subscriber('/camera/depth/camera_info', CameraInfo, camera_info_callback)
    rospy.Subscriber('/camera/depth/points', PointCloud2, preexisting_pointcloud_callback)

    # Publishers for the processed images and point clouds.
    image_pub = rospy.Publisher('/edge_detection/input_image', Image, queue_size=1)
    edge_pub = rospy.Publisher('/edge_detection/edge_image', Image, queue_size=1)
    pointcloud_pub = rospy.Publisher('/edge_detection/edge_points', PointCloud2, queue_size=1)
    preexisting_pointcloud_pub = rospy.Publisher('/edge_detection/preexisting_points', PointCloud2, queue_size=1)

    rospy.loginfo("Edge Detection Subscriber Node Started.")
    rospy.spin()


if __name__ == "__main__":
    main()
