#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import struct

# Initialize CV Bridge
bridge = CvBridge()

# Publishers
image_pub = None
edge_pub = None
pointcloud_pub = None
preexisting_pointcloud_pub = None

# Global variable for camera info
camera_info = None

# Callback for RGB image
def image_callback(msg):
    global image_pub, edge_pub

    rospy.loginfo("Received RGB image.")

    # Convert ROS Image message to OpenCV format
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logerr(f"Failed to convert RGB image: {e}")
        return

    # Convert to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)

    # Apply Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)

    # Convert edges back to ROS Image
    edge_image_msg = bridge.cv2_to_imgmsg(edges, encoding="mono8")

    # Publish both original image and edge image
    image_pub.publish(msg)
    edge_pub.publish(edge_image_msg)

    rospy.loginfo("Published edge-detected image.")

# Callback for Depth image
def depth_callback(msg):
    global camera_info, pointcloud_pub

    if camera_info is None:
        rospy.logwarn("Camera info not received yet.")
        return

    rospy.loginfo("Received depth image.")

    # Convert ROS Image message to OpenCV format
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        rospy.loginfo(f"Depth image size: {depth_image.shape}")
    except Exception as e:
        rospy.logerr(f"Failed to convert depth image: {e}")
        return

    # Extract camera intrinsics
    fx, fy = camera_info.K[0], camera_info.K[4]
    cx, cy = camera_info.K[2], camera_info.K[5]

    # Detect edges in depth image
    edges = cv2.Canny(depth_image.astype(np.uint8), 50, 150)
    points = []

    # Convert edge pixels to 3D points
    for v in range(edges.shape[0]):
        for u in range(edges.shape[1]):
            if edges[v, u] > 0:  # If edge pixel
                z = depth_image[v, u] / 1000.0  # Convert depth to meters
                if z > 0:  # Ignore invalid depth
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append((x, y, z))

    rospy.loginfo(f"Generated {len(points)} 3D points.")

    # Publish the point cloud
    publish_point_cloud(points, msg.header)

# Callback for Camera Info
def camera_info_callback(msg):
    global camera_info
    camera_info = msg
    rospy.loginfo("Camera info received.")

# Callback for Pre-existing Point Cloud
def preexisting_pointcloud_callback(msg):
    global preexisting_pointcloud_pub

    rospy.loginfo("Received pre-existing point cloud.")

    # Republish the pre-existing point cloud (optional)
    preexisting_pointcloud_pub.publish(msg)

# Publish Point Cloud
def publish_point_cloud(points, header):
    global pointcloud_pub

    rospy.loginfo(f"Publishing {len(points)} points as PointCloud2.")

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]

    pointcloud_data = []
    for p in points:
        pointcloud_data.append(struct.pack('fff', *p))

    pointcloud_msg = PointCloud2()
    pointcloud_msg.header = header
    pointcloud_msg.height = 1
    pointcloud_msg.width = len(points)
    pointcloud_msg.fields = fields
    pointcloud_msg.is_bigendian = False
    pointcloud_msg.point_step = 12
    pointcloud_msg.row_step = pointcloud_msg.point_step * len(points)
    pointcloud_msg.is_dense = True
    pointcloud_msg.data = b''.join(pointcloud_data)

    pointcloud_pub.publish(pointcloud_msg)

# Main function
def main():
    global image_pub, edge_pub, pointcloud_pub, preexisting_pointcloud_pub

    rospy.init_node('edge_detection_subscriber')

    # Subscribers
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)  # RGB image
    rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_callback)  # Depth image
    rospy.Subscriber('/camera/depth/camera_info', CameraInfo, camera_info_callback)  # Camera parameters
    rospy.Subscriber('/camera/depth/points', PointCloud2, preexisting_pointcloud_callback)  # Pre-existing point cloud

    # Publishers
    image_pub = rospy.Publisher('/edge_detection/input_image', Image, queue_size=1)
    edge_pub = rospy.Publisher('/edge_detection/edge_image', Image, queue_size=1)
    pointcloud_pub = rospy.Publisher('/edge_detection/edge_points', PointCloud2, queue_size=1)
    preexisting_pointcloud_pub = rospy.Publisher('/edge_detection/preexisting_points', PointCloud2, queue_size=1)

    rospy.loginfo("Edge Detection Subscriber Node Started.")
    rospy.spin()

if __name__ == "__main__":
    main()
