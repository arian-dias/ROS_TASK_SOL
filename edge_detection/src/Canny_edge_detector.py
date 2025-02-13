import os
import cv2
import numpy as np

def canny_edge_detector(input_dir=os.path.expanduser("~/catkin_ws/src/edge_detection_ros_challenge/edge_detection_ros/edge_detection/data"), 
                              output_dir=os.path.expanduser("~/catkin_ws/src/edge_detection_ros_challenge/edge_detection_ros/edge_detection/results"), 
                              low_threshold=60, high_threshold=200):
    '''
    This function applies the canny edge algorithm, it converts the image to grayscale and applies gaussian blur.

    Steps:

    - Checking th output directory 
    - We pre-process the image and displaying the green overlay on the image.
    Args:

    - input_dir: Input directory containing the data.
    - output_dir: Saving the output data
    - low and high threshold: Modifying the threshold for the canny edge algorithm.
    
    '''

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Process each image in the directory
    for filename in os.listdir(input_dir):
        if filename.lower().endswith(('png', 'jpg', 'jpeg')):
            # Read the image
            img_path = os.path.join(input_dir, filename)
            image = cv2.imread(img_path)
            
            if image is None:
                print(f"Error loading image: {filename}")
                continue
            
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)
            edges = cv2.Canny(blurred, low_threshold, high_threshold)
            
            # Create a green overlay for edges
            green_edges = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
            green_edges[:, :, 1] = edges  # Set edges to green channel
            
            # Blend the original image and the edge-detected image
            result = cv2.addWeighted(image, 0.8, green_edges, 1.0, 0)
            
            # Save the output image
            output_path = os.path.join(output_dir, filename)
            cv2.imwrite(output_path, result)
            print(f"Processed: {filename}")

if __name__ == "__main__":
    canny_edge_detector()
