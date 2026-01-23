#!/usr/bin/env python3
'''
# Team ID:          1002
# Theme:            Krishi Drone
# Author List:      Sathyagith, Rudra, Shavya, Nagendra
# Purpose:          Real-time infected plant detection for bottom-half blocks
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json



def extract_ground(img, min_area=4000):
    '''
    Purpose:
    ---
    Crop the input image to retain only the plant tray region by removing surrounding soil.

    Input Arguments:
    ---
    `img` :  [ numpy.ndarray ]
        Input BGR image
    `min_area` :  [ int ]
        Minimum contour area to be considered as ground

    Returns:
    ---
    `cropped_img` :  [ numpy.ndarray ]
        Cropped image containing plant tray
    `x` :  [ int ]
        X-coordinate of top-left corner of cropped area
    `y` :  [ int ]
        Y-coordinate of top-left corner of cropped area
    `w` :  [ int ]
        Width of cropped area
    `h` :  [ int ]
        Height of cropped area

    Example call:
    ---
    cropped, x, y, w, h = extract_ground(image)
    '''
    # Convert image to HSV for robust color segmentation
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([179, 50, 255])
    mask = cv2.inRange(img_hsv, lower_white, upper_white)

    # Morphological operations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=5)
    mask = cv2.dilate(mask, kernel, iterations=2)
    white_mask = np.ones_like(mask, dtype=np.uint8) * 255
    border_width = 20

 
    white_mask[:border_width, :] = 0           # Top border
    white_mask[-border_width:, :] = 0          # Bottom border
    white_mask[:, :border_width] = 0           # Left border
    white_mask[:, -border_width:] = 0          # Right border
    
    mask = cv2.bitwise_and(white_mask, mask)
    cv2.imwrite("mask.jpg",mask)

    # Find contours of ground/white regions
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]

    # If no large contours found, return original image
    if not filtered:
        return img, 0, 0, img.shape[1], img.shape[0]

    # Bounding rectangle around all contours
    x, y, w, h = cv2.boundingRect(np.concatenate(filtered))
    return img[y:y+h, x:x+w]

def sort_corners(pts):
    rect = np.zeros((4,2), dtype=np.float32)
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1).reshape(-1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

def detect_aruco_markers(image):
    dicts_to_try = [
        cv2.aruco.DICT_4X4_50, cv2.aruco.DICT_4X4_100,
        cv2.aruco.DICT_5X5_50, cv2.aruco.DICT_5X5_100,
        cv2.aruco.DICT_6X6_250, cv2.aruco.DICT_7X7_1000
    ]
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    for dict_id in dicts_to_try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)
        if ids is not None and len(ids) >= 4:
            return corners, ids
    return None, None

def warp_image(image, src_pts):
    widthA = np.linalg.norm(src_pts[2]-src_pts[3])
    widthB = np.linalg.norm(src_pts[1]-src_pts[0])
    w = int(max(widthA, widthB))
    heightA = np.linalg.norm(src_pts[1]-src_pts[2])
    heightB = np.linalg.norm(src_pts[0]-src_pts[3])
    h = int(max(heightA, heightB))
    dst_pts = np.array([[0,0],[w-1,0],[w-1,h-1],[0,h-1]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    return cv2.warpPerspective(image, M, (w,h))

def remove_white_border(img, threshold=220):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = gray < threshold
    coords = np.argwhere(mask)
    if coords.size == 0:
        return img
    y0, x0 = coords.min(axis=0)
    y1, x1 = coords.max(axis=0)+1
    
    return img[y0:y1, x0:x1]

def divide_bottom_blocks(warped):
    h = warped.shape[0]
    bottom_half = warped[h//2: , :]
    w = bottom_half.shape[1]
    block_width = w // 2

    block1 = bottom_half[:, :block_width]
    block2 = bottom_half[:, block_width:]
    
    cv2.imwrite('block111.jpg',block1)
    cv2.imwrite('block112.jpg',block2)

    block1 = extract_ground(block1)
    block2 = extract_ground(block2)

    return block1, block2

def detect_infected_plant(block, block_id="P1"):
    h, w = block.shape[:2]
    cell_w = w // 2
    cell_h = h // 3
    infected_ratios = {}
    plant_labels = ["A", "B", "C", "D", "E", "F"]
    lower_infect = np.array([10,80,80])
    upper_infect = np.array([35,255,255])
    idx = 0
    for col in range(2):
        for row in range(3):
            x1, x2 = col*cell_w, (col+1)*cell_w
            y1, y2 = row*cell_h, (row+1)*cell_h
            plant = block[y1:y2, x1:x2]
            hsv = cv2.cvtColor(plant, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_infect, upper_infect)
            if block_id=='P2':
            	cv2.imwrite(f"Plant{row}{col}.jpg", plant)
            	cv2.imwrite(f"Block2.jpg",block)
            else:
            	cv2.imwrite(f"Block1.jpg",block)
          

            ratio = np.sum(mask>0)/float(plant.shape[0]*plant.shape[1]+1e-6)
            infected_ratios[plant_labels[idx]] = ratio
            idx += 1
    return f"{block_id}{max(infected_ratios,key=infected_ratios.get)}"



class InfectedPlantsNode(Node):
    def __init__(self):
        super().__init__('infected_plants_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(String, '/detected_plants', 10)
        self.get_logger().info("Infected Plants Node Started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        corners, ids = detect_aruco_markers(frame)
        if ids is None:
            #self.get_logger().info("No ArUco markers detected")
            return

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        avg_pts = np.array([c[0].mean(axis=0) for c in corners], dtype=np.float32)
        warped = warp_image(frame, sort_corners(avg_pts))
        warped = remove_white_border(warped)
        
        cv2.imwrite('wwarrped.jpg',warped)

        block1, block2 = divide_bottom_blocks(warped)
        infected1 = detect_infected_plant(block1, "P1")
        infected2 = detect_infected_plant(block2, "P2")

        result = [infected1, infected2]
        msg_out = String()
        msg_out.data = json.dumps(result)
        self.publisher_.publish(msg_out)
        self.get_logger().info(f"Detected infected plants: {result}")

        # Optional visualization
        cv2.imshow("Original", frame)
        cv2.imshow("Block 1", block1)
        cv2.imshow("Block 2", block2)
        cv2.waitKey(1)

# -------------------- Main --------------------

def main(args=None):
    rclpy.init(args=args)
    node = InfectedPlantsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
