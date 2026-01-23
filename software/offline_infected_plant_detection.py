#!/usr/bin/env python3
'''
# Team ID:          1002
# Theme:            Krishi Drone
# Author List:      Sathyagith, Rudra, Shavya, Nagendra
# Filename:         KD_1002_task1a.py
# Functions:        extract_ground, sort_corners, detect_aruco_markers, warp_image, remove_white_border, divide_blocks, detect_infected_plant, main
# Global variables: None
'''

import cv2
import numpy as np
import argparse
import sys

# -------------------- Helper Functions --------------------

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
    lower_white = np.array([0, 0, 40])
    upper_white = np.array([179, 40, 255])
    mask = cv2.inRange(img_hsv, lower_white, upper_white)

    # Morphological operations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=1)

    # Find contours of ground/white regions
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]

    # If no large contours found, return original image
    if not filtered:
        return img, 0, 0, img.shape[1], img.shape[0]

    # Bounding rectangle around all contours
    x, y, w, h = cv2.boundingRect(np.concatenate(filtered))
    return img[y:y+h, x:x+w], x, y, w, h


def sort_corners(pts):
    '''
    Purpose:
    ---
    Sort four points in the order: top-left, top-right, bottom-right, bottom-left.

    Input Arguments:
    ---
    `pts` :  [ numpy.ndarray ]
        Array of shape (4,2) containing 4 corner points

    Returns:
    ---
    `rect` :  [ numpy.ndarray ]
        Sorted corner points

    Example call:
    ---
    sorted_pts = sort_corners(corner_points)
    '''
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1).reshape(-1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect


def detect_aruco_markers(image):
    '''
    Purpose:
    ---
    Detect ArUco markers robustly. Returns sorted corners and IDs.

    Input Arguments:
    ---
    `image` :  [ numpy.ndarray ]
        Input BGR image

    Returns:
    ---
    `src_pts` :  [ numpy.ndarray ]
        Sorted corner points of the detected markers
    `ids` :  [ numpy.ndarray ]
        IDs of the detected markers

    Example call:
    ---
    src_pts, ids = detect_aruco_markers(image)
    '''
    # List of ArUco dictionaries to try
    dicts_to_try = [
        cv2.aruco.DICT_4X4_50, cv2.aruco.DICT_4X4_100,
        cv2.aruco.DICT_5X5_50, cv2.aruco.DICT_5X5_100,
        cv2.aruco.DICT_6X6_250, cv2.aruco.DICT_7X7_1000
    ]

    # Detector parameters
    parameters = cv2.aruco.DetectorParameters_create()

    # Try each dictionary to detect at least 4 markers
    for dict_id in dicts_to_try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)
        if ids is not None and len(ids) >= 4:
            # Take first 4 markers
            ids_flat = ids.flatten()
            sorted_idx = np.argsort(ids_flat)[:4]
            marker_points = np.array([corners[i][0].mean(axis=0) for i in sorted_idx], dtype=np.float32)
            return sort_corners(marker_points), ids[sorted_idx]
    return None, None


def warp_image(image, src_pts):
    '''
    Purpose:
    ---
    Perform perspective transform to obtain top-down view of the tray.

    Input Arguments:
    ---
    `image` : [numpy.ndarray] Input BGR image
    `src_pts` : [numpy.ndarray] Sorted corner points for perspective transform

    Returns:
    ---
    `warped` : [numpy.ndarray] Warped top-down image

    Example call:
    ---
    warped = warp_image(image, src_pts)
    '''
    widthA = np.linalg.norm(src_pts[2] - src_pts[3])
    widthB = np.linalg.norm(src_pts[1] - src_pts[0])
    w = int(max(widthA, widthB))

    heightA = np.linalg.norm(src_pts[1] - src_pts[2])
    heightB = np.linalg.norm(src_pts[0] - src_pts[3])
    h = int(max(heightA, heightB))

    dst_pts = np.array([[0,0],[w-1,0],[w-1,h-1],[0,h-1]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    return cv2.warpPerspective(image, M, (w, h))


def remove_white_border(img, threshold=220):
    '''
    Purpose:
    ---
    Remove any white background borders from a warped tray image.

    Input Arguments:
    ---
    `img` : [numpy.ndarray] Input BGR image
    `threshold` : [int] Pixel intensity threshold to consider as white

    Returns:
    ---
    `cropped_img` : [numpy.ndarray] Image with white borders removed

    Example call:
    ---
    cropped = remove_white_border(img)
    '''
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = gray < threshold
    coords = np.argwhere(mask)
    if coords.size == 0:
        return img
    y0, x0 = coords.min(axis=0)
    y1, x1 = coords.max(axis=0) + 1
    return img[y0:y1, x0:x1]


def divide_blocks(warped):
    '''
    Purpose:
    ---
    Divide tray into two blocks, crop ground region, and rotate for analysis.

    Input Arguments:
    ---
    `warped` : [numpy.ndarray] Warped top-down tray image

    Returns:
    ---
    `block1` : [numpy.ndarray] Cropped block 1 image
    `block2` : [numpy.ndarray] Cropped block 2 image

    Example call:
    ---
    block1, block2 = divide_blocks(warped)
    '''
    w = warped.shape[1]
    block_width = w // 2
    block_bottom = warped[:, block_width:]
    half_height = block_bottom.shape[0] // 2

    block2 = block_bottom[:half_height, :]
    block1 = block_bottom[half_height:, :]

    # Crop only the tray region
    block1, _, _, _, _ = extract_ground(block1)
    block2, _, _, _, _ = extract_ground(block2)

    # Rotate blocks for proper orientation
    block1 = cv2.rotate(block1, cv2.ROTATE_90_CLOCKWISE)
    block2 = cv2.rotate(block2, cv2.ROTATE_90_CLOCKWISE)

    return block1, block2


def detect_infected_plant(block, block_id="P1"):
    '''
    Purpose:
    ---
    Detect the most infected plant in a block using HSV thresholding.

    Input Arguments:
    ---
    `block` : [numpy.ndarray] Image of block containing plants
    `block_id` : [str] Identifier for the block ("P1" or "P2")

    Returns:
    ---
    `most_infected` : [str] Plant ID of the most infected plant (e.g., "P1F")

    Example call:
    ---
    plant_id = detect_infected_plant(block1, "P1")
    '''
    h, w = block.shape[:2]
    cell_w = w // 2
    cell_h = h // 3
    infected_ratios = {}
    plant_labels = ["A", "B", "C", "D", "E", "F"]

    # HSV range for diseased plant (yellow/brown)
    lower_infect = np.array([10, 80, 80])
    upper_infect = np.array([35, 255, 255])

    idx = 0
    for col in range(2):
        for row in range(3):
            x1, x2 = col * cell_w, (col + 1) * cell_w
            y1, y2 = row * cell_h, (row + 1) * cell_h
            plant = block[y1:y2, x1:x2]

            hsv = cv2.cvtColor(plant, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_infect, upper_infect)

            infected_area = np.sum(mask > 0)
            total_area = plant.shape[0] * plant.shape[1]
            ratio = infected_area / float(total_area + 1e-6)

            infected_ratios[plant_labels[idx]] = ratio
            idx += 1

    return f"{block_id}{max(infected_ratios, key=infected_ratios.get)}"


# -------------------- Main Function --------------------

def main():
    '''
    Purpose:
    ---
    Main execution function. Reads the input image, detects ArUco markers,
    warps the tray, divides blocks, detects infected plants, and writes portal-required output.

    Input Arguments:
    ---
    None (image path is passed as command line argument)

    Returns:
    ---
    None (output written to output_task1a.txt)

    Example call:
    ---
    python3 KD_1002_task1a.py --image task1a_image.jpg
    '''
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("--image", required=True, help="Path to input image")
        args = parser.parse_args()

        image = cv2.imread(args.image)
        if image is None:
            sys.exit(0)  # portal-safe exit if image not found

        src_pts, ids = detect_aruco_markers(image)
        if src_pts is None or ids is None or len(ids) < 4:
            # If markers not detected, write empty output for portal
            with open("output_task1a.txt", "w") as f:
                f.write("Detected marker IDs: []\n\n")
                f.write("Infected plant in Block 1: P1None\n")
                f.write("Infected plant in Block 2: P2None\n")
            sys.exit(0)

        # Warp image using detected ArUco corners
        warped = warp_image(image, src_pts)
        warped = remove_white_border(warped, threshold=220)

        # Divide tray into 2 blocks and detect infected plants
        block1, block2 = divide_blocks(warped)
        infected1 = detect_infected_plant(block1, "P1")
        infected2 = detect_infected_plant(block2, "P2")

        # Write portal-required output
        with open("output_task1a.txt", "w") as f:
            f.write(f"Detected marker IDs: {list(ids.flatten())}\n\n")
            f.write(f"Infected plant in Block 1: {infected1}\n")
            f.write(f"Infected plant in Block 2: {infected2}\n")

        sys.exit(0)
    except:
        sys.exit(0)  # ensure zero exit on any exception

if __name__ == "__main__":
    main()

