# FUNZIONA SOLO SUL PC

import cv2
import numpy as np

def map_to_defined_colors(hsv_frame):
    """
    Efficiently map the HSV frame to defined colors based on hue, saturation, and value using vectorized operations.
    """
    # Create an output frame initialized to black
    mapped_frame = np.zeros_like(hsv_frame)

    # Extract the HSV channels
    hue = hsv_frame[:, :, 0]
    sat = hsv_frame[:, :, 1]
    val = hsv_frame[:, :, 2]

    # Apply the conditions for each color
    # BLACK
    black_mask = val < 50
    mapped_frame[black_mask] = [0, 0, 0]

    # WHITE (Only very light values with low saturation)
    white_mask = (sat < 30) & (val >= 200)
    mapped_frame[white_mask] = [0, 0, 255]

    # GREY (Low saturation, moderate value)
    grey_mask = (sat < 30) & (val >= 50) & (val < 200)
    mapped_frame[grey_mask] = [0, 0, 128]

    # RED
    red_mask = ((hue > 160) | (hue <= 10)) & (sat >= 30) & (val >= 50)
    mapped_frame[red_mask] = [0, 255, 255]

    # PINK
    pink_mask = ((hue > 160) | (hue <= 10)) & (sat >= 30) & (sat < 167) & (val >= 50)
    mapped_frame[pink_mask] = [170, 50, 255]

    # ORANGE
    orange_mask = (hue > 10) & (hue <= 22) & (sat >= 30) & (val >= 50)
    mapped_frame[orange_mask] = [15, 255, 255]

    # YELLOW
    yellow_mask = (hue > 22) & (hue <= 35) & (sat >= 30) & (val >= 50)
    mapped_frame[yellow_mask] = [30, 255, 255]

    # GREEN
    green_mask = (hue > 35) & (hue <= 75) & (sat >= 30) & (val >= 50)
    mapped_frame[green_mask] = [60, 255, 255]

    # BLUE
    blue_mask = (hue > 75) & (hue <= 100) & (sat >= 30) & (val >= 50)
    mapped_frame[blue_mask] = [90, 255, 255]

    # PURPLE (Only when saturation is high and hue is in purple range)
    purple_mask = (hue > 100) & (hue <= 135) & (sat >= 100) & (val >= 50)
    mapped_frame[purple_mask] = [135, 255, 255]

    return mapped_frame

def create_motion_mask(prev_frame, curr_frame):
    """
    Create a mask that highlights areas of motion by comparing the previous and current frames.
    """
    # Convert frames to grayscale
    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)

    # Compute absolute difference between current and previous frame
    diff_frame = cv2.absdiff(prev_gray, curr_gray)

    # Threshold the difference to create a binary motion mask
    _, motion_mask = cv2.threshold(diff_frame, 50, 255, cv2.THRESH_BINARY)

    return motion_mask

def reduce_noise(frame):
    """
    Apply Gaussian Blur to reduce noise in static regions.
    """
    return cv2.GaussianBlur(frame, (5, 5), 0)

# Main loop for capturing and processing frames
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Initialize previous frame for motion detection
_, prev_frame = cap.read()

while True:
    _, curr_frame = cap.read()

    # Create motion mask from previous and current frame
    motion_mask = create_motion_mask(prev_frame, curr_frame)
    
    # Convert current frame to HSV
    hsv_frame = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2HSV)

    # Map each pixel to the defined color
    mapped_hsv_frame = map_to_defined_colors(hsv_frame)

    # For static regions (motion_mask == 0), apply noise reduction
    static_regions = (motion_mask == 0)

    # Apply noise reduction (Gaussian Blur) only in static regions
    smoothed_frame = reduce_noise(mapped_hsv_frame)

    # Keep the color from the current mapped frame in static regions
    mapped_hsv_frame[static_regions] = smoothed_frame[static_regions]

    # Convert the final frame back to BGR for display
    smoothed_bgr_frame = cv2.cvtColor(mapped_hsv_frame, cv2.COLOR_HSV2BGR)

    # Mirror the image horizontally
    mirrored_frame = cv2.flip(smoothed_bgr_frame, 1)

    # Show the final mirrored frame
    cv2.imshow("Mirrored Frame with Motion Detection and Noise Reduction", mirrored_frame)

    # Update the previous frame
    prev_frame = curr_frame

    if cv2.waitKey(10) & 0xFF == ord('q'): 
       cap.release()
       cv2.destroyAllWindows()
       break

cap.release()
cv2.destroyAllWindows()