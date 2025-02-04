import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

while True:
    _, frame1 = cap.read()
    frame2 = cv2.flip(frame1, 1)
    frame = cv2.convertScaleAbs(frame2, 1, 1)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    height, width, _ = frame.shape

    # Center coordinates
    cx = int(width / 2)
    cy = int(height / 2)

    # Define a 100x100 region around the center
    roi_size = 50  # Half-size of the ROI
    roi = hsv_frame[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]

    # Calculate the mean color in the ROI
    mean_color = cv2.mean(roi)[:3]  # Ignore alpha channel
    hue_value = int(mean_color[0])
    sat_value = int(mean_color[1])
    value_value = int(mean_color[2])

    # Determine the predominant color
    color = "Undefined"
    if value_value < 50:
        color = "BLACK"
    elif sat_value < 50:
        color = "WHITE"
    else:
     if hue_value < 5:
        color = "RED"
     elif hue_value < 22:
        color = "ORANGE"
     elif hue_value < 33:
        color= "YELLOW"
     elif hue_value < 78:
        color = " GREEN"
     elif hue_value < 131:
        color = "BLUE"
     elif hue_value < 167:
        color = "VIOLET"
     else:
        color = "RED"

    print(f"Mean HSV: {hue_value}, {sat_value}, {value_value}")

    # Display the ROI and the determined color
    pixel_center_bgr = frame[cy, cx]
    b, g, r = int(pixel_center_bgr[0]), int(pixel_center_bgr[1]), int(pixel_center_bgr[2])
    cv2.putText(frame, color, (20, 70), 0, 2, (0, 255, 0), 4)
    cv2.rectangle(frame, (cx - roi_size, cy - roi_size), (cx + roi_size, cy + roi_size), (0, 255, 0), 2)

    # Show the frame
    cv2.imshow("Frame", frame)
    if cv2.waitKey(10) & 0xFF == ord('q'): 
       cap.release()
       cv2.destroyAllWindows()
       break



