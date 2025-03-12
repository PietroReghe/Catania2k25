# QUESTO CODE MOSTRA SOLO COLORE IMPOSTATO
# COLORI NON DIMPOSTATI SONO NERO
# RASPBERRY

from picamera2 import Picamera2
import cv2
import numpy as np

def main():
    picam2 = Picamera2()
    picam2.start()
    
    print("Press 'q' to quit.")
    
    # Define the color range for detection (e.g., red color)
    lower_bound = np.array([0, 120, 70])  # Lower bound of HSV
    upper_bound = np.array([10, 255, 255])  # Upper bound of HSV

    while True:
        # Capture a frame
        frame = picam2.capture_array()
        
        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create a mask for the specific color range
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # Apply the mask to the original frame
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Display the original frame and the result
        cv2.imshow("Original", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("Detected Color", result)
        
        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Clean up
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
