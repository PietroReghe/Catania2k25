#CODE CHE RILEVA IL COLORE ROSSO
#QUESTO CODE E' UNA PARTE DEL CODE Test3 

# Python code for Red Color detection

from picamera2 import Picamera2
import numpy as np 
import cv2 

# Capturing video through webcam 

while(True):
    # Start a while loop 
    picam2 = Picamera2()
    picam2.start()
    
    print("Press 'q' to quit.")
    
    while(True): 
        
        # Reading the video from the 
        # webcam in image frames 
        imageFrame = picam2.capture_array()

        # Convert the imageFrame in 
        # BGR(RGB color space) to 
        # HSV(hue-saturation-value) 
        # color space 
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 

        # Set range for red color and 
        # define mask 
        red_lower = np.array([120, 87, 111], np.uint8) 
        red_upper = np.array([180, 255, 255], np.uint8) 
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 

        # Morphological Transform, Dilation 
        # for each color and bitwise_and operator 
        # between imageFrame and mask determines 
        # to detect only that particular color 
        kernel = np.ones((5, 5), "uint8") 
        
        # For red color 
        red_mask = cv2.dilate(red_mask, kernel) 
        res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                                mask = red_mask) 
 
        # Creating contour to track red color 
        contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        print(" ") 
        
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            if(area > 300): 
                print("e stato rilevato il colore ROSSO")
                x, y, w, h = cv2.boundingRect(contour) 
                imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2) 
                
                cv2.putText(imageFrame, "Red Colour", (x, y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
                            (0, 0, 255))   
                        

 
                
        # Program Termination 
        cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame) 
        if cv2.waitKey(10) & 0xFF == ord('q'): 
            webcam.release() 
            cv2.destroyAllWindows() 
            break
