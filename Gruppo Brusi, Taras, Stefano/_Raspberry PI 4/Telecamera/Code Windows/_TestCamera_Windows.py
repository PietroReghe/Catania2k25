# Import library numpy for array
import cv2

# Capturing video through webcam
webcam = cv2.VideoCapture(0)

# Start a while loopcv2.imshow("Multiple Color Detection in Real-TIme", imageFrame) 
while(1):
    _, imageFrame = webcam.read()
    mirrored_frame = cv2.flip(imageFrame, 1)
    cv2.imshow("Multiple Color Detection in Real-TIme", mirrored_frame) 
    if cv2.waitKey(10) & 0xFF == ord('q'): 
        webcam.release() 
        cv2.destroyAllWindows() 
        break