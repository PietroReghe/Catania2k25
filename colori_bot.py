import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

while True:
    _, frame1 = cap.read()
    frame2 = cv2.flip(frame1, 1)
    frame = cv2.convertScaleAbs(frame2, 1, 1)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    height, width, _ = frame.shape
    
    #center
    cx = int (width /2)
    cy = int (height / 2)
    
    #pick pixel value
    pixel_center = hsv_frame[cy, cx]
    hue_value = pixel_center[0]
    sat_value = pixel_center[1]
    value_value = pixel_center[2]
    
    #define colors
    thing = "Undefined"
    
    if value_value < 50:
        thing = "VOID"
    elif sat_value < 30:
        thing = "PARETE"
    else:
        if hue_value > 130 or hue_value < 5:
            if sat_value > 160:
                thing = "MACCHINA R"
            else:
                thing = "OSTACOLO"
        elif hue_value < 45:
            thing = "MACCHINA Y"
        elif hue_value < 90:
            thing = "MACCHINA G"
        else:
            thing = "MACCHINA B"
    
    pixel_center_bgr = frame[cy, cx]
    b, g, r =int(pixel_center_bgr[0]), int(pixel_center_bgr[1]), int(pixel_center_bgr[2])
    cv2.putText(frame, thing, (20, 70), 0, 2, (0, 255, 0), 3) #where, what, position, font, width, size, colour, width
    cv2.circle(frame, (cx, cy), 0, (0, 255, 0), 12) #where, position, diameter, colour, width
    
    #end of loop
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)
    if key == 27:
       break
   
cap.release()
cv2.destroyAllWindows()
