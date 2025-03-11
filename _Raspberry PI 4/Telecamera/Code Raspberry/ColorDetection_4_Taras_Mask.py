from picamera2 import Picamera2
import numpy as np
import cv2

# Inizializza la fotocamera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

while True:
    frame = picam2.capture_array()
    frame = cv2.flip(frame, 1)  # Ribalta l'immagine orizzontalmente
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    height, width, _ = frame.shape

    # Coordinate del centro
    cx = width // 2
    cy = height // 2

    # Definizione della regione di interesse (ROI)
    roi_size = 50
    roi = hsv_frame[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]

    # Calcolo del colore medio nella ROI
    mean_color = cv2.mean(roi)[:3]
    hue_value = int(mean_color[0])
    sat_value = int(mean_color[1])
    value_value = int(mean_color[2])

    # Determina il colore predominante
    color = "Undefined"
    if value_value < 50:
        color = "BLACK"
    elif sat_value < 50:
        color = "WHITE"
    else:
        if hue_value < 5 or hue_value >= 167:
            color = "RED"
        elif hue_value < 22:
            color = "ORANGE"
        elif hue_value < 33:
            color = "YELLOW"
        elif hue_value < 78:
            color = "GREEN"
        elif hue_value < 131:
            color = "BLUE"
        elif hue_value < 167:
            color = "VIOLET"
    
    print(f"Mean HSV: {hue_value}, {sat_value}, {value_value}, Detected Color: {color}")

    # Mostra il colore rilevato sullo schermo
    cv2.putText(frame, color, (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4)
    cv2.rectangle(frame, (cx - roi_size, cy - roi_size), (cx + roi_size, cy + roi_size), (0, 255, 0), 2)

    # Visualizza il frame
    cv2.imshow("Color Detection", frame)
    
    # Esci premendo 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()