# Python code for Red Color detection

from picamera2 import Picamera2
import numpy as np 
import cv2 
import serial

def read_camera_pi():
    cap = cv2.VideoCapture(0)  # Inizializza la videocamera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    picam2 = Picamera2()
    picam2.start()

    while True:
        ret, frame1 = cap.read()
        if not ret:
            print("Errore nel leggere il frame dalla videocamera")
            break

        frame2 = cv2.flip(frame1, 1)
        frame = cv2.convertScaleAbs(frame2, 1, 1)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        height, width, _ = frame.shape

        # Center coordinates
        cx = width // 2
        cy = height // 2

        # Define a 100x100 region around the center
        roi_size = 50  # Half-size of the ROI

        # Controlla che la ROI non esca dai limiti dell'immagine
        if cy - roi_size < 0 or cy + roi_size >= height or cx - roi_size < 0 or cx + roi_size >= width:
            print("ROI fuori dai limiti dell'immagine")
            continue

        roi = hsv_frame[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]

        # Calcola il colore medio nella ROI
        mean_color = cv2.mean(roi)[:3]  # Ignora il canale alpha
        hue_value = int(mean_color[0])
        sat_value = int(mean_color[1])
        value_value = int(mean_color[2])

        # Determina il colore predominante
        color = "Undefined"
        car_color = None  # Memorizza il colore del veicolo

        if value_value < 50:
            color = "BLACK"
        elif sat_value < 50:
            color = "WHITE"
        else:
            if hue_value < 5 or hue_value >= 167:
                color = "RED"
                car_color = "REDCAR"
            elif hue_value < 22:
                color = "ORANGE"
            elif hue_value < 33:
                color = "YELLOW"
                car_color = "YELLOWCAR"
            elif hue_value < 78:
                color = "GREEN"
                car_color = "GREENCAR"
            elif hue_value < 131:
                color = "BLUE"
                car_color = "BLUECAR"
            elif hue_value < 167:
                color = "VIOLET"

        print(f"Mean HSV: {hue_value}, {sat_value}, {value_value}, Detected Color: {color}")

        # Mostra il risultato sullo schermo
        cv2.putText(frame, color, (20, 70), 0, 2, (0, 255, 0), 4)
        cv2.rectangle(frame, (cx - roi_size, cy - roi_size), (cx + roi_size, cy + roi_size), (0, 255, 0), 2)

        cv2.imshow("Color Detection", frame)

        # Se viene premuto 'q', esce dal loop
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    # Rilascia le risorse
    cap.release()
    cv2.destroyAllWindows()

    return car_color  # Restituisce il colore dell'auto se rilevato


# Esegui la funzione
if __name__ == "__main__":
    detected_color = read_camera_pi()
    if detected_color:
        print(f"Veicolo rilevato: {detected_color}")
