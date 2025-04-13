# FUNZIONA SOLO SU RASPBERRY
# CODE COMPLETO 

from picamera2 import Picamera2
import numpy as np
import cv2
import serial

# Funzioni per la comunicazione seriale

def send(value):
    """Invia un messaggio e aspetta conferma 'ok' da Arduino."""
    while True:
        ser.write(value)
        line = ser.readline().decode('utf-8').rstrip()
        if line == "ok":
            break

def receive():
    """Riceve un messaggio da Arduino e risponde con 'ok'."""
    while True:
        line = ser.readline().decode('utf-8').rstrip()
        #print("gay")
        if line:  # Controlla se la stringa non  vuota
            #print("mucca")
            ser.write(b"ok\n")  # Conferma la ricezione
            return line

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

def rileva_blocco():
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
    
    return color


ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0)
ser.reset_input_buffer()

#main
while True:
    msg = receive()     #rimane in loop nella funzione fino a quando non riceve un messaggio
    if msg == "c": #camera
        colore = rileva_blocco()
        if colore == "RED":
            #print("rosso")
            send(b"r\n")    #red
            msg = receive()
        if colore != "RED":
            send(b"n\n")    #null
            msg = receive()