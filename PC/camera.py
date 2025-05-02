from PC.commands import BLUE_CAR, GREEN, RED_CAR, YELLOW
from picamera2 import Picamera2
from typing import List
from time import sleep
import numpy as np 
import cv2 

path0 = '/home/pi/Desktop/Raspberry/Catania_2025/image0.jpg'  # Percorso in cui salvare la foto
path1 = '/home/pi/Desktop/Raspberry/Catania_2025/image1.jpg'  # Percorso in cui salvare la foto


mean_color = [0, 0, 0]

camera = Picamera2()
camera_config = camera.create_still_configuration({"size": (1920, 1080)})  # Risoluzionw 1920x1080
camera.configure(camera_config)

def read_color_from_img(img0, img1):
   
   img0 = cv2.convertScaleAbs(img0, 1, 1) 
   img1 = cv2.convertScaleAbs(img1, 1, 1) 
   
   hsv_frame0 = cv2.cvtColor(img0, cv2.COLOR_BGR2HSV)
   hsv_frame1 = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
   
   height, width, _ = img0.shape

   # Coordinate del centro
   cx = int(width / 2)
   cy = int(height / 2)

   # Definire una regione 100x100 attorno al centro
   roi_size = 50  # Metà dimensione della ROI
   roi0 = hsv_frame0[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]
   roi1 = hsv_frame1[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]

   # Calcolare il colore medio nella ROI
   mean_color0 = cv2.mean(roi0)[:3]  # Ignorare il canale alpha
   mean_color1 = cv2.mean(roi1)[:3]
   
   for i in range (0,3,+1):
      mean_color[i] = (mean_color0[i] + mean_color1[i]) / 2
   
   hue_value = int(mean_color[0])
   sat_value = int(mean_color[1])
   value_value = int(mean_color[2])

   if hue_value < 10 or hue_value > 135:
      color = RED_CAR
   elif hue_value < 58:
      color = YELLOW
   elif hue_value < 93:
      color =  GREEN
   elif hue_value < 135:
      color = BLUE_CAR
   else:
      color = None
         
   print(hue_value, sat_value, value_value)
   return color
   
def read_from_camera() -> str | None:


   # Inizializza la fotocamera
   camera.start_preview()
   sleep(2)  # Attendi che la fotocamera si avvii

   # Cattura un'immagine di prova
   while True:
         
         camera.start()              # Avvia la fotocamera
         sleep(2)                  # Attendi che la fotocamera si avvii
         
         camera.capture_file(path0)  # Salva l'immagine nella cartella
         sleep(0.1)
         camera.capture_file(path1)  # Salva l'immagine nella cartella
         
         camera.stop()               # Ferma la fotocamera
         
         img0 = cv2.imread(path0)    # Leggi l'immagine salvata
         img1 = cv2.imread(path1)

         color = read_color_from_img(img0, img1)      # Ricava colore e fai la media delle immagini
         print(color)
         return color

