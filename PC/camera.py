from commands import BLUE_CAR, GREEN, RED_CAR, YELLOW
from picamera2 import Picamera2
from typing import List
from time import sleep
from PIL import Image
import numpy as np
import colorsys

path0 = './image0.jpg'
path1 = './image1.jpg'

mean_color = [0, 0, 0]

camera = Picamera2()
camera_config = camera.create_still_configuration({"size": (1920, 1080)})
camera.configure(camera_config)

def read_color_from_img(img0, img1):
    # Converti le immagini in array numpy
    np_img0 = np.array(img0)
    np_img1 = np.array(img1)

    height, width, _ = np_img0.shape

    # Coordinate centro
    cx = int(width / 2)
    cy = int(height / 2)
    roi_size = 50

    # Estrai le ROI
    roi0 = np_img0[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]
    roi1 = np_img1[cy - roi_size:cy + roi_size, cx - roi_size:cx + roi_size]

    # Media RGB
    mean_rgb0 = roi0.mean(axis=(0, 1)) / 255.0  # Normalizza per HSV
    mean_rgb1 = roi1.mean(axis=(0, 1)) / 255.0

    mean_rgb = (mean_rgb0 + mean_rgb1) / 2

    # Converti in HSV (valori da 0 a 1)
    h, s, v = colorsys.rgb_to_hsv(*mean_rgb)

    # Converti H da [0,1] a [0,180] per coerenza con OpenCV
    hue_value = int(h * 180)
    sat_value = int(s * 255)
    value_value = int(v * 255)

    mean_color[0] = hue_value
    mean_color[1] = sat_value
    mean_color[2] = value_value

    # Classificazione colore
    if hue_value < 10 or hue_value > 135:
        color = RED_CAR
    elif hue_value < 58:
        color = YELLOW
    elif hue_value < 93:
        color = GREEN
    elif hue_value < 135:
        color = BLUE_CAR
    else:
        color = None

    print("Colori ananlizzati:",hue_value, sat_value, value_value)
    return color

def read_from_camera() -> str | None:
    while True:
        camera.start()
        sleep(2)

        camera.capture_file(path0)
        sleep(0.1)
        camera.capture_file(path1)
        camera.stop()

        # Apri immagini con PIL
        img0 = Image.open(path0).convert("RGB")
        img1 = Image.open(path1).convert("RGB")

        color = read_color_from_img(img0, img1)
        print("Colore finale: ",color)
        return color

