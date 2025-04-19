import time
import numpy as np
from picamera import PiCamera
from PIL import Image
def read_from_camera():
    # === PARAMETRI CONFIGURABILI ===
    image_path = 'photo.jpg'
    square_size = 50  # dimensione del quadrato centrale in pixel
    color_thresholds = {
        "BIANCO": ([200, 200, 200], [255, 255, 255]),
        "BLU":    ([0, 0, 100],    [80, 80, 255]),
        "ROSSO":  ([100, 0, 0],    [255, 80, 80]),
        "GIALLO": ([150, 150, 0],  [255, 255, 100]),
    }

    # === SCATTA FOTO ===
    camera = PiCamera()
    camera.resolution = (640, 480)
    time.sleep(2)  # attesa per esposizione
    camera.capture(image_path)
    camera.close()

    # === ELABORA IMMAGINE ===
    image = Image.open(image_path)
    pixels = np.array(image)

    h, w, _ = pixels.shape
    center_x, center_y = w // 2, h // 2

    half = square_size // 2
    crop = pixels[center_y - half:center_y + half, center_x - half:center_x + half]

    mean_color = crop.mean(axis=(0, 1)).astype(int)  # RGB medio
    print(f"Colore medio nel quadrato: {mean_color}")

    # === DETERMINA IL COLORE ===
    def classify_color(rgb):
        for name, (low, high) in color_thresholds.items():
            if all(low[i] <= rgb[i] <= high[i] for i in range(3)):
                return name
        return "COLORE NON RICONOSCIUTO"

    risultato = classify_color(mean_color)
    print("Colore rilevato:", risultato)
