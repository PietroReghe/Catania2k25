from picamera2 import Picamera2
import numpy as np
import time
import colorsys

thing = "Undefined"

def read_from_camera():
    global thing

    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()

    time.sleep(2)  # Tempo per inizializzazione

    while True:
        frame = picam2.capture_array()

        if frame is None:
            print("⚠️ Frame non valido, riprovo...")
            continue

        # Flip orizzontale (come cv2.flip(frame, 1))
        frame = np.flip(frame, axis=1)

        height, width, _ = frame.shape
        cx = width // 2
        cy = height // 2

        # BGR to HSV conversion per un solo pixel
        b, g, r = frame[cy, cx] / 255.0
        h, s, v = colorsys.rgb_to_hsv(r, g, b)

        hue_value = int(h * 180)      # OpenCV usa H in [0,180]
        sat_value = int(s * 255)
        thing_value = int(v * 255)

        # 🔍 Logica colori
        if thing_value < 50:
            thing = "VOID"
        elif sat_value < 30:
            thing = "PARETE"
        else:
            if hue_value > 145 or hue_value < 5:
                if sat_value > 180:
                    thing = "REDCAR"
                else:
                    thing = "OSTACOLO"
            elif hue_value < 45:
                thing = "YELLOWCAR"
            elif hue_value < 90:
                thing = "GREENCAR"
            else:
                thing = "BLUECAR"

        print(f"🎯 Al centro: {thing} | HSV: ({hue_value}, {sat_value}, {thing_value})")

        # Esci con 'q' (puoi usare un contatore o input)
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            break

    picam2.stop()

# Avvia
read_from_camera()
