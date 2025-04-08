import cv2
import time

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

thing = "Undefined"

def read_from_camera():
    global thing

    if not cap.isOpened():
        print("❌ Errore: videocamera non trovata o non accessibile.")
        return

    time.sleep(2)  # Tempo per inizializzazione

    while True:
        ret, frame1 = cap.read()

        # ✅ Controllo: frame valido?
        if not ret or frame1 is None:
            print("⚠️ Frame non valido, riprovo...")
            continue

        frame2 = cv2.flip(frame1, 1)
        frame = cv2.convertScaleAbs(frame2, 1, 1)

        try:
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        except cv2.error as e:
            print(f"Errore nella conversione HSV: {e}")
            continue

        height, width, _ = frame.shape
        cx = int(width / 2)
        cy = int(height / 2)

        pixel_center = hsv_frame[cy, cx]
        hue_value = pixel_center[0]
        sat_value = pixel_center[1]
        thing_value = pixel_center[2]

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

        pixel_center_bgr = frame[cy, cx]
        b, g, r = int(pixel_center_bgr[0]), int(pixel_center_bgr[1]), int(pixel_center_bgr[2])

        cv2.putText(frame, thing, (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
        cv2.circle(frame, (cx, cy), 10, (0, 255, 0), 2)

        cv2.imshow("Raspberry Pi Cam", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Avvia
read_from_camera()
