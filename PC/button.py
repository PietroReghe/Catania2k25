import RPi.GPIO as GPIO
import subprocess
import time

# Configura il pin GPIO
BUTTON_GPIO = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

try:
    print("In attesa della pressione del pulsante...")
    while True:
        if GPIO.input(BUTTON_GPIO) == GPIO.LOW:  # Il pulsante è premuto
            print("Bottone premuto! Eseguo main.py...")
            subprocess.run(["python3", "main.py"])  # Esegue lo script main.py
            time.sleep(0.5)  # Evita pressioni multiple ravvicinate
        time.sleep(0.1)  # Riduce l'uso della CPU
except KeyboardInterrupt:
    print("Uscita dal programma.")
    GPIO.cleanup()
