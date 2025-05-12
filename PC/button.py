import RPi.GPIO as GPIO
import subprocess
import time
import os

# Configura GPIO
SWITCH_PIN = 17  # GPIO17 (modifica se necessario)
GPIO.setmode(GPIO.BCM)
GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Path all'ambiente virtuale e script
VENV_ACTIVATE = "/home/robocope/Documents/Catania2k25/venv-cam2/bin/activate"
SCRIPT_PATH = "/home/robocope/Documents/Catania2k25/main.py"

def run_script():
    print("Levetta ON, lancio script...")
    cmd = f"/bin/bash -c 'source {VENV_ACTIVATE} && python {SCRIPT_PATH}'"
    subprocess.Popen(cmd, shell=True)

try:
    print("In attesa della levetta su ON (GPIO17)...")
    while True:
        if GPIO.input(SWITCH_PIN) == GPIO.HIGH:
            run_script()
            # Aspetta che la levetta torni su OFF per evitare rilanci
            while GPIO.input(SWITCH_PIN) == GPIO.HIGH:
                time.sleep(0.1)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Interrotto manualmente.")
finally:
    GPIO.cleanup()
