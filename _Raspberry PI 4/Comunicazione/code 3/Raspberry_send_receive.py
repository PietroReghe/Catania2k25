import serial
import time

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
        if line:  # Controlla se la stringa non è vuota
            ser.write(b"ok\n")  # Conferma la ricezione
            return line

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()

    while True:
        send(b"Hello from Raspberry Pi!\n")
        response = receive()
        print("Ricevuto da Arduino:", response)
        
        if response == "Done":
            print("Completato!")
            break  # Esci dal loop se Arduino risponde "Done"

        time.sleep(1)
