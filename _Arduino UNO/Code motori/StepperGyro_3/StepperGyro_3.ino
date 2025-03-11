//include librerie
#include "twoStepper.h" //stepper ruote
#include "Wire.h" //comunicazione I2C per sensori

Command robot = Command();  //creazione oggetto master (gestione motori)

int rpm = 800;

void setup() {
  Serial.begin(9600);
  robot.set();
}

void loop() {
  robot.turnBothWheels(10, rpm);
  delay(500);
}
