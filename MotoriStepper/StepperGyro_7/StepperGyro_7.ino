//include librerie
#include "twoSteppers.h"  // Stepper ruote
#include "Wire.h"         // Comunicazione I2C per sensori

Robot robot = Robot();  // Creazione oggetto master (gestione motori)

int velocita_vai = 2000;
int velocita_gira = 2000;

void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  robot.set();
}

void loop() {
}
