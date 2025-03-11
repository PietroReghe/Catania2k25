//include librerie
#include "twoStepper.h" //stepper ruote
#include "Wire.h" //comunicazione I2C per sensori

Command robot = Command();  //creazione oggetto master (gestione motori)

int rpm = 1000;

void setup() {  
  Serial.begin(9600);
  robot.set();
}

void loop() {
  // distance (mm), steps / second, direction, acceleration
    robot.go(100, rpm, "ahead", true);
}
