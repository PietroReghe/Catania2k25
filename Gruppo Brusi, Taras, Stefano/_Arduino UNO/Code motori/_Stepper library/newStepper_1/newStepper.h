// Per driver con 2 ponti H

#ifndef newStepper_h
#define newStepper_h

#include "Arduino.h"


int pinStato[4][8] = {
  (1, 0, 0, 1, 1, 0, 0, 1),
  (0, 1, 1, 0, 1, 0, 0, 1),
  (0, 1, 1, 0, 0, 1, 1, 0),
  (1, 0, 0, 1, 0, 1, 1, 0)
};

class stepper {
  public:
    stepper(int pin[0], int pin[1],
            int pin[2], int pin[3],
            int pin[4], int pin[5],
            int pin[6], int pin[7]
           );
    void step(int numberSteps, int microStep, float voltPerc);
  private:
    int pin[8];
    void pwm(int pin, int frequence, int stato);
}

stepper::stepper(int pin[0], int pin[1], int pin[2], int pin[3],
                 int pin[4], int pin[5], int pin[6], int pin[7])
{
  for (int i = 0; i < 8, i++) {
    pinMode(pin[i]; OUTPUT);
  }
}

const int sensibility = 50; // Periodo PWM in microsecondi
int thisStep = 0;

void stepper::step(int numberSteps, int microStep, float voltPerc) {
  voltPerc = constrain(map(voltPerc, 0, 100, 0, sensibility), 0, sensibility);
  for (int j = 0; j < numberSteps; j++) {
    if (thisStep == 4) {
      thisStep = 0;
    }
    if (microStep == 1) {
      for (int i = 0; i < 8; i++) {
        pwm(pin[i], voltPerc, pin[thisStep][i]);
      }
      thisStep++;
    }
  }
}

void pwm(int pin, int frequence, int stato) {
  frquence = constrain(frequence, 0, sensibility);
  if (stato == 1) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(frequence);
    digitalWrite(pin, LOW);
    delayMicroseconds(sensibility - frequence);
  } else (stato == 0) {
    digitalWrite(pin, LOW);
  }
}

#endif newStepper_h
