// Per driver con 2 ponti H

#ifndef newStepper_h
#define newStepper_h

#include "Arduino.h"

int pinStato[4][8] = {
  {1, 0, 0, 1, 1, 0, 0, 1},
  {0, 1, 1, 0, 1, 0, 0, 1},
  {0, 1, 1, 0, 0, 1, 1, 0},
  {1, 0, 0, 1, 0, 1, 1, 0}
};

class stepper {
  public:
    stepper(int p0, int p1, int p2, int p3, int p4, int p5, int p6, int p7);
    void step(int numberSteps, int microStep, float voltPerc);
  private:
    int pin[8];
    void pwm(int pin, int frequence, int stato);
};

stepper::stepper(int p0, int p1, int p2, int p3, int p4, int p5, int p6, int p7) {
  pin[0] = p0; pin[1] = p1; pin[2] = p2; pin[3] = p3;
  pin[4] = p4; pin[5] = p5; pin[6] = p6; pin[7] = p7;
  for (int i = 0; i < 8; i++) {
    pinMode(pin[i], OUTPUT);
  }
}

const int sensibility = 50; // Periodo PWM in microsecondi
int thisStep = 0;

void stepper::step(int numberSteps, int microStep, float voltPerc) {
  voltPerc = map(voltPerc, 0, 100, 0, sensibility);
  voltPerc = constrain(voltPerc, 0, sensibility);
  for (int j = 0; j < numberSteps; j++) {
    if (thisStep == 4) {
      thisStep = 0;
    }
    if (microStep == 1) {
      for (int i = 0; i < 8; i++) {
        pwm(pin[i], voltPerc, pinStato[thisStep][i]);
      }
      thisStep++;
    }
  }
}

void stepper::pwm(int pin, int frequence, int stato) {
  frequence = constrain(frequence, 0, sensibility);
  if (stato == 1) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(frequence);
    digitalWrite(pin, LOW);
    delayMicroseconds(sensibility - frequence);
  } else {
    digitalWrite(pin, LOW);
  }
}

#endif // newStepper_h
