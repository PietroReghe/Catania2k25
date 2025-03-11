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

class Stepper {
  public:
    Stepper(int p0, int p1, int p2, int p3, int p4, int p5, int p6, int p7);
    volatile int timer;
    void step(int numberSteps, int microStep, float voltPerc);
    void speed(timer);
  private:
    int pin[8];
};

Stepper::Stepper(int p0, int p1, int p2, int p3, int p4, int p5, int p6, int p7) {
  pin[0] = p0; pin[1] = p1; pin[2] = p2; pin[3] = p3;
  pin[4] = p4; pin[5] = p5; pin[6] = p6; pin[7] = p7;
  for (int i = 0; i < 8; i++) {
    pinMode(pin[i], OUTPUT);
  }
}

void Stepper::speed(timer) {
  return timer;
}

void Stepper::step(int numberSteps) {
  if (millis() - prevMillis > timer) {
    for (int j = 0; j < numberSteps; j++) {
      if (thisStep == 4) {
        thisStep = 0;
      }
      for (int i = 0; i < 8; i++) {
        digitalWrite(pin[i], pinStato[thisStep][i]);
      }
      thisStep++;
    }
    prevMillis = millis();
  }
}

#endif // newStepper_h
