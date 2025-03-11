/*


// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
/// \author  Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2009 Mike McCauley
// $Id: ConstantSpeed.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>

#define STEPPERDX_STEP_PIN 2
#define STEPPERDX_DIR_PIN 3

AccelStepper stepper(AccelStepper::DRIVER, STEPPERDX_STEP_PIN, STEPPERDX_DIR_PIN); //motore destro

void setup()
{  
   stepper.setMaxSpeed(12500); // 8 microsteps
   stepper.setSpeed(1000);  
}

void loop()
{  
   stepper.runSpeed();
}










#define directionPin 3
#define stepPin 2

#define stepsPerRevolution 6400

void setup() {
  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
}

void loop() {
  for (int i = 0; i < stepsPerRevolution; i++) {
    digitalWrite(directionPin, HIGH);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(20);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(20);
  }
}
*/
