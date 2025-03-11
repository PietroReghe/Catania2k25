/*
   LIBRERIA GESTIONE MOTORI RUOTE

   libreria creata da noi per utilizzare attraverso funzioni più agevoli e
   intuitive i motori e gli spostamenti del robot
*/
#ifndef twoStepper_h
#define twoStepper_h

//include librerie
#include "Arduino.h" //include la libreria di arduino per poter utilizzare i pin
#include <AccelStepper.h> // libreria utilizzata per muovere gli stepper
#include <Wire.h>
#include <MPU6050.h>

// Definizione valori costanti
#define MICROSTEPS 4 // MICROSTEPSs per ogni passo
#define stepsPerLap 200 // passi interi per lap
#define diameterWheels 65  // diametro ruote in mm
#define distanceWheels 200   // distanza tra le ruote in mm
const int constAcc = 1;

// The DX Stepper pins
#define STEPPERDX_DIR_PIN 4
#define STEPPERDX_STEP_PIN 5
// The SX stepper pins
#define STEPPERSX_DIR_PIN 2
#define STEPPERSX_STEP_PIN 3

#define tempo_fine 208000

// Define some steppers and the pins the will use
AccelStepper stepperDX(AccelStepper::DRIVER, STEPPERDX_STEP_PIN, STEPPERDX_DIR_PIN); //motore destro
AccelStepper stepperSX(AccelStepper::DRIVER, STEPPERSX_STEP_PIN, STEPPERSX_DIR_PIN); //motore sinistro

MPU6050 mpu;
float setpoint = 0;
float input, output;
float Kp = 2, Ki = 0.1, Kd = 0.7;
float previousError = 0, integral = 0;
int velocita;
int counter = 0;

// Funzione per far interrompere il robot allo scadere del tempo
void interrompi() {
  if (millis() >= tempo_fine) {
    Serial.println("finito");
    while (true) {}
  }
}

// classe
class Command {
  public:
    Command();

    void set();
    void go(int lenght, int rpm, String wayTravel, String accel);
    void turn(int degree, int rpm, String wayTurn, String wayTravel);
    void turnBothWheels(int degree, int rpm, String wayTurn);
};

Command::Command() {}


float getGyroAngle() {
  /*
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    Serial.println(gz / 131.0);
    return gz / 131.0; // Convert raw data to degrees per second
  */
  Serial.println(mpu.getRotationZ() / 131.0);
  return mpu.getRotationZ() / 131.0;
}

float PIDControl(float setpoint, float input) {
  float error = setpoint - input;
  integral += error;
  float derivative = error - previousError;
  previousError = error;
  return Kp * error + Ki * integral + Kd * derivative;
}

// ==================================================
//                      SETTING
// ==================================================
//tutti i valori si devono intendere come passi/secondo --> il numero di passi del motore è 3200 (200*16)
void Command::set() {
  stepperDX.setMaxSpeed(4000.0);
  stepperSX.setMaxSpeed(4000.0);
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  Serial.println("go");
}



unsigned long stride;
int steps = 0; // variabile direzionale
int savePos;

// ==================================================
//                        GO
// ==================================================

void Command::go(int lenght, int rpm, String wayTravel, String accel) {
  if (wayTravel == "ahead") {
    steps = 1; // mantieni direzione invariata
  } else if (wayTravel == "back") {
    steps = -1; // inverti direzione
  }

  if (accel == "on") {
    //Serial.println("1");
    //Serial.println(abs(lenght) / (PI * diameterWheels)* stepsPerLap);
    stepperDX.setCurrentPosition(0);
    for (int i = 300; i < rpm; i += constAcc) {
      stepperDX.move(1);
      stepperSX.move(1);
      stepperDX.setSpeed(i * steps); // la velocità è negativa o positiva in base alla direzione
      stepperSX.setSpeed(i * steps);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
    }
    savePos = stepperDX.currentPosition();

    stride = abs(lenght) / (PI * diameterWheels) * stepsPerLap - savePos * 2 / MICROSTEPS;
    //Serial.println(stride);
    stepperDX.setCurrentPosition(0); // imposta come 0 la posizione di partenza
    velocita = rpm * steps;
    while (abs(stepperDX.currentPosition()) != (stride * MICROSTEPS)) { //raggiungi posizione da lunghezza calcolata
      if (counter == 10) {
        input = getGyroAngle();
        setpoint = 0;
        output = PIDControl(setpoint, input);
        counter = 0;
        stepperDX.setSpeed(velocita + output);
        stepperSX.setSpeed(velocita - output);
      }
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      counter++;
    }

    stepperDX.setCurrentPosition(0);
    for (int i = rpm; i > 300; i -= constAcc) {
      stepperDX.move(1);
      stepperSX.move(1);
      stepperDX.setSpeed(i * steps);
      stepperSX.setSpeed(i * steps);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
    }
    savePos += stepperDX.currentPosition();
    //Serial.println(savePos / MICROSTEPS);
  }
  else if (accel == "off") {
    stride = abs(lenght) / (PI * diameterWheels) * stepsPerLap;

    velocita = rpm * steps;
    stepperDX.setSpeed(velocita);
    stepperSX.setSpeed(velocita);
    stepperDX.setCurrentPosition(0); // imposta come 0 la posizione di partenza

    while (abs(stepperDX.currentPosition()) != (stride * MICROSTEPS)) { //raggiungi posizione da lunghezza calcolata
      if (counter == 10) {
        input = getGyroAngle();
        setpoint = 0;
        output = PIDControl(setpoint, input);
        counter = 0;
        stepperDX.setSpeed(velocita + output);
        stepperSX.setSpeed(velocita - output);
      }
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      counter++;
    }
  }
}



// ==================================================
//                      TURN
// ==================================================

//funzione per sterzare con una sola ruota (l'altra fa da perno)
void Command::turn(int degree, int rpm, String wayTurn, String wayTravel) {
  //converti misura in deg in radianti (arduino utilizza i radianti)
  float rad = abs(degree) * 17.45329 / 1000;
  // calcola la distanza che la ruota deve percorrere
  int lenght = rad * distanceWheels;
  //calcola i passi che lo stepper deve compiere
  unsigned long stride = abs(lenght) / (PI * diameterWheels) * stepsPerLap;

  int steps = 0; //variabile direzionale

  if (wayTravel == "ahead") {
    steps = 1;
  }
  if (wayTravel == "back") {
    steps = -1; //inverti la direzione
  }

  if (wayTurn == "right") {
    stepperSX.setCurrentPosition(0);
    while (abs(stepperSX.currentPosition()) != (stride * MICROSTEPS)) {
      interrompi();
      stepperSX.setSpeed(rpm * steps);
      stepperSX.runSpeed();
      interrompi();
    }
  }
  if (wayTurn == "left") {
    stepperDX.setCurrentPosition(0);
    while (stepperDX.currentPosition() != (stride * MICROSTEPS)) {
      interrompi();
      stepperDX.setSpeed(rpm * steps);
      stepperDX.runSpeed();
      interrompi();
    }
  }
}



// ==================================================
//                   TURN BOTH WHEELS
// ==================================================

//funzione per sterzare con entrambe le ruote (il centro dell'asse posteriore resta fermo)
void Command::turnBothWheels(int degree, int rpm, String wayTurn) {
  //conversione degrees in radianti
  float rad = (float)abs(degree) * 17.45329 / 1000.00;
  //calcolo distanza necessaria da compiere per singola ruota
  int lenght = rad * distanceWheels * 0.5;
  unsigned long stride = (abs(lenght) / (PI * diameterWheels)) * (stepsPerLap * MICROSTEPS);
  int steps = 0;

  if (wayTurn == "right") {
    steps = -1;
  }
  //inverti la direzione
  if (wayTurn == "left") {
    steps = 1;
  }

  stepperDX.setCurrentPosition(0);
  while (abs(stepperDX.currentPosition()) != stride) {
    stepperDX.setSpeed(rpm * steps);
    stepperSX.setSpeed(-rpm * steps);
    stepperDX.runSpeed();
    stepperSX.runSpeed();
  }
}


#endif twoStepper_h
