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

#define MICROSTEP 16 // microsteps per ogni passo
#define stepsPerLap 200 // passi interi per lap
#define diameterWheels 6.5  // diametro ruote in cm
#define distanceWheels 25   // distanza tra le ruote in cm

// The DX Stepper pins
#define STEPPERDX_DIR_PIN 5
#define STEPPERDX_STEP_PIN 2
// The SX stepper pins
#define STEPPERSX_DIR_PIN 7
#define STEPPERSX_STEP_PIN 4

// Define some steppers and the pins the will use
AccelStepper stepperDX(AccelStepper::DRIVER, STEPPERDX_STEP_PIN, STEPPERDX_DIR_PIN); //motore destro
AccelStepper stepperSX(AccelStepper::DRIVER, STEPPERSX_STEP_PIN, STEPPERSX_DIR_PIN); //motore sinistro

//classe comando
class Command {
  public:
    Command();

    void set();
    void go(int lenght, int rpm, String wayTravel);
    void turn(int degree, int rpm, String wayTurn, String wayTravel);
    void turnBothWheels(int degree, int rpm, String wayTurn);
};
Command::Command() {}

//procedura di setting iniziale con valori massimi di velocità e di accelerazione
//tutti i valori si devono intendere come passi/secondo --> il numero di passi del motore è 3200 (200*16)
void Command::set() {
  stepperDX.setMaxSpeed(10000.0);
  stepperDX.setAcceleration(500.0);
  stepperSX.setMaxSpeed(10000.0);
  stepperSX.setAcceleration(500.0);
}

//funzione per andare avanti
void Command::go(int lenght, int rpm, String wayTravel) {
  int steps = 0; // variabile direzionale

  if (wayTravel == "ahead") {
    steps = 1; // mantieni direzione invariata
  }

  if (wayTravel == "back") {
    steps = -1; // inverti direzione
  }

  //funzione per calcolare il numero di passi necessari da compiere per gli stepper
  //numero passi = lunghezza / circonferenza ruota * stepsPerLap
  unsigned long stride = abs(lenght) / (PI * diameterWheels) * stepsPerLap;

  //lo stepperDX è il motore di riferimento per i comandi
  stepperDX.setCurrentPosition(0); // imposta come 0 la posizione di partenza
  while (abs(stepperDX.currentPosition()) != (stride * MICROSTEP)) { //raggiungi posizione da lunghezza calcolata
    stepperDX.setSpeed(rpm * steps); // la velocità è negativa o positiva in base alla direzione
    stepperSX.setSpeed(rpm * steps);
    stepperDX.runSpeed();
    stepperSX.runSpeed();
    stepperDX.stop();
    stepperSX.stop();
  }
}

//funzione per sterzare con una sola ruota (l'altra fa da perno)
void Command::turn(int degree, int rpm, String wayTurn, String wayTravel) {
  //converti misura in deg in radianti (arduino utilizza i radianti)
  float rad = abs(degree) * 17.45 / 1000;
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
    while (abs(stepperSX.currentPosition()) != (stride * MICROSTEP)) {
      stepperSX.setSpeed(rpm * steps);
      stepperSX.runSpeed();
      stepperSX.stop();
    }
  }
  if (wayTurn == "left") {
    stepperDX.setCurrentPosition(0);
    while (stepperDX.currentPosition() != (stride * MICROSTEP)) {
      stepperDX.setSpeed(rpm * steps);
      stepperDX.runSpeed();
      stepperDX.stop();
    }
  }
}

//funzione per sterzare con entrambe le ruote (il centro dell'asse posteriore resta fermo)
void Command::turnBothWheels(int degree, int rpm, String wayTurn) {
  //conversione degrees in radianti
  float rad = abs(degree) * 17.45 / 1000;
  //calcolo distanza necessaria da compiere per singola ruota
  int lenght = rad * distanceWheels * 0.5;
  unsigned long stride = abs(lenght) / (PI * diameterWheels) * stepsPerLap;
  int steps = 0;

  if (wayTurn == "right") {
    steps = -1;
  }
  //inverti la direzione
  if (wayTurn == "left") {
    steps = 1;
  }

  stepperDX.setCurrentPosition(0);
  while (abs(stepperDX.currentPosition()) != (stride * MICROSTEP)) {
    stepperDX.setSpeed(rpm * steps);
    stepperSX.setSpeed(-rpm * steps);
    stepperDX.runSpeed();
    stepperSX.runSpeed();
    stepperDX.stop();
    stepperSX.stop();
  }
}
#endif twoStepper_h
