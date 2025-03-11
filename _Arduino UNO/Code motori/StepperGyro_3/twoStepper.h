/*
   LIBRERIA GESTIONE MOTORI RUOTE

   libreria creata da noi per utilizzare attraverso funzioni più agevoli e
   intuitive i motori e gli spostamenti del robot







   PROBLEMA

   FUNZIONE TURN BOTH WHEELS
   IL SECONDO WHILE NON FUNZIONA E NON SO IL PERCHE'
   FUNZIONA SOLO CON I NUMERI NEGATIVI, CON QUELLI POSITIVI NO
*/
#ifndef twoStepper_h
#define twoStepper_h

//include librerie
#include "Arduino.h" //include la libreria di arduino per poter utilizzare i pin
#include <AccelStepper.h> // libreria utilizzata per muovere gli stepper
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//#define INTERRUPT_PIN 2

// Definizione valori costanti
#define MICROSTEPS 4 // MICROSTEPSs per ogni passo
#define stepsPerLap 200 // passi interi per lap
#define diameterWheels 65  // diametro ruote in cm
#define distanceWheels 25   // distanza tra le ruote in cm
const int constAcc = 1;

// The DX Stepper pins
#define STEPPERDX_DIR_PIN 4
#define STEPPERDX_STEP_PIN 5
// The SX stepper pins
#define STEPPERSX_DIR_PIN 6
#define STEPPERSX_STEP_PIN 3

#define tempo_fine 208000

#define INTERRUPT_PIN 2

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// Define some steppers and the pins the will use
AccelStepper stepperDX(AccelStepper::DRIVER, STEPPERDX_STEP_PIN, STEPPERDX_DIR_PIN); //motore destro
AccelStepper stepperSX(AccelStepper::DRIVER, STEPPERSX_STEP_PIN, STEPPERSX_DIR_PIN); //motore sinistro

MPU6050 mpu;

float setpoint = 0;
float input, output;
float Kp = 5, Ki = 0.0, Kd = 2;
float previousError = 0, integral = 0;
int velocita;
int counter = 0;

void dmpDataReady();
int getGyroAngle();
float PIDControl(float setpoint, float input);

// Funzione per far interrompere il robot allo scadere del tempo
void interrompi() {
  if (millis() >= tempo_fine) {
    Serial.println("finito");
    while (true) {}
  }
}

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// classe
class Command {
  public:
    Command();

    void set();
    void go(int lenght, int rpm, String wayTravel, String accel);
    void turn(int degree, int rpm, String wayTurn, String wayTravel);
    void turnBothWheels(int d, int rpm);
};

Command::Command() {}

int getGyroAngle() {
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    int value = euler[0] * 180 / M_PI;
    return value;
    // return (euler[1] * 180 / M_PI);
    // return (euler[2] * 180 / M_PI);
  }
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

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    while (true) {}
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
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
    velocita = rpm * steps;
    for (int i = 300; i < rpm; i += constAcc) {
      stepperDX.move(1);
      stepperSX.move(1);
      stepperDX.setSpeed(velocita);
      stepperSX.setSpeed(velocita);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
    }
    savePos = stepperDX.currentPosition();

    stride = abs(lenght) / (PI * diameterWheels) * stepsPerLap - savePos * 2 / MICROSTEPS;
    //Serial.println(stride);
    stepperDX.setCurrentPosition(0); // imposta come 0 la posizione di partenza
    while (abs(stepperDX.currentPosition()) != (stride * MICROSTEPS)) { //raggiungi posizione da lunghezza calcolata
      stepperDX.setSpeed(rpm * steps); // la velocità è negativa o positiva in base alla direzione
      stepperSX.setSpeed(rpm * steps);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
    }

    stepperDX.setCurrentPosition(0);
    for (int i = rpm; i > 300; i -= constAcc) {
      stepperDX.move(1);
      stepperSX.move(1);
      stepperDX.setSpeed(i * steps); // la velocità è negativa o positiva in base alla direzione
      stepperSX.setSpeed(i * steps);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
    }
    savePos += stepperDX.currentPosition();

  } else if (accel == "off") {

    // velocità motori
    velocita = rpm * steps;

    // numero passi
    stride = abs(lenght) / (PI * diameterWheels) * stepsPerLap;
    // imposta come 0 la posizione di partenza
    stepperDX.setCurrentPosition(0);
    bool controll = true;

    //raggiungi posizione da lunghezza calcolata
    while (abs(stepperDX.currentPosition()) != (stride * MICROSTEPS)) {
      if (controll == true) {
        stepperDX.setSpeed(velocita);
        stepperSX.setSpeed(velocita);
      }
      if (counter == 100) {
        input = getGyroAngle();
        setpoint = 0;
        output = PIDControl(setpoint, input);
        counter = 0;
        controll = false;
      }
      stepperDX.setSpeed(velocita + output);
      stepperSX.setSpeed(velocita - output);
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

bool pregnante(int angolo) {
  //Serial.println(getGyroAngle());
  if (getGyroAngle() < angolo) {
    return true;
  } else if (getGyroAngle() > angolo) {
    return false;
  } else if (getGyroAngle() == angolo) {
    return;
  }
}

// ==================================================
//                   TURN BOTH WHEELS
// ==================================================

//funzione per sterzare con entrambe le ruote (il centro dell'asse posteriore resta fermo)
void Command::turnBothWheels(int d, int rpm) {
  bool sam = true;
  while (pregnante(d) == true) {
    Serial.println("  basso");
    stepperDX.setSpeed(-rpm);
    stepperSX.setSpeed(rpm);
    stepperDX.runSpeed();
    stepperSX.runSpeed();
    sam = false;
  }
  if (sam == false) {
    return;
  }
  while (pregnante(d) == false) {
    Serial.println("  alto");
    stepperDX.setSpeed(rpm);
    stepperSX.setSpeed(-rpm);
    stepperDX.runSpeed();
    stepperSX.runSpeed();
    sam = false;
  }
  if (sam == false) {
    return;
  }
}

#endif twoStepper_h
