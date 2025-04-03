#ifndef TWO_STEPPER_H
#define TWO_STEPPER_H

//#define NO_GIROSCOPIO
#define NO_GIROSCOPIO
bool giroscopio_attivo = true;  // Variabile di stato che abilita le funzioni con il giroscopio
                                // Se il giroscopio fallisce il codice procede lo stesso senza il giroscopio

// Librerie
#include "Arduino.h"       // include la libreria di arduino per poter utilizzare i pin
#include <AccelStepper.h>  // libreria utilizzata per muovere gli stepper
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Costanti
#define micropassi 4                   // micropassi per ogni passo
#define numeroPassi 200                // Passi interi per giro
#define diametroRuote 63               // Diametro ruote in mm
#define distanzaRuote 172              // Distanza tra le ruote in mm
#define durata 208000                  // Durata gara in millis
const int K_volte_misura_angolo = 50;  // Fattore contatore numero passi tra due rilevazioni dell'angolo
                                       // Più è grande, più la possibilità che non si fermi all'angolo stabilito è maggiore
                                       // Più è piccolo, più la possibiltà che rilevi misure "false" è maggiore
const float K_angolo = 0.9865f;        // Fattore di correzione per calcolo angolo
const int K_accelerazione = 1;         // Fattore incremento accelerazione


// ===================================================================
// ==============          PINS LED - STEPPER        =================
// ===================================================================

// Pin LED errore
#define LED_ERRORE 12
// Pins motore destro
#define STEPPERDX_DIR_PIN 6
#define STEPPERDX_STEP_PIN 5
// Pins motore sinistro
#define STEPPERSX_DIR_PIN 4
#define STEPPERSX_STEP_PIN 3

// Steppers
AccelStepper stepperDX(AccelStepper::DRIVER, STEPPERDX_STEP_PIN, STEPPERDX_DIR_PIN);  // motore destro
AccelStepper stepperSX(AccelStepper::DRIVER, STEPPERSX_STEP_PIN, STEPPERSX_DIR_PIN);  // motore sinistro


// ===================================================================
// ================          VARIABILI TEST        ===================
// ===================================================================

const int velocita_motori_vai = 1500;
const int velocita_motori_gira = 600;

// ===================================================================
// =============          VARIABILI GIROSCOPIO        ================
// ===================================================================

#ifdef GIROSCOPIO
// Interrupt pin
#define INTERRUPT_PIN 2

// Variabili giroscopio
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Girscopio
MPU6050 mpu;

// Variabili PID
float setpoint = 0;
float input, output;
float Kp = 20, Ki = 0.0, Kd = 2;
float previousError = 0, integral = 0;

// Variabile velocità motori
int velocita = 0;
// Variabile K_volte_misura_angolo
int K_volte_misura_angolo = 0;
// angolo misurato dal giroscopio
float angoloMisura = 0.0f;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// Funzione giroscopio
void dmpDataReady();

// Funzione interrupt per giroscopio
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void rilevaAngolo() {
  if (!dmpReady) {
    Serial.println("impallato");
    return;
  }
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    // Serial.print("euler\t");
    // Serial.println(euler[0] * 180 / M_PI);
    angoloMisura = float(euler[0] * 180 / M_PI);
  }
}

float PIDControl(float setpoint, float input) {
  float error = setpoint - input;
  integral += error;
  float derivative = error - previousError;
  previousError = error;
  return Kp * error + Ki * integral + Kd * derivative;
}
#endif


// Funzione di interruzione
void interrompi() {
  if (millis() >= durata) {
    Serial.println("finito");
    while (true) {
    }
  }
}


// ===================================================================
// =================          CLASSE ROBOT        ====================
// ===================================================================

class Robot {
public:
  Robot();

  void set();
  void vai(int length, int rpm, String verso, String accelerazione);
  void gira(int AngoloGradi, int rpm, String direzione, String verso);
  void giraRuote(int AngoloGradi, int rpm);
  void test();
};


Robot::Robot() {}


// ==================================================================
// ===================          SETTING        ======================
// ==================================================================

#ifdef GIROSCOPIO
void Robot::set() {
  stepperDX.setMaxSpeed(2000.0);
  stepperSX.setMaxSpeed(2000.0);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("\nSend any character to begin DMP programming and demo: "));

  delay(100);  // delay di attesa per inizializzare la comunicazione con in gyro

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip

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
    giroscopio_attivo = true;
    return;
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("------ ERRORE -------  DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    giroscopio_attivo = false;
    return;
  }
}
#endif

#ifdef NO_GIROSCOPIO
void Robot::set() {
  stepperDX.setMaxSpeed(4000.0);
  stepperSX.setMaxSpeed(4000.0);
  return;
}
#endif


// ==================================================================
// ===================           VAI           ======================
// ==================================================================

unsigned long passiDaCompiere;  // Passi da compiere
int statoVerso = 0;             // Variabile direzionale
int savePos;                    // Variabile salvataggio posizione

float ampiezzaAngolo = 0.0f;
float memoriaAngoli = 0.0f;

#ifdef GIROSCOPIO
void Robot::vai(int length, int rpm, String verso, String accelerazione) {
  if (giroscopio_attivo == true) {
    // Angolo da utilizzare come riferimento per andare dritti
    rilevaAngolo();
    float angoloSet = angoloMisura;

    if (verso == "avanti") {
      statoVerso = 1;  // mantieni direzione invariata
    } else if (verso == "indietro") {
      statoVerso = -1;  // inverti direzione
    }

    // ============== ACCELERAZIONE ==============
    if (accelerazione == "on") {
      // Imposta come 0 la posizione di partenza
      stepperDX.setCurrentPosition(0);
      stepperSX.setCurrentPosition(0);
      // Velocità motori a regime
      // Velocità negativa o positiva in base a statoVerso
      velocita = rpm * statoVerso;

      // ---------- Accelerazione ----------
      // La variabile "i" gestisce incrementa la velocità ogni microstep
      for (int i = 100; i < rpm; i += K_accelerazione) {
        stepperDX.move(1);
        stepperSX.move(1);
        stepperDX.setSpeed(i * statoVerso);
        stepperSX.setSpeed(-i * statoVerso);
        stepperDX.runSpeed();
        stepperSX.runSpeed();
      }

      // Salva la posizione attuale
      savePos = abs(stepperDX.currentPosition());
      // Calcola i passi che lo stepper deve compiere
      passiDaCompiere = abs(length) / (PI * diametroRuote) * numeroPassi - savePos * 2 / micropassi;
      // Imposta come 0 la posizione di partenza
      stepperDX.setCurrentPosition(0);
      stepperSX.setCurrentPosition(0);
      // Imposta la correzione dell'errore a 0
      output = 0;
      int passiValore = (passiDaCompiere * micropassi * 2);
      // ---------- Movimento a velcoità costante ----------
      while ((abs(stepperDX.currentPosition()) + abs(stepperSX.currentPosition())) < passiValore) {
        // Calcola l'errore ogni 100 micropassi
        if (K_volte_misura_angolo > 100) {
          rilevaAngolo();
          input = angoloMisura;
          output = 0;
          setpoint = angoloSet;
          output = PIDControl(setpoint, input);
          K_volte_misura_angolo = 0;
          //Serial.println((abs(stepperDX.currentPosition()) + abs(stepperSX.currentPosition())));
        }
        stepperDX.setSpeed(velocita - output);
        stepperSX.setSpeed(-1 * (velocita + output));
        stepperDX.runSpeed();
        stepperSX.runSpeed();
        // Incrementa il K_volte_misura_angolo
        K_volte_misura_angolo++;
      }

      // ---------- Decelerazione ----------
      stepperDX.setCurrentPosition(0);
      for (int i = rpm; i > 100; i -= K_accelerazione) {
        stepperDX.move(1);
        stepperSX.move(1);
        stepperDX.setSpeed(i * statoVerso);
        stepperSX.setSpeed(-i * statoVerso);
        stepperDX.runSpeed();
        stepperSX.runSpeed();
      }
      // savePos += stepperDX.currentPosition();

      // ============== NO ACCELERAZIONE ==============
    } else if (accelerazione == "off") {
      // velocità motori
      velocita = rpm * statoVerso;
      // numero passi
      passiDaCompiere = abs(length) / (PI * diametroRuote) * numeroPassi;
      // imposta come 0 la posizione di partenza
      stepperDX.setCurrentPosition(0);
      stepperSX.setCurrentPosition(0);
      // Imposta output PID a 0
      output = 0;
      K_volte_misura_angolo = 0;

      // raggiungi posizione da lunghezza calcolata
      while ((abs(stepperDX.currentPosition()) + abs(stepperSX.currentPosition())) < (passiDaCompiere * micropassi * 2)) {
        // Calcola errore ogni 100 micropassi
        if (K_volte_misura_angolo > 100) {
          rilevaAngolo();
          input = angoloMisura;
          setpoint = angoloSet;
          output = PIDControl(setpoint, input);
          K_volte_misura_angolo = 0;
        }
        stepperDX.setSpeed(velocita - output);
        stepperSX.setSpeed(-1 * (velocita + output));
        stepperDX.runSpeed();
        stepperSX.runSpeed();
        K_volte_misura_angolo++;
      }
    }
    stepperDX.stop();
    stepperSX.stop();
  } else if (giroscopio_attivo == false) {
    if (verso == "avanti") {
      statoVerso = 1;  // mantieni direzione invariata
    } else if (verso == "indietro") {
      statoVerso = -1;  // inverti direzione
    }

    if (accelerazione == "on") {
      //Serial.println("1");
      //Serial.println(abs(length) / (PI * diametroRuote)* numeroPassi);
      stepperDX.setCurrentPosition(0);
      for (int i = 300; i < rpm; i += K_accelerazione) {
        stepperDX.move(1);
        stepperSX.move(1);
        stepperDX.setSpeed(i * statoVerso);  // la velocità è negativa o positiva in base alla direzione
        stepperSX.setSpeed(-i * statoVerso);
        stepperDX.runSpeed();
        stepperSX.runSpeed();
      }
      savePos = stepperDX.currentPosition();

      passiDaCompiere = abs(length) / (PI * diametroRuote) * numeroPassi - savePos * 2 / micropassi;
      //Serial.println(passiDaCompiere);
      stepperDX.setCurrentPosition(0);                                             // imposta come 0 la posizione di partenza
      while (abs(stepperDX.currentPosition()) < (passiDaCompiere * micropassi)) {  //raggiungi posizione da lunghezza calcolata
        stepperDX.setSpeed(rpm * statoVerso);                                      // la velocità è negativa o positiva in base alla direzione
        stepperSX.setSpeed(-rpm * statoVerso);
        stepperDX.runSpeed();
        stepperSX.runSpeed();
      }

      stepperDX.setCurrentPosition(0);
      for (int i = rpm; i > 300; i -= K_accelerazione) {
        stepperDX.move(1);
        stepperSX.move(1);
        stepperDX.setSpeed(i * statoVerso);  // la velocità è negativa o positiva in base alla direzione
        stepperSX.setSpeed(-i * statoVerso);
        stepperDX.runSpeed();
        stepperSX.runSpeed();
      }
      savePos += stepperDX.currentPosition();

    } else if (accelerazione == "off") {

      // numero passi
      passiDaCompiere = abs(length) / (PI * diametroRuote) * numeroPassi;
      // imposta come 0 la posizione di partenza
      stepperDX.setCurrentPosition(0);

      //raggiungi posizione da lunghezza calcolata
      while (abs(stepperDX.currentPosition()) < (passiDaCompiere * micropassi)) {
        // velocità motori
        stepperDX.setSpeed(rpm * statoVerso);
        stepperSX.setSpeed(-rpm * statoVerso);
        stepperDX.runSpeed();
        stepperSX.runSpeed();
      }
    }
  }
}
#endif

#ifdef NO_GIROSCOPIO
void Robot::vai(int length, int rpm, String verso, String accelerazione) {
  if (verso == "avanti") {
    statoVerso = 1;  // mantieni direzione invariata
  } else if (verso == "indietro") {
    statoVerso = -1;  // inverti direzione
  }

  if (accelerazione == "on") {
    //Serial.println("1");
    //Serial.println(abs(length) / (PI * diametroRuote)* numeroPassi);
    stepperDX.setCurrentPosition(0);
    for (int i = 300; i < rpm; i += K_accelerazione) {
      stepperDX.move(1);
      stepperSX.move(1);
      stepperDX.setSpeed(i * statoVerso);  // la velocità è negativa o positiva in base alla direzione
      stepperSX.setSpeed(-i * statoVerso);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
    }
    savePos = stepperDX.currentPosition();

    passiDaCompiere = abs(length) / (PI * diametroRuote) * numeroPassi - savePos * 2 / micropassi;
    //Serial.println(passiDaCompiere);
    stepperDX.setCurrentPosition(0);                                             // imposta come 0 la posizione di partenza
    while (abs(stepperDX.currentPosition()) < (passiDaCompiere * micropassi)) {  //raggiungi posizione da lunghezza calcolata
      stepperDX.setSpeed(rpm * statoVerso);                                      // la velocità è negativa o positiva in base alla direzione
      stepperSX.setSpeed(-rpm * statoVerso);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
    }

    stepperDX.setCurrentPosition(0);
    for (int i = rpm; i > 300; i -= K_accelerazione) {
      stepperDX.move(1);
      stepperSX.move(1);
      stepperDX.setSpeed(i * statoVerso);  // la velocità è negativa o positiva in base alla direzione
      stepperSX.setSpeed(-i * statoVerso);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
    }
    savePos += stepperDX.currentPosition();

  } else if (accelerazione == "off") {

    // numero passi
    passiDaCompiere = abs(length) / (PI * diametroRuote) * numeroPassi;
    // imposta come 0 la posizione di partenza
    stepperDX.setCurrentPosition(0);

    //raggiungi posizione da lunghezza calcolata
    while (abs(stepperDX.currentPosition()) < (passiDaCompiere * micropassi)) {
      // velocità motori
      stepperDX.setSpeed(rpm * statoVerso);
      stepperSX.setSpeed(-rpm * statoVerso);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
    }
  }
}
#endif


// ==================================================================
// =============         GIRA UNA SOLA RUOTA         ================
// ==================================================================

#ifdef GIROSCOPIO  // MdB
void Robot::gira(int AngoloGradi, int rpm, String direzione, String verso) {
  if (giroscopio_attivo == true) {
    memoriaAngoli += float(AngoloGradi) * K_angolo;  //diminuendo il fattore moltiplicativo l'angolo diminuisce
                                                     //il valore corretto sarebbe circa 0.9865f
    int n = abs(int(memoriaAngoli) / 180) + 2;
    float versoAngolo = 0.0f;

    if (memoriaAngoli >= 0.0f) {
      versoAngolo = 1.0f;
    }
    if (memoriaAngoli < 0.0f) {
      versoAngolo = -1.0f;
    }
    // Normalizza l'angolo tra -180 e +180
    if (n % 2 == 0) {
      ampiezzaAngolo = (float(abs(int(memoriaAngoli) % 180)) + 0.3f) * versoAngolo;
    } else if (n % 2 == 1) {
      ampiezzaAngolo = (float(-180 + abs(int(memoriaAngoli) % 180)) + 0.3f) * versoAngolo;
    }

    if (direzione == "sinistra") {
      rpm = -rpm;
    }

    if (verso == "avanti") {
      statoVerso = -1;
    }
    if (verso == "indietro") {
      statoVerso = 1;  // inverti la direzione
    }

    // angoloMisura = angolo misurato dal giroscipio
    while (angoloMisura < (ampiezzaAngolo - 0.4) || angoloMisura > (ampiezzaAngolo + 0.4)) {
      if (K_volte_misura_angolo > 2) {
        rilevaAngolo();
        K_volte_misura_angolo = 0;
      }
      K_volte_misura_angolo++;
      if (direzione == "sinistra") {
        stepperDX.moveTo(1);
        stepperDX.setSpeed(-rpm * statoVerso);
        stepperDX.runSpeed();
      } else {
        stepperSX.moveTo(1);
        stepperSX.setSpeed(-rpm * statoVerso);
        stepperSX.runSpeed();
      }
    }
    return;
  } else if (giroscopio_attivo == false) {
    // converti misura in deg in radianti (arduino utilizza i radianti)
    float rad = abs(AngoloGradi) * 17.45329 / 1000;
    // calcola la distanza che la ruota deve percorrere
    int length = rad * distanzaRuote;
    // calcola i passi che lo stepper deve compiere
    unsigned long passiDaCompiere = abs(length) / (PI * diametroRuote) * numeroPassi;

    int statoVerso = 0;  // variabile direzionale

    if (verso == "avanti") {
      statoVerso = -1;
    }
    if (verso == "indietro") {
      statoVerso = 1;  // inverti la direzione
    }

    if (direzione == "destra") {
      stepperSX.setCurrentPosition(0);
      while (abs(stepperSX.currentPosition()) < abs(passiDaCompiere * micropassi)) {
        interrompi();
        stepperSX.setSpeed(rpm * statoVerso);
        stepperSX.runSpeed();
        interrompi();
      }
    }
    if (direzione == "sinistra") {
      stepperDX.setCurrentPosition(0);
      while (abs(stepperDX.currentPosition()) < abs(passiDaCompiere * micropassi)) {
        interrompi();
        stepperDX.setSpeed(rpm * statoVerso);
        stepperDX.runSpeed();
        interrompi();
      }
    }
    return;
  }
}
#endif

#ifdef NO_GIROSCOPIO
void Robot::gira(int AngoloGradi, int rpm, String direzione, String verso) {
  // converti misura in deg in radianti (arduino utilizza i radianti)
  float rad = abs(AngoloGradi) * 17.45329 / 1000;
  // calcola la distanza che la ruota deve percorrere
  int length = rad * distanzaRuote;
  // calcola i passi che lo stepper deve compiere
  unsigned long passiDaCompiere = abs(length) / (PI * diametroRuote) * numeroPassi;

  int statoVerso = 0;  // variabile direzionale

  if (verso == "avanti") {
    statoVerso = -1;
  }
  if (verso == "indietro") {
    statoVerso = 1;  // inverti la direzione
  }

  if (direzione == "destra") {
    stepperSX.setCurrentPosition(0);
    while (abs(stepperSX.currentPosition()) < abs(passiDaCompiere * micropassi)) {
      interrompi();
      stepperSX.setSpeed(rpm * statoVerso);
      stepperSX.runSpeed();
      interrompi();
    }
  }
  if (direzione == "sinistra") {
    stepperDX.setCurrentPosition(0);
    while (abs(stepperDX.currentPosition()) < abs(passiDaCompiere * micropassi)) {
      interrompi();
      stepperDX.setSpeed(rpm * statoVerso);
      stepperDX.runSpeed();
      interrompi();
    }
  }
  return;
}
#endif


// ==================================================================
// ==============        GIRA ENTRAMBE LE RUOTE        ==============
// ==================================================================

#ifdef GIROSCOPIO
void Robot::giraRuote(int AngoloGradi, int rpm) {
  if (giroscopio_attivo == true) {
    memoriaAngoli += float(AngoloGradi) * K_angolo;  //diminuendo il fattore moltiplicativo l'angolo diminuisce
                                                     //il valore corretto sarebbe circa K_angolo
    int n = abs(int(memoriaAngoli) / 180) + 2;
    float versoAngolo = 0.0f;

    if (memoriaAngoli >= 0.0f) {
      versoAngolo = 1.0f;
    }
    if (memoriaAngoli < 0.0f) {
      versoAngolo = -1.0f;
    }
    // Normalizza l'angolo tra -180 e +180
    if (n % 2 == 0) {
      ampiezzaAngolo = (float(abs(int(memoriaAngoli) % 180)) + 0.3f) * versoAngolo;
    } else if (n % 2 == 1) {
      ampiezzaAngolo = (float(-180 + abs(int(memoriaAngoli) % 180)) + 0.3f) * versoAngolo;
    }

    if (AngoloGradi > 0) {
      rpm = -rpm;
    }

    Serial.println(ampiezzaAngolo);
    // angoloMisura = angolo misurato dal giroscipio
    while (angoloMisura < (ampiezzaAngolo - 0.4) || angoloMisura > (ampiezzaAngolo + 0.4)) {
      rilevaAngolo();
      stepperDX.moveTo(1);
      stepperSX.moveTo(1);
      stepperDX.setSpeed(rpm);
      stepperSX.setSpeed(rpm);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
    }
    return;
  } else if (giroscopio_attivo == false) {
    // conversione degrees in radianti
    float rad = abs(AngoloGradi) * 17.45329 / 1000;
    // calcolo distanza necessaria da compiere per singola ruota
    int length = rad * distanzaRuote * 0.5;
    unsigned long passiDaCompiere = abs(length) / (PI * diametroRuote) * numeroPassi;
    int direzione = 0;
    if (AngoloGradi < 0) {
      direzione = -1;
    } else if (AngoloGradi > 0) {
      direzione = 1;
    }
    stepperDX.setCurrentPosition(0);
    while (abs(stepperDX.currentPosition()) < abs(passiDaCompiere * micropassi)) {
      stepperDX.setSpeed(rpm * direzione);
      stepperSX.setSpeed(rpm * direzione);
      stepperDX.runSpeed();
      stepperSX.runSpeed();
      interrompi();
    }
  }
}
#endif

#ifdef NO_GIROSCOPIO
void Robot::giraRuote(int AngoloGradi, int rpm) {
  // conversione degrees in radianti
  float rad = abs(AngoloGradi) * 17.45329 / 1000;
  // calcolo distanza necessaria da compiere per singola ruota
  int length = rad * distanzaRuote * 0.5;
  unsigned long passiDaCompiere = abs(length) / (PI * diametroRuote) * numeroPassi;
  int direzione = 0;
  if (AngoloGradi < 0) {
    direzione = -1;
  } else if (AngoloGradi > 0) {
    direzione = 1;
  }
  stepperDX.setCurrentPosition(0);
  while (abs(stepperDX.currentPosition()) < abs(passiDaCompiere * micropassi)) {
    stepperDX.setSpeed(rpm * direzione);
    stepperSX.setSpeed(rpm * direzione);
    stepperDX.runSpeed();
    stepperSX.runSpeed();
    interrompi();
  }
}


// ==================================================================
// =======================        TEST        ======================= 
// ==================================================================

void Robot::test(){
}

#endif
#endif