// Definizione dei pin delle bobine
#define COIL1 8   // Bobina A
#define COIL2 9   // Bobina B
#define COIL3 10  // Bobina C
#define COIL4 11  // Bobina D

// Numero di microstep per passo completo
#define MICROSTEPS 16

// Sequenza di 16 microstep per passo
const int microStepSequence[MICROSTEPS][4] = {
  {255,   0,   0,   0},  // Step 1
  {255, 128,   0,   0},  // Step 2
  {255, 255,   0,   0},  // Step 3
  {128, 255,   0,   0},  // Step 4
  {  0, 255,   0,   0},  // Step 5
  {  0, 255, 128,   0},  // Step 6
  {  0, 255, 255,   0},  // Step 7
  {  0, 128, 255,   0},  // Step 8
  {  0,   0, 255,   0},  // Step 9
  {  0,   0, 255, 128},  // Step 10
  {  0,   0, 255, 255},  // Step 11
  {  0,   0, 128, 255},  // Step 12
  {  0,   0,   0, 255},  // Step 13
  {128,   0,   0, 255},  // Step 14
  {255,   0,   0, 255},  // Step 15
  {255,   0,   0, 128}   // Step 16
};

// Variabili di controllo del motore
int stepIndex = 0;  // Indice della sequenza corrente
int stepDelay = 50;  // Delay tra microstep in microsecondi (velocità motore)

void setup() {
  // Configura i pin come output
  pinMode(COIL1, OUTPUT);
  pinMode(COIL2, OUTPUT);
  pinMode(COIL3, OUTPUT);
  pinMode(COIL4, OUTPUT);
}

void loop() {
  // Simula il microstepping del motore passo-passo
  setMicroStep(stepIndex);

  // Passa al microstep successivo (16 microsteps per ciclo completo)
  stepIndex = (stepIndex + 1) % MICROSTEPS;

  // Ritardo per controllare la velocità del motore
  delayMicroseconds(stepDelay);
}

// Funzione per impostare un microstep utilizzando PWM software simulato
void setMicroStep(int step) {
  softwarePWM(COIL1, microStepSequence[step][0]);
  softwarePWM(COIL2, microStepSequence[step][1]);
  softwarePWM(COIL3, microStepSequence[step][2]);
  softwarePWM(COIL4, microStepSequence[step][3]);
}

// Funzione per simulare PWM sui pin digitali
void softwarePWM(int pin, int dutyCycle) {
  if (dutyCycle > 0) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(dutyCycle * 4);  // Simula il duty cycle con durata proporzionale
  }
  digitalWrite(pin, LOW);
}
