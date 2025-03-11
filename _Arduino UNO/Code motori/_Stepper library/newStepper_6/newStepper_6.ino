// Definizione dei pin delle bobine
#define COIL1 8   // Bobina A
#define COIL2 9   // Bobina B
#define COIL3 10  // Bobina C
#define COIL4 11  // Bobina D

// Numero di microstep per passo completo
#define MICROSTEPS 16

// Sequenza di 16 microstep per passo
// Ogni microstep ha un valore di duty cycle per ogni bobina (A, B, C, D)
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
int stepDelay = 1000;  // Ritardo tra microstep in microsecondi (velocità motore)

// Frequenza PWM simulata (per regolazione fine)
int pwmFrequency = 2000; // Frequenza PWM in Hz
unsigned long pwmPeriod; // Periodo del PWM in microsecondi
unsigned long lastPwmTime = 0;

void setup() {
  // Configura i pin come output
  pinMode(COIL1, OUTPUT);
  pinMode(COIL2, OUTPUT);
  pinMode(COIL3, OUTPUT);
  pinMode(COIL4, OUTPUT);

  // Calcola il periodo del PWM
  pwmPeriod = 1000000 / pwmFrequency;
}

void loop() {
  unsigned long currentTime = micros();

  // Verifica il tempo trascorso per il controllo del PWM
  if (currentTime - lastPwmTime >= pwmPeriod) {
    // Aggiorna l'uscita delle bobine per il microstep corrente
    setMicroStep(stepIndex);

    // Passa al prossimo microstep (16 microsteps per ciclo completo)
    stepIndex = (stepIndex + 1) % MICROSTEPS;

    // Salva l'ultimo tempo del PWM
    lastPwmTime = currentTime;
  }

  // Ritardo per regolare la velocità del motore
  delayMicroseconds(stepDelay);
}

// Funzione per impostare un microstep utilizzando il PWM simulato
void setMicroStep(int step) {
  softwarePWM(COIL1, microStepSequence[step][0]);
  softwarePWM(COIL2, microStepSequence[step][1]);
  softwarePWM(COIL3, microStepSequence[step][2]);
  softwarePWM(COIL4, microStepSequence[step][3]);
}

// Funzione per simulare PWM sui pin digitali (accensione proporzionale delle bobine)
void softwarePWM(int pin, int dutyCycle) {
  if (dutyCycle > 0) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(dutyCycle * 10);  // Proporzionale al duty cycle
  }
  digitalWrite(pin, LOW);
}

// Funzione per invertire la direzione del motore (in caso di necessità)
void reverseDirection() {
  stepIndex = (stepIndex + (MICROSTEPS / 2)) % MICROSTEPS;
}
