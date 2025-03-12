// Definizione dei pin delle bobine del motore passo-passo
#define COIL1 8  // Bobina A
#define COIL2 9  // Bobina B
#define COIL3 10 // Bobina C
#define COIL4 11 // Bobina D

// Parametri PWM software
int pwmDutyCycle = 100;   // Duty cycle in percentuale (0-100)
int pwmFrequency = 1000; // Frequenza PWM in Hz
unsigned long pwmPeriod;
unsigned long lastPwmTime = 0;
bool pwmState = LOW;

// Tempo di ritardo tra gli step del motore (maggiore valore = velocità minore)
int stepDelay = 500;  // Microsecondi tra ogni microstep

// Sequenza di microstepping a 8 fasi
const int microStepSequence[8][4] = {
  {1, 0, 0, 0},  // Step 1: A
  {1, 1, 0, 0},  // Step 2: A+B
  {0, 1, 0, 0},  // Step 3: B
  {0, 1, 1, 0},  // Step 4: B+C
  {0, 0, 1, 0},  // Step 5: C
  {0, 0, 1, 1},  // Step 6: C+D
  {0, 0, 0, 1},  // Step 7: D
  {1, 0, 0, 1}   // Step 8: D+A
};

void setup() {
  // Configurazione dei pin delle bobine come output
  pinMode(COIL1, OUTPUT);
  pinMode(COIL2, OUTPUT);
  pinMode(COIL3, OUTPUT);
  pinMode(COIL4, OUTPUT);

  // Calcolo del periodo PWM in microsecondi
  pwmPeriod = 1000000 / pwmFrequency;
}

void loop() {
  static int stepIndex = 0;
  unsigned long currentTime = micros();

  // Gestione PWM software
  if (pwmState == HIGH && (currentTime - lastPwmTime >= (pwmDutyCycle * pwmPeriod) / 100)) {
    powerOffCoils();  // Spegne le bobine
    pwmState = LOW;
    lastPwmTime = currentTime;
  }

  if (pwmState == LOW && (currentTime - lastPwmTime >= pwmPeriod - (pwmDutyCycle * pwmPeriod) / 100)) {
    setMicroStep(stepIndex);  // Imposta il microstep corrente
    pwmState = HIGH;
    lastPwmTime = currentTime;

    // Avanza allo step successivo dopo il ritardo specificato
    stepIndex = (stepIndex + 1) % 8;  // 8 microstep ciclici
  }

  delayMicroseconds(stepDelay);
}

// Funzione per impostare un microstep specifico
void setMicroStep(int step) {
  digitalWrite(COIL1, microStepSequence[step][0]);
  digitalWrite(COIL2, microStepSequence[step][1]);
  digitalWrite(COIL3, microStepSequence[step][2]);
  digitalWrite(COIL4, microStepSequence[step][3]);
}

// Funzione per spegnere tutte le bobine (riduce il consumo)
void powerOffCoils() {
  digitalWrite(COIL1, LOW);
  digitalWrite(COIL2, LOW);
  digitalWrite(COIL3, LOW);
  digitalWrite(COIL4, LOW);
}
