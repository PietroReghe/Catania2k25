#include <AccelStepper.h>

#define STEP_PIN 2    // Pin STEP del driver A4988
#define DIR_PIN 3     // Pin DIR del driver A4988

// Creazione dell'oggetto AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
    stepper.setMaxSpeed(12000);     // Imposta la velocità massima in passi/sec
    stepper.setAcceleration(2000);  // Imposta l'accelerazione in passi/sec^2
}

void loop() {
    // Fase di accelerazione
    stepper.moveTo(4000);  // Numero di passi per la fase di accelerazione
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }

    // Fase di mantenimento della velocità costante
    stepper.setSpeed(10000);
    for (int i = 0; i < 10000; i++) {  // Esegui passi a velocità costante
        stepper.runSpeed();
    }
 

    // Fase di decelerazione
    stepper.moveTo(0); // Torna alla posizione iniziale
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    
    delay(2000); // Aspetta prima di ripetere il ciclo
}
