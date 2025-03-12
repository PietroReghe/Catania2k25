COSA CONTENGONO LE CARTELLE

_Stepper library  -->  Librerie Stepper.h arduino
Stepper_1  -->  Libreria creata da noi per il funzionamento di due stepper in contemporanea
                Libreria utilizzata lo scorso anno, funziona
Stepper_2  -->  Modifiche libreria Stepper_1
StepperGyro  -->  Primo test giroscopio
                  Code creato da Chat-GPT, non funziona da solo
                  Il giroscopio funziona con le accelerazioni, poco accurato
                  Non si può usare per gli le curve, ma solo per procedere dritto
                  Calcolo errore tramite un PID
StepperGyro_1  -->  Secondo test giroscopio
                    Code StepperGyro unito con code Stepper_1
                    Poco efficiente
StepperGyro_2  -->  Simile a code StepperGyro_1
                    Poco efficiente
StepperGyro_3  -->  Code che utilizza una libreria più complessa per il giroscopio, questa
                    libreria utilizza un pin interrupt (pin 2) per correggere l'errore
                    ==> CODE MIGLIORE
                    ==> PROBLEMA = FUNZIONE TURN_BOTH_WHEELS NON FUNZIONA CORRETTAMENTE
                    