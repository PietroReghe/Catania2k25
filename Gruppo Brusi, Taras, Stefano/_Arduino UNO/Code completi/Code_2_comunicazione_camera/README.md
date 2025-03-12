Code di comunicazione con attesa risposta all'invio di ogni messaggio tra Arduino 
e Raspberry. Il code è basato su conferme per ogni messaggio inviato.

Raspberry risponde a raspberry con "fatto" quando raspberry invia un segnale:
"y" se il colore rilevato è il rosso
"n" se il colore rilevato è diverso dal rosso

Arduino risponde sempre con "fatto" (poi questa funzione dovrà essere ampliata)
