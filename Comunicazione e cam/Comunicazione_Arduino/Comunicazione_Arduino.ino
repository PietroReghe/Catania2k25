String ricevi();
void invia(String line);

int dt1, tp1 = 0;

#define t1 50

#define giallo 4
#define rosso 3
#define verde 2

long i = 0;

void setup() {
  Serial.begin(9600);
  pinMode(giallo, OUTPUT);
  pinMode(rosso, OUTPUT);
  pinMode(verde, OUTPUT);
}

String ricevuto = "";

void loop() {
  
  invia("rileva");

  ricevuto = ricevi();
  invia(ricevuto);

  if(ricevuto == "rosso"){
    digitalWrite(rosso, HIGH);
    digitalWrite(giallo, LOW);
    digitalWrite(verde, LOW);
    invia("rosso");
  } else if (ricevuto == "giallo") {
    digitalWrite(rosso, LOW);
    digitalWrite(giallo, HIGH);
    digitalWrite(verde, LOW);
    invia("giallo");
  } else if (ricevuto == "verde") {
    digitalWrite(rosso, LOW);
    digitalWrite(giallo, LOW);
    digitalWrite(verde, HIGH);
    invia("verde");
  } else if (ricevuto == "blu") {
    digitalWrite(rosso, LOW);
    digitalWrite(giallo, LOW);
    digitalWrite(verde, LOW);
    invia("blu");
  }

  delay(10000);
}
