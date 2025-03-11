String ricevi();
void invia(String line);
void blink(int pin);

int dt1;
long tp1;
long i = 0;

#define t1 50  // Tempo di lampeggio

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  String msg = ricevi();
  if (msg == "Hello from Raspberry Pi!") {
    invia("Done");
  }
}
