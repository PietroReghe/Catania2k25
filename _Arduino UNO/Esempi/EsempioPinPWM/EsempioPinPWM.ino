#define pin LED_BUILTIN
#define pin2 2

int fPer = 30;
#define sens 100 // sens > 100

void setup() {
  Serial.begin(9600);
  pinMode(pin, OUTPUT);
  fPer = constrain(map(fPer, 0, 100, 0, sens), 0, sens);
  Serial.print(fPer);
}

void loop() {
  digitalWrite(pin, 1);
  digitalWrite(pin2, 1);
  delayMicroseconds(fPer);
  digitalWrite(pin, 0);
  digitalWrite(pin2, 0);
  delayMicroseconds(sens - fPer);
}
