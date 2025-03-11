void setup() {
  Serial.begin(300);
}

void loop() {
  Serial.println(map(analogRead(A0), 0, 1024, 0, 255));
}
