

int sensorValue = 0;
float sensorValueDigital;
void setup() {
  Serial.begin(9600);

}

void loop() {
  sensorValue = analogRead(A0); // read sensor
  sensorValueDigital = sensorValue*(5.0/1023.0);
  Serial.println(sensorValueDigital);  // send to Serial Monitor
  delay(500);
}