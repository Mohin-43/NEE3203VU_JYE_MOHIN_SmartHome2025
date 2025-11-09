void loop() {
  sensorValue = analogRead(A0);
  
  // Display sensor value in Serial Monitor
  Serial.print("Sensor Value: ");
  Serial.println(sensorValue);
  
  // REVERSE map: high sensor = few LEDs, low sensor = many LEDs
  int numLEDs = map(sensorValue, 0, 1023, 16, 0);  // Notice: 16 to 0 (reversed!)
  
  // Display LED count in Serial Monitor
  Serial.print("LEDs Active: ");
  Serial.print(numLEDs);
  Serial.println("/16");
  Serial.println("---------------------------------");
  
  ring.clear();
  for (int i = 0; i < numLEDs; i++) {
    ring.setPixelColor(i, ring.Color(150, 200, 0));
  }
  ring.show();
  
  delay(1000);
}