int pwmPin = 9;
int tachPin = 10;
int POTPin = A0;
#define POT_MAX_OHMS 50000.0;

void setup() {
  pinMode(pwmPin, OUTPUT);
  pinMode(tachPin, INPUT_PULLUP);
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

// Ramp Down 10 intervals
/* void loop() {
  analogWrite(pwmPin, 255);  
  Serial.println("Starting at full speed...");
  delay(5000);

  for (int i = 0; i < 10; i++) {
    int duty = map(i, 0, 9, 255, 30);
    analogWrite(pwmPin, duty);

    period = pulseIn(tachPin, HIGH, 1000000); 
    int rpm = (period > 0) ? 60.0 * 1000000.0 / (period * 2.0) : 0;

    Serial.print("Level ");
    Serial.print(i + 1);
    Serial.print(" | PWM ");
    Serial.print(duty);
    Serial.print(" | RPM ");
    Serial.println(rpm);

    delay(1000);  
  }

  Serial.println("Finished cycle.\n");
  delay(5000);
} */

void loop() {

  // Potentiometer Set Up
  int potValue = analogRead(POTPin);
  int FanSpeed = map(potValue, 0, 1023, 0, 255);
int BlinkInterval = map(FanSpeed, 0, 2000, 1000, 50);
  

  digitalWrite(LED_BUILTIN, HIGH);
  delay(BlinkInterval/4);
  digitalWrite(LED_BUILTIN, LOW);
  delay(BlinkInterval/4);
  
  
  // Serial output in ohms
  float potOhms = (potValue / 1023.0) * POT_MAX_OHMS;
    Serial.print("POT Value (Ohms): ");
  Serial.println(potOhms);
  Serial.println("----------------");

analogWrite(pwmPin, FanSpeed);

int period = pulseIn(tachPin, HIGH, 1000000); 
    int rpm = (period > 0) ? 60.0 * 1000000.0 / (period * 2.0) : 0;
    Serial.print(" PWM ");
    Serial.print(FanSpeed);
    Serial.print(" | RPM ");
    Serial.println(rpm);

}
