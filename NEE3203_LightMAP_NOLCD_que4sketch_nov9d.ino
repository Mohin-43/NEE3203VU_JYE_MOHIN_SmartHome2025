#include <Adafruit_NeoPixel.h>

// Sensor pins
const int lightSensorPin = A0;   // Light sensor (phototransistor)
const int distancePin = A1;      // Distance sensor (3-pin)

int lightValue = 0;
long distance = 0;

// NeoPixel setup
#define LED_PIN    12
#define LED_COUNT  16

Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Thresholds
#define PRESENCE_DISTANCE 50  // Person detected within 50cm
#define DARK_THRESHOLD 450    // Room is dark if light < 450

void setup() {
  Serial.begin(9600);
  
  // Initialize NeoPixel
  ring.begin(); 
  ring.show();
  ring.setBrightness(200);
  
  Serial.println("=================================");
  Serial.println("Smart LED System Started");
  Serial.println("=================================");
  Serial.println();
}

void loop() {
  // Read light sensor (phototransistor on A0)
  lightValue = analogRead(lightSensorPin);
  
  // Read distance sensor (3-pin on A1)
  distance = readDistance();
  
  // Display sensor readings in Serial Monitor
  Serial.println("---------------------------------");
  Serial.print("Light Sensor: ");
  Serial.print(lightValue);
  Serial.print(" | Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // LOGIC: Control LEDs based on BOTH sensors
  bool personPresent = (distance > 0 && distance < PRESENCE_DISTANCE);
  bool isDark = (lightValue < DARK_THRESHOLD);
  
  if (personPresent && isDark) {
    // Person detected + Dark room = Turn on LEDs based on light level
    Serial.println("Status: Person Detected + Dark Room");
    int numLEDs = map(lightValue, 0, 1023, 16, 0);
    numLEDs = constrain(numLEDs, 0, 16);
    Serial.print("Action: Lighting ");
    Serial.print(numLEDs);
    Serial.println(" LEDs (yellow-green)");
    controlLEDsByLight(lightValue);
    
  } else if (personPresent && !isDark) {
    // Person detected + Bright room = Just show presence indicator
    Serial.println("Status: Person Detected + Bright Room");
    Serial.println("Action: 4 Blue LEDs (presence indicator)");
    ring.clear();
    for (int i = 0; i < 4; i++) {
      ring.setPixelColor(i, ring.Color(0, 100, 255));  // Blue indicator
    }
    ring.show();
    
  } else if (!personPresent && isDark) {
    // No person + Dark room = Dim night light
    Serial.println("Status: No Person + Dark Room");
    Serial.println("Action: 1 Dim LED (night light)");
    ring.clear();
    ring.setPixelColor(0, ring.Color(50, 50, 0));  // Single dim LED
    ring.show();
    
  } else {
    // No person + Bright room = LEDs off
    Serial.println("Status: No Person + Bright Room");
    Serial.println("Action: All LEDs OFF");
    ring.clear();
    ring.show();
  }
  
  Serial.println();
  delay(1000);
}

// Function to read distance from 3-pin sensor on A1
long readDistance() {
  long duration;
  
  // Send trigger pulse
  pinMode(distancePin, OUTPUT);
  digitalWrite(distancePin, LOW);
  delayMicroseconds(2);
  digitalWrite(distancePin, HIGH);
  delayMicroseconds(10);
  digitalWrite(distancePin, LOW);
  
  // Receive echo
  pinMode(distancePin, INPUT);
  duration = pulseIn(distancePin, HIGH, 30000);  // 30ms timeout
  
  // Convert to distance (cm)
  if (duration == 0) return -1;  // No echo received
  return duration / 58.2;
}

// Function to control LEDs based on light intensity
void controlLEDsByLight(int lightValue) {
  // Map light sensor value to number of LEDs (REVERSED)
  // High light value = bright room = few LEDs
  // Low light value = dark room = many LEDs
  int numLEDs = map(lightValue, 0, 1023, 16, 0);
  
  // Make sure it's within valid range
  numLEDs = constrain(numLEDs, 0, 16);
  
  ring.clear();
  for (int i = 0; i < numLEDs; i++) {
    ring.setPixelColor(i, ring.Color(150, 200, 0));  // Yellow-green
  }
  ring.show();
}