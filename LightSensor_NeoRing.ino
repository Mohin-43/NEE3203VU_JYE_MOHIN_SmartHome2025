#include <Adafruit_NeoPixel.h>

// --- Pins ---
const int lightSensorPin = A0;

// --- NeoPixel ---
#define LED_PIN    12
#define LED_COUNT  16
Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- Thresholds ---
#define DARK_THRESHOLD 7   // anything >8 considered “dark” for your room

// --- Variables ---
int lightValue = 0;
int currentLEDs = 0;
unsigned long lastUpdate = 0;
const int updateInterval = 5;  // ms between LED steps

void setup() {
  Serial.begin(9600);
  ring.begin();
  ring.show();
  ring.setBrightness(20);


}

void loop() {
  // --- Read light sensor ---
  lightValue = analogRead(lightSensorPin);

  // --- Debug ---
  Serial.println("---------------------------------");
  Serial.print("Light Raw: "); Serial.println(lightValue);

  // --- Logic ---
  if (lightValue > DARK_THRESHOLD) {
    // Dark room → LEDs respond gradually
    int targetLEDs = map(lightValue, 7, 15, 0, LED_COUNT);
    targetLEDs = constrain(targetLEDs, 0, LED_COUNT);
    smoothLEDUpdate(targetLEDs, ring.Color(150, 200, 0));
  } else {
    // Bright room → just 2 blue LEDs
    showBlueIndicator();
  }

  delay(100);
}

// --- Functions ---

void smoothLEDUpdate(int targetLEDs, uint32_t color) {
  unsigned long now = millis();
  if (now - lastUpdate < updateInterval) return;
  lastUpdate = now;

  if (currentLEDs < targetLEDs) currentLEDs++;
  else if (currentLEDs > targetLEDs) currentLEDs--;

  ring.clear();
  for (int i = 0; i < currentLEDs; i++) {
    ring.setPixelColor(i, color);
  }
  ring.show();
}

// Bright room indicator → only 2 blue LEDs
void showBlueIndicator() {
  ring.clear();
  for (int i = 0; i < 2; i++) {
    ring.setPixelColor(i, ring.Color(0, 100, 255));
  }
  ring.show();
  currentLEDs = 2;  // update internal counter
}
