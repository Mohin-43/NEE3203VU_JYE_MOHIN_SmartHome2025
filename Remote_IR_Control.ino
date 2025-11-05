#include <Arduino.h>
#include <IRremote.hpp>

#define IR_RECEIVE_PIN 2

// LED pins: {Red, Yellow, Green}
const int ledPins[] = {8, 7, 6};
const int NUM_LEDS = 3;
int activeLED = 0;

const unsigned long DEFAULT_BLINK_PERIOD = 300;

// --- Blinker Class (Robust Version) ---
class Blinker {
private:
  int ledPin;
  unsigned long blinkPeriod;
  unsigned long lastToggleTime;
  bool isBlinking;
  bool isStaticOn;
  bool ledState;  // Track the current actual pin state

public:
  Blinker(int pin) {
    ledPin = pin;
    pinMode(ledPin, OUTPUT);
    isBlinking = false;
    isStaticOn = false;
    ledState = LOW;
    lastToggleTime = millis();
    blinkPeriod = DEFAULT_BLINK_PERIOD;
    digitalWrite(ledPin, ledState);
  }

  void setStatic(bool on) {
    isBlinking = false;
    isStaticOn = on;
    ledState = on ? HIGH : LOW;
    digitalWrite(ledPin, ledState);
  }

  void startBlink(unsigned long period) {
    isBlinking = true;
    blinkPeriod = period;
    lastToggleTime = millis();
    ledState = HIGH;
    digitalWrite(ledPin, ledState);
  }

  void stopBlink() {
    isBlinking = false;
    digitalWrite(ledPin, isStaticOn ? HIGH : LOW);
  }

  void toggleBlink() {
    // Toggle blinking ON/OFF while preserving last static state
    if (isBlinking) {
      stopBlink();
    } else {
      startBlink(blinkPeriod);
    }
  }

  void adjustSpeed(bool faster) {
    if (!isBlinking) return;
    if (faster) blinkPeriod = max(50UL, blinkPeriod / 2);
    else blinkPeriod = min(1000UL, blinkPeriod * 2);
  }

  void update() {
    if (isBlinking) {
      unsigned long now = millis();
      if (now - lastToggleTime >= blinkPeriod) {
        lastToggleTime = now;
        ledState = !ledState;
        digitalWrite(ledPin, ledState);
      }
    }
  }
};

// --- Create LED Objects ---
Blinker leds[NUM_LEDS] = {
  Blinker(ledPins[0]),
  Blinker(ledPins[1]),
  Blinker(ledPins[2])
};

void setup() {
  Serial.begin(115200);
  Serial.println(F("START: Robust Multi-LED Blinker with IR Control"));
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}

void loop() {
  if (IrReceiver.decode()) {
    IrReceiver.resume();
    if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      uint16_t command = IrReceiver.decodedIRData.command;
      Serial.print(F("Received command: 0x"));
      Serial.println(command, HEX);

      // --- Command Mappings ---
      if (command == 0x45) { // "ON" - Turn all ON (static)
        for (int i = 0; i < NUM_LEDS; i++) leds[i].setStatic(true);

      } else if (command == 0x44) { // "OFF" - Turn all OFF
        for (int i = 0; i < NUM_LEDS; i++) leds[i].setStatic(false);

      } else if (command == 0x07) { // "EQ" - Cycle active LED
        activeLED = (activeLED + 1) % NUM_LEDS;
        Serial.print(F("Active LED changed to index "));
        Serial.println(activeLED);

      } else if (command == 0x0C) { // "PLAY" - Active LED ON
        leds[activeLED].setStatic(true);

      } else if (command == 0x18) { // "DOWN" - Active LED OFF
        leds[activeLED].setStatic(false);

      } else if (command == 0x5E) { // "UP" - Toggle Blink on active LED
        leds[activeLED].toggleBlink();

      } else if (command == 0x40) { // "VOL-" - Blink Faster
        leds[activeLED].adjustSpeed(true);

      } else if (command == 0x43) { // "VOL+" - Blink Slower
        leds[activeLED].adjustSpeed(false);
      }
    }
  }

  // Continuous non-blocking update for all LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].update();
  }
}
