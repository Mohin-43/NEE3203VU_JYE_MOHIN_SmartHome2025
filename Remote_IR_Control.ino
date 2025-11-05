#include <Arduino.h>
#include <IRremote.hpp> 

#define IR_RECEIVE_PIN 2  
int Red =  8;
int Yellow = 7;
int Green = 6;
const unsigned long period  =1/5;
unsigned long startMillis;
unsigned long currentMillis;
int activeLED = 0;  

// Add these lines at the top:
bool ledStates[3] = {false, false, false};  // Track state of each LED individually

void updateLEDs() {
  digitalWrite(Red,    ledStates[0] ? HIGH : LOW);
  digitalWrite(Green,  ledStates[1] ? HIGH : LOW);
  digitalWrite(Yellow, ledStates[2] ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);

  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  Serial.print(F("Ready to receive IR signals at pin "));
  Serial.println(IR_RECEIVE_PIN);

  pinMode(Red, OUTPUT);
  pinMode(Yellow, OUTPUT);
  pinMode(Green, OUTPUT);
startMillis = millis();
}

void loop() {
  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
      Serial.println(F("Received noise or unknown protocol"));
      IrReceiver.printIRResultRawFormatted(&Serial, true);
      IrReceiver.resume();
    } else {
      IrReceiver.resume();
      IrReceiver.printIRResultShort(&Serial);
      IrReceiver.printIRSendUsage(&Serial);
    }
    Serial.println();

    if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
      Serial.println(F("Repeat received."));
    } else {
      if (IrReceiver.decodedIRData.command == 0x45) {
        Serial.println(F("Received command ON."));
        digitalWrite(Red, HIGH);
        digitalWrite(Green, HIGH);
        digitalWrite(Yellow, HIGH);

        // Keep ledStates in sync with all ON
        ledStates[0] = ledStates[1] = ledStates[2] = true;

      } else if (IrReceiver.decodedIRData.command == 0x44) {
        Serial.println(F("Received command PAUSE."));
        digitalWrite(Red, LOW);
        digitalWrite(Green, LOW);
        digitalWrite(Yellow, LOW);

        // Keep ledStates in sync with all OFF
        ledStates[0] = ledStates[1] = ledStates[2] = false;

      } else if (IrReceiver.decodedIRData.command == 0x07) {  // EQ button
        Serial.println(F("EQ pressed — cycling LED"));
        activeLED = (activeLED + 1) % 3;
        digitalWrite(Red,    activeLED == 0 ? HIGH : LOW);
        digitalWrite(Green,  activeLED == 1 ? HIGH : LOW);
        digitalWrite(Yellow, activeLED == 2 ? HIGH : LOW);
      }
      else if (IrReceiver.decodedIRData.command == 0x0C) {  
        Serial.print(F("Active LED On: "));
        Serial.println(activeLED);
        ledStates[activeLED] = true;
        updateLEDs();
      }
      else if (IrReceiver.decodedIRData.command == 0x18) {  
        Serial.print(F("Active LED Off: "));
        Serial.println(activeLED);
        ledStates[activeLED] = false;
        updateLEDs();
        }
        else if (IrReceiver.decodedIRData.command == 0x5E) {  
        Serial.print(F("Active LED Blink at 5Hz: "));
        currentMillis = millis();
            if (currentMillis - startMillis >= period){ digitalWrite(activeLED , !digitalRead(activeLED));
            startMillis = currentMillis;}
        }
        else if (IrReceiver.decodedIRData.command == 0x40) {  
        Serial.print(F("Active LED Blink Faster: "));
        Serial.println(activeLED);
        ledStates[activeLED] = false;
        updateLEDs();
        }
        else if (IrReceiver.decodedIRData.command == 0x43) {  
        Serial.print(F("Active LED Blink Slower: "));
        Serial.println(activeLED);
        ledStates[activeLED] = false;
        updateLEDs();
        }
    }
  }
}
