#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <Wire.h>
#include "epd1in54_V2.h"
#include "imagedata.h"
#include "epdpaint.h"
#include <stdio.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"
#include "DFRobot_INA219.h"
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include <math.h>
#include <IRremote.h>

// ========================================
// Pin Definitions
// ========================================
#define PWM_PIN 6
#define TACH_PIN 4
#define POT_PIN A0
#define DISTANCE_PIN A4
#define GLOW_BIT_PIN 5
#define AL_ADDR 0x48
#define POT_MAX_OHMS 50000.0

// New pins for IR + Ring
#define IR_PIN 2
#define RING_PIN 3
#define RING_COUNT 16

// ========================================
#define LED_COUNT 8
Adafruit_NeoPixel glowBit(LED_COUNT, GLOW_BIT_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ring(RING_COUNT, RING_PIN, NEO_GRB + NEO_KHZ800);

// ========================================
Epd epd;
unsigned char image[1024], blankBase[5000];
Paint paint(image, 0, 0);
#define COLORED 0
#define UNCOLORED 1

SparkFun_Ambient_Light light(AL_ADDR);
Adafruit_BME680 bme(&Wire1);
DFRobot_INA219_IIC ina219(&Wire1, INA219_I2C_ADDRESS4);
SFE_MAX1704X fuelGauge(MAX1704X_MAX17048);

float gain = .25, temperature = 0, humidity = 0, pressure = 0, gasResistance = 0;
float batteryVoltage = 0, batterySOC = 0, batteryChangeRate = 0;
float busVoltage = 0, shuntVoltage = 0, current_mA = 0, power_mW = 0, totalEnergy_mWh = 0;
long luxVal = 0;
long distance = 0;
int Integtime = 700, potValue = 0, fanSpeed = 0, blinkInterval = 500, updateCount = 0;
float potOhms = 0;
bool ina219Ready = false, fuelGaugeReady = false, lightReady = false, ledState = false;

unsigned long lastSensorRead = 0, lastEnergyUpdate = 0, lastFanUpdate = 0, lastLEDToggle = 0;
unsigned long lastSmartLEDUpdate = 0, lastEPaperUpdate = 0;
const unsigned long sensorReadInterval = 500;
const unsigned long ePaperInterval = 3000;
const unsigned long fanUpdateInterval = 100;
const unsigned long smartLEDInterval = 100;
const unsigned long fullRefreshCycle = 20;

const unsigned long ringAnimInterval = 40;
unsigned long lastRingAnimUpdate = 0;

// === IR Codes (low-byte NEC commands) ===
#define IR_CODE_POWER      0x45
#define IR_CODE_VOL_UP     0x46
#define IR_CODE_FUNC_STOP  0x47
#define IR_CODE_REW        0x44
#define IR_CODE_PLAY_PAUSE 0x40
#define IR_CODE_FF         0x43
#define IR_CODE_DOWN       0x07
#define IR_CODE_UP         0x15
#define IR_CODE_EQ         0x09
#define IR_CODE_ST_REPT    0x19
#define IR_CODE_0          0x16
#define IR_CODE_1          0x0C
#define IR_CODE_2          0x18
#define IR_CODE_3          0x5E
#define IR_CODE_4          0x08
#define IR_CODE_5          0x1C
#define IR_CODE_6          0x5A
#define IR_CODE_7          0x42
#define IR_CODE_8          0x52
#define IR_CODE_9          0x4A

bool ringOn = true;
uint8_t ringBrightness = 100;
uint8_t ringMode = 0;
uint32_t ringColor = 0xFFFFFF;
uint16_t ringAnimStep = 0;

uint32_t colorRed     = Adafruit_NeoPixel::Color(255, 0,   0);
uint32_t colorGreen   = Adafruit_NeoPixel::Color(0,   255, 0);
uint32_t colorBlue    = Adafruit_NeoPixel::Color(0,   0,   255);
uint32_t colorYellow  = Adafruit_NeoPixel::Color(255, 200, 0);
uint32_t colorCyan    = Adafruit_NeoPixel::Color(0,   255, 255);
uint32_t colorMagenta = Adafruit_NeoPixel::Color(255, 0,   180);
uint32_t colorOrange  = Adafruit_NeoPixel::Color(255, 80,  0);
uint32_t colorPurple  = Adafruit_NeoPixel::Color(160, 0,   200);
uint32_t colorWhite   = Adafruit_NeoPixel::Color(255, 255, 255);

void setupIR() {
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
  Serial.println("IR Receiver initialized.");
}

void handleIRCommand(uint16_t cmd) {
  Serial.print("IR Command Received: 0x");
  Serial.println(cmd, HEX);
  switch (cmd) {
    case IR_CODE_POWER:
      ringOn = !ringOn;
      break;
    case IR_CODE_VOL_UP:
      ringBrightness = (ringBrightness <= 245) ? ringBrightness + 10 : 255;
      ring.setBrightness(ringBrightness);
      break;
    case IR_CODE_DOWN:  // use DOWN for brightness down
      ringBrightness = (ringBrightness >= 10) ? ringBrightness - 10 : 0;
      ring.setBrightness(ringBrightness);
      break;
    case IR_CODE_FUNC_STOP:
      ringMode = (ringMode + 1) % 4;
      break;
    case IR_CODE_PLAY_PAUSE:
      ringMode = 0;
      break;
    case IR_CODE_1: ringColor = colorRed;     ringMode = 0; break;
    case IR_CODE_2: ringColor = colorGreen;   ringMode = 0; break;
    case IR_CODE_3: ringColor = colorBlue;    ringMode = 0; break;
    case IR_CODE_4: ringColor = colorYellow;  ringMode = 0; break;
    case IR_CODE_5: ringColor = colorCyan;    ringMode = 0; break;
    case IR_CODE_6: ringColor = colorMagenta; ringMode = 0; break;
    case IR_CODE_7: ringColor = colorOrange;  ringMode = 0; break;
    case IR_CODE_8: ringColor = colorPurple;  ringMode = 0; break;
    case IR_CODE_9: ringColor = colorWhite;   ringMode = 0; break;
    case IR_CODE_0: ringColor = colorWhite;   ringMode = 0; break;
    case IR_CODE_UP:
      ringColor = Adafruit_NeoPixel::Color(random(0,255), random(0,255), random(0,255));
      ringMode = 0;
      break;
    case IR_CODE_REW:
      ringMode = (ringMode == 0) ? 3 : ringMode - 1;
      break;
    case IR_CODE_FF:
      ringMode = (ringMode + 1) % 4;
      break;
    default:
      break;
  }
}

uint32_t wheel(byte pos) {
  pos = 255 - pos;
  if (pos < 85) {
    return Adafruit_NeoPixel::Color(255 - pos * 3, 0, pos * 3);
  } else if (pos < 170) {
    pos -= 85;
    return Adafruit_NeoPixel::Color(0, pos * 3, 255 - pos * 3);
  } else {
    pos -= 170;
    return Adafruit_NeoPixel::Color(pos * 3, 255 - pos * 3, 0);
  }
}

void updateRingAnimation() {
  if (!ringOn) {
    ring.clear();
    ring.show();
    return;
  }
  switch (ringMode) {
    case 0:
      for (int i = 0; i < RING_COUNT; i++) ring.setPixelColor(i, ringColor);
      ring.show();
      break;
    case 1:
      for (int i = 0; i < RING_COUNT; i++)
        ring.setPixelColor(i, wheel((i * 256 / RING_COUNT + ringAnimStep) & 255));
      ring.show();
      ringAnimStep++;
      break;
    case 2: {
      uint8_t dot = ringAnimStep % RING_COUNT;
      ring.clear();
      ring.setPixelColor(dot, ringColor);
      ring.show();
      ringAnimStep++;
    } break;
    case 3: {
      float phase = (ringAnimStep % 200) / 200.0;
      float s = (sin(phase * 2 * PI) + 1) / 2.0;
      uint8_t r = ((ringColor >> 16) & 0xFF) * s;
      uint8_t g = ((ringColor >> 8)  & 0xFF) * s;
      uint8_t b = ( ringColor        & 0xFF) * s;
      for (int i = 0; i < RING_COUNT; i++) ring.setPixelColor(i, r, g, b);
      ring.show();
      ringAnimStep++;
    } break;
  }
}

// (All your existing functions: scanI2C, readFanControl, readSensors, readDistance, updateSmartLEDs,
// displayLine, displayFullScreen, updateDisplayPartial, printSensorData stay unchanged)
// For brevity they are omitted here—keep them exactly as before.

// Setup and loop modifications: just ensure IR decoding and ring animation calls remain.
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  // (rest of your existing setup code ...)
  glowBit.begin(); glowBit.show(); glowBit.setBrightness(100);
  ring.begin(); ring.setBrightness(ringBrightness); ring.show();
  setupIR();
  // (rest unchanged)
}

void loop() {
  unsigned long currentMillis = millis();
  if (IrReceiver.decode()) {
    uint16_t cmd = IrReceiver.decodedIRData.command;
    handleIRCommand(cmd);
    IrReceiver.resume();
  }
  // (rest of your existing loop logic: fan, smart LEDs, sensors, e-paper)
  if (currentMillis - lastRingAnimUpdate >= ringAnimInterval) {
    lastRingAnimUpdate = currentMillis;
    updateRingAnimation();
  }
}

/*
If using older IRremote (<3.0), replace decode code with legacy version as previously noted.
*/