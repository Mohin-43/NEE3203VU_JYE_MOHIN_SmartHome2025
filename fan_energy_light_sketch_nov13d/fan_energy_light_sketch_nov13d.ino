// ========================================
// Integrated Smart LED & Sensor Monitor System
// ========================================

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

// ========================================
// Pin Definitions
// ========================================
#define PWM_PIN 6              // Fan control
#define TACH_PIN 4             // Fan tachometer
#define POT_PIN A0             // Potentiometer
#define DISTANCE_PIN A4        // Distance sensor (3-pin ultrasonic)
#define GLOW_BIT_PIN 5         // NeoPixel Glow Bit Light 1x8
#define AL_ADDR 0x48           // Ambient light sensor I2C address
#define POT_MAX_OHMS 50000.0

// ========================================
// NeoPixel Glow Bit Setup (1x8)
// ========================================
#define LED_COUNT 8            // Changed from 16 to 8
Adafruit_NeoPixel glowBit(LED_COUNT, GLOW_BIT_PIN, NEO_GRB + NEO_KHZ800);

// Smart LED Thresholds
#define PRESENCE_DISTANCE 50   // Person detected within 50cm
#define DARK_THRESHOLD 100     // Room is dark if light < 100 lux (adjusted for VEML6030)

// ========================================
// E-paper Setup
// ========================================
Epd epd;
unsigned char image[1024], blankBase[5000];
Paint paint(image, 0, 0);
#define COLORED 0
#define UNCOLORED 1

// ========================================
// Sensor Objects
// ========================================
SparkFun_Ambient_Light light(AL_ADDR);
Adafruit_BME680 bme(&Wire1);
DFRobot_INA219_IIC ina219(&Wire1, INA219_I2C_ADDRESS4);
SFE_MAX1704X fuelGauge(MAX1704X_MAX17048);

// ========================================
// Sensor Data Variables
// ========================================
float gain = .25, temperature = 0, humidity = 0, pressure = 0, gasResistance = 0;
float batteryVoltage = 0, batterySOC = 0, batteryChangeRate = 0;
float busVoltage = 0, shuntVoltage = 0, current_mA = 0, power_mW = 0, totalEnergy_mWh = 0;
long luxVal = 0;
long distance = 0;
int Integtime = 700, potValue = 0, fanSpeed = 0, blinkInterval = 500, updateCount = 0;
float potOhms = 0;
bool ina219Ready = false, fuelGaugeReady = false, ledState = false;

// ========================================
// Timing Variables
// ========================================
unsigned long lastUpdate = 0, lastEnergyUpdate = 0, lastFanUpdate = 0, lastLEDToggle = 0;
unsigned long lastSmartLEDUpdate = 0;
const unsigned long updateInterval = 1000, fanUpdateInterval = 100, smartLEDInterval = 1000;

// ========================================
// Setup Function
// ========================================
void setup() {
  Serial.begin(115200);
  while(!Serial && millis() < 3000) delay(10);
  
  Serial.println("========================================");
  Serial.println("Integrated Smart LED & Sensor Monitor");
  Serial.println("========================================");
  
  // Pin setup
  pinMode(PWM_PIN, OUTPUT);
  pinMode(TACH_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(GLOW_BIT_PIN, OUTPUT);
  Serial.println("Pins initialized");
  
  // Initialize NeoPixel Glow Bit (1x8)
  glowBit.begin(); 
  glowBit.show();
  glowBit.setBrightness(200);
  Serial.println("NeoPixel Glow Bit (1x8) initialized");
  
  // Initialize I2C
  Wire1.begin();
  delay(500);
  Serial.println("\nInitializing sensors...");
  scanI2C();
  
  // Initialize VEML6030 Light Sensor
  if(light.begin(Wire1)) {
    Serial.println("✓ VEML6030 Light Sensor ready!"); 
    light.setGain(gain);
    light.setIntegTime(Integtime);
  } else {
    Serial.println("✗ VEML6030 failed!");
  }
  
  // Initialize BME680 Environmental Sensor
  if (!bme.begin()) {
    Serial.println("✗ BME680 failed!");
  } else {
    Serial.println("✓ BME680 Environmental Sensor ready!");
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
  }
  
  // Initialize INA219 Power Monitor
  Serial.println("Initializing INA219 Power Monitor...");
  int retryCount = 0;
  while(ina219.begin() != true && retryCount < 5) {
    Serial.print("INA219 failed! Attempt: ");
    Serial.println(++retryCount);
    delay(1000);
  }
  if(retryCount < 5) {
    Serial.println("✓ INA219 Power Monitor ready!");
    ina219.linearCalibrate(1000, 1000);
    ina219Ready = true;
  } else {
    Serial.println("✗ INA219 init failed after 5 attempts.");
  }
  
  // Initialize MAX17048 Battery Fuel Gauge
  Serial.println("Initializing MAX17048 Battery Fuel Gauge...");
  if (fuelGauge.begin(Wire1)) {
    Serial.println("✓ MAX17048 Battery Monitor ready!");
    fuelGaugeReady = true;
    fuelGauge.quickStart();
    delay(200);
    fuelGauge.setThreshold(10);
  } else {
    Serial.println("✗ MAX17048 failed!");
  }
  
  // E-paper setup
  Serial.println("\nInitializing e-Paper display...");
  epd.LDirInit();
  epd.Clear();
  readSensors();
  readFanControl();
  displayFullScreen();
  delay(2000);
  
  Serial.println("Switching to partial update mode...");
  epd.HDirInit();
  memset(blankBase, 0xFF, sizeof(blankBase));
  epd.DisplayPartBaseImage(blankBase);
  
  lastEnergyUpdate = lastFanUpdate = lastLEDToggle = lastSmartLEDUpdate = millis();
  
  Serial.println("\n========================================");
  Serial.println("System Ready!");
  Serial.println("Smart LED System Active");
  Serial.println("Battery backup enabled");
  Serial.println("========================================\n");
}

// ========================================
// Main Loop
// ========================================
void loop() {
  unsigned long currentMillis = millis();
  
  // Fast fan control update
  if (currentMillis - lastFanUpdate >= fanUpdateInterval) {
    lastFanUpdate = currentMillis;
    readFanControl();
    analogWrite(PWM_PIN, fanSpeed);
  }
  
  // LED blink based on fan speed
  if (currentMillis - lastLEDToggle >= blinkInterval) {
    lastLEDToggle = currentMillis;
    digitalWrite(LED_BUILTIN, ledState = !ledState);
  }
  
  // Smart LED control update (independent timing)
  if (currentMillis - lastSmartLEDUpdate >= smartLEDInterval) {
    lastSmartLEDUpdate = currentMillis;
    updateSmartLEDs();
  }
  
  // Slow sensor/display update
  if (currentMillis - lastUpdate >= updateInterval) {
    lastUpdate = currentMillis;
    updateCount++;
    readSensors();
    
    if (ina219Ready) {
      totalEnergy_mWh += (power_mW * (currentMillis - lastEnergyUpdate) / 3600000.0);
      lastEnergyUpdate = currentMillis;
    }
    
    if (updateCount >= 20) {
      Serial.println("Periodic full refresh...");
      epd.LDirInit();
      epd.Clear();
      displayFullScreen();
      delay(1000);
      epd.HDirInit();
      epd.DisplayPartBaseImage(blankBase);
      updateCount = 0;
    } else {
      updateDisplayPartial();
    }
    
    printSensorData();
  }
}

// ========================================
// Smart LED Control Functions
// ========================================

// Function to read distance from 3-pin sensor on A4
long readDistance() {
  int raw = analogRead(DISTANCE_PIN);
  if (raw < 10) return -1;  // too low = out of range
  float voltage = raw * (5.0 / 1023.0);
  long distance = 27.86 * pow(voltage, -1.15); // empirical formula for GP2Y0A21YK0F
  return distance;
}

// Main Smart LED update function
void updateSmartLEDs() {
  // Read distance sensor
  distance = readDistance();
  
  // luxVal is already updated in readSensors()
  
  // Display sensor readings in Serial Monitor
  Serial.println("----- Smart LED Status -----");
  Serial.print("Light Sensor: ");
  Serial.print(luxVal);
  Serial.print(" lux | Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // LOGIC: Control LEDs based on BOTH sensors
  bool personPresent = (distance > 0 && distance < PRESENCE_DISTANCE);
  bool isDark = (luxVal < DARK_THRESHOLD);
  
  if (personPresent && isDark) {
    // Person detected + Dark room = Turn on LEDs based on light level
    Serial.println("Status: Person Detected + Dark Room");
    int numLEDs = map(luxVal, 0, 500, 8, 0);  // Adjusted for 8 LEDs
    numLEDs = constrain(numLEDs, 0, 8);
    Serial.print("Action: Lighting ");
    Serial.print(numLEDs);
    Serial.println(" LEDs (yellow-green)");
    controlLEDsByLight(luxVal);
    
  } else if (personPresent && !isDark) {
    // Person detected + Bright room = Just show presence indicator
    Serial.println("Status: Person Detected + Bright Room");
    Serial.println("Action: 2 Blue LEDs (presence indicator)");  // Changed from 4 to 2
    glowBit.clear();
    for (int i = 0; i < 2; i++) {  // Changed from 4 to 2 LEDs
      glowBit.setPixelColor(i, glowBit.Color(0, 100, 255));  // Blue indicator
    }
    glowBit.show();
    
  } else if (!personPresent && isDark) {
    // No person + Dark room = Dim night light
    Serial.println("Status: No Person + Dark Room");
    Serial.println("Action: 1 Dim LED (night light)");
    glowBit.clear();
    glowBit.setPixelColor(0, glowBit.Color(50, 50, 0));  // Single dim LED
    glowBit.show();
    
  } else {
    // No person + Bright room = LEDs off
    Serial.println("Status: No Person + Bright Room");
    Serial.println("Action: All LEDs OFF");
    glowBit.clear();
    glowBit.show();
  }
  Serial.println();
}

// Function to control LEDs based on light intensity (lux)
void controlLEDsByLight(long luxValue) {
  // Map lux value to number of LEDs (REVERSED)
  // High lux = bright room = few LEDs
  // Low lux = dark room = many LEDs
  int numLEDs = map(luxValue, 0, 500, 8, 0);  // Changed from 16 to 8
  
  // Make sure it's within valid range
  numLEDs = constrain(numLEDs, 0, 8);  // Changed from 16 to 8
  
  glowBit.clear();
  for (int i = 0; i < numLEDs; i++) {
    glowBit.setPixelColor(i, glowBit.Color(150, 200, 0));  // Yellow-green
  }
  glowBit.show();
}

// ========================================
// Sensor Reading Functions
// ========================================

void readFanControl() {
  potValue = analogRead(POT_PIN);
  potOhms = (potValue / 1023.0) * POT_MAX_OHMS;
  fanSpeed = map(potValue, 0, 1023, 0, 255);
  blinkInterval = constrain(map(fanSpeed, 0, 255, 1000, 50), 50, 1000);
}

void readSensors() {
  // Read ambient light sensor (VEML6030)
  luxVal = light.readLight();
  
  // Read environmental sensor (BME680)
  if (bme.performReading()) {
    temperature = bme.temperature;
    humidity = bme.humidity;
    pressure = bme.pressure / 100.0;
    gasResistance = bme.gas_resistance / 1000.0;
  }
  
  // Read battery fuel gauge
  if (fuelGaugeReady) {
    batteryVoltage = fuelGauge.getVoltage();
    batterySOC = fuelGauge.getSOC();
    batteryChangeRate = fuelGauge.getChangeRate();
    if (fuelGauge.getAlert()) {
      Serial.println("⚠️  WARNING: Battery critically low!");
    }
  }
  
  // Read power monitor
  if (ina219Ready) {
    busVoltage = ina219.getBusVoltage_V();
    shuntVoltage = ina219.getShuntVoltage_mV();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    if (busVoltage < 0 || busVoltage > 32) busVoltage = 0;
  }
}

// ========================================
// Display Functions
// ========================================

void displayLine(int &yPos, const char* text, bool partial = false) {
  paint.Clear(UNCOLORED);
  paint.DrawStringAt(5, 0, text, &Font12, COLORED);
  if (partial) {
    epd.SetFrameMemoryPartial(paint.GetImage(), 0, yPos, paint.GetWidth(), paint.GetHeight());
  } else {
    epd.SetFrameMemory(paint.GetImage(), 0, yPos, paint.GetWidth(), paint.GetHeight());
  }
  yPos += 16;
}

void displayFullScreen() {
  char buf[30];
  int yPos = 0;
  
  // Title
  paint.SetWidth(200);
  paint.SetHeight(20);
  paint.Clear(COLORED);
  paint.DrawStringAt(5, 2, "Smart LED Monitor", &Font12, UNCOLORED);
  epd.SetFrameMemory(paint.GetImage(), 0, yPos, paint.GetWidth(), paint.GetHeight());
  yPos = 22;
  
  paint.SetHeight(16);
  sprintf(buf, "Temp: %.1fC", temperature); displayLine(yPos, buf);
  sprintf(buf, "Humid: %.1f%%", humidity); displayLine(yPos, buf);
  sprintf(buf, "Press: %.0fhPa", pressure); displayLine(yPos, buf);
  sprintf(buf, "Light: %ldLux", luxVal); displayLine(yPos, buf);
  sprintf(buf, "Dist: %ldcm", distance); displayLine(yPos, buf);
  
  yPos += 2; displayLine(yPos, "--- Fan ---"); 
  sprintf(buf, "Speed: %d%%", map(fanSpeed, 0, 255, 0, 100)); displayLine(yPos, buf);
  sprintf(buf, potOhms >= 1000 ? "Pot: %.1fkOhm" : "Pot: %.0fOhm", 
          potOhms >= 1000 ? potOhms/1000.0 : potOhms); 
  displayLine(yPos, buf);
  
  yPos += 2; displayLine(yPos, "--- Power ---");
  
  if (fuelGaugeReady) {
    sprintf(buf, "Batt: %.1f%%", batterySOC); displayLine(yPos, buf);
    sprintf(buf, "BatV: %.2fV", batteryVoltage); displayLine(yPos, buf);
  }
  if (ina219Ready) {
    sprintf(buf, "Curr: %.1fmA", current_mA); displayLine(yPos, buf);
    sprintf(buf, "Powr: %.1fmW", power_mW); displayLine(yPos, buf);
  }
  
  epd.DisplayFrame();
  Serial.println("Full frame displayed");
}

void updateDisplayPartial() {
  char buf[30];
  int yPos = 22;
  paint.SetWidth(200);
  paint.SetHeight(16);
  
  sprintf(buf, "Temp: %.1fC", temperature); displayLine(yPos, buf, true);
  sprintf(buf, "Humid: %.1f%%", humidity); displayLine(yPos, buf, true);
  sprintf(buf, "Press: %.0fhPa", pressure); displayLine(yPos, buf, true);
  sprintf(buf, "Light: %ldLux", luxVal); displayLine(yPos, buf, true);
  sprintf(buf, "Dist: %ldcm", distance); displayLine(yPos, buf, true);
  
  yPos += 34;
  sprintf(buf, "Speed: %d%%", map(fanSpeed, 0, 255, 0, 100)); displayLine(yPos, buf, true);
  sprintf(buf, potOhms >= 1000 ? "Pot: %.1fkOhm" : "Pot: %.0fOhm", 
          potOhms >= 1000 ? potOhms/1000.0 : potOhms);
  displayLine(yPos, buf, true);
  
  yPos += 34;
  
  if (fuelGaugeReady) {
    sprintf(buf, "Batt: %.1f%%", batterySOC); displayLine(yPos, buf, true);
    sprintf(buf, "BatV: %.2fV", batteryVoltage); displayLine(yPos, buf, true);
  }
  if (ina219Ready) {
    sprintf(buf, "Curr: %.1fmA", current_mA); displayLine(yPos, buf, true);
    sprintf(buf, "Powr: %.1fmW", power_mW); displayLine(yPos, buf, true);
  }
  
  epd.DisplayPartFrame();
}

void printSensorData() {
  Serial.println("========================================");
  Serial.println("=== Environmental Sensors ===");
  Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
  Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" hPa");
  Serial.print("Gas Resistance: "); Serial.print(gasResistance); Serial.println(" KOhms");
  Serial.print("Ambient Light: "); Serial.print(luxVal); Serial.println(" Lux");
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");
  
  Serial.println("\n=== Fan Control ===");
  Serial.print("Potentiometer: "); Serial.print(potOhms); Serial.println(" Ohms");
  Serial.print("Fan Speed: "); Serial.print(map(fanSpeed, 0, 255, 0, 100)); 
  Serial.print("% (PWM: "); Serial.print(fanSpeed); Serial.println(")");
  
  Serial.println("\n=== Power Monitoring ===");
  if (fuelGaugeReady) {
    Serial.print("Battery Voltage: "); Serial.print(batteryVoltage); Serial.println(" V");
    Serial.print("Battery SOC: "); Serial.print(batterySOC); Serial.println(" %");
    Serial.print("Battery Change Rate: "); Serial.print(batteryChangeRate); Serial.println(" %/hr");
  }
  if (ina219Ready) {
    Serial.print("Bus Voltage: "); Serial.print(busVoltage); Serial.println(" V");
    Serial.print("Current Draw: "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power Consumption: "); Serial.print(power_mW); Serial.println(" mW");
    Serial.print("Total Energy Used: ");
    if (totalEnergy_mWh < 1000) {
      Serial.print(totalEnergy_mWh); Serial.println(" mWh");
    } else {
      Serial.print(totalEnergy_mWh / 1000.0); Serial.println(" Wh");
    }
  }
  Serial.println("========================================\n");
}

void scanI2C() {
  int nDevices = 0;
  Serial.println("Scanning I2C bus (Wire1 - QWIIC)...");
  for(byte address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    if (Wire1.endTransmission() == 0) {
      Serial.print("  Device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  if (nDevices == 0) {
    Serial.println("  No I2C devices found!\n");
  } else {
    Serial.print("  "); Serial.print(nDevices); Serial.println(" device(s) found.\n");
  }
}