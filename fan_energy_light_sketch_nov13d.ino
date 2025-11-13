// ========================================
// Integrated Sensor Monitor & Fan Controller with Smart LED
// ========================================

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
#include <Adafruit_NeoPixel.h>

// Pin Definitions
#define PWM_PIN 6
#define POT_PIN A0
#define DISTANCE_PIN A4     // Distance sensor (3-pin ultrasonic) - ANALOG PIN
#define LED_PIN 5           // GlowBit Stick 1x8
#define LED_COUNT 8         // 8 LEDs on GlowBit Stick
#define AL_ADDR 0x48
#define POT_MAX_OHMS 50000.0

// Smart LED Thresholds
#define PRESENCE_DISTANCE 50  // Person detected within 50cm
#define DARK_THRESHOLD 100    // Room is dark if lux < 100

// E-paper setup
Epd epd;
unsigned char image[1024], blankBase[5000];
Paint paint(image, 0, 0);
#define COLORED 0
#define UNCOLORED 1

// Sensor objects
SparkFun_Ambient_Light light(AL_ADDR);
Adafruit_BME680 bme(&Wire1);
DFRobot_INA219_IIC ina219(&Wire1, INA219_I2C_ADDRESS4);
SFE_MAX1704X fuelGauge(MAX1704X_MAX17048);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Sensor data
float gain = .25, temperature = 0, humidity = 0, pressure = 0, gasResistance = 0;
float batteryVoltage = 0, batterySOC = 0, batteryChangeRate = 0;
float busVoltage = 0, shuntVoltage = 0, current_mA = 0, power_mW = 0, totalEnergy_mWh = 0;
long luxVal = 0;
int Integtime = 700, potValue = 0, fanSpeed = 0, blinkInterval = 500, updateCount = 0;
float potOhms = 0;
long distance = 0;
unsigned long lastPulseDuration = 0; // For debugging
bool ina219Ready = false, fuelGaugeReady = false, ledState = false;

// Timing
unsigned long lastUpdate = 0, lastEnergyUpdate = 0, lastFanUpdate = 0, lastLEDToggle = 0, lastLEDUpdate = 0;
const unsigned long updateInterval = 1000, fanUpdateInterval = 100, ledUpdateInterval = 500;

void setup() {
  Serial.begin(115200);
  while(!Serial && millis() < 3000) delay(10);
  Serial.println("========================================\nIntegrated Sensor Monitor & Fan Controller with Smart LED\n========================================");
  
  // Pin setup
  pinMode(PWM_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  // Distance pin (A4) configured dynamically in readDistance()
  
  Serial.println("Fan control pins initialized");
  Serial.print("Distance sensor on pin: A4 (analog pin ");
  Serial.print(DISTANCE_PIN);
  Serial.println(")");
  Serial.print("GlowBit Stick on pin: ");
  Serial.println(LED_PIN);
  
  // Initialize NeoPixel LED strip
  strip.begin();
  strip.show();
  strip.setBrightness(200);
  Serial.println("GlowBit Stick 1x8 initialized on pin 5");
  
  Wire1.begin();
  delay(500);
  Serial.println("\nInitializing sensors...");
  scanI2C();
  
  // Initialize all sensors
  if(light.begin(Wire1)) {
    Serial.println("✓ VEML6030 Light Sensor ready!"); 
    light.setGain(gain);
    light.setIntegTime(Integtime);
  } else Serial.println("✗ VEML6030 failed!");
  
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
  } else Serial.println("✗ INA219 init failed after 5 attempts.");
  
  Serial.println("Initializing MAX17048 Battery Fuel Gauge...");
  if (fuelGauge.begin(Wire1)) {
    Serial.println("✓ MAX17048 Battery Monitor ready!");
    fuelGaugeReady = true;
    fuelGauge.quickStart();
    delay(200);
    fuelGauge.setThreshold(10);
  } else Serial.println("✗ MAX17048 failed!");
  
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
  
  lastEnergyUpdate = lastFanUpdate = lastLEDToggle = lastLEDUpdate = millis();
  Serial.println("\n========================================\nSystem Ready!\nBattery backup enabled\n========================================\n");
  
  // Test distance sensor with DETAILED debugging
  Serial.println("========================================");
  Serial.println("DISTANCE SENSOR DIAGNOSTICS");
  Serial.println("========================================");
  Serial.println("Testing distance sensor on pin A4...");
  Serial.println("Place an object 10-100cm away from sensor");
  Serial.println("----------------------------------------");
  
  for(int i = 0; i < 10; i++) {
    long testDist = readDistance();
    Serial.print("Reading #");
    Serial.print(i+1);
    Serial.print(" | Duration: ");
    Serial.print(lastPulseDuration);
    Serial.print(" µs | Distance: ");
    
    if (testDist >= 0) {
      Serial.print(testDist);
      Serial.println(" cm ✓");
    } else if (lastPulseDuration == 0) {
      Serial.println("NO ECHO - Check wiring!");
    } else if (testDist == -1) {
      Serial.print("OUT OF RANGE (");
      Serial.print(lastPulseDuration / 58.2);
      Serial.println(" cm)");
    }
    delay(300);
  }
  
  Serial.println("========================================\n");
}

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
  
  // Smart LED control based on presence and light level
  if (currentMillis - lastLEDUpdate >= ledUpdateInterval) {
    lastLEDUpdate = currentMillis;
    distance = readDistance();
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
    } else updateDisplayPartial();
    
    printSensorData();
  }
}

void readFanControl() {
  potValue = analogRead(POT_PIN);
  potOhms = (potValue / 1023.0) * POT_MAX_OHMS;
  fanSpeed = map(potValue, 0, 1023, 0, 255);
  blinkInterval = constrain(map(fanSpeed, 0, 255, 1000, 50), 50, 1000);
}

void readSensors() {
  luxVal = light.readLight();
  if (bme.performReading()) {
    temperature = bme.temperature;
    humidity = bme.humidity;
    pressure = bme.pressure / 100.0;
    gasResistance = bme.gas_resistance / 1000.0;
  }
  if (fuelGaugeReady) {
    batteryVoltage = fuelGauge.getVoltage();
    batterySOC = fuelGauge.getSOC();
    batteryChangeRate = fuelGauge.getChangeRate();
    if (fuelGauge.getAlert()) Serial.println("⚠️  WARNING: Battery critically low!");
  }
  if (ina219Ready) {
    busVoltage = ina219.getBusVoltage_V();
    shuntVoltage = ina219.getShuntVoltage_mV();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    if (busVoltage < 0 || busVoltage > 32) busVoltage = 0;
  }
}

// Read distance from 3-pin ultrasonic sensor on ANALOG pin A4 with detailed debugging
long readDistance() {
  unsigned long duration;
  
  // Send trigger pulse
  pinMode(DISTANCE_PIN, OUTPUT);
  digitalWrite(DISTANCE_PIN, LOW);
  delayMicroseconds(5);  // Increased stabilization time
  digitalWrite(DISTANCE_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(DISTANCE_PIN, LOW);
  
  // Receive echo
  pinMode(DISTANCE_PIN, INPUT);
  duration = pulseIn(DISTANCE_PIN, HIGH, 50000);  // Increased timeout to 50ms
  
  // Store duration for debugging
  lastPulseDuration = duration;
  
  // Convert to distance (cm)
  if (duration == 0) {
    // No echo received - sensor may not be connected or no object in range
    return -1;
  }
  
  long dist = duration / 58.2;
  
  // More lenient range validation for testing (2cm to 500cm)
  if (dist < 2) {
    return -1;  // Too close
  }
  if (dist > 500) {
    return -1;  // Too far
  }
  
  return dist;
}

// Smart LED control logic
void updateSmartLEDs() {
  bool personPresent = (distance > 0 && distance < PRESENCE_DISTANCE);
  bool isDark = (luxVal < DARK_THRESHOLD);
  
  if (personPresent && isDark) {
    // Person detected + Dark room = Turn on LEDs based on light level
    int numLEDs = map(luxVal, 0, DARK_THRESHOLD, LED_COUNT, 0);
    numLEDs = constrain(numLEDs, 0, LED_COUNT);
    strip.clear();
    for (int i = 0; i < numLEDs; i++) {
      strip.setPixelColor(i, strip.Color(150, 200, 0));  // Yellow-green
    }
    strip.show();
    
  } else if (personPresent && !isDark) {
    // Person detected + Bright room = Just show presence indicator
    strip.clear();
    for (int i = 0; i < 2; i++) {  // 2 LEDs for 8-LED stick
      strip.setPixelColor(i, strip.Color(0, 100, 255));  // Blue indicator
    }
    strip.show();
    
  } else if (!personPresent && isDark) {
    // No person + Dark room = Dim night light
    strip.clear();
    strip.setPixelColor(0, strip.Color(50, 50, 0));  // Single dim LED
    strip.show();
    
  } else {
    // No person + Bright room = LEDs off
    strip.clear();
    strip.show();
  }
}

void displayLine(int &yPos, const char* text, bool partial = false) {
  paint.Clear(UNCOLORED);
  paint.DrawStringAt(5, 0, text, &Font12, COLORED);
  if (partial) epd.SetFrameMemoryPartial(paint.GetImage(), 0, yPos, paint.GetWidth(), paint.GetHeight());
  else epd.SetFrameMemory(paint.GetImage(), 0, yPos, paint.GetWidth(), paint.GetHeight());
  yPos += 16;
}

void displayFullScreen() {
  char buf[30];
  int yPos = 0;
  
  // Title
  paint.SetWidth(200);
  paint.SetHeight(20);
  paint.Clear(COLORED);
  paint.DrawStringAt(5, 2, "Smart Control System", &Font12, UNCOLORED);
  epd.SetFrameMemory(paint.GetImage(), 0, yPos, paint.GetWidth(), paint.GetHeight());
  yPos = 22;
  
  paint.SetHeight(16);
  sprintf(buf, "Temp: %.1fC", temperature); displayLine(yPos, buf);
  sprintf(buf, "Humid: %.1f%%", humidity); displayLine(yPos, buf);
  sprintf(buf, "Light: %ldLux", luxVal); displayLine(yPos, buf);
  if (distance >= 0) {
    sprintf(buf, "Dist: %ldcm", distance);
  } else {
    sprintf(buf, "Dist: --");
  }
  displayLine(yPos, buf);
  yPos += 2; displayLine(yPos, "--- Fan ---"); 
  sprintf(buf, "Speed: %d%%", map(fanSpeed, 0, 255, 0, 100)); displayLine(yPos, buf);
  sprintf(buf, potOhms >= 1000 ? "Pot: %.1fkOhm" : "Pot: %.0fOhm", potOhms >= 1000 ? potOhms/1000.0 : potOhms); 
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
  sprintf(buf, "Light: %ldLux", luxVal); displayLine(yPos, buf, true);
  if (distance >= 0) {
    sprintf(buf, "Dist: %ldcm", distance);
  } else {
    sprintf(buf, "Dist: --");
  }
  displayLine(yPos, buf, true);
  yPos += 34;
  sprintf(buf, "Speed: %d%%", map(fanSpeed, 0, 255, 0, 100)); displayLine(yPos, buf, true);
  sprintf(buf, potOhms >= 1000 ? "Pot: %.1fkOhm" : "Pot: %.0fOhm", potOhms >= 1000 ? potOhms/1000.0 : potOhms);
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
  Serial.println("========================================\n=== Environmental Sensors ===");
  Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
  Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" hPa");
  Serial.print("Gas Resistance: "); Serial.print(gasResistance); Serial.println(" KOhms");
  Serial.print("Ambient Light: "); Serial.print(luxVal); Serial.println(" Lux");
  
  // Enhanced distance sensor output
  Serial.print("Distance Sensor: ");
  if (distance >= 0) {
    Serial.print(distance); 
    Serial.print(" cm [Duration: ");
    Serial.print(lastPulseDuration);
    Serial.println(" µs]");
  } else {
    if (lastPulseDuration == 0) {
      Serial.println("NO ECHO RECEIVED - Check sensor connection!");
    } else {
      Serial.print("Out of range [Duration: ");
      Serial.print(lastPulseDuration);
      Serial.print(" µs = ");
      Serial.print(lastPulseDuration / 58.2);
      Serial.println(" cm]");
    }
  }
  
  Serial.println("\n=== Smart LED Status ===");
  bool personPresent = (distance > 0 && distance < PRESENCE_DISTANCE);
  bool isDark = (luxVal < DARK_THRESHOLD);
  if (personPresent && isDark) {
    Serial.println("Person Detected + Dark Room -> LEDs ON (light-based)");
  } else if (personPresent && !isDark) {
    Serial.println("Person Detected + Bright Room -> Blue indicator");
  } else if (!personPresent && isDark) {
    Serial.println("No Person + Dark Room -> Night light");
  } else {
    Serial.println("No Person + Bright Room -> LEDs OFF");
  }
  
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