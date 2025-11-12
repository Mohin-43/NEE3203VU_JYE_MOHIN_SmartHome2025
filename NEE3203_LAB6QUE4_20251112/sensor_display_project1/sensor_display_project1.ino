#include <SPI.h>
#include <Wire.h>
#include "epd1in54_V2.h"
#include "imagedata.h"
#include "epdpaint.h"
#include <stdio.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"

// E-paper definitions
Epd epd;
unsigned char image[1024];
Paint paint(image, 0, 0);
#define COLORED     0
#define UNCOLORED   1

// Sensor definitions
#define AL_ADDR 0x48
#define SEALEVELPRESSURE_HPA (1013.25)

SparkFun_Ambient_Light light(AL_ADDR);
Adafruit_BME680 bme(&Wire1);

// Sensor variables
float gain = .25;
int Integtime = 700;
long luxVal = 0;
float temperature = 0;
float humidity = 0;
float pressure = 0;
float gasResistance = 0;

// Update interval (in milliseconds)
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 1000; // Update every 5 seconds
int updateCount = 0;

// Create a blank base image buffer
unsigned char blankBase[5000];

void setup()
{
  Serial.begin(115200);
  while(!Serial && millis() < 3000) delay(10);
  
  Wire1.begin();
  delay(100);
  
  // Initialize sensors
  Serial.println("Initializing sensors...");
  
  if(light.begin(Wire1)) {
    Serial.println("VEML6030 ready!"); 
    light.setGain(gain);
    light.setIntegTime(Integtime);
  } else {
    Serial.println("VEML6030 failed!");
  }
  
  if (!bme.begin()) {
    Serial.println("BME680 failed!");
  } else {
    Serial.println("BME680 ready!");
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
  }
  
  // Initialize e-Paper display with full refresh ONCE
  Serial.println("Initializing e-Paper...");
  epd.LDirInit();
  epd.Clear();
  
  // Do initial sensor reading
  readSensors();
  
  // Display everything with full refresh initially
  displayFullScreen();
  
  delay(2000);
  
  // Switch to partial update mode
  Serial.println("Switching to partial update mode...");
  epd.HDirInit();
  
  // Create a completely blank/white base image
  memset(blankBase, 0xFF, sizeof(blankBase)); // 0xFF = all white
  epd.DisplayPartBaseImage(blankBase);
  
  Serial.println("System ready - using partial updates!");
}

void loop()
{
  unsigned long currentMillis = millis();
  
  // Update display periodically
  if (currentMillis - lastUpdate >= updateInterval) {
    lastUpdate = currentMillis;
    updateCount++;
    
    // Read sensor data
    readSensors();
    
    // Do a full refresh every 20 updates to prevent ghosting buildup
    if (updateCount >= 20) {
      Serial.println("Periodic full refresh to clear ghosting...");
      epd.LDirInit();
      epd.Clear();
      displayFullScreen();
      delay(1000);
      
      // Switch back to partial mode
      epd.HDirInit();
      epd.DisplayPartBaseImage(blankBase);
      updateCount = 0;
    } else {
      // Use smooth partial update
      updateDisplayPartial();
    }
    
    // Print to serial for debugging
    printSensorData();
  }
}

void displayFullScreen() {
  // Display title and all sensor data using full frame update
  char buffer[30];
  
  // Title
  paint.SetWidth(200);
  paint.SetHeight(24);
  paint.Clear(COLORED);
  paint.DrawStringAt(20, 4, "Sensor Monitor", &Font16, UNCOLORED);
  epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight());
  
  // Sensor readings
  paint.SetWidth(200);
  paint.SetHeight(20);
  
  // Temperature
  paint.Clear(UNCOLORED);
  sprintf(buffer, "Temp: %.1f C", temperature);
  paint.DrawStringAt(5, 2, buffer, &Font16, COLORED);
  epd.SetFrameMemory(paint.GetImage(), 0, 30, paint.GetWidth(), paint.GetHeight());
  
  // Humidity
  paint.Clear(UNCOLORED);
  sprintf(buffer, "Humid: %.1f %%", humidity);
  paint.DrawStringAt(5, 2, buffer, &Font16, COLORED);
  epd.SetFrameMemory(paint.GetImage(), 0, 55, paint.GetWidth(), paint.GetHeight());
  
  // Pressure
  paint.Clear(UNCOLORED);
  sprintf(buffer, "Press: %.0f hPa", pressure);
  paint.DrawStringAt(5, 2, buffer, &Font16, COLORED);
  epd.SetFrameMemory(paint.GetImage(), 0, 80, paint.GetWidth(), paint.GetHeight());
  
  // Gas
  paint.Clear(UNCOLORED);
  sprintf(buffer, "Gas: %.1f KOhm", gasResistance);
  paint.DrawStringAt(5, 2, buffer, &Font16, COLORED);
  epd.SetFrameMemory(paint.GetImage(), 0, 105, paint.GetWidth(), paint.GetHeight());
  
  // Light
  paint.Clear(UNCOLORED);
  sprintf(buffer, "Light: %ld Lux", luxVal);
  paint.DrawStringAt(5, 2, buffer, &Font16, COLORED);
  epd.SetFrameMemory(paint.GetImage(), 0, 130, paint.GetWidth(), paint.GetHeight());
  
  epd.DisplayFrame();
  Serial.println("Full frame displayed");
}

void updateDisplayPartial() {
  // Update ONLY the sensor values using partial refresh (no flicker!)
  char buffer[30];
  
  paint.SetWidth(200);
  paint.SetHeight(20);
  
  // Temperature
  paint.Clear(UNCOLORED);
  sprintf(buffer, "Temp: %.1f C", temperature);
  paint.DrawStringAt(5, 2, buffer, &Font16, COLORED);
  epd.SetFrameMemoryPartial(paint.GetImage(), 0, 30, paint.GetWidth(), paint.GetHeight());
  
  // Humidity
  paint.Clear(UNCOLORED);
  sprintf(buffer, "Humid: %.1f %%", humidity);
  paint.DrawStringAt(5, 2, buffer, &Font16, COLORED);
  epd.SetFrameMemoryPartial(paint.GetImage(), 0, 55, paint.GetWidth(), paint.GetHeight());
  
  // Pressure
  paint.Clear(UNCOLORED);
  sprintf(buffer, "Press: %.0f hPa", pressure);
  paint.DrawStringAt(5, 2, buffer, &Font16, COLORED);
  epd.SetFrameMemoryPartial(paint.GetImage(), 0, 80, paint.GetWidth(), paint.GetHeight());
  
  // Gas
  paint.Clear(UNCOLORED);
  sprintf(buffer, "Gas: %.1f KOhm", gasResistance);
  paint.DrawStringAt(5, 2, buffer, &Font16, COLORED);
  epd.SetFrameMemoryPartial(paint.GetImage(), 0, 105, paint.GetWidth(), paint.GetHeight());
  
  // Light
  paint.Clear(UNCOLORED);
  sprintf(buffer, "Light: %ld Lux", luxVal);
  paint.DrawStringAt(5, 2, buffer, &Font16, COLORED);
  epd.SetFrameMemoryPartial(paint.GetImage(), 0, 130, paint.GetWidth(), paint.GetHeight());
  
  epd.DisplayPartFrame();
  Serial.println("Partial update");
}

void readSensors() {
  // Read light sensor
  luxVal = light.readLight();
  
  // Read BME680 sensor
  if (bme.performReading()) {
    temperature = bme.temperature;
    humidity = bme.humidity;
    pressure = bme.pressure / 100.0;
    gasResistance = bme.gas_resistance / 1000.0;
  }
}

void printSensorData() {
  Serial.println("=== Sensor Readings ===");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");
  
  Serial.print("Gas Resistance: ");
  Serial.print(gasResistance);
  Serial.println(" KOhms");
  
  Serial.print("Ambient Light: ");
  Serial.print(luxVal);
  Serial.println(" Lux");
  
  Serial.println();
}