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
const unsigned long updateInterval = 5000; // Update every 5 seconds

void setup()
{
  Serial.begin(115200);
  
  // Initialize I2C
  Wire1.begin();
  
  // Initialize sensors
  Serial.println("Initializing sensors...");
  
  // Initialize VEML6030 Light Sensor
  if(light.begin(Wire1)) {
    Serial.println("VEML6030 Light Sensor ready!"); 
    light.setGain(gain);
    light.setIntegTime(Integtime);
  } else {
    Serial.println("Could not communicate with VEML6030!");
  }
  
  // Initialize BME680
  if (!bme.begin()) {
    Serial.println("Could not find BME680 sensor!");
  } else {
    Serial.println("BME680 sensor ready!");
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
  }
  
  // Initialize e-Paper display
  Serial.println("Initializing e-Paper display...");
  epd.LDirInit();
  epd.Clear();
  
  // Display startup screen
  displayStartupScreen();
  
  delay(2000);
  
  // Prepare for continuous updates
  epd.HDirInit();
  epd.DisplayPartBaseImage(IMAGE_DATA);
  
  Serial.println("System ready!");
}

void loop()
{
  unsigned long currentMillis = millis();
  
  // Update display periodically
  if (currentMillis - lastUpdate >= updateInterval) {
    lastUpdate = currentMillis;
    
    // Read sensor data
    readSensors();
    
    // Update display with sensor data
    updateDisplay();
    
    // Print to serial for debugging
    printSensorData();
  }
}

void displayStartupScreen() {
  paint.SetWidth(200);
  paint.SetHeight(24);
  
  paint.Clear(COLORED);
  paint.DrawStringAt(10, 4, "Sensor Monitor", &Font16, UNCOLORED);
  epd.SetFrameMemory(paint.GetImage(), 0, 10, paint.GetWidth(), paint.GetHeight());
  
  paint.Clear(UNCOLORED);
  paint.DrawStringAt(10, 4, "Initializing...", &Font16, COLORED);
  epd.SetFrameMemory(paint.GetImage(), 0, 35, paint.GetWidth(), paint.GetHeight());
  
  epd.DisplayFrame();
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

void updateDisplay() {
  char buffer[30];
  
  paint.SetWidth(200);
  paint.SetHeight(20);
  
  // Display Temperature
  paint.Clear(UNCOLORED);
  sprintf(buffer, "Temp: %.1f C", temperature);
  paint.DrawStringAt(5, 2, buffer, &Font16, COLORED);
  epd.SetFrameMemoryPartial(paint.GetImage(), 0, 10, paint.GetWidth(), paint.GetHeight());
  
  // Display Humidity
  paint.Clear(UNCOLORED);
  sprintf(buffer, "Humidity: %.1f %%", humidity);
  paint.DrawStringAt(5, 2, buffer, &Font16, COLORED);
  epd.SetFrameMemoryPartial(paint.GetImage(), 0, 35, paint.GetWidth(), paint.GetHeight());
  
  // Display Pressure
  paint.Clear(UNCOLORED);
  sprintf(buffer, "Press: %.1f hPa", pressure);
  paint.DrawStringAt(5, 2, buffer, &Font16, COLORED);
  epd.SetFrameMemoryPartial(paint.GetImage(), 0, 60, paint.GetWidth(), paint.GetHeight());
  
  // Display Gas Resistance
  paint.Clear(UNCOLORED);
  sprintf(buffer, "Gas: %.1f KOhm", gasResistance);
  paint.DrawStringAt(5, 2, buffer, &Font16, COLORED);
  epd.SetFrameMemoryPartial(paint.GetImage(), 0, 85, paint.GetWidth(), paint.GetHeight());
  
  // Display Light Level
  paint.Clear(UNCOLORED);
  sprintf(buffer, "Light: %ld Lux", luxVal);
  paint.DrawStringAt(5, 2, buffer, &Font16, COLORED);
  epd.SetFrameMemoryPartial(paint.GetImage(), 0, 110, paint.GetWidth(), paint.GetHeight());
  
  // Update the display
  epd.DisplayPartFrame();
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