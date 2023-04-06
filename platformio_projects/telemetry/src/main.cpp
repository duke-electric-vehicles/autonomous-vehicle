#include <SPI.h>
#include <SD.h>
#include "INA190.hpp"
#include <NEO6M.cpp>
#include <algorithm>
#include <vector>
#include "DataCollection.h"













#include <FastLED.h>

// Define the number of LEDs and the data output pin
#define NUM_LEDS 3
#define DATA_PIN 33
CRGB leds[NUM_LEDS];

void cycleLED(){
  // Set the hue value of the LEDs based on the current time
  uint8_t hue = millis() / 6;
  
  // Loop through each LED and set its color based on the hue value
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(hue + (i * 40), 255, 255);
  }
  
  // Show the updated colors on the LEDs
  FastLED.show();
  
  // Wait for a short period of time before updating the colors again
  delayMicroseconds(1000000 / 240); // 240Hz
}


// Sensors and device mapping
const int CURRENT_SENSE_PIN = 21;//21;
const int HALL_INPUT_PIN = 26;
const int VBAT_INPUT_PIN = 17;
const int LED_DIN_PIN = 33;

// Comms
const int SCL_PIN = 24;
const int SDA_PIN = 25;
const int RX_PIN = 7;
const int TX_PIN = 8;

// Consts
const float VBAT_GAIN = 10;
const float VBAT_COMPENSATION = 0.93;
const int updateRate = 5;   // in Hz

// Sensors and devices
// INA190 currSense(CURRENT_SENSE_PIN);  // pin 21, R = 1mOhm
NEO6M gps; 

// Replace these with your sensor reading functions
double getVoltage() { 
  int samples = 10;

  float sum = 0;
  for(int i = 0; i < samples; i++){
    int voltageIn = analogRead(VBAT_INPUT_PIN);
    float batteryVoltage = voltageIn * VBAT_GAIN *  (3.3 / 1023);
    sum += batteryVoltage;
  }
  return VBAT_COMPENSATION * (sum / samples); 
}

double getVoltage_MD() {
  const int samples = 1000; /*calculated sample size based on 1% margin of error and desired confidence level*/;
  std::vector<float> sampleData(samples);

  for (int i = 0; i < samples; i++) {
    sampleData[i] = getVoltage();
  }

  std::sort(sampleData.begin(), sampleData.end());
  float median;

  if (samples % 2 == 0) {
    median = (sampleData[samples / 2 - 1] + sampleData[samples / 2]) / 2.0f;
  } else {
    median = sampleData[samples / 2];
  }

  return VBAT_COMPENSATION * static_cast<double>(median);
}





double getCurrent() { 
  // int samples = 10;
  // float sum = 0;
  // for(int i = 0; i < samples; i++){
  //   sum += currSense.getCurrent();
  // }
  // return (sum / samples ); 
  return -1000.0;
}

double getCurrent_MD() {
  const int samples = 1000; /*calculated sample size based on 1% margin of error and desired confidence level*/;
  std::vector<float> sampleData(samples);

  for (int i = 0; i < samples; i++) {
    sampleData[i] = currSense.getCurrent();
  }

  std::sort(sampleData.begin(), sampleData.end());
  float median;

  if (samples % 2 == 0) {
    median = (sampleData[samples / 2 - 1] + sampleData[samples / 2]) / 2.0f;
  } else {
    median = sampleData[samples / 2];
  }

  return static_cast<double>(median);
}

double getPower() {
  return currSense.getCurrent() * getVoltage(); 
}

double getVelocity() { return 0; }  // determined by hall sensor
double getEnergy() { return 0; }    // todo: calculate total energy by integrating power
double getDistance() { return 0; }  // todo: same as above
double getElapsedTime() { return 0; } // 
double getLatitude() { return gps.getLatitude(); }
double getLongitude() { return gps.getLongitude(); }
double getAltitude() { return gps.getAltitude(); }

const int chipSelect = BUILTIN_SDCARD;
const unsigned long logInterval = 1000;
unsigned long previousMillis = 0;


void initSD(){
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1);
  }
  Serial.println("Card initialized.");

  File dataFile = SD.open("data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Voltage(V),Current(A),Power(W),Velocity(m/s),Energy(J),Distance(m),Elapsed Time(ms),Latitude(DD.dddd),Longitude(DD.dddd),Altitude(m)");
    dataFile.close();
    Serial.println("Created data.csv");
  } else {
    Serial.println("Error creating data.csv");
  }
}

struct SensorData {
  using Func = double (*)();
  Func func;
  int decimalPlaces;
};

std::vector<SensorData> sensorDataFunctions = {
  {getVoltage, 2},
  {getCurrent, 2},
  {getPower, 2},
  {getVelocity, 2},
  {getEnergy, 2},
  {getDistance, 2},
  {getElapsedTime, 0},
  {getLatitude, 4},
  {getLongitude, 4},
  {getAltitude, 2}
};

void logData() {
  File dataFile = SD.open("data.csv", FILE_WRITE);
  if (dataFile) {
    for (auto& sensorData : sensorDataFunctions) {
      double data = sensorData.func();
      dataFile.print(data, sensorData.decimalPlaces);
      dataFile.print(&sensorData != &sensorDataFunctions.back() ? "," : "\n");
    }

    dataFile.close();
    Serial.println("Logged data");
  } else {
    Serial.println("Error opening data.csv");
  }
}

void setup() {  
  // Serial.begin(9600);
  // while (!Serial) {
  //   ;
  // }

  // Initialize the FastLED library
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(64); // Set the brightness of the LEDs (0-255)

  // initSD();
  // gps.initialize();

  // collectData();

}

// primary function here is to update and log dataplayo
void loop() { 


  Serial.print("VOLTAGE = "); Serial.println(getVoltage_MD(), 4);
  Serial.print("CURRENT = "); Serial.println(getCurrent_MD(), 4);
  // Serial.print("POWER = "); Serial.println(getPower(), 4);

  // // gps.update();
  delay(200);
  cycleLED();
}

// Create a CRGB array to store the color values for each LED


