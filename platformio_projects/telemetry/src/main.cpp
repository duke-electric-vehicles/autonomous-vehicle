#include <SPI.h>
#include <SD.h>
#include <vector>

// Replace these with your sensor reading functions
double getVoltage() { return 0; }
double getCurrent() { return 0; }
double getPower() { return 0; }
double getVelocity() { return 0; }
double getEnergy() { return 0; }
double getDistance() { return 0; }
double getElapsedTime() { return 0; }
double getLatitude() { return 0; }
double getLongitude() { return 0; }
double getAltitude() { return 0; }

const int chipSelect = BUILTIN_SDCARD;
const unsigned long logInterval = 1000;
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

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

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= logInterval) {
    previousMillis = currentMillis;
    logData();
  }
}