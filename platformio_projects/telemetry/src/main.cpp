#include <SPI.h>
#include <SD.h>
#include <INA190.cpp>
#include <NEO6M.cpp>
#include <vector>

// Sensors and device mapping
const int CURRENT_SENSE_PIN = 21;
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
const float CURR_SENSE_SHUNT_R = 0.001;
const int updateRate = 5;   // in Hz

// Sensors and devices
INA190 currSense(CURRENT_SENSE_PIN, CURR_SENSE_SHUNT_R);  // pin 21, R = 1mOhm
NEO6M gps; 

// Replace these with your sensor reading functions
double getVoltage() { 
  int voltageIn = analogRead(VBAT_INPUT_PIN);
  float batteryVoltage = voltageIn * VBAT_GAIN *  (3.3 / 1023);
  return batteryVoltage; 
}

double getCurrent() { 
  return currSense.getCurrent(); 
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
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  initSD();
  gps.initialize();
}

// primary function here is to update and log data
void loop() { 
  gps.update();
  delay(200);
}