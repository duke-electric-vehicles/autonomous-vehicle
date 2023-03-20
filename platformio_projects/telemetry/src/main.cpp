#include <SPI.h>
#include <SD.h>

/**
 * @author Diego
 * 
 **/

// Replace these with your sensor reading functions
float getVoltage() { return 0; }
float getCurrent() { return 0; }
float getPower() { return 0; }
float getVelocity() { return 0; }
float getEnergy() { return 0; }
float getDistance() { return 0; }
unsigned long getElapsedTime() { return 0; }
float getLatitude() { return 0; }
float getLongitude() { return 0; }
float getAltitude() { return 0; }

const int chipSelect = BUILTIN_SDCARD; // Use the built-in SD card slot on Teensy 4.1
const unsigned long logInterval = 1000; // Log data every 1000 milliseconds (1 second)
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for the serial monitor to open
  }

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1);
  }
  Serial.println("Card initialized.");

  // Create a new CSV file
  File dataFile = SD.open("data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Voltage(V),Current(A),Power(W),Velocity(m/s),Energy(J),Distance(m),Elapsed Time(ms),Latitude(DD.dddd),Longitude(DD.dddd),Altitude(m)");
    dataFile.close();
    Serial.println("Created data.csv");
  } else {
    Serial.println("Error creating data.csv");
  }
}

void logData() {
  // Get sensor data
  float voltage = getVoltage();
  float current = getCurrent();
  float power = getPower();
  float velocity = getVelocity();
  float energy = getEnergy();
  float distance = getDistance();
  unsigned long elapsedTime = getElapsedTime();
  float latitude = getLatitude();
  float longitude = getLongitude();
  float altitude = getAltitude();

  // Log data to the CSV file
  File dataFile = SD.open("data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.print(voltage, 2);
    dataFile.print(",");
    dataFile.print(current, 2);
    dataFile.print(",");
    dataFile.print(power, 2);
    dataFile.print(",");
    dataFile.print(velocity, 2);
    dataFile.print(",");
    dataFile.print(energy, 2);
    dataFile.print(",");
    dataFile.print(distance, 2);
    dataFile.print(",");
    dataFile.print(elapsedTime);
    dataFile.print(",");
    dataFile.print(latitude, 4);
    dataFile.print(",");
    dataFile.print(longitude, 4);
    dataFile.print(",");
    dataFile.println(altitude, 2);
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
