#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <SPI.h>
#include <SD.h>
#include <INA190.cpp>
#include <NEO6M.cpp>
#include <vector>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Include float64 (idk what msg type it should be)
//#include <my_custom_msgs/msg/sensor_data.h>
#include <std_msgs/msg/float64.h>

// ... (all the constants, global variables, and function definitions from diego's code)

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


// void initSD(){
//   Serial.print("Initializing SD card...");
//
//   if (!SD.begin(chipSelect)) {
//     Serial.println("Card failed, or not present");
//     while (1);
//   }
//   Serial.println("Card initialized.");
//
//   File dataFile = SD.open("data.csv", FILE_WRITE);
//   if (dataFile) {
//     dataFile.println("Voltage(V),Current(A),Power(W),Velocity(m/s),Energy(J),Distance(m),Elapsed Time(ms),Latitude(DD.dddd),Longitude(DD.dddd),Altitude(m)");
//     dataFile.close();
//     Serial.println("Created data.csv");
//   } else {
//     Serial.println("Error creating data.csv");
//   }
// }

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

// void logData() {
//   File dataFile = SD.open("data.csv", FILE_WRITE);
//   if (dataFile) {
//     for (auto& sensorData : sensorDataFunctions) {
//       double data = sensorData.func();
//       dataFile.print(data, sensorData.decimalPlaces);
//       dataFile.print(&sensorData != &sensorDataFunctions.back() ? "," : "\n");
//     }
//
//     dataFile.close();
//     Serial.println("Logged data");
//   } else {
//     Serial.println("Error opening data.csv");
//   }
// }

// void data_setup() {
//   Serial.begin(9600);
//   while (!Serial) {
//     ;
//   }
//
//   // initSD();
//   // gps.initialize();
// }

// primary function here is to update and log data
// void data_loop() { 
//   gps.update();
//   // delay(200);
// }

// Global variables for micro-ROS
rcl_publisher_t voltage_pub;
rcl_publisher_t current_pub;
rcl_publisher_t power_pub;
std_msgs__msg__Float64 voltage_msg;
std_msgs__msg__Float64 current_msg;
std_msgs__msg__Float64 power_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// ... (RCCHECK and RCSOFTCHECK macros, error_loop function from the publisher node code)

#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  // Serial.println("publishing");
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Fill in the message data
    voltage_msg.data = getVoltage();
    current_msg.data = getCurrent();
    power_msg.data = getPower();
    //msg.velocity = getVelocity();
    //msg.energy = getEnergy();
    //msg.distance = getDistance();
    //msg.elapsed_time = getElapsedTime();
    //msg.latitude = getLatitude();
    //msg.longitude = getLongitude();
    //msg.altitude = getAltitude();

    // Publish the message
    RCSOFTCHECK(rcl_publish(&voltage_pub, &voltage_msg, NULL));
    RCSOFTCHECK(rcl_publish(&current_pub, &current_msg, NULL));
    RCSOFTCHECK(rcl_publish(&power_pub, &power_msg, NULL));
  }
}

void setup() {
  // data_setup(); // Initialize data logging
  Serial.begin(115200);
  set_microros_serial_transports(Serial); // Initialize micro-ROS transports
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  while (rmw_uros_ping_agent(100, 5) != RMW_RET_OK) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(300);
  }

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "pdb_node", "", &support));
  RCCHECK(rclc_publisher_init_default(
    &voltage_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "pdb_voltage"));
  RCCHECK(rclc_publisher_init_default(
    &current_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "pdb_current"));
  RCCHECK(rclc_publisher_init_default(
    &power_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "pdb_power"));
  //
  // Initialize timer
  const unsigned int timer_timeout = 500;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Initialize executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  // Serial.println("finished setup");
}

void loop() {
  // data_loop(); // Update data and log it
  // delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
