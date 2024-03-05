#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float64.h>

#include "util.h"

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

    // Publish the message
    RCSOFTCHECK(rcl_publish(&voltage_pub, &voltage_msg, NULL));
    RCSOFTCHECK(rcl_publish(&current_pub, &current_msg, NULL));
    RCSOFTCHECK(rcl_publish(&power_pub, &power_msg, NULL));
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("start");

  set_microros_serial_transports(Serial); // Initialize micro-ROS transports
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  while (rmw_uros_ping_agent(100, 5) != RMW_RET_OK) {
    Serial.println("here");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  digitalWrite(LED_PIN, LOW);  

  Serial.println("getting allocator");
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
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}