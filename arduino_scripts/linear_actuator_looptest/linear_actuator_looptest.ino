#include <Servo.h>

/**
 * Diego Miranda
 * March 2nd, 2021
 * 
 * This program runs a simple test on the linear actuator 
 * by applying a forward voltage, zero voltage, and backwards 
 * voltage. 
 * 
 * NOTE: If using the RageBoard v2, at least two PWM signals
 * must be sent. In this case, we are running one motor and they
 * are identical.
 */

Servo sig1, sig2;
int still, forwards, backwards;

void setup() {
  //Motor setup
  sig1.attach(3);
  sig2.attach(5);

  //Vars
  still = 1500;
  forwards = 1100;
  backwards = 1900;
}

void loop() {
  //Repeat forwards and backwards
  //Initial voltage = 0
  sig1.writeMicroseconds(still);
  sig2.writeMicroseconds(still);
  delay(200);

  //Forwards voltage
  sig1.writeMicroseconds(forwards);
  sig2.writeMicroseconds(forwards);
  delay(1000);

  //Zero voltage
  sig1.writeMicroseconds(still);
  sig2.writeMicroseconds(still);
  delay(200);

  //Backwards voltage
  sig1.writeMicroseconds(backwards);
  sig2.writeMicroseconds(backwards);
  delay(1000);
}
