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
void setup() {
  //Motor setup
  Servo sig1, sig2;
  sig1.attach(3);
  sig2.attach(5);

  //Vars
  int still = 1500;
  int forwards = 1100;
  int backwards = 1900;

  //Initial voltage = 0
  sig1.writeMicroseconds(still);
  sig2.writeMicroseconds(still);
  delay(500);

  //Forwards voltage
  sig1.writeMicroseconds(forwards);
  sig2.writeMicroseconds(forwards);
  delay(3000);

  //Zero voltage
  sig1.writeMicroseconds(still);
  sig2.writeMicroseconds(still);
  delay(500);

  //Backwards voltage
  sig1.writeMicroseconds(backwards);
  sig2.writeMicroseconds(backwards);
  delay(3000);
}

void loop() {
  //TODO: Implement square wave test
}
