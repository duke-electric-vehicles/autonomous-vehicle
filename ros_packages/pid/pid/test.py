#include <Arduino.h>
#include <Servo.h>

#define LED_PIN 13
const int trigPin = 9;
const int echoPin = 10;
float duration, distance;
Servo myServo;
float kp = 30.0;
float kd = 20;
float ki = 0;
float perr = 0;
float target = 70;
float integral = 0;
int integralcounter = 0;

void setup() {  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);  

  myServo.attach(3);
  myServo.writeMicroseconds(1500);
  delay(1000);
}


void loop() {
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(100);

  integralcounter++;
  
  if(integralcounter == 250)
  {
    integral = 0;
    integralcounter = 0;
  }

  float err = distance - target;
  integral = integral + err;
  float speed = 1500 + err*kp + ki*integral - kd*(err-perr);

  if(distance > target)
  {
    myServo.writeMicroseconds(speed);
    /*if(speed < 1600)
    {
      myServo.writeMicroseconds(1600);
    }
    else
    {
      myServo.writeMicroseconds(speed);
    }*/
    

  }
  else if (distance < target)
  {
    //int num = speed - 1500;
    myServo.writeMicroseconds(speed);
    /*
    if(speed > 1400)
    {
      myServo.writeMicroseconds(1400);
    }
    else
    {
      myServo.writeMicroseconds(speed);
    }*/
  }
  else
  {
    myServo.writeMicroseconds(1500);
  }
  err = perr;
}
