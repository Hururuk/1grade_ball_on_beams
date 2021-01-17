#include <Servo.h>
Servo myservo;

#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10
#define _DUTY_MAX 1850
#define _DUTY_NEU 1475
#define _DUTY_MIN 950

float raw_dist, raw_dist_prev;


void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED,1);
  
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(1475);
}

void loop() {
  // put your main code here, to run repeatedly:
}
