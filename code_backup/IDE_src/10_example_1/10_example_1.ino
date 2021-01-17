#include <Servo.h>
#define PIN_SERVO 10

Servo myservo;

void setup() {
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(1500);
  delay(1000);
  Serial.begin(57600);
  Serial.print(myservo.read());
}

void loop() {
  //myservo.write(0);
  //delay(500);
  //Serial.println(myservo.read());
  //myservo.write(90);
  //delay(500);
  //myservo.write(180);
  //delay(500);
  //myservo.write(90);
  //delay(500);
    // add code here.
  while(1) {
    myservo.writeMicroseconds(1450);
  }
}
