#include <Servo.h>
Servo myservo;

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10
#define _DUTY_MAX 1850
#define _DUTY_NEU 1475
#define _DUTY_MIN 950

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);
  
// initialize serial port
  Serial.begin(57600);
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 300 / 260.05 * raw_dist + 19.7;
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(dist_cali > 156 && dist_cali <224) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);

  if(dist_cali >= 255) {
    myservo.writeMicroseconds(_DUTY_MIN);
  }else {
    myservo.writeMicroseconds(_DUTY_MAX);
  }
  delay(20);
}
