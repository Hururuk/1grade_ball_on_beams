#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0
// Framwork setting
#define _DIST_TARGET 255
#define _DIST_MIN 90
#define _DIST_MAX 410
// Distance sensor
#define _DIST_ALPHA 0.2 //ema_alpha
// Servo range
#define _DUTY_MIN 1200
#define _DUTY_NEU 1475
#define _DUTY_MAX 1800
// Servo speed control
#define _SERVO_ANGLE 30 //servo_angle
#define _SERVO_SPEED 300 //servo_speed
#define _RAMPUP_TIME 0.5
// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 30
#define _INTERVAL_SERIAL 100
// PID parameters
#define _KP 1.2 
#define _KD 70
#define _KI 0.6 //0.3 
#define _ITERM_MAX 50 //50 //item max
// EMA filter
#define EMA_ALPHA 0.35
// etc
#define START _DUTY_MIN + 100
#define END _DUTY_MAX - 100
// global variables
float dist_min, dist_max, dist_raw, dist_cali;
float ema_dist = 0;
float samples_num = 3;
// Servo instance
Servo myservo;
// Distance sensor
float dist_target;
// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;
// Servo speed control
int duty_chg_max, duty_chg_per_interval, duty_chg_adjust;
int toggle_interval, toggle_interval_cnt;
float pause_time;
int duty_target, duty_curr;
// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;
// cali IR_DISTANCE
const float coE[] = {0.0000207, -0.0058357, 1.53848999, 23.2994353};

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO);

  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  dist_target = _DIST_TARGET;

  event_servo = true;
  event_dist = true;
  event_serial = true;

  duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr);
  Serial.begin(57600);

  // servo related variables
  duty_chg_max = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _INTERVAL_SERVO / 1000;
  duty_chg_adjust = (float) duty_chg_max * _INTERVAL_SERVO / _RAMPUP_TIME;
  duty_chg_per_interval = 0;

  pause_time = 0.5;
  toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / _INTERVAL_SERVO;
  toggle_interval_cnt = toggle_interval;

  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;
}

float ir_distance(void) {
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float under_noise_filter(void) {
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    delayMicroseconds(1500);
  }
  return largestReading;
}

float ir_distance_filtered(void) {
  int currReading;
  int lowestReading = 1024;
  for (int i =0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading ;}
  }
  ema_dist = EMA_ALPHA * lowestReading + (1-EMA_ALPHA) * ema_dist;
  float dist_cali = coE[0] * pow(ema_dist, 3) + coE[1] * pow(ema_dist, 2) + coE[2] * ema_dist + coE[3];
  return dist_cali;
}

void loop() {
  // Event_sampling_time
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) event_dist = true;
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) event_servo = true;
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) event_serial = true;

  // Event_dist_handlers
  if (event_dist) {
    event_dist = false;
    // get a distance
    dist_raw = ir_distance_filtered();
    dist_cali = ir_distance_filtered();
    // PID control logic
    error_curr = dist_target - dist_cali;
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    if(abs(iterm) > _ITERM_MAX) iterm = 0;
    control = pterm + dterm + iterm;
    // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;
    if (duty_target > _DUTY_MAX) {duty_target = _DUTY_MAX;}
    if (duty_target < _DUTY_MIN) {duty_target = _DUTY_MIN;}

    error_prev = error_curr;
    last_sampling_time_dist = millis();
  }

  // Event_servo_handlers
  if (event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target
    if(duty_target > duty_curr) {
      if(duty_chg_per_interval < duty_chg_max) {
        duty_chg_per_interval += duty_chg_adjust;
        if(duty_chg_per_interval > duty_chg_max) duty_chg_per_interval = duty_chg_max;
      }
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    } else if(duty_target < duty_curr) {
      if(duty_chg_per_interval > -duty_chg_max) {
        duty_chg_per_interval -= duty_chg_adjust;
        if(duty_chg_per_interval < -duty_chg_max) duty_chg_per_interval = -duty_chg_max;
      }
      duty_curr += duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    } else {
      duty_chg_per_interval = 0;
    }

    if(toggle_interval_cnt >= toggle_interval) {
      toggle_interval_cnt = 0;
      if(duty_curr < START + 200) duty_target = END;
      else if(duty_curr > END - 200) duty_target = START;
    }else {
      toggle_interval_cnt++;
    }
    
    // update servo position
    myservo.writeMicroseconds(duty_curr);

    last_sampling_time_servo = millis();
  }
  // Event_serial_handlers
  if (event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");

    last_sampling_time_serial = millis();
  }
}
