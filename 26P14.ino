#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0
// Framwork setting
#define _DIST_TARGET 205 //255 - 55(Sensor)
#define _DIST_MIN 90
#define _DIST_MAX 410
// Distance sensor
#define _DIST_ALPHA 0.2 //ema_alpha
// Servo range
#define _DUTY_MIN 1175
#define _DUTY_NEU 1475
#define _DUTY_MAX 1675
// Servo speed control
#define _SERVO_ANGLE 30 //servo_angle
#define _SERVO_SPEED 300 //servo_speed
// Event periods
#define _INTERVAL_DIST 30
#define _INTERVAL_SERVO 30
#define _INTERVAL_SERIAL 100
// PID parameters
#define _KP_UP 1.2
#define _KP_DW 0.6
#define _KD_UP 50
#define _KD_DW 35
// EMA filter
#define EMA_ALPHA 0.2
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
int duty_chg_per_interval;
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
  myservo.writeMicroseconds(_DUTY_NEU);
  Serial.begin(57600);

  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000);
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
    dist_raw = ir_distance();
    dist_cali = ir_distance_filtered();
    dist_raw = dist_cali;
    // PID control logic
    error_curr = dist_target - dist_cali;
    if(error_curr < 0) {
      pterm = _KP_UP * error_curr;
      dterm = _KD_UP * (error_curr - error_prev);
    }else{
      pterm = _KP_DW * error_curr;
      dterm = _KD_DW * (error_curr - error_prev);
    }
    control = pterm + dterm;
    
    /*control = _KP * pterm;
    // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;
    if (duty_target > _DUTY_MAX) {duty_target = _DUTY_MAX;}
    if (duty_target < _DUTY_MIN) {duty_target = _DUTY_MIN;}

    dterm = _KD * (error_curr - error_prev);
    control = dterm;*/
    
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
    if (duty_curr > duty_target) {duty_curr -= duty_chg_per_interval ;}
    if (duty_curr < duty_target) {duty_curr += duty_chg_per_interval + 10;}
    // update servo position
    myservo.writeMicroseconds(duty_curr);

    last_sampling_time_servo = millis();
  }

  // Event_serial_handlers
  if (event_serial) {
    event_serial = false;
    /*Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm, -1000, 1000, 510, 610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target, 1000, 2000, 410, 510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr, 1000, 2000, 410, 510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");*/
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");

    last_sampling_time_serial = millis();
  }
}
