#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9
#define PIN_SERVO 10
#define PIN_IR    A0

// Event interval parameters
#define _INTERVAL_DIST    50    // distance sensor interval (unit: ms)
#define _INTERVAL_SERVO   20    // servo interval (unit: ms)
#define _INTERVAL_SERIAL  100   // serial interval (unit: ms)

// EMA filter configuration for the IR distance sensor
#define _EMA_ALPHA 0.4    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.

// Servo adjustment - Set _DUTY_MAX, _NEU, _MIN with your own numbers
#define _DUTY_MAX 3000    // maximum servo pulse width (us)
#define _DUTY_NEU 1000    // neutral position servo pulse width (us)
#define _DUTY_MIN 300   // minimum servo pulse width (us)

#define _SERVO_ANGLE_DIFF 90  // Replace with |D - E| degree
#define _SERVO_SPEED       70  // servo speed in degrees per second

#define _BANGBANG_RANGE    400  // duty up and down for bangbang control

// Target Distance
#define _DIST_TARGET    175 // Center of the rail (unit:mm)

// global variables

Servo myservo;

float dist_ema;     // unit: mm

int duty_change_per_interval; // maximum duty difference per interval
int duty_target;    // Target duty
int duty_current;   // Current duty
int duty_adj;       // Level adjustment

unsigned long last_sampling_time_dist;   // unit: msec
unsigned long last_sampling_time_servo;  // unit: msec
unsigned long last_sampling_time_serial; // unit: msec

bool event_dist, event_servo, event_serial; // event triggered?

void setup()
{
  // initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  myservo.attach(PIN_SERVO);

  duty_target = duty_current = _DUTY_NEU;
  myservo.writeMicroseconds(duty_current);

  // initialize serial port
  Serial.begin(1000000);  
    
  // convert angular speed into duty change per interval.
  duty_change_per_interval = 
    (float)(_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / _SERVO_ANGLE_DIFF) * (_INTERVAL_SERVO / 1000.0); 
}

void loop()
{
  unsigned long time_curr = millis();
  
  // wait until next event time
  if (time_curr >= (last_sampling_time_dist + _INTERVAL_DIST)) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if (time_curr >= (last_sampling_time_servo + _INTERVAL_SERVO)) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if (time_curr >= (last_sampling_time_serial + _INTERVAL_SERIAL)) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }
    
  if (event_dist) {
    float dist_filtered; // unit: mm
    event_dist = false;

    // get a distance reading from the distance sensor
    dist_filtered = volt_to_distance(ir_sensor_filtered(10, 0.5, 0));
    dist_ema = _EMA_ALPHA * dist_filtered + (1.0 - _EMA_ALPHA) * dist_ema;

    // bang bang control
    if (dist_ema > _DIST_TARGET) {
      duty_target = _DUTY_NEU - _BANGBANG_RANGE;
      digitalWrite(PIN_LED, 0);
    } else if (dist_ema < _DIST_TARGET) {
      duty_target = _DUTY_NEU + _BANGBANG_RANGE;
      digitalWrite(PIN_LED, 1);
    }
  }
  
  if (event_servo) {
    event_servo = false;
     
    // adjust duty_current toward duty_target by duty_change_per_interval
    if (duty_target > duty_current) {
      duty_current += duty_change_per_interval;
      if (duty_current > duty_target)
          duty_current = duty_target;
    } else if (duty_target < duty_current) {
      duty_current -= duty_change_per_interval;
      if (duty_current < duty_target)
        duty_current = duty_target;
    }
    
    // servo arm protection
    if (duty_current < _DUTY_MIN)
      duty_current = _DUTY_MIN;
    else if (duty_current > _DUTY_MAX) 
      duty_current = _DUTY_MAX;

    // update servo position
    myservo.writeMicroseconds(duty_current);
  }
  
  if (event_serial) {
    event_serial = false;
    
    // output the read value to the serial port
    Serial.print("TARGET:"); Serial.print(_DIST_TARGET);
    Serial.print(",DIST:"); Serial.print(dist_ema);
    Serial.print(",duty_target:"); Serial.print(duty_target);
    Serial.print(",duty_current:"); Serial.println(duty_current);
  }
}

float volt_to_distance(int a_value)
{
  // Replace line into your own equation
  return 515 + (-1.39 * a_value) + (9.16e-4 * a_value * a_value);
}

int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose)
{
  unsigned int *ir_val, ret_val;
  unsigned int start_time;
 
  if (verbose >= 2)
    start_time = millis(); 

  if ((n == 0) || (n > 100) || (position < 0.0) || (position > 1))
    return 0;
    
  if (position == 1.0)
    position = 0.999;

  ir_val = (unsigned int *)malloc(sizeof(unsigned int) * n);
  if (ir_val == NULL)
    return 0;

  for (int i = 0; i < n; i++) {
    ir_val[i] = analogRead(PIN_IR);
  }

  qsort(ir_val, n, sizeof(unsigned int), compare);
  ret_val = ir_val[(unsigned int)(n * position)];
  free(ir_val);

  return ret_val;
}
