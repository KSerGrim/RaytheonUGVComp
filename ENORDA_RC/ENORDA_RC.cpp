#include "Arduino.h"
#include "ENORDA_RC.h"

ENORDA_RC::ENORDA_RC(int pin_rudd, int pin_elev, int pin_gear)
{
  _pin_elev = pin_elev;
  _pin_rudd = pin_rudd;
  _pin_gear = pin_gear;
}

// midpoint is idle controller value (~1450), diff is control variation (~300), threshold is for noise filter (~20)
void ENORDA_RC::init(int midpoint_elev, int midpoint_rudd, int diff, int threshold)
{
  pinMode(_pin_rudd, INPUT);
  pinMode(_pin_elev, INPUT);
  pinMode(_pin_gear, INPUT);
  _midpoint_elev = midpoint_elev;
  _midpoint_rudd = midpoint_rudd;
  _diff = diff;
  _threshold = threshold;
  _max_speed = 255;
  _positive_logic = 1;
}

// polls the RC controller and adjusts motor drive (pass by reference &rightspeed) -- !en to disable polling
void ENORDA_RC::pollController(char en, int* right_motor_speed, int* left_motor_speed, int* right_motor_dir, int* left_motor_dir, int* ch_switch)
{
  

  int _elev, _rudd, _gear;
  int _right_speed, _left_speed, _right_dir, _left_dir, toggle;

  // read RC values
  // values range from -DIFF to +DIFF
  _elev = pulseIn(_pin_elev, HIGH) - _midpoint_elev;
  _rudd = pulseIn(_pin_rudd, HIGH) - _midpoint_rudd;
  _gear = pulseIn(_pin_gear, HIGH) - _midpoint_elev;

  // superimpose speed
  _right_speed = _elev + _rudd;
  _left_speed = _elev - _rudd;

  // set directions
  // positive logic 1 means forward direction is 1
  if (_right_speed > _threshold) {
    _right_dir = _positive_logic;
  } else if (_right_speed < (0 - _threshold)) {
    _right_dir = !_positive_logic;
  } else {
    _right_speed = 0;
    _right_dir = _positive_logic;
  }

  if (_left_speed > _threshold) {
    _left_dir = _positive_logic;
  } else if (_left_speed < (0 - _threshold)) {
    _left_dir = !_positive_logic;
  } else {
    _left_speed = 0;
    _left_dir = _positive_logic;
  }
  // ----------------------------------------------

  // set switch value
  toggle = _gear > 0;

  *ch_switch = toggle;

  // update referenced values if enabled
  if (en == 0) return;
  _right_speed = map(abs(_right_speed), 0, _diff, 0, _max_speed);
  _left_speed = map(abs(_left_speed), 0, _diff, 0, _max_speed);
  *right_motor_speed = constrain(_right_speed, 0, _max_speed);
  *left_motor_speed = constrain(_left_speed, 0, _max_speed);
  *right_motor_dir = _right_dir;
  *left_motor_dir = _left_dir;
}



void ENORDA_RC::setMaximumSpeed(int max_speed)
{
  _max_speed = max_speed;
}

void ENORDA_RC::setLogicPolarity(int positive_logic) 
{
  _positive_logic = positive_logic;
}

int ENORDA_RC::readElev() {
  return pulseIn(_pin_elev, HIGH);
}

int ENORDA_RC::readRudd() {
  return pulseIn(_pin_rudd, HIGH);
}

int ENORDA_RC::readGear() {
  return pulseIn(_pin_gear, HIGH);
}