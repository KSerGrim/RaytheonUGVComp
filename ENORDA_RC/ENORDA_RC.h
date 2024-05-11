#ifndef ENORDA_RC_h
#define ENORDA_RC_h

#include "Arduino.h"

class ENORDA_RC
{
  public:
    ENORDA_RC(int pin_rudd, int pin_elev, int pin_gear);
    void pollController(char en, int* right_motor_speed, int* left_motor_speed, int* right_motor_dir, int* left_motor_dir, int* ch_switch);
    void setMaximumSpeed(int max_speed);
    void setLogicPolarity(int positive_logic);
    void init(int midpoint_elev, int midpoint_rudd, int diff, int threshold);
    int readRudd();
    int readElev();
    int readGear();
  private:
    int _pin_rudd;
    int _pin_elev;
    int _pin_gear;
    int _midpoint_rudd;
    int _midpoint_elev;
    int _diff;
    int _threshold;
    int _max_speed;
    int _positive_logic;
};

#endif