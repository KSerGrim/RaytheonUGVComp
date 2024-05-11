/*
!!!       UNTESTED CODE DO NOT USE        !!!
Written by an aspiring analog IC design engineer
(Adam)
*/

#ifndef ENORDA_FDC2214_h
#define ENORDA_FDC2214_h

#include "Arduino.h"
#include "Wire.h"

class ENORDA_FDC2214
{
  public:
    ENORDA_FDC2214(char);
    void i2c_init_fdc2214();
    void i2c_sleep_fdc2214(bool);
    uint32_t i2c_read_fdc2214();
    double get_capacitance_pf(uint32_t);
  private:
    char ch;
};

#endif