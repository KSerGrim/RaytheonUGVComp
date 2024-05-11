#include "Arduino.h"
#include "Wire.h"
#include "ENORDA_FDC2214.h"

/*
!!!       UNTESTED CODE DO NOT USE        !!!
Written by an aspiring analog IC design engineer
(Adam)
*/

#define ADDR_DEVICE_FDC2214 0x2A
#define ADDR_REG_CONFIG_FDC2214 0x1A
#define ADDR_REG_MUX_CONFIG_FDC2214 0x1B

static const char ADDR_REG_DATA_MSB_FDC2214[4] = { 0x00, 0x02, 0x04, 0x06 };
static const char ADDR_REG_DATA_LSB_FDC2214[4] = { 0x01, 0x03, 0x05, 0x07 };
static const char ADDR_REG_CLOCK_DIVIDERS_FDC2214[4] = { 0x14, 0x15, 0x16, 0x17};
static const char ADDR_REG_DRIVE_CURRENT_FDC2214[4] = { 0x1E, 0x1F, 0x20, 0x21 };

ENORDA_FDC2214::ENORDA_FDC2214(char channel) {
  ch = channel;
}

// initialize the basic registers
// initialize the basic registers of channel
void ENORDA_FDC2214::i2c_init_fdc2214 () {



  /*
  Note for Drive current:
  Sensor drive current: to ensure that the oscillation amplitude is between 1.2V and 1.8V, measure the
  oscillation amplitude on an oscilloscope and adjust the IDRIVE value

  Also:
  FOR ANY OTHER PROBLEMS REFER TO "10.2.3.2 Recommended Initial Register Configuration Values"
  */

  Wire.beginTransmission(ADDR_DEVICE_FDC2214); 
  Wire.write(ADDR_REG_CLOCK_DIVIDERS_FDC2214[ch]); // ch0 clock config
  Wire.write(0x20); // 0010 0000 divide fin by 2 (max 5 MHz)
  Wire.write(0x01); // 0000 0001 divide fclk by 1 (40 MHz)
  Wire.endTransmission();

  Wire.beginTransmission(ADDR_DEVICE_FDC2214); 
  Wire.write(ADDR_REG_MUX_CONFIG_FDC2214); // mux address
  Wire.write(0x02); // 0000 0010
  Wire.write(0x0D); // 0000 1101
  Wire.endTransmission();

  Wire.beginTransmission(ADDR_DEVICE_FDC2214);
  Wire.write(ADDR_REG_CONFIG_FDC2214); // config address
  Wire.write(0b00011100 | (ch << 6)); // (2 variable bits)01 1100 
  Wire.write(0x81); // 1000 0001
  Wire.endTransmission();

  Wire.beginTransmission(ADDR_DEVICE_FDC2214);
  Wire.write(ADDR_REG_DRIVE_CURRENT_FDC2214[ch]);
  Wire.write(0xF8); // 1111 1000 0000 0000
  Wire.write(0x00);
  Wire.endTransmission();
}

// water sensor sleep toggle
void ENORDA_FDC2214::i2c_sleep_fdc2214 (bool enable) {

  /*
  For power saving:
  We can enter sleep mode using config register, but after coming out of 
  sleep mode wait 16384 / 40 000 000 seconds for first conversion
  */

  if (enable) {
    Wire.beginTransmission(ADDR_DEVICE_FDC2214);
    Wire.write(ADDR_REG_CONFIG_FDC2214); // config address
    Wire.write(0x3C); // 0011 1100 
    Wire.write(0x01); // 0000 0001
    Wire.endTransmission();
  } else {
    Wire.beginTransmission(ADDR_DEVICE_FDC2214);
    Wire.write(ADDR_REG_CONFIG_FDC2214); // config address
    Wire.write(0x1C); // 0001 1100 
    Wire.write(0x01); // 0000 0001
    Wire.endTransmission();
  }

}

// read 28 bits of sensor data
uint32_t ENORDA_FDC2214::i2c_read_fdc2214 () {
  uint32_t data = 0;
  uint32_t temp = 0;

  Wire.beginTransmission(ADDR_DEVICE_FDC2214);
  Wire.write(ADDR_REG_DATA_MSB_FDC2214[ch]); // set register to DATA_CH0
  Wire.endTransmission();

  Wire.requestFrom(ADDR_DEVICE_FDC2214, 2);    // request 2 bytes

  if (Wire.available() == 2) {  // read 2 bytes
    temp = Wire.read();
    data = data | (temp << 24);
    temp = Wire.read();
    data = data | (temp << 16);
  }

  Wire.beginTransmission(ADDR_DEVICE_FDC2214);
  Wire.write(ADDR_REG_DATA_LSB_FDC2214[ch]); // set register to DATA_LSB_CH0
  Wire.endTransmission();

  Wire.requestFrom(ADDR_DEVICE_FDC2214, 2);    // request 2 bytes

  if (Wire.available() == 2) {  // read 2 bytes
    temp = Wire.read();
    data = data | (temp << 8);
    temp = Wire.read();
    data = data | temp;
  }

  return data;
}

double ENORDA_FDC2214::get_capacitance_pf(uint32_t data) {
  double f_sensor = (40E6 * (data * 2) * 1.0) / 268435456;
  double c_sensor = (1.0 / (18E-6 * pow(2 * 3.14 * f_sensor, 2)) - 33E-12) * 1E12;
  //Serial.println(c_sensor);
  return (c_sensor);
}
