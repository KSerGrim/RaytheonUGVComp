// FDC2214 library example code

#include "ENORDA_FDC2214.h"

// init water sensor class on channel 0
ENORDA_FDC2214 water_sensor(0);

void setup() {
  Serial.begin(9600); // begin serial
  Wire.begin(); // begin i2c
  digitalWrite(SDA, LOW); // disable pull up resistor on SDA
  digitalWrite(SCL, LOW); // disable pull up resistor on SCL
  water_sensor.i2c_init_fdc2214(); // init water sensor after i2c

}

void loop() {
  uint32_t data = water_sensor.i2c_read_fdc2214(); // read raw water sensor data
  double cap_pf = water_sensor.get_capacitance_pf(data); // convert raw data to capacitance in pF
  Serial.print( cap_pf, DEC ); // print
  Serial.print(" pF\n");
  delay(1000); // do this every second
}