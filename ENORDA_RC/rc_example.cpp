#include "ENORDA_RC.h"

// rudder pin, elevation pin, gear pin
ENORDA_RC rcController(38, 37, 36);

int rightSpeed, leftSpeed, rightDir, leftDir, killswitch;

void setup() {
  Serial.begin(9600);
  rcController.init(1444, 1428, 300, 30); // midpoint_elev, midpoint_rudd, difference, threshold
  rcController.setMaximumSpeed(255); // max speed to be mapped to motors
  rcController.setLogicPolarity(1); // forward direction is 1, if 0, forward direction is 0
}

void loop() {
  // if en == 0, motors will NOT be updated, but the killswitch will be
  rcController.pollController(1, &rightSpeed, &leftSpeed, &rightDir, &leftDir, &killswitch);

  // log it
  Serial.print("rightSpeed:"); Serial.println(rightSpeed);
  Serial.print("leftSpeed:"); Serial.println(leftSpeed);
  Serial.print("rightDir:"); Serial.println(rightDir * 50); // just so you can see in serial plot
  Serial.print("leftDir:"); Serial.println(leftDir * 50);
  Serial.print("kill:"); Serial.println(killswitch * 75);
  Serial.print("rudd:");Serial.println(rcController.readRudd());
  Serial.print("elev:");Serial.println(rcController.readElev());
}