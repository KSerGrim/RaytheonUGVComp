// #define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
// MPU is PWR SDA(20), SCL(21), TX(18), RX(19)
// Encoder front_Right_Enc(21, 35);
// Encoder front_Left_Enc(20, 37);

Encoder back_Right_Enc(19, 39);
Encoder back_Left_Enc(18, 41);
//   avoid using pins with LEDs attached

unsigned long currentTime = 0; // Corrected: Declare currentTime to store the last update time
const float PPR = 150.0;  // Encoder Pulses Per Revolution
const float wheelDiameter = 0.065;  // Wheel diameter in meters
const float pi = 3.14159265358979323846;
const float wheelCircumference = pi * wheelDiameter; // Calculate wheel circumference for distance traveled per revolution
int period = 250;

long int oldPosition_FrontRight = 0; // Initialized to store the last position
long int oldPosition_FrontLeft = 0;

long int oldPosition_BackRight = 0; // Initialized to store the last position
long int oldPosition_BackLeft = 0;

float PPS_FR, RPS_FR, RPM_FR, MPH_FR;
float PPS_FL, RPS_FL, RPM_FL, MPH_FL;

float PPS_BR, RPS_BR, RPM_BR, MPH_BR;
float PPS_BL, RPS_BL, RPM_BL, MPH_BL;

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
}

void loop() {
  unsigned long currentMillis = millis(); // Use a temporary variable for the current time
  if(currentMillis - currentTime >= period){
    // long int newPosition_FrontRight = front_Right_Enc.read();
    // long int newPosition_FrontLeft = front_Left_Enc.read();
    long int newPosition_BackRight = back_Right_Enc.read();
    long int newPosition_BackLeft = back_Left_Enc.read();
    
    // Average the difference in positions for both encoders
    // float avgPositionDifference = ((newPosition_FrontRight - oldPosition_FrontRight) + (newPosition_FrontLeft - oldPosition_FrontLeft)) / 2.0;
    // PPS = avgPositionDifference * 1000.0 / (currentMillis - currentTime); // Calculate PPS based on the average position difference
    
    // PPS_FR = 1000*(newPosition_FrontRight - oldPosition_FrontRight) / (millis() - currentTime);
    // PPS_FL = 1000*(newPosition_FrontLeft - oldPosition_FrontLeft) / (millis() - currentTime);
    PPS_BR = 1000*(newPosition_BackRight - oldPosition_BackRight) / (millis() - currentTime);
    PPS_BL = 1000*(newPosition_BackLeft - oldPosition_BackLeft) / (millis() - currentTime);
    
    // RPS_FR = PPS_FR / PPR; // Calculate RPS
    // RPS_FL = PPS_FL / PPR; // Calculate RPS
    RPS_BR = PPS_BR / PPR; // Calculate RPS
    RPS_BL = PPS_BL / PPR; // Calculate RPS
    
    // RPM_FR = RPS_FR * 60;
    // RPM_FL = RPS_FL * 60;
    RPM_BR = RPS_BR * 60;
    RPM_BL = RPS_BL * 60;
    
    // MPH_FR = (wheelCircumference * RPM_FR * 60) / 1609.34; // Convert meters per minute to miles per hour
    // MPH_FL = (wheelCircumference * RPM_FL * 60) / 1609.34; // Convert meters per minute to miles per hour
    MPH_BR = (wheelCircumference * RPM_BR * 60) / 1609.34; // Convert meters per minute to miles per hour
    MPH_BL = (wheelCircumference * RPM_BL * 60) / 1609.34; // Convert meters per minute to miles per hour
    
    // Serial.print("PPS_FR: ");
    // Serial.println(PPS_FR);
    // Serial.print("RPS_FRL: ");
    // Serial.println(RPS_FRL);
    // Serial.print("RPM_FR: ");
    // Serial.println(RPM_FR);
    // Serial.print("MPH_FR: ");
    // Serial.println(MPH_FR);
    // Serial.println();

    // Serial.print("PPS_FL: ");
    // Serial.println(PPS_FL);
    // Serial.print("RPS_FL: ");
    // Serial.println(RPS_FL);
    // Serial.print("RPM_FL: ");
    // Serial.println(RPM_FL);
    // Serial.print("MPH_FL: ");
    // Serial.println(MPH_FL);

    Serial.print("PPS_BR: ");
    Serial.println(PPS_BR);
    // Serial.print("RPS_BR: ");
    // Serial.println(RPS_BR);
    // Serial.print("RPM_BR: ");
    // Serial.println(RPM_BR);
    // Serial.print("MPH_BR: ");
    // Serial.println(MPH_BR);
    // Serial.println();

    Serial.print("PPS_BL: ");
    Serial.println(PPS_BL);
    // Serial.print("RPS_BL: ");
    // Serial.println(RPS_BL);
    // Serial.print("RPM_BL: ");
    // Serial.println(RPM_BL);
    // Serial.print("MPH_BL: ");
    // Serial.println(MPH_BL);
    Serial.println();

    // Update old positions for the next calculation
    // oldPosition_FrontRight = newPosition_FrontRight;
    // oldPosition_FrontLeft = newPosition_FrontLeft;

    // Update old positions for the next calculation
    oldPosition_BackRight = newPosition_BackRight;
    oldPosition_BackLeft = newPosition_BackLeft;
    currentTime = millis(); 
  }
}
