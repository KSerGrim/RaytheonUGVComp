#include <PinChangeInterrupt.h>
#include <LiquidCrystal.h>
#include "ENORDA_FDC2214.h"
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <RTClib.h>
#include <Adafruit_GPS.h>
#include <PID_v1.h>
#include <Encoder.h>
#include <QMC5883LCompass.h>
#include <FastLED.h>

/*  GLOBAL VARIABLES  */
unsigned long currentTime = 0;
const float PPR = 12.0;              // Encoder Pulses Per Revolution
const float wheelDiameter = 0.0538;  // Wheel diameter in meters
const float pi = 3.14159265358979323846;
const float wheelCircumference = pi * wheelDiameter;  // Calculate wheel circumference for distance traveled per revolution
double distanceToTarget = 30;
// float distanceTravelled = 0;
double speed = .17;
float originalLat = 0.0;
float originalLon = 0.0;
int initialHeading = 0, previousHeading = 0, currentHeading = 0;
volatile bool ppsFlag = false;
volatile int quarterSecTicker = 0;

/* DC MOTORS WITH ENCODERS */
// MPU is PWR SDA(20), SCL(21), TX(18), RX(19)
// Front Motor Left motor
Encoder front_Right_Enc(2, 35);
const int FrontAPWM = 5;         // PWM pin for motor A speed control
const int front_Motor_A1 = A12;  // Motor A direction pin 1
const int front_Motor_A2 = A13;  // Motor A direction pin 2
float PPS_FL, RPS_FL, RPM_FL;
double MPH_FL;
long int oldPosition_FrontLeft = 0;

// Front Motor Right motor
Encoder front_Left_Enc(3, 37);
const int FrontBPWM = 6;         // PWM pin for motor B speed control
const int front_Motor_B1 = A14;  // Motor B direction pin 1
const int front_Motor_B2 = A15;  // Motor B direction pin 2
float PPS_FR, RPS_FR, RPM_FR;
double MPH_FR;
long int oldPosition_FrontRight = 0;

// Back Motor Left motor
Encoder back_Left_Enc(43, 19);
const int BackAPWM = 12;       // PWM pin for motor A speed control
const int back_Motor_A1 = A4;  // Motor A direction pin 1
const int back_Motor_A2 = A5;  // Motor A direction pin 2
float PPS_BL, RPS_BL, RPM_BL;
double MPH_BL;
long int oldPosition_BackLeft = 0;

// Back Motor Right motor
Encoder back_Right_Enc(18, 41);
const int BackBPWM = 13;       // PWM pin for motor B speed control
const int back_Motor_B1 = A6;  // Motor B direction pin 1
const int back_Motor_B2 = A7;  // Motor B direction pin 2
float PPS_BR, RPS_BR, RPM_BR;
double MPH_BR;
long int oldPosition_BackRight = 0;

/* PID CONTROLLERS */
double Input_FR = 0, Input_FL = 0, Input_BR = 0, Input_BL = 0;  // Input of Motor
double Output_FR, Output_FL, Output_BR, Output_BL = 50;         //FRONT motors are flipped

double Setpoint_RightTracks, Setpoint_LeftTracks, Setpoint_BL, Setpoint_Output;
double default_LeftTracks = speed, default_BL = speed;  //left tracks need a speed offset due to mechanical system error
double default_RightTracks = speed;
//kpki = 100 kd = 5.25
// double kpfr = 75.0; double kifr = 75.0; double kdfr = 5.75;
// double kpfl = 75.0; double kifl = 75.0; double kdfl = 5.75;
// double kpbr = 85.0; double kibr = 85.0; double kdbr = 5.75;
// double kpbl = 75.0; double kibl = 75.0; double kdbl = 5.75;

double kpfr = 20.0;
double kifr = 25.0;
double kdfr = 7.5;
double kpfl = 20.0;
double kifl = 30.0;
double kdfl = 6.75;
double kpbr = 20.0;
double kibr = 30.0;
double kdbr = 6.75;
double kpbl = 20.0;
double kibl = 25.0;
double kdbl = 7.75;

// double kp = 50.0; double ki = 50.0; double kd = 5.0;
// Kp Ki Kd
// PID pid_FR(&MPH_FR, &Input_FR, &Setpoint_LeftTracks, kp, ki, kd, DIRECT);   //IS FRONT LEFT****
// PID pid_FL(&MPH_FL, &Input_FL, &Setpoint_RightTracks, kp, ki, kd, DIRECT);  //IS FRONT RIGHT****
// PID pid_BR(&MPH_BR, &Input_BR, &Setpoint_RightTracks, kp, ki, kd, DIRECT);
// PID pid_BL(&MPH_BL, &Input_BL, &Setpoint_LeftTracks, kp, ki, kd, DIRECT);

PID pid_FR(&MPH_FR, &Input_FR, &Setpoint_LeftTracks, kpfr, kifr, kdfr, DIRECT);   //IS FRONT LEFT****
PID pid_FL(&MPH_FL, &Input_FL, &Setpoint_RightTracks, kpfl, kifl, kdfl, DIRECT);  //IS FRONT RIGHT****
PID pid_BR(&MPH_BR, &Input_BR, &Setpoint_RightTracks, kpbr, kibr, kdbr, DIRECT);
PID pid_BL(&MPH_BL, &Input_BL, &Setpoint_LeftTracks, kpbl, kibl, kdbl, DIRECT);

double Setpoint_Comp, Input_Comp, Output_Comp;

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false
#define sdPin 4
#define LED_PIN_1 11
#define LED_NUM 200
#define SFM_PIN 22
#define SD_CHIP_SELECT_PIN 53
#define ppsPin 10
#define BL_Secondary_Interrupt 43
#define LOG_FILE_NAME "water_log_and_GPS.txt"
#define GPSSerial Serial2


Adafruit_GPS GPS(&GPSSerial);
QMC5883LCompass compass;
ENORDA_FDC2214 water_sensor(0);
File logFile;
RTC_DS3231 rtc;
DateTime now;


//Fast LED
CRGB leds[LED_NUM];

//One time execute boolean
bool doOnce = true;
bool Serial_Toggle = true;

void setup() {

  Serial.begin(115200);
  // Wire.begin();

  init_Motors();
  init_PIDs();
  init_Compass_with_PID();
  init_Adafruit_GPS();
  init_LCD();
  init_Water_Sensor();
  init_LEDS();
}

void loop() {
  uint32_t data = water_sensor.i2c_read_fdc2214();  // read raw water sensor data
  double cap_pf = water_sensor.get_capacitance_pf(data);

  if (doOnce) {
    init_Compass_with_PID();
    doOnce = false;
  }

  /* INTERRUPT TO SYNC */
  if (ppsFlag) {
    driveMotorsForward();
    //   // Reset the flag
    ppsFlag = false;
    // readRawMagnetometerData();
    // float azimuth = calculateAzimuth();
    readGPS();
    readCompassData();

    if (GPS.fix) {
      calculate_GPS_Data();
    } else {
      //   lcd.clear();
      //   lcd.setCursor(0, 1);
      //   lcd.print("No Signal");
    }
    //Synchronize time-sensitive tasks here
    //check_Water_Sensor(cap_pf);
  }

  /* With GPS */
  if (distanceToTarget > 2.0) {
    driveMotorsForward();
    // check_Water_Sensor(cap_pf);

  } else {
    stopMotors();
  }

  /* With Encoders */
  // if(distanceTravelled < 100){
  //   driveMotorsForward();
  // } else {
  //     stopMotors();
  // }

  if (millis() - currentTime >= 100) {
    currentTime = millis();
    long int newPosition_FrontRight = front_Right_Enc.read();
    long int newPosition_FrontLeft = front_Left_Enc.read();
    long int newPosition_BackRight = back_Right_Enc.read();
    long int newPosition_BackLeft = back_Left_Enc.read();

    MPH_FR = calculate_Speed_In_MPH(newPosition_FrontRight, oldPosition_FrontRight);
    MPH_FL = calculate_Speed_In_MPH(newPosition_FrontLeft, oldPosition_FrontLeft);
    MPH_BR = calculate_Speed_In_MPH(newPosition_BackRight, oldPosition_BackRight);
    MPH_BL = calculate_Speed_In_MPH(newPosition_BackLeft, oldPosition_BackLeft);

    if (Serial_Toggle = true) {
      Serial.println("Speed");
      Serial.print(MPH_FR);
      Serial.print(" ");
      Serial.print(MPH_FL);
      Serial.print(" ");
      Serial.print(MPH_BR);
      Serial.print(" ");
      Serial.print(MPH_BL);
      Serial.println();
    }


    // double gap = abs(Setpoint_LeftTracks - MPH_BL);  //distance away from setpoint
    // if (gap < .03) {                                 //we're close to setpoint, use conservative tuning parameters
    //   pid_BL.SetTunings(consKd, consKp, consKi);
    // } else {
    //   //we're far from setpoint, use aggressive tuning parameters
    //   pid_BL.SetTunings(aggKd, aggKp, aggKi);
    // }

    compute_PIDs();

    // Apply the PID output to control the motor
    analogWrite(FrontBPWM, Input_FR);
    analogWrite(FrontAPWM, Input_FL);
    analogWrite(BackBPWM, Input_BR);
    analogWrite(BackAPWM, Input_BL);

    // distanceTravelled = calculateDistanceFromEncoder(newPosition_BackRight);

    // Update old positions for the next calculation
    oldPosition_FrontRight = newPosition_FrontRight;
    oldPosition_FrontLeft = newPosition_FrontLeft;
    oldPosition_BackRight = newPosition_BackRight;
    oldPosition_BackLeft = newPosition_BackLeft;
  }
}

/* MOVEMENT FUNCTIONS */
void driveMotorsForward() {
  // Set the motors to move forward
  digitalWrite(front_Motor_A1, HIGH);
  digitalWrite(front_Motor_A2, LOW);

  digitalWrite(front_Motor_B1, HIGH);
  digitalWrite(front_Motor_B2, LOW);

  digitalWrite(back_Motor_B1, LOW);
  digitalWrite(back_Motor_B2, HIGH);

  digitalWrite(back_Motor_A1, LOW);
  digitalWrite(back_Motor_A2, HIGH);
}

void turnMotorsLeft() {
  // Set the motors to move forward
  digitalWrite(front_Motor_A1, HIGH);  // Reverse direction for left front motor
  digitalWrite(front_Motor_A2, LOW);
  digitalWrite(back_Motor_A1, LOW);  // Keep or slow down left back motor
  digitalWrite(back_Motor_A2, HIGH);

  digitalWrite(front_Motor_B1, LOW);  // Right front motor moves forward
  digitalWrite(front_Motor_B2, HIGH);
  digitalWrite(back_Motor_B1, HIGH);  // Right back motor moves forward
  digitalWrite(back_Motor_B2, LOW);
}

void turnMotorsRight() {
  // Keep the left side motors moving forward to initiate a right turn
  digitalWrite(front_Motor_A1, LOW);  // Left front motor moves forward
  digitalWrite(front_Motor_A2, HIGH);
  digitalWrite(back_Motor_A1, HIGH);  // Left back motor moves forward
  digitalWrite(back_Motor_A2, LOW);

  // Slow down or reverse the right side motors to complete the turn
  digitalWrite(front_Motor_B1, HIGH);  // Reverse direction for right front motor
  digitalWrite(front_Motor_B2, LOW);
  digitalWrite(back_Motor_B1, LOW);  // Keep or slow down right back motor
  digitalWrite(back_Motor_B2, HIGH);
}

void stopMotors() {
  digitalWrite(front_Motor_A1, LOW);
  digitalWrite(front_Motor_A2, LOW);
  digitalWrite(front_Motor_B1, LOW);
  digitalWrite(front_Motor_B2, LOW);

  digitalWrite(back_Motor_A1, LOW);
  digitalWrite(back_Motor_A2, LOW);
  digitalWrite(back_Motor_B1, LOW);
  digitalWrite(back_Motor_B2, LOW);
}

/* GPS AND COMPASS FUNCTIONS */
float A[3][3] = {
  { 0.014188, -0.002833, -0.001193 },
  { -0.002833, 0.014178, -0.001184 },
  { -0.001193, -0.001184, 0.012449 }
};

// Hard iron correction vector (3x1)
float b[3] = { -1249.955084, -470.904569, 2750.184400 };  // Replace these with your actual calibration values

float comp_offset_x = b[0], comp_offset_y = b[1], comp_offset_z = b[2];
float comp_scale_x = (A[0][0] + A[0][1] + A[0][2]), comp_scale_y = (A[1][0] + A[1][1] + A[1][2]), comp_scale_z = (A[2][0] + A[2][1] + A[2][2]);

void readCompassData() {
  int currentBearing, input_comp_init = 0;

  compass.read();
  currentHeading = compass.getAzimuth();
  Serial.println(currentHeading);

  // Calculate deviation from initial heading
  Input_Comp = currentHeading - initialHeading;

  //Keep the input within -180 to 180 degrees
  if (Input_Comp > 180) {
    Input_Comp -= 360;
  } else if (Input_Comp < -180) {
    Input_Comp += 360;
  }

  // If deviation is within -3 to 3 degrees, consider it as no deviation
  if (abs(Input_Comp) < 4) {
    Input_Comp = 0;
    if (previousHeading < -3 && (Setpoint_LeftTracks != default_LeftTracks)) {
      //VEERS TO THE RIGHT
      Setpoint_RightTracks += (0.2);  //(Setpoint_RightTracks) * (0.1);
      Setpoint_LeftTracks += (0.1);   //(Setpoint_LeftTracks) * (0.05);
      Setpoint_BL += (0.1);           //(Setpoint_BL) * (0.05);
      Setpoint_RightTracks = constrain(Setpoint_RightTracks, default_RightTracks, default_RightTracks * 1.5);
      Setpoint_BL = constrain(Setpoint_BL, default_BL * 0.9, default_BL * 1.1);
      Setpoint_LeftTracks = constrain(Setpoint_LeftTracks, default_LeftTracks * 0.9, default_LeftTracks * 1.1);
    }
    if (previousHeading > 3 && (Setpoint_RightTracks != Setpoint_RightTracks)) {
      //VEERS TO THE LEFT
      Setpoint_RightTracks += (0.1);  //(Setpoint_RightTracks) * (0.05);//
      Setpoint_LeftTracks += (0.2);   //(Setpoint_LeftTracks) * (0.1);//
      Setpoint_BL += (0.1);           //(Setpoint_BL) * (0.1);//
      Setpoint_RightTracks = constrain(Setpoint_RightTracks, default_RightTracks * 0.9, default_RightTracks * 1.1);
      Setpoint_BL = constrain(Setpoint_BL, default_BL, default_BL * 1.25);
      Setpoint_LeftTracks = constrain(Setpoint_LeftTracks, default_LeftTracks, default_LeftTracks * 1.4);
    }
    Setpoint_LeftTracks = default_LeftTracks;
    Setpoint_RightTracks = default_RightTracks;
    Setpoint_BL = default_BL;
    previousHeading = 0;
  } else if (Input_Comp > 3) {
    Setpoint_LeftTracks += (0.1);   //(Setpoint_LeftTracks) * (0.1);//
    Setpoint_BL += (0.1);           //(Setpoint_BL) * (0.05);//
    Setpoint_RightTracks -= (0.1);  //(Setpoint_RightTracks) * (0.2);//
    Setpoint_BL = constrain(Setpoint_BL, default_BL, default_BL * 1.5);
    Setpoint_LeftTracks = constrain(Setpoint_LeftTracks, default_LeftTracks, default_LeftTracks * 1.75);
    Setpoint_RightTracks = constrain(Setpoint_RightTracks, default_RightTracks * .25, default_RightTracks);
  } else if (Input_Comp < -3) {
    Setpoint_RightTracks += (0.1);  //(Setpoint_RightTracks) * (0.1);//
    Setpoint_LeftTracks -= (0.1);   //(Setpoint_LeftTracks) * (0.2);//
    Setpoint_BL -= (0.1);           //(Setpoint_BL) * (0.25);
    Setpoint_RightTracks = constrain(Setpoint_RightTracks, default_RightTracks, default_RightTracks * 2.0);
    Setpoint_BL = constrain(Setpoint_BL, default_BL * .25, default_BL);
    Setpoint_LeftTracks = constrain(Setpoint_LeftTracks, default_LeftTracks * .25, default_LeftTracks);
  }
  previousHeading = currentHeading;
}

void readGPS() {
  char c = GPS.read();
  if ((c) && (GPSECHO))
    Serial.write(c);
  if (GPS.newNMEAreceived()) {
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA()))  // this also sets the newNMEAreceived() flag to false
      return;                        // we can fail to parse a sentence in which case we should just wait for another
  }
}

void calculate_GPS_Data() {
  double currentLat = GPS.latitudeDegrees;
  double currentLon = GPS.longitudeDegrees;
  double targetLat, targetLon;

  calculateTargetPoint(currentLat, currentLon, distanceToTarget, initialHeading, targetLat, targetLon);

  double currentBearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);

  distanceToTarget = calculateDistance(currentLat, currentLon, targetLat, targetLon);

  // float mph;
  // mph = (GPS.speed) * 1.15;

  // Serial.print("Speed (mph): ");
  // Serial.println(mph);
  // Serial.print("Angle: ");
  // Serial.println(GPS.angle);
  // Serial.print("Bearing: ");
  // Serial.println(currentBearing);
  // Serial.print("distanceToTarget: ");
  // Serial.println(distanceToTarget);
}

void check_Water_Sensor(double cap_pf) {
  if (cap_pf >= 700.0) {  // Trigger visual and audio cues (400 for hand, 700 for water)
    //logData(true, cap_pf);  // Log "HIT"; NOTE IF LOGDATA FAILS IT WILL JUMP TO PRIOR STATEMENT
    while (1 > 0) {
      vaCues();
    }
  } else {
    //logData(false, cap_pf);  // Log "NO HIT"
  }
}

/* CALCULATION FUNCTIONS */
float calculate_Speed_In_MPH(long int newPosition, long int oldPosition) {
  float deltaT = 100.00;                                     // Time since last position read
  float PPS = 10.00 * (newPosition - oldPosition) / deltaT;  // Pulses per second
  float RPS = abs(PPS) / PPR;                                // Revolutions per second
  float RPM = RPS * 60.00;                                   // Revolutions per minute
  float MPH = (wheelCircumference * RPM * 60.00) / 1609.34;  // Miles per hour
  return MPH;
}

double calculateBearing(double lat1, double long1, double lat2, double long2) {
  double deltaLong = radians(long2 - long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double x = sin(deltaLong) * cos(lat2);
  double y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(deltaLong);
  double bearing = atan2(x, y);
  bearing = degrees(bearing);
  return fmod((bearing + 360), 360);  // Normalize to 0-359 degrees
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000;  // Earth's radius in meters
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);

  double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = R * c;

  return distance;  // Distance in meters
}

float calculateDistanceFromEncoder(long int totalPulses) {
  float totalRevolutions = totalPulses / (float)PPR;
  float distanceTraveled = totalRevolutions * wheelCircumference;  // Distance = number of revolutions * circumference

  return distanceTraveled;  // Distance traveled in the same unit as the circumference
}

void calculateTargetPoint(double startLat, double startLon, double distance, double bearing, double& targetLat, double& targetLon) {
  double R = 6378137;  // Earth's radius in meters
  double bearingRad = radians(bearing);
  double dByR = distance / R;

  double lat1 = radians(startLat);
  double lon1 = radians(startLon);

  targetLat = asin(sin(lat1) * cos(dByR) + cos(lat1) * sin(dByR) * cos(bearingRad));
  targetLon = lon1 + atan2(sin(bearingRad) * sin(dByR) * cos(lat1), cos(dByR) - sin(lat1) * sin(targetLat));

  targetLat = degrees(targetLat);
  targetLon = degrees(targetLon);
}

/* VISUAL AND AUDIO FUNCTIONS */
void vaCues() {
  fill_solid(leds, LED_NUM, CRGB::Green);
  stopMotors();
  FastLED.show();
  int soundLevel = analogRead(SFM_PIN);  // Adjust threshold as needed
  tone(SFM_PIN, 1000);                   // Generate sound on digital pin 9
  delay(500);                            // Sound duration
  noTone(SFM_PIN);
  fill_solid(leds, LED_NUM, CRGB::Yellow);
  FastLED.show();
  delay(500);
  tone(SFM_PIN, 1000);
  fill_solid(leds, LED_NUM, CRGB::White);
  FastLED.show();
  delay(500);
  noTone(SFM_PIN);
}

/* 
The UGV logs a message of a Ã¢ÂÂSoakÃ¢ÂÂ with the following information:
i. RTXDC_2024
ii. Team Name [e.g. UTA, UTD, UT, UTEP, SMU, TAMU]
iii. Drone Type
iv. Action: Soaked!
v. ArUco Marker ID
vi. Time
vii. GPS Location
*/
// Log water hit
void logData(bool hit, double capacitance) {
  // Get current time
  now = rtc.now();

  if (hit) {
    logFile.println("RTXDC_2024");
    logFile.print("UTD_UGV_Soaked!_00");
    logFile.print(now.timestamp());  // Print timestamp
    double currentLat = GPS.latitudeDegrees;
    double currentLon = GPS.longitudeDegrees;
    logFile.print("_");
    logFile.print(currentLat);
    logFile.print("_");
    logFile.print(currentLon);
  } else {
    logFile.print("NO HIT | ");
    logFile.print(now.timestamp());  // Print timestamp
    logFile.print(" | Capacitance: ");
    logFile.print(capacitance);
    logFile.print(" pF");
  }
  logFile.println();
}

/*  INITIATION FUNCTIONS */
void init_Motors() {
  pinMode(FrontAPWM, OUTPUT);
  pinMode(front_Motor_A1, OUTPUT);
  pinMode(front_Motor_A2, OUTPUT);

  pinMode(FrontBPWM, OUTPUT);
  pinMode(front_Motor_B1, OUTPUT);
  pinMode(front_Motor_B2, OUTPUT);

  pinMode(BackAPWM, OUTPUT);
  pinMode(back_Motor_A1, OUTPUT);
  pinMode(back_Motor_A2, OUTPUT);

  pinMode(BackBPWM, OUTPUT);
  pinMode(back_Motor_B1, OUTPUT);
  pinMode(back_Motor_B2, OUTPUT);

  analogWrite(FrontAPWM, 150);
  analogWrite(FrontBPWM, 150);
  analogWrite(BackAPWM, 150);
  analogWrite(BackBPWM, 200);

  // pinMode(BL_Secondary_Interrupt, INPUT);  // or INPUT, depending on your wiring
  // attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(BL_Secondary_Interrupt), BL_ISR2, RISING);
}

void init_PIDs() {
  pid_FR.SetMode(AUTOMATIC);
  pid_FL.SetMode(AUTOMATIC);
  pid_BR.SetMode(AUTOMATIC);
  pid_BL.SetMode(AUTOMATIC);
  pid_FL.SetSampleTime(50);  // Adjust the sample time as needed
  pid_FR.SetSampleTime(50);  // Adjust the sample time as needed
  pid_BL.SetSampleTime(50);  // Adjust the sample time as needed
  pid_BR.SetSampleTime(50);  // Adjust the sample time as needed
  pid_FR.SetOutputLimits(0, 255);
  pid_FL.SetOutputLimits(0, 255);
  pid_BR.SetOutputLimits(0, 255);
  pid_BL.SetOutputLimits(0, 255);
}

void init_Compass_with_PID() {
  Setpoint_Comp = 0;
  compass.init();
  compass.setMagneticDeclination(2, 38);
  compass.setSmoothing(5, true);
  compass.setCalibrationOffsets(comp_offset_x, comp_offset_y, comp_offset_z);
  compass.setCalibrationScales(comp_scale_x, comp_scale_y, comp_scale_z);
  compass.read();

  initialHeading = compass.getAzimuth();
  Setpoint_Comp = initialHeading;

  Setpoint_LeftTracks = default_LeftTracks;
  Setpoint_RightTracks = default_RightTracks;
  Setpoint_BL = default_BL;
}

void init_Adafruit_GPS() {
  GPS.begin(9600);  // Initialize GPS module

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // Request RMC and GGA NMEA sentences
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Capture original position
  if (GPS.fix) {
    originalLat = GPS.latitudeDegrees;
    originalLon = GPS.longitudeDegrees;
  }

  // pinMode(ppsPin, INPUT);  // or INPUT, depending on your wiring
  // attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ppsPin), ppsPulse, RISING);
}

void init_LCD() {
  //VSS & K = 0V, VDD = 3.3V, A = 5V
  const int rs = 30, rw = 32, en = 39, d0 = 23, d1 = 25, d2 = 27, d3 = 29, d4 = 31, d5 = 33, d6 = 47, d7 = 49;
  const int vo_pin = 28;
  LiquidCrystal lcd(rs, rw, en, d0, d1, d2, d3, d4, d5, d6, d7);
  pinMode(SD_CHIP_SELECT_PIN, OUTPUT);  // Set SD card chip select pin as output
  lcd.begin(16, 2);
  digitalWrite(en, HIGH);
  delay(5);
  digitalWrite(en, LOW);
  pinMode(vo_pin, OUTPUT);
  analogWrite(vo_pin, 15);

  //ETCG Notes -- Uncomment While Loop if You REQUIRE SD Card

  if (!SD.begin(SD_CHIP_SELECT_PIN)) {
    lcd.setCursor(0, 0);
    lcd.print("SD Card Not Found");
    delay(100);
  }


  //ETCG Notes -- Write to SD Card
  logFile = SD.open(LOG_FILE_NAME, FILE_WRITE);
  if (!logFile) {
    // Serial.println("Error opening log file");
    return;
  }
}

void init_Water_Sensor() {
  pinMode(LED_PIN_1, OUTPUT);  // Set LED pin as output
  pinMode(SFM_PIN, INPUT);     // Set SFM-27 pin as input

  digitalWrite(SDA, LOW);  // disable pull up resistor on SDA
  digitalWrite(SCL, LOW);  // disable pull up resistor on SCL

  water_sensor.i2c_init_fdc2214();  // init water sensor after i2c
}

void compute_PIDs() {
  pid_FR.Compute();
  pid_FL.Compute();
  pid_BR.Compute();
  pid_BL.Compute();
}

void ppsPulse(void) {
  ppsFlag = true;
}

void BL_ISR2(void) {
  back_Left_Enc.interruptArgs;
}

void init_LEDS() {
  FastLED.addLeds<WS2811, LED_PIN_1, BRG>(leds, LED_NUM);
  FastLED.setBrightness(50);
}

/* DEBUG FUNCTIONS */
void printToPython() {
  Serial.print(currentTime);
  Serial.print(" ");
  Serial.print(MPH_FR);  //Serial.print(" ");Serial.print(Input_FR);
  Serial.print(" ");
  Serial.print(MPH_FL);  //Serial.print(" ");Serial.print(Input_FL);
  Serial.print(" ");
  Serial.print(MPH_BR);  //Serial.print(" ");Serial.print(Input_BR);
  Serial.print(" ");
  Serial.print(MPH_BL);  //Serial.print(" ");Serial.print(Input_BL);
  Serial.println();
}