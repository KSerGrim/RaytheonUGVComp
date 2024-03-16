//TODO: get compass to work wtih left and right turn
#include <PID_v1.h>
#include <Encoder.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

/*  GLOBAL VARIABLES  */
int pwm;
unsigned long currentTime = 0;
const float PPR = 12.0;  // Encoder Pulses Per Revolution
const float wheelDiameter = 0.0538;  // Wheel diameter in meters
const float pi = 3.14159265358979323846;
const float wheelCircumference = pi * wheelDiameter; // Calculate wheel circumference for distance traveled per revolution
int period = 100;
float distanceTraveled = 0;
float totalRevolutions = 0;

/* DC MOTORS WITH ENCODERS */
// MPU is PWR SDA(20), SCL(21), TX(18), RX(19)
// Front Motor Left motor
Encoder front_Left_Enc(2, 35);
const int FrontAPWM = 12; // PWM pin for motor A speed control
const int front_Motor_A1 = A4;   // Motor A direction pin 1
const int front_Motor_A2 = A5;   // Motor A direction pin 2
float PPS_FL, RPS_FL, RPM_FL;
double MPH_FL;
long int oldPosition_FrontLeft = 0;

// Front Motor Right motor
Encoder front_Right_Enc(3, 37);
const int FrontBPWM = 13; // PWM pin for motor B speed control
const int front_Motor_B1 = A6;   // Motor B direction pin 1
const int front_Motor_B2 = A7;   // Motor B direction pin 2
float PPS_FR, RPS_FR, RPM_FR;
double MPH_FR;
long int oldPosition_FrontRight = 0;

// Back Motor Left motor
Encoder back_Left_Enc(18, 39);
const int BackAPWM = 5; // PWM pin for motor A speed control
const int back_Motor_A1 = A12;   // Motor A direction pin 1
const int back_Motor_A2 = A13;   // Motor A direction pin 2
float PPS_BL, RPS_BL, RPM_BL;
double MPH_BL;
long int oldPosition_BackLeft = 0;

// Back Motor Right motor
Encoder back_Right_Enc(19, 41);
const int BackBPWM = 6; // PWM pin for motor B speed control  
const int back_Motor_B1 = A14;   // Motor B direction pin 1
const int back_Motor_B2 = A15;   // Motor B direction pin 2
float PPS_BR, RPS_BR, RPM_BR;
double MPH_BR;
long int oldPosition_BackRight = 0;

/* M80 GPS /W COMPASS */
TinyGPSPlus gps;
QMC5883LCompass compass;
char initialDirection = 0;
double oldLat, targetLat = 0;
double oldLong, targetLong = 0;

/* PID CONTROLLERS */
double Input_FR = 0, Input_FL = 0, Input_BR = 0, Input_BL = 0; // Input of Motor
double Output_FR, Output_FL, Output_BR, Output_BL = 50; // 
 // Ki Kp Kd
double Setpoint = .17;

//Not calibrated yet
double Setpoint_Comp, Input_Comp, Output_Comp;
double Kp=1.0, Ki=0.05, Kd=0.25;
PID PID_compass(&Input_Comp, &Output_Comp, &Setpoint_Comp, Kp, Ki, Kd, DIRECT);


PID pid_FL(&MPH_FL, &Input_FL, &Setpoint, 50.0, 50.0, 0.00, DIRECT);
PID pid_BR(&MPH_BR, &Input_BR, &Setpoint, 50.0, 50.0, 0.00, DIRECT);
PID pid_BL(&MPH_BL, &Input_BL, &Setpoint, 50.0, 50.0, 0.00, DIRECT);
PID pid_FR(&MPH_FR, &Input_FR, &Setpoint, 50.0, 70.0, 0.00, DIRECT);

void setup() {
  pwm = 75; //Starting Speed

  Serial.begin(115200);
  Wire.begin();

  init_Motors();
  init_PIDs();
  // init_GPS_Compass();
}

void loop() {
  double currentLat, currentLong;

  driveMotorsForward();
  //turnRight();
  turnLeft();
  // driveMotorsForward();
  // turnLeft();
  unsigned long currentMillis = millis(); // Use a temporary variable for the current time
  if(currentMillis - currentTime >= period){
    long int newPosition_FrontRight = front_Right_Enc.read();
    long int newPosition_FrontLeft = front_Left_Enc.read();
    long int newPosition_BackRight = back_Right_Enc.read();
    long int newPosition_BackLeft = back_Left_Enc.read();

    MPH_FR = calculate_Speed_In_MPH(newPosition_FrontRight, oldPosition_FrontRight);
    MPH_FL = calculate_Speed_In_MPH(newPosition_FrontLeft, oldPosition_FrontLeft);
    MPH_BR = calculate_Speed_In_MPH(newPosition_BackRight,oldPosition_BackRight);
    MPH_BL = calculate_Speed_In_MPH(newPosition_BackLeft, oldPosition_BackLeft);
    
    pid_FR.Compute();
    pid_FL.Compute();
    pid_BR.Compute();
    pid_BL.Compute();

    // Apply the PID output to control the motor
    analogWrite(FrontBPWM, Input_FR);
    analogWrite(FrontAPWM, Input_FL);
    analogWrite(BackBPWM, Input_BR);
    analogWrite(BackAPWM, Input_BL);

    calculate_Distance_L();

    // readCompassData();
    // getCurrentGPSPosition(&currentLat, &currentLong);
    // if (Serial3.available() > 0){
    //   char c = Serial3.read(); // Read the incoming byte
    //   if (gps.encode(c))
    //     displayGPSData();
    // }

    // printToPython();

    // Update old positions for the next calculation
    oldPosition_FrontRight = newPosition_FrontRight;
    oldPosition_FrontLeft = newPosition_FrontLeft;
    oldPosition_BackRight = newPosition_BackRight;
    oldPosition_BackLeft = newPosition_BackLeft;

    currentTime = millis();
  }
}

/* MOVEMENT FUNCTIONS */
void driveMotorsForward() {
  // Set the motors to move forward
  digitalWrite(front_Motor_A1, HIGH);
  digitalWrite(front_Motor_A2, LOW);
  digitalWrite(front_Motor_B1, HIGH);
  digitalWrite(front_Motor_B2, LOW);

  digitalWrite(back_Motor_A1, LOW);
  digitalWrite(back_Motor_A2, HIGH);
  digitalWrite(back_Motor_B1, LOW);
  digitalWrite(back_Motor_B2, HIGH);
}

void stopMotors() {
  // Turn off motor PWM signals
  analogWrite(FrontAPWM, 0);
  analogWrite(FrontBPWM, 0);
  analogWrite(BackAPWM, 0);
  analogWrite(BackBPWM, 0);
}
void turnLeft() {
  // Assuming that setting a motor's direction pin HIGH and the other LOW makes it move forward
  // To turn right, the left motors should move forward, and the right motors should move backward
  
  // right motors forward
  digitalWrite(front_Motor_A1, HIGH);
  digitalWrite(front_Motor_A2, LOW);
  digitalWrite(back_Motor_A1, HIGH);
  digitalWrite(back_Motor_A2, LOW);

  // left motors backward
  digitalWrite(front_Motor_B1, LOW);
  digitalWrite(front_Motor_B2, HIGH);
  digitalWrite(back_Motor_B1, LOW);
  digitalWrite(back_Motor_B2, HIGH);

  // Set a speed for turning
  analogWrite(FrontAPWM, pwm); // pwm should be set to the desired speed
  analogWrite(BackAPWM, pwm);
  analogWrite(FrontBPWM, pwm);
  analogWrite(BackBPWM, pwm);
  
  delay(1000); // Placeholder for a delay, should be calibrated for your robot

  // Stop motors after turning
  stopMotors();
}
void turnRight() {
  // To turn left, the right motors should move forward, and the left motors should move backward
  
  // left motors forward
  digitalWrite(front_Motor_B1, HIGH);
  digitalWrite(front_Motor_B2, LOW);
  digitalWrite(back_Motor_B1, HIGH);
  digitalWrite(back_Motor_B2, LOW);

  // right motors backward
  digitalWrite(front_Motor_A1, LOW);
  digitalWrite(front_Motor_A2, HIGH);
  digitalWrite(back_Motor_A1, LOW);
  digitalWrite(back_Motor_A2, HIGH);

  // Set a speed for turning
  analogWrite(FrontAPWM, pwm); // pwm should be set to the desired speed
  analogWrite(BackAPWM, pwm);
  analogWrite(FrontBPWM, pwm);
  analogWrite(BackBPWM, pwm);

  delay(1000); // Placeholder for a delay, should be calibrated for your robot

  // Stop motors after turning
  stopMotors();
}

/* GPS AND COMPASS FUNCTIONS */
void readCompassData(){
  int x, y, z, a, b;
	char myArray[3];
	
	compass.read();
  
	x = compass.getX(); y = compass.getY(); z = compass.getZ();
	a = compass.getAzimuth();
	b = compass.getBearing(a);

	compass.getDirection(myArray, a);
  if( initialDirection == ' '){
    // initialDirection = compass.getDirection(myArray, a);
  }

	Serial.print("X: "); Serial.print(x);
  Serial.print(" Y: "); Serial.print(y);
  Serial.print(" Z: "); Serial.print(z);
  Serial.print(" Azimuth: "); Serial.print(a);
  Serial.print(" Bearing: "); Serial.print(b);
  Serial.print(" Direction: "); Serial.print(myArray[0]); Serial.print(myArray[1]); Serial.print(myArray[2]);
}

float getCurrentGPSPosition(double *latitude, double *longitude){
  // Read and process GPS data
  while (Serial3.available() > 0) {   //Might need to change from while to if in case it is in constant loop
    char c = Serial3.read();
    if (gps.encode(c)) {
      if (gps.location.isValid()) {
        // Process valid GPS location
        displayGPSData();

        *latitude = gps.location.lat();
        *longitude = gps.location.lng();
      }
    }
  }
}

void displayGPSData() {
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6); // 6 decimal places
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("Location: Not Available");
  }

  if (gps.date.isValid()) {
    Serial.print("Date: "); Serial.print(gps.date.month());
    Serial.print("/"); Serial.print(gps.date.day());
    Serial.print("/"); Serial.println(gps.date.year());
  } else {
    Serial.println("Date: Not Available");
  }

  if (gps.time.isValid()) {
		if (gps.time.hour() < 10) Serial.print(F("0"));
		Serial.print(gps.time.hour());
		Serial.print(F(":"));
		if (gps.time.minute() < 10) Serial.print(F("0"));
		Serial.print(gps.time.minute());
		Serial.print(F(":"));
		if (gps.time.second() < 10) Serial.print(F("0"));
		Serial.print(gps.time.second());
		Serial.print(F("."));
		if (gps.time.centisecond() < 10) Serial.print(F("0"));
		Serial.print(gps.time.centisecond());
	} else {
		Serial.print(F("INVALID"));
	}

  Serial.println();
}

/* CALCULATION FUNCTIONS */
float calculate_Speed_In_MPH(long int newPosition, long int oldPosition) {
  float deltaT = (millis() - currentTime); // Time since last position read
  float PPS = 10 * (newPosition - oldPosition) / deltaT; // Pulses per second
  float RPS = abs(PPS) / PPR; // Revolutions per second
  float RPM = RPS * 60; // Revolutions per minute
  float MPH = (wheelCircumference * RPM * 60) / 1609.34; // Miles per hour
  return MPH;
}

float calculate_Distance_L(){
  float PPI = PPR / wheelCircumference;
  distanceTraveled = totalRevolutions / PPI;

  Serial.print("Distance: "); Serial.println(distanceTraveled);
  return distanceTraveled; // Distance traveled in the same unit as the circumference
}

double calculateBearing(double lat1, double long1, double lat2, double long2) {
  double deltaLong = radians(long2 - long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double x = sin(deltaLong) * cos(lat2);
  double y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(deltaLong);
  double bearing = atan2(x, y);
  bearing = degrees(bearing);
  return fmod((bearing + 360), 360); // Normalize to 0-359 degrees
}

/*  INITIATION FUNCTIONS */
void init_Motors(){
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

  analogWrite(FrontAPWM, pwm);
  analogWrite(FrontBPWM, pwm);
  analogWrite(BackAPWM, pwm);
  analogWrite(BackBPWM, pwm);
}

void init_PIDs(){
  pid_FR.SetMode(AUTOMATIC);
  pid_FL.SetMode(AUTOMATIC);
  pid_BR.SetMode(AUTOMATIC);
  pid_BL.SetMode(AUTOMATIC);
  pid_FL.SetSampleTime(100); // Adjust the sample time as needed
  pid_FR.SetSampleTime(100); // Adjust the sample time as needed
  pid_BL.SetSampleTime(100); // Adjust the sample time as needed
  pid_BR.SetSampleTime(100); // Adjust the sample time as needed
  pid_FR.SetOutputLimits(0, 255);
  pid_FL.SetOutputLimits(0, 255);
  pid_BR.SetOutputLimits(0, 255);
  pid_BL.SetOutputLimits(0, 255);
}

void init_GPS_Compass(){
  Serial3.begin(115200);
  compass.init();
  Serial.println("This will provide calibration settings for your QMC5883L chip. When prompted, move the magnetometer in all directions until the calibration is complete.");
  Serial.println("Calibration will begin in 5 seconds.");
  delay(5000);
  Serial.println("CALIBRATING. Keep moving your sensor...");
  compass.calibrate();

  Serial.println("DONE. Copy the lines below and paste it into your projects sketch.);");
  Serial.println();
  Serial.print("compass.setCalibrationOffsets(");
  Serial.print(compass.getCalibrationOffset(0));
  Serial.print(", ");
  Serial.print(compass.getCalibrationOffset(1));
  Serial.print(", ");
  Serial.print(compass.getCalibrationOffset(2));
  Serial.println(");");
  Serial.print("compass.setCalibrationScales(");
  Serial.print(compass.getCalibrationScale(0));
  Serial.print(", ");
  Serial.print(compass.getCalibrationScale(1));
  Serial.print(", ");
  Serial.print(compass.getCalibrationScale(2));
  Serial.println(");");
  // compass.setCalibrationOffsets(-1989.00, 1456.00, 1197.00);
}

/* DEBUG FUNCTIONS */
void printToPython(){
  Serial.print(currentTime);
    Serial.print(" ");Serial.print(MPH_FR);//Serial.print(" ");Serial.print(Input_FR);
    Serial.print(" ");Serial.print(MPH_FL);//Serial.print(" ");Serial.print(Input_FL);
    Serial.print(" ");Serial.print(MPH_BR);//Serial.print(" ");Serial.print(Input_BR);
    Serial.print(" ");Serial.print(MPH_BL);//Serial.print(" ");Serial.print(Input_BL);
    Serial.println();
}