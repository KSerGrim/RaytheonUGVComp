#include <PID_v1.h>

#include <Encoder.h>
#include <util/atomic.h>


// MPU is PWR SDA(20), SCL(21), TX(18), RX(19)
double Setpoint = 100; // Desired speed of wheel 1
double input1; // Current speed of wheel 1
double output1; // Output to adjust speed of wheel 1

// Define PID controllers for each wheel
PID pid_FR(&input1, &output1, &Setpoint, 2.0, 1.0, 0.0, DIRECT);

// Front Motor A (Left motor)
Encoder front_Left_Enc(20, 35);
const int FrontAPWM = 2; // PWM pin for motor A speed control
const int front_Motor_A1 = A4;   // Motor A direction pin 1
const int front_Motor_A2 = A5;   // Motor A direction pin 2

// Front Motor B (Right motor)
Encoder front_Right_Enc(21, 37);
const int FrontBPWM = 3; // PWM pin for motor B speed control
const int front_Motor_B1 = A6;   // Motor B direction pin 1
const int front_Motor_B2 = A7;   // Motor B direction pin 2

// Back Motor A (Left motor)
Encoder back_Left_Enc(18, 39);
const int BackAPWM = 6; // PWM pin for motor A speed control
const int back_Motor_A1 = A14;   // Motor A direction pin 1
const int back_Motor_A2 = A15;   // Motor A direction pin 2

// Back Motor B (Right motor)
Encoder back_Right_Enc(19, 41);
const int BackBPWM = 5; // PWM pin for motor B speed control  
const int back_Motor_B1 = A12;   // Motor B direction pin 1
const int back_Motor_B2 = A13;   // Motor B direction pin 2

// Time tracking
unsigned long lastTime;
unsigned long currentTime = 0; // Corrected: Declare currentTime to store the last update time
const float PPR = 20.0;  // Encoder Pulses Per Revolution
const float wheelDiameter = 0.065;  // Wheel diameter in meters
const float pi = 3.14159265358979323846;
const float wheelCircumference = pi * wheelDiameter; // Calculate wheel circumference for distance traveled per revolution
int period = 250;
const long targetSpeedPPS = 1500; // Target speed in Pulses Per Second (PPS)

const float distanceMeters = 9.164;  // 10 yards in meters
const float gearRatio = 1.0;  // Adjust this if there's a gear reduction between the motor and encoder

long int oldPosition_FrontRight = 0; // Initialized to store the last position
long int oldPosition_FrontLeft = 0;

long int oldPosition_BackRight = 0; // Initialized to store the last position
long int oldPosition_BackLeft = 0;

float PPS_FR, RPS_FR, RPM_FR, MPH_FR;
float PPS_FL, RPS_FL, RPM_FL, MPH_FL;

float PPS_BR, RPS_BR, RPM_BR, MPH_BR;
float PPS_BL, RPS_BL, RPM_BL, MPH_BL;

volatile float old_RPM_FL = 0;
volatile float old_RPM_FR = 0;
volatile float old_RPM_BL = 0;
volatile float old_RPM_BR = 0;


// Calculate the number of revolutions needed to cover the distance
float revolutionsRequired = distanceMeters / wheelCircumference;

// Calculate the total number of pulses needed to cover the distance
long pulsesNeeded = round(revolutionsRequired * PPR * gearRatio);

int pwm = 50; // Initial PWM for motor A
// int back_pwmA = 0; // Initial PWM for motor A
// int back_pwmB = 0; // Initial PWM for motor B



float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pid_FR.SetMode(AUTOMATIC);
  pid_FR.SetSampleTime(100); // Adjust the sample time as needed
  pid_FR.SetOutputLimits(0, 255);
  Setpoint = 50;
  lastTime = millis();
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
  // pinMode(front_Encoder_A1, INPUT);
  // pinMode(front_Encoder_A2, INPUT);
  // pinMode(front_Encoder_B1, INPUT);
  // pinMode(front_Encoder_B2, INPUT);
  // pinMode(back_Encoder_A1, INPUT);
  // pinMode(back_Encoder_A2, INPUT);
  // pinMode(back_Encoder_B1, INPUT);
  // pinMode(back_Encoder_B2, INPUT);

  // // Configure encoder pins as inputs and attach interrupts
  // pinMode(back_Encoder_A1, INPUT_PULLUP);
  // pinMode(back_Encoder_A2, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(back_Encoder_A1), countEncoderA, RISING);

  // pinMode(front_Encoder_B1, INPUT_PULLUP);
  // pinMode(front_Encoder_B2, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(front_Encoder_B1), countEncoderB, RISING);
  
}

void loop() {
  // read the position in an atomic block
  // to avoid potential misreads
  
  // ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
  //   old_RPM_FL = RPM_FL;
  //   old_RPM_FR = RPM_FR;
  //   old_RPM_BL = RPM_BL;
  //   old_RPM_BR = RPM_BR;
  //   }
  // delay(10000);
    driveMotorsForward();
    // delay(10000);
    // spin();
    // delay(10000);
    // stopMotors();
  unsigned long currentMillis = millis(); // Use a temporary variable for the current time
  if(currentMillis - currentTime >= period){
    // input1 = front_Right_Enc.read();
    long int newPosition_FrontRight = front_Right_Enc.read();
    long int newPosition_FrontLeft = front_Left_Enc.read();
    long int newPosition_BackRight = back_Right_Enc.read();
    long int newPosition_BackLeft = back_Left_Enc.read();

    // Average the difference in positions for both encoders
    // float avgPositionDifference = ((newPosition_FrontRight - oldPosition_FrontRight) + (newPosition_FrontLeft - oldPosition_FrontLeft)) / 2.0;
    // PPS = avgPositionDifference * 1000.0 / (currentMillis - currentTime); // Calculate PPS based on the average position difference
    
    float deltaT = (millis() - currentTime);
    PPS_FR = calculateSpeed(newPosition_FrontRight, oldPosition_FrontRight, deltaT);
    input1 = PPS_FR;
    pid_FR.Compute();
    // Apply the PID output to control the motor
    analogWrite(FrontBPWM, output1);
    // adjust_EncodersWithPID(PPS_FR, targetSpeedPPS, FrontBPWM);
    PPS_FL = calculateSpeed(newPosition_FrontLeft, oldPosition_FrontLeft, deltaT);
    PPS_BR = calculateSpeed(newPosition_BackRight,oldPosition_BackRight, deltaT);
    PPS_BL = calculateSpeed(newPosition_BackLeft, oldPosition_BackLeft, deltaT);
    Serial.print(PPS_FR);
    // Serial.print(" ");Serial.print(PPS_FL);
    // Serial.print(" ");Serial.print(PPS_BR);
    // Serial.print(" ");Serial.print(PPS_BL);
    Serial.println();
    
    RPS_FR = calculateRPS(PPS_FR); // Calculate RPS
    RPS_FL = calculateRPS(PPS_FL); // Calculate RPS
    RPS_BR = calculateRPS(PPS_BR); // Calculate RPS
    RPS_BL = calculateRPS(PPS_BL); // Calculate RPS
    
    RPM_FR = calculateRPM(RPS_FR);
    RPM_FL = calculateRPM(RPS_FL);
    RPM_BR = calculateRPM(RPS_BR);
    RPM_BL = calculateRPM(RPS_BL);
    // runFilter(RPM_FL, old_RPM_FL, deltaT, FrontAPWM, front_Motor_A1, front_Motor_A2, newPosition_FrontLeft, oldPosition_FrontLeft);
    // runFilter(RPM_FR, old_RPM_FR, deltaT, FrontBPWM, front_Motor_B1, front_Motor_B2, newPosition_FrontLeft, oldPosition_FrontRight);

    
    MPH_FR = calculateMPH(RPM_FR); // Convert meters per minute to miles per hour
    MPH_FL = calculateMPH(RPM_FL); // Convert meters per minute to miles per hour
    MPH_BR = calculateMPH(RPM_BR); // Convert meters per minute to miles per hour
    MPH_BL = calculateMPH(RPM_BL); // Convert meters per minute to miles per hour
    // Serial.print(" PPS_FR: ");
    // Serial.print(PPS_FR);
    // Serial.print(" \tPPS_BL: ");
    // Serial.print(PPS_FL);
    // Serial.print(" PPS_BR: ");

    // Serial.print(" PPS_BR: ");
    // Serial.print(PPS_BR);
    // Serial.print(" \tPPS_BL: ");
    // Serial.print(PPS_BL);
    // Serial.println();

    // Update old positions for the next calculation
    oldPosition_FrontRight = newPosition_FrontRight;
    oldPosition_FrontLeft = newPosition_FrontLeft;

    // Update old positions for the next calculation
    oldPosition_BackRight = newPosition_BackRight;
    oldPosition_BackLeft = newPosition_BackLeft;
    currentTime = millis();
  }
}

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

  analogWrite(FrontAPWM, pwm);
  analogWrite(FrontBPWM, pwm);
  analogWrite(BackAPWM, pwm);
  analogWrite(BackBPWM, pwm);
}

void stopMotors() {
  // Turn off motor PWM signals
  analogWrite(FrontAPWM, 0);
  analogWrite(FrontBPWM, 0);
  analogWrite(BackAPWM, 0);
  analogWrite(BackBPWM, 0);
}

// PID Constants
const float Kp = 1; // Proportional gain - Adjust based on testing
const float Ki = 0.0; // Integral gain - For future implementation
const float Kd = 0.0; // Derivative gain - For future implementation
float eintegral = 0;

long previousError = 0;
long integral = 0;

void adjust_EncodersWithPID(long currentSpeed, long targetSpeed, int PWM, float deltaT) {
  long error = currentSpeed - targetSpeed;
  // integral += error;
  long derivative = (error - previousError)/deltaT;

  integral = eintegral * e*deltaT;
  
  long adjustment = Kp * error + Ki * integral + Kd * derivative;
  float pwr = fabs(adjustment);

  int newPWM = adjustPWM(pwm, pwr); // Assuming pwmA is the PWM value for the motor you're adjusting
  analogWrite(PWM, newPWM); // Assuming BackAPWM is the PWM pin for the motor
  
  previousError = error;
}

int adjustPWM(int currentPWM, long adjustment) {
  int newPWM = currentPWM + adjustment;
  return constrain(newPWM, 0, 255); // Keep PWM within bounds
}

float calculateSpeed(long int newPosition, long int oldPosition, float deltaT) {
  float PPS = (newPosition - oldPosition) / (1000/deltaT);
  // Serial.print("PPS: ");
  // Serial.println(PPS);
  return PPS; // Placeholder
}

float calculateRPS(float PPS){
  float RPS = PPS / PPR; // Calculate RPS
  // Serial.print("RPS: ");
  // Serial.println(RPS);
  return RPS;
}

float calculateRPM(float RPS){
  float RPM = RPS * 60;
  // Serial.print("RPM: ");
  // Serial.println(RPM);
  return RPM;
}

float calculateMPH(float RPM){
  float MPH = (wheelCircumference * RPM * 60) / 1609.34; // Convert meters per minute to miles per hour
  // Serial.print("MPH: ");
  // Serial.println(MPH);
  return MPH;
}

void spin() {
  // Set the motors to move forward

  analogWrite(FrontAPWM, 255);
  analogWrite(FrontBPWM, 255);
  analogWrite(BackAPWM, 255);
  analogWrite(BackBPWM, 255);

  digitalWrite(front_Motor_A1, LOW);
  digitalWrite(front_Motor_A2, HIGH);
  digitalWrite(back_Motor_A1, HIGH);
  digitalWrite(back_Motor_A2, LOW);
  
  digitalWrite(front_Motor_B1, HIGH);
  digitalWrite(front_Motor_B2, LOW);
  digitalWrite(back_Motor_B1, LOW);
  digitalWrite(back_Motor_B2, HIGH);

}

void runFilter(float v1, float v2, float deltaT, int PWM, int IN1, int IN2, long int EncoderA_Counts, long int EncoderB_Counts){
  // Compute velocity with method 1
  long currT = millis();

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;
  
  long error = EncoderA_Counts - EncoderB_Counts; // Calculate the difference in encoder counts
  
  integral += error; // Accumulate the error over time (Integral part)
  long derivative = error - previousError; // Calculate the rate of error change (Derivative part)

  // Set a target
  float vt = 100*(sin(currT/1e6)>0);
  // Compute the control signal u
  float kp = 5;
  float kd = 0;
  float ki = 0;
  float e = vt-v1Filt;
  eintegral = eintegral + e*deltaT;
  
  float u = kp*e + ki*eintegral + kd*derivative;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  setMotor(dir,pwr,PWM,IN1,IN2);

  Serial.print(u);
  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.println();
  previousError = error; // Remember the error for next time
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}