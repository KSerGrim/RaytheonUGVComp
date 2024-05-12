#include <ENORDA_RC.h>
#include "ENORDA_FDC2214.h"
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <RTClib.h>
#include <FastLED.h> 
#include <Adafruit_GPS.h>

/*  GLOBAL VARIABLES  */
unsigned long currentTime = 0;

/* DC MOTORS WITH ENCODERS */
int BackLeftPWM = 2;
int BackRightPWM = 3;
const int back_Motor_Left1 = A4;  // Motor A direction pin 1
const int back_Motor_Left2 = A5;  // Motor A direction pin 2
const int back_Motor_Right1 = A6;  // Motor B direction pin 1
const int back_Motor_Right2 = A7;  // Motor B direction pin 2
int Input_BR, Input_BL, rightDir, leftDir;
int killswitch;

#define LED_PIN_1 11
#define LED_NUM 20
#define SFM_PIN 22
#define SD_CHIP_SELECT_PIN 53
#define LOG_FILE_NAME "waterLog.txt"
#define RUDD 35
#define ELEV 37
#define GEAR 39
#define GPSECHO true
#define LEFT_ARDUPILOT_PIN 44
#define RIGHT_ARDUPILOT_PIN 46
// what's the name of the hardware serial port?
#define GPSSerial Serial2

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);
ENORDA_FDC2214 water_sensor(0);
ENORDA_RC rcController(RUDD, ELEV, GEAR); // rudder pin, elevation pin, gear pin
File logFile;
RTC_DS3231 rtc;
DateTime now;
CRGB leds[LED_NUM]; //Fast LED

bool SerialToggle = false;

void setup() {
  // init motor
  pinMode(LEFT_ARDUPILOT_PIN, INPUT);
  pinMode(RIGHT_ARDUPILOT_PIN, INPUT);

  if (SerialToggle == true) {
    Serial.begin(115200);
  }
  
  Wire.begin();
  // Serial.println("This should have been read");
  rtc.begin();
  init_Water_Sensor();
  
  init_LEDS();
  init_SD();
  rcController.init(1458, 1440, 300, 30); // midpoint_elev, midpoint_rudd, difference, threshold
  rcController.setMaximumSpeed(255); // max speed to be mapped to motors
  rcController.setLogicPolarity(1); // forward direction is 1, if 0, forward direction is 0
  delay(15000);
}

void loop() {
  // adams code
  // if en == 0, motors will NOT be updated, but the killswitch will be
  rcController.pollController(killswitch, &Input_BR, &Input_BL, &rightDir, &leftDir, &killswitch);
  pollArdupilot(!killswitch, &Input_BR, &Input_BL, &rightDir, &leftDir);
  driveMotors();

  // log it
  if (SerialToggle == true) {
    Serial.print("rightSpeed:"); Serial.println(Input_BR);
    Serial.print("leftSpeed:"); Serial.println(Input_BL);
    Serial.print("rightDir:"); Serial.println(rightDir * 50); // just so you can see in serial plot
    Serial.print("leftDir:"); Serial.println(leftDir * 50);
    Serial.print("kill:"); Serial.println(killswitch * 75);
    Serial.print("rudd:");Serial.println(rcController.readRudd());
    Serial.print("elev:");Serial.println(rcController.readElev()); 
  }

  // USER CONTROL CODE
  // RETURNS HERE IF KILLSWITCHED
  // if (killswitch) return;

  if (millis() - currentTime >= 75) {
    float ellapsedTime = millis() - currentTime;
    currentTime = millis();

    uint32_t data = water_sensor.i2c_read_fdc2214();  // read raw water sensor data
    double cap_pf = water_sensor.get_capacitance_pf(data);
    // Serial.println(cap_pf);
    check_Water_Sensor(cap_pf);
  }
}

void check_Water_Sensor(double cap_pf) {
  if (cap_pf >= 700.0) {  // Trigger visual and audio cues (400 for hand, 700 for water)
    // Serial.println("I'm in WaterHole");
    logData(true, cap_pf);  // Log "HIT"; NOTE IF LOGDATA FAILS IT WILL JUMP TO PRIOR STATEMENT
    while (1) {
      vaCues();
    }
  } else {
    //logData(false, cap_pf);  // Log "NO HIT"
  }
}

/* VISUAL AND AUDIO FUNCTIONS */
void vaCues() {
  FastLED.setBrightness(100);
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

void logData(bool hit, double capacitance) {
  // Get current time
  // Serial.println("logData");
  // Serial.println("I'm in LogData");
  
// Serial.println("I'm in LogData");
  if (hit) {
    logFile.println();
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
    // logFile.print("NO HIT | ");
    // logFile.print(now.timestamp());  // Print timestamp
    // logFile.print(" | Capacitance: ");
    // logFile.print(capacitance);
    // logFile.print(" pF");
  }
  logFile.println();
  logFile.close();
}

/*  INITIATION FUNCTIONS */


void init_Water_Sensor() {
  pinMode(LED_PIN_1, OUTPUT);  // Set LED pin as output
  pinMode(SFM_PIN, INPUT);     // Set SFM-27 pin as input

  digitalWrite(SDA, LOW);  // disable pull up resistor on SDA
  digitalWrite(SCL, LOW);  // disable pull up resistor on SCL

  water_sensor.i2c_init_fdc2214();  // init water sensor after i2c
}

void init_GPS() {
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);
}

void init_SD(){
  if (!SD.begin(SD_CHIP_SELECT_PIN)) {
    while (1);
  }
  logFile = SD.open(LOG_FILE_NAME, FILE_WRITE);
}
void init_LEDS() {
  FastLED.addLeds<WS2811, LED_PIN_1, BRG>(leds, LED_NUM);
  delay(100);
  fill_solid(leds, LED_NUM, 0xFFFFFF);
  delay(100);
  FastLED.setBrightness(255);
  delay(100);
}

void pollArdupilot(char en, int* rightSpeed, int* leftSpeed, int* rightDir, int* leftDir) {
  if (en == 0) return;

  int readLeft, readRight, readLeftDir, readRightDir;
  int positive_logic = 1;
  int max_speed = 255;
  int MIDPOINT = 1500;
  int DIFF =  500;
  int THRES = 30;

  readLeft = MIDPOINT - pulseIn(LEFT_ARDUPILOT_PIN, HIGH);
  readRight = MIDPOINT - pulseIn(RIGHT_ARDUPILOT_PIN, HIGH);
  
  if (readRight > THRES) readRightDir = positive_logic;
  else if (readRight < (0 - THRES)) readRightDir = !positive_logic;
  else {
    readRightDir = positive_logic;
    readRight = 0;
  }

  if (readLeft > THRES) readLeftDir = positive_logic;
  else if (readLeft < (0 - THRES)) readLeftDir = !positive_logic;
  else {
    readLeftDir = positive_logic;
    readLeft = 0;
  }

  readLeft = map(abs(readLeft), 0, DIFF, 0, max_speed);
  readRight = map(abs(readRight), 0, DIFF, 0, max_speed);

  *leftSpeed = constrain(readLeft, 0, max_speed);
  *rightSpeed = constrain(readRight, 0, max_speed);
  
  *leftDir = readLeftDir;
  *rightDir = readRightDir;  
}

void driveMotors() {
  digitalWrite(back_Motor_Right1, !rightDir);
  digitalWrite(back_Motor_Right2, rightDir);

  digitalWrite(back_Motor_Left1, !leftDir);
  digitalWrite(back_Motor_Left2, leftDir);

  analogWrite(BackRightPWM, Input_BR);
  analogWrite(BackLeftPWM, Input_BL);
}

void stopMotors () {
  Input_BL = 0;
  Input_BR = 0;
  analogWrite(BackRightPWM, 0);
  analogWrite(BackLeftPWM, 0);
}