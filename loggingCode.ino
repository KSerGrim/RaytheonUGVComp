#include "ENORDA_FDC2214.h"
#include <Wire.h>
#include <FDC1004.h>
#include <SD.h>
#include <SPI.h>
#include <RTClib.h> // Include the RTC library for timekeeping

// Define LED pins
#define LED_PIN_1 13

// Define SFM-27 sound sensor pin
#define SFM_PIN 22

// Define chip select pin for SD card module
#define SD_CHIP_SELECT_PIN 43

// Define file name for logging
#define LOG_FILE_NAME "water_log.txt"

// init water sensor class on channel 0
ENORDA_FDC2214 water_sensor(0);

File logFile; // File object for logging

// Create an RTC object
RTC_DS3231 rtc;

void setup() {
  Serial.begin(9600); // begin serial
  Wire.begin(); // begin i2c
  pinMode(LED_PIN_1, OUTPUT); // Set LED pin as output
  pinMode(SFM_PIN, INPUT); // Set SFM-27 pin as input
  pinMode(SD_CHIP_SELECT_PIN, OUTPUT); // Set SD card chip select pin as output
  digitalWrite(SDA, LOW); // disable pull up resistor on SDA
  digitalWrite(SCL, LOW); // disable pull up resistor on SCL
  
  // Initialize SD card
  if (!SD.begin(SD_CHIP_SELECT_PIN)) {
    Serial.println("Error initializing SD card");
    return;
  }

  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println("Error initializing RTC");
    return;
  }

  // Open log file
  logFile = SD.open(LOG_FILE_NAME, FILE_WRITE);
  if (!logFile) {
    Serial.println("Error opening log file");
    return;
  }
  
  water_sensor.i2c_init_fdc2214(); // init water sensor after i2c
}

// main loop
void loop() {
  uint32_t data = water_sensor.i2c_read_fdc2214(); // read raw water sensor data
  double cap_pf = water_sensor.get_capacitance_pf(data); // convert raw data to capacitance in pF
  Serial.print(cap_pf, DEC); // print
  Serial.print(" pF\n");
  
  // Check if capacitance is above threshold (20pF)
  if (cap_pf >= 20.0) {
    vaCues(); // Trigger visual and audio cues
    logData(true, cap_pf); // Log "HIT"
  } else {
    logData(false, cap_pf); // Log "NO HIT"
  }
  
  delay(1); // Log every millisecond
}

// Visual and audio cues
void vaCues() {
  // Turn on LEDs
  digitalWrite(LED_PIN_1, HIGH);
  
  // Check SFM-27 sensor reading
  int soundLevel = analogRead(SFM_PIN);
  if (soundLevel > 500) { // Adjust threshold as needed
    Serial.println("High sound level detected!"); // For debugging
    tone(9, 1000); // Generate sound on digital pin 9
  }
  
  delay(500); // Sound duration
  
  // Turn off LEDs and stop sound
  digitalWrite(LED_PIN_1, LOW);
  noTone(9);
}

// Log water hit
void logData(bool hit, double capacitance) {
  // Get current time
  DateTime now = rtc.now();

  // Write data to log file
  if (hit) {
    logFile.print("HIT | ");
  } else {
    logFile.print("NO HIT | ");
  }
  logFile.print(now.timestamp()); // Print timestamp
  logFile.print(" | Capacitance: ");
  if (capacitance >= 20.0) {
    logFile.print("<b>");
    logFile.print(capacitance);
    logFile.print(" pF");
    logFile.print("</b>");
  } else {
    logFile.print(capacitance);
    logFile.print(" pF");
  }
  logFile.println();
}
