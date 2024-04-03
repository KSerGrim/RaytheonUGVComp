#include <PinChangeInterrupt.h>
#include <LiquidCrystal.h>
#include <Adafruit_GPS.h>

#define GPSSerial Serial2

double distanceToTarget = 30;
float speed = 0;
float originalLat = 0.0;
float originalLon = 0.0;
int initialHeading = 0, previousHeading = 0;
double targetLat, targetLon;
volatile bool ppsFlag = false;
const int rs = 51, rw = 53, en = 39, d0 = 23, d1 = 25, d2 = 27, d3 = 29, d4 = 31, d5 = 33, d6 = 47, d7 = 49;
const int vo_pin = 28;
#define ppsPin 52
LiquidCrystal lcd(rs, rw, en, d0, d1, d2, d3, d4, d5, d6, d7);
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();


void setup() {
  Serial.begin(115200);
  init_Adafruit_GPS();
  init_LCD();
}

void loop()  // run over and over again
{
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    // Serial.print(GPS.lastNMEA());    // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA()))  // this also sets the newNMEAreceived() flag to false
      return;                        // we can fail to parse a sentence in which case we should just wait for another
  }
  
  // read data from the GPS in the 'main loop'
  if (ppsFlag) {
    // Reset the flag
    ppsFlag = false;

    // Synchronize time-sensitive tasks here
    Serial.println("PPS pulse detected, syncing tasks.");
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC);
    Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC);
    Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC);
    Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);

    if (GPS.fix) {
      display_GPS_Data_On_LCD();
    } else {
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("No Signal");
    }
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 250) {
    timer = millis();  // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC);
    Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC);
    Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC);
    Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);

    // if (GPS.fix) {
    //   Serial.print("Location: ");
    //   Serial.print(GPS.latitude, 4);
    //   Serial.print(GPS.lat);
    //   Serial.print(", ");
    //   Serial.print(GPS.longitude, 4);
    //   Serial.println(GPS.lon);
    //   Serial.print("Speed (knots): ");
    //   Serial.println(GPS.speed);
    //   Serial.print("Angle: ");
    //   Serial.println(GPS.angle);
    //   Serial.print("Altitude: ");
    //   Serial.println(GPS.altitude);
    //   Serial.print("Satellites: ");
    //   Serial.println((int)GPS.satellites);
    //   Serial.print("Antenna status: ");
    //   Serial.println((int)GPS.antenna);
    // }
    if (GPS.fix) {
      display_GPS_Data_On_LCD();
    } else {
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("No Signal");
    }
  }
}

void display_GPS_Data_On_LCD() {
  double currentLat = GPS.latitudeDegrees;
  double currentLon = GPS.longitudeDegrees;

  // Distance forward in meters and current heading
  double currentBearing = calculateBearing(originalLat, originalLon, currentLat, currentLon);

  calculateTargetPoint(currentLat, currentLon, distanceToTarget, currentBearing, targetLat, targetLon);

  distanceToTarget = calculateDistance(currentLat, currentLon, targetLat, targetLon);

  float mph;
  mph = (GPS.speed) * 1.15;

  Serial.println("Location in Degrees: ");
  Serial.print(GPS.latitudeDegrees, 8);
  Serial.print(", ");
  Serial.println(GPS.longitudeDegrees, 8);

  Serial.print("Speed (mph): ");
  Serial.println(mph);
  Serial.print("Angle: ");
  Serial.println(GPS.angle);
  Serial.print("Bearing: ");
  Serial.println(currentBearing);
  Serial.print("Satellites: ");
  Serial.println((int)GPS.satellites);
  Serial.print("distanceToTarget: ");
  Serial.println(distanceToTarget);

  //ETCG Notes -- Print to LCD Screen
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Spd: ");
  lcd.setCursor(5, 0);
  lcd.print(mph);

  lcd.setCursor(0, 1);
  lcd.print("Dist: ");
  lcd.setCursor(6, 1);
  lcd.print(distanceToTarget);
  delay(5);
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

void init_LCD() {
  lcd.begin(16, 2);
  digitalWrite(en, HIGH);
  delay(5);
  digitalWrite(en, LOW);
  pinMode(vo_pin, OUTPUT);
  analogWrite(vo_pin, 55);
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
  // Ask for firmware version
  Serial.println(PMTK_Q_RELEASE);

  // Capture original position
  if (GPS.fix) {
    originalLat = GPS.latitudeDegrees;
    originalLon = GPS.longitudeDegrees;
  }

  pinMode(ppsPin, INPUT);  // or INPUT, depending on your wiring
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ppsPin), ppsPulse, RISING);

}
void ppsPulse(void) {
  ppsFlag = true;
}