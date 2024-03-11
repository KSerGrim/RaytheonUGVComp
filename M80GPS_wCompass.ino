#include <TinyGPS++.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

TinyGPSPlus gps;
QMC5883LCompass compass;

void setup() {
  Serial.begin(115200); // Start serial communication with the computer
  Serial1.begin(9600); // Start Serial1 to communicate with GPS module
  
  Wire.begin(); // Initialize I2C for compass, if needed
  compass.init();
  Serial.println("This will provide calibration settings for your QMC5883L chip. When prompted, move the magnetometer in all directions until the calibration is complete.");
  Serial.println("Calibration will begin in 5 seconds.");
  delay(5000);
  Serial.println("CALIBRATING. Keep moving your sensor...");
  compass.calibrate();

// compass.setCalibrationOffsets(-1989.00, 1456.00, 1197.00);
// compass.setCalibrationScales(1.01, 1.16, 0.87);

// 2/26 Newest
// compass.setCalibrationOffsets(-410.00, 289.00, -841.00);
// compass.setCalibrationScales(1.03, 0.91, 1.08);

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

}

void loop() {
  readCompassData();
  // while (Serial1.available() > 0)
  //   if (gps.encode(Serial1.read()))
  //     displayGPSData();

  // if (millis() > 5000 && gps.charsProcessed() < 10)
  // {
  //   Serial.println(F("No GPS detected: check wiring."));
  //   while(true);
  // }
  Serial.println();

	delay(250);
}

void readCompassData(){
  int x, y, z, a, b;
	char myArray[3];
	
	compass.read();
  
	x = compass.getX();
	y = compass.getY();
	z = compass.getZ();
	
	a = compass.getAzimuth();
	
	b = compass.getBearing(a);

	compass.getDirection(myArray, a);
  
  
	Serial.print("X: ");
	Serial.print(x);

	Serial.print(" Y: ");
	Serial.print(y);

	Serial.print(" Z: ");
	Serial.print(z);

	Serial.print(" Azimuth: ");
	Serial.print(a);

	Serial.print(" Bearing: ");
	Serial.print(b);

	Serial.print(" Direction: ");
	Serial.print(myArray[0]);
	Serial.print(myArray[1]);
	Serial.print(myArray[2]);
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

  if (gps.altitude.isValid()) {
    Serial.print("Altitude: ");
    Serial.print(gps.altitude.meters());
    Serial.println(" meters");
  } else {
    Serial.println("Altitude: Not Available");
  }

  if (gps.satellites.isValid()) {
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
  } else {
    Serial.println("Satellites: Not Available");
  }

  if (gps.date.isValid()) {
    Serial.print("Date: ");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  } else {
    Serial.println("Date: Not Available");
  }

  if (gps.time.isValid()) {
    Serial.print("Time: ");
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());
  } else {
    Serial.println("Time: Not Available");
  }

  Serial.println();
  delay(1000); // Update rate
}
