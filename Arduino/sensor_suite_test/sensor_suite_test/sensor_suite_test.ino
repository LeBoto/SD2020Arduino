// Author: Alex Stubbles
// Description: This code echos sensor data from an accelerometer, GPS, thermometer and altimeter to the computer
//
// Thermometer/Altimeter: BPM388
// GPS: Ultimate GPS Breakout v3
// Accelerometer: LSM9DS1 9-DOF
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LSM9DS1.h>
// *******************GPS SETUP*******************
// Connect the GPS TX (transmit) pin to Digital 9
// Connect the GPS RX (receive) pin to Digital 6
SoftwareSerial mySerial(9, 6); // You can change the pin numbers to match your wiring
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,1000*2F"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_Q_RELEASE "$PMTK605*31"
// *******************TEMP/ALT SETUP*******************
#define BMP_SCK A0
#define BMP_MISO A1
#define BMP_MOSI A2
#define BMP_CS A3

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
// *******************ACCEL SETUP*******************
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

String alt;
String accel;
String gps;

void accSensor()
{
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}
void setup() {
  Serial.begin(115200);
  while (!Serial);
  // Start temp/alt
  if (!bmp.begin()) {
    digitalWrite(13, HIGH);
    while (1);
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  mySerial.begin(9600);
  delay(2000);
  mySerial.println(PMTK_Q_RELEASE);
  // you can send various commands to get it started
  mySerial.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  mySerial.println(PMTK_SET_NMEA_UPDATE_10HZ);
  if (!lsm.begin())
  {
    digitalWrite(13, HIGH);
    while (1);
  }
  accSensor();
}

void loop() {
  while (mySerial.available()) {
    char c = mySerial.read();
    Serial.write(c);
  }
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 
  alt = "$ALTTEMP," + bmp.temperature + "," + bmp.pressure / 100.0 + "," + bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(alt);
  Serial.print("$ACC,");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print(a.acceleration.z);
  Serial.print(",");
  Serial.print(m.magnetic.x);
  Serial.print(",");
  Serial.print(m.magnetic.y);
  Serial.print(",");
  Serial.print(m.magnetic.z);
  Serial.print(",");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print(g.gyro.z);
  Serial.println();
}
