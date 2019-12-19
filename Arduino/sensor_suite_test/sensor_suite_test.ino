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
SoftwareSerial mySerial(9, 6); // (TX, RX)
Adafruit_GPS GPS(&mySerial);
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,1000*2F"
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

char dat[10];
char dat_string[100];

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
  GPS.begin(9600);
  delay(2000);
  GPS.sendCommand(PMTK_Q_RELEASE);
  // you can send various commands to get it started
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  if (!lsm.begin())
  {
    digitalWrite(13, HIGH);
    while (1);
  }
  accSensor();
}

void loop() {
  strcpy(dat_string, "beep,");
  if (GPS.newNMEAreceived()) {
    strcpy(dat_string, "boop,");
    char *stringptr = GPS.lastNMEA();
  
    if (!GPS.parse(stringptr))   // this also sets the newNMEAreceived() flag to false
        return;
    strcat(dat_string, stringptr);
  }
//  while (mySerial.available()) {
//    char c = mySerial.read();
//    Serial.write(c);
////    strcat(dat_string, c);
//  }
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
//  dtostrf(bmp.temperature * 9.0/5.0 + 32.0, 3, 3, dat);
//  strcat(dat_string, dat);
//  strcat(dat_string, ",");
//  dtostrf(bmp.pressure / 100.0, 3, 3, dat);
//  strcat(dat_string, dat);
//  strcat(dat_string, ",");
//  dtostrf(bmp.readAltitude(SEALEVELPRESSURE_HPA), 3, 3, dat);
//  strcat(dat_string, dat);
//  strcat(dat_string, ",");
//  dtostrf(a.acceleration.x, 3, 3, dat);
//  strcat(dat_string, dat);
//  strcat(dat_string, ",");
//  dtostrf(a.acceleration.y, 3, 3, dat);
//  strcat(dat_string, dat);
//  strcat(dat_string, ",");
//  dtostrf(a.acceleration.z, 3, 3, dat);
//  strcat(dat_string, dat);
//  strcat(dat_string, ",");
//  dtostrf(m.magnetic.x, 3, 3, dat);
//  strcat(dat_string, dat);
//  strcat(dat_string, ",");
//  dtostrf(m.magnetic.y, 3, 3, dat);
//  strcat(dat_string, dat);
//  strcat(dat_string, ",");
//  dtostrf(m.magnetic.z, 3, 3, dat);
//  strcat(dat_string, dat);
//  strcat(dat_string, ",");
//  dtostrf(g.gyro.x, 3, 3, dat);
//  strcat(dat_string, dat);
//  strcat(dat_string, ",");
//  dtostrf(g.gyro.y, 3, 3, dat);
//  strcat(dat_string, dat);
//  strcat(dat_string, ",");
//  dtostrf(g.gyro.z, 3, 3, dat);
//  strcat(dat_string, dat);
  Serial.println(dat_string);
}
