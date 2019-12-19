// Author: Alex Stubbles
// Description: This code echos sensor data from an accelerometer, GPS, thermometer and altimeter to the computer
//
// Thermometer/Altimeter: BPM388
// GPS: Ultimate GPS Breakout v3
// Accelerometer: LSM9DS1 9-DOF
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <avr/sleep.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
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

# define BUFF 100
#define chipSelect 4
#define ledPin 13

char aMessage[BUFF]; 
char cha;
byte messageSize;
File logfile;

void error(uint8_t errno) {
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}

void accSensor()
{
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}
void setup() {
  Serial.begin(115200);
//  while (!Serial);
  if (!bmp.begin()) {
    digitalWrite(13, HIGH);
    while (1);
  }
  if (!lsm.begin())
  {
    while (1);
  }
  accSensor();
  if (!SD.begin(chipSelect)) {
    Serial.println("Card init. failed!");
    error(2);
  }
  char filename[15];
  strcpy(filename, "GPSLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  Serial.println(filename);
  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    error(3);
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
}

void loop() {
  if (mySerial.available()) {
    for (int c = 0; c <BUFF;c++) aMessage[c]=0;      // clear aMessage in prep for new message
    messageSize = 0;                                 // set message size to zero

    while (mySerial.available()) {            // loop through while data is available
      cha = mySerial.read();                  // get character
      aMessage[messageSize]=cha;              // append to aMessage
      messageSize++;                          // bump message size
      delay(10);                              // just to slow the reads down a bit
    } // while
    aMessage[messageSize]='\n';               // set last character to a null
//    Serial.println(aMessage);
    uint8_t stringsize = strlen(aMessage);
    Serial.println(stringsize);
    if (stringsize != logfile.write((uint8_t *)aMessage, stringsize))    //write the string to the SD file
        error(4);
    if (strstr(aMessage, "RMC"))   logfile.flush();
  } // if available
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
}

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
