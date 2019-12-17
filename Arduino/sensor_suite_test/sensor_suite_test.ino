// Author: Alex Stubbles
// Description: This code echos sensor data from an accelerometer, GPS, thermometer and altimeter to the computer
//
// Thermometer/Altimeter: BPM388
// GPS: Ultimate GPS Breakout v3
// Accelerometer: LSM9DS1 9-DOF
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <avr/sleep.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LSM9DS1.h>
// *******************GPS SETUP*******************
// Connect the GPS TX (transmit) pin to Digital 9
// Connect the GPS RX (receive) pin to Digital 6
SoftwareSerial mySerial(9, 6); // You can change the pin numbers to match your wiring
Adafruit_GPS GPS(&mySerial);
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,1000*2F"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
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

// *******************SD LOG SETUP*******************
// Set the pins used
#define chipSelect 4
#define ledPin 13

File logfile;

// read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) {
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 0;
  if (c <= 'F')
    return (c - 'A')+10;
}

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
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
  // 3.) Setup the gyroscope
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}
void setup() {
  Serial.begin(115200);
  while (!Serial);
  pinMode(ledPin, OUTPUT);
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  Serial.println("Sensor Suit Test");
  // Start temp/alt
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
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
  mySerial.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  mySerial.println(PMTK_SET_NMEA_UPDATE_10HZ);
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
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
  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("Couldnt create ");
    Serial.println(filename);
    error(3);
  }
  Serial.print("Writing to ");
  Serial.println(filename);
}

void loop() {
  while (mySerial.available()) {
    char c = mySerial.read();
    Serial.write(c);
  }
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 

  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data

    // Don't call lastNMEA more than once between parse calls!  Calling lastNMEA
    // will clear the received flag and can cause very subtle race conditions if
    // new data comes in before parse is called again.
    char *stringptr = GPS.lastNMEA();

    if (!GPS.parse(stringptr))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another

    // Sentence parsed!
    Serial.println("Log");

    uint8_t stringsize = strlen(stringptr);
    if (stringsize != logfile.write((uint8_t *)stringptr, stringsize))    //write the string to the SD file
        error(4);
    if (strstr(stringptr, "RMC") || strstr(stringptr, "GGA"))   logfile.flush();
    Serial.println();
  }
}
