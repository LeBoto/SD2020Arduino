// Including all libraries for the SD card, altimeter, and GPS
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <avr/sleep.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"


// GPS setup
SoftwareSerial mySerial(9, 6); // (TX, RX)
Adafruit_GPS GPS(&mySerial);
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,1000*2F"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_Q_RELEASE "$PMTK605*31"


// Altimeter Setup
Adafruit_BMP3XX bmp(A3, A2, A1,  A0); // CS, OSI, ISO, SCK


// SD card Setup
#define chipSelect 4
#define ledPin 13
#define savePin 8
byte currentBuff;
const byte numBuff = 128;
const byte numGPS = 80;
const byte numTemp = 4;
char log_buff[numBuff];
char gps_buff[numGPS];
char temp[numTemp]; 
char gpsChar;
File logfile;

bool checksum(char pos[numGPS]){
  byte start_with = 0;
  byte end_with = 0;
  byte CRC = 0;
  for ( int index = 0; index < 80; index++) {
    gpsChar = pos[index];

    if ( gpsChar == '$') {
      start_with = index;
    }

    if (gpsChar == '*') {
      end_with = index;
    }
  }
  for (byte x = start_with + 1; x < end_with; x++) { // XOR every character in between '$' and '*'
    CRC = CRC ^ pos[x];
  }
  if (CRC == 0) return true;
  else false;
}
void error(uint8_t errno) {
  // Function that will blink "errno" amount of times when called
  /* Error convention:
   *  1-3: Startup error [Alt, GPS, SD]
   *  4: Failure to open log
   *  5: Failure to write to SD
   */
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


void setup() {
  Serial.begin(115200); // Start Serial connnection
  if(!bmp.begin()) error(1); // Start Altimeter
  if(!GPS.begin(9600)) error(2); // Start GPS
  if (!SD.begin(chipSelect)) error(3); // Start SD card
  // create if does not exist, do not open existing, write, sync after write
  char filename[15];
  strcpy(filename, "GPSLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    if (! SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE); // Open file
  if(!logfile ) error(4);

  // Set up GPS and Altimeter
  GPS.sendCommand(PMTK_Q_RELEASE);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
}

void loop() {
  digitalWrite(13, LOW);
  if (!bmp.performReading()) return; // Attempt to read data from Altimeter
  if (mySerial.available()) { // Checking for GPS data
    currentBuff = 0;                          // Start writing at the beggining of the char array
    while (mySerial.available()) {            // loop through while data is available
      gpsChar = mySerial.read();              // get character
      gps_buff[currentBuff] = gpsChar;        // append to aMessage
      currentBuff++;                          // bump message size
      delay(5);                               // just to slow the reads down a bit
    }
    if(!checksum(gps_buff)) return;
    Serial.print(gps_buff);
//    uint8_t stringsize = strlen(gps_buff);
//    if (stringsize != logfile.write((uint8_t *)gps_buff, stringsize)) error(5);  //write the string to the SD file
//    if (strstr(gps_buff, "RMC"))   logfile.flush();
    gps_buff[0] = '\0';
  }
}
