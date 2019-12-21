#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <avr/sleep.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LSM9DS1.h>
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
SoftwareSerial mySerial(9, 6); // (TX, RX)
Adafruit_GPS GPS(&mySerial);
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,1000*2F"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_Q_RELEASE "$PMTK605*31"
// Altimeter Setup
Adafruit_BMP3XX bmp(A3, A2, A1,  A0); // CS, OSI, ISO, SCK
/* set to true to only log to SD when GPS has a fix, for debugging, keep it false */
#define LOG_FIXONLY false

// this keeps track of whether we're using the interrupt
// off by default!
#ifndef ESP8266 // Sadly not on ESP8266
boolean usingInterrupt = true;
#endif

// Set the pins used
#define chipSelect 4
#define ledPin 13

File logfile;
char csvLine[128];
char tempLine[84];
char tempVal[8];

// read a Hex value and return the decimal equivalent
//uint8_t parseHex(char c) {
//  if (c < '0')
//    return 0;
//  if (c <= '9')
//    return c - '0';
//  if (c < 'A')
//    return 0;
//  if (c <= 'F')
//    return (c - 'A')+10;
//}

// blink out an error code
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

void setup() {
  // for Leonardos, if you want to debug SD issues, uncomment this line
  // to see serial output
  //while (!Serial);

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  if(!bmp.begin()) error(1);
//  Serial.println("\r\nUltimate GPSlogger Shield");
  pinMode(ledPin, OUTPUT);

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

//  if (!SD.begin(chipSelect)) {
//    Serial.println("Card init. failed!");
//    error(2);
//  }
//  char filename[15];
//  strcpy(filename, "GPSLOG00.csv");
//  for (uint8_t i = 0; i < 100; i++) {
//    filename[6] = '0' + i/10;
//    filename[7] = '0' + i%10;
//    // create if does not exist, do not open existing, write, sync after write
//    if (! SD.exists(filename)) {
//      break;
//    }
//  }
//
//  logfile = SD.open(filename, FILE_WRITE);
//  if( ! logfile ) {
////    Serial.print("Couldnt create ");
////    Serial.println(filename);
//    error(3);
//  }
//  Serial.print("Writing to ");
//  Serial.println(filename);

  // connect to the GPS at the desired rate
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
//  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For logging data, we don't suggest using anything but either RMC only or RMC+GGA
  // to keep the log files at a reasonable size
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 100 millihertz (once every 10 seconds), 1Hz or 5Hz update rate

  // Turn off updates on antenna status, if the firmware permits it
  GPS.sendCommand(PGCMD_NOANTENNA);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
#ifndef ESP8266 // Not on ESP8266
  useInterrupt(true);
#endif

//  Serial.println("Ready!");
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
#ifndef ESP8266 // Not on ESP8266
ISR(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  #ifdef UDR0
      if (GPSECHO)
        if (c) UDR0 = c;
      // writing direct to UDR0 is much much faster than Serial.print
      // but only one character can be written at a time.
  #endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  }
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif // ESP8266

void loop() {
  // if a sentence is received, we can check the checksum, parse it...
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
//    Serial.println("OK");
    if (LOG_FIXONLY && !GPS.fix) {
//      Serial.print("No Fix");
      return;
    }
    if (!bmp.performReading()) return;
    // Rad. lets log it!
    uint8_t stringsize = strlen(stringptr);
//    if (stringsize != logfile.write((uint8_t *)stringptr, stringsize))    //write the string to the SD file
//        error(4);
    dtostrf(bmp.temperature, 8, 5, tempVal);
    sprintf(tempLine,"%s,%s",tempVal,stringptr);
    tempVal[0] = "\0";
    dtostrf(bmp.pressure/100.0, 8, 4, tempVal);
    sprintf(csvLine,"%s,%s",tempVal,tempLine);
    Serial.println(csvLine);
//    strcat(stringptr, csvLine);
//    uint8_t stringsize = strlen(csvLine);
//    if (stringsize != logfile.write((uint8_t *)csvLine, stringsize))    //write the string to the SD file
//        error(4);
//    if (strstr(stringptr, "RMC") || strstr(stringptr, "GGA"))   logfile.flush();
//    Serial.println("LOG");
  }
  csvLine[0] = "\0";
}
