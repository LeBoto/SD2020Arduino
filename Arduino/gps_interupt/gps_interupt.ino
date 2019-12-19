// This code just echos whatever is coming from the GPS unit to the
// serial monitor, handy for debugging!
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(9, 6);
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;

void setup()  
{    
 // connect at 115200 so we can read the GPS fast enuf and
 // also spit it out
 Serial.begin(115200);
 Serial.println("Adafruit GPS library basic test!");

 // 9600 NMEA is the default baud rate for MTK - some use 4800
 GPS.begin(9600);
 
 //turn on RMC (recommended minimum) and GGA (fix data) including altitude
 GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 
 // Set the update rate
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

 // Request updates on antenna status, comment out to keep quiet
 GPS.sendCommand(PGCMD_ANTENNA);

 // the nice thing about this code is you can have a timer0 interrupt go off
 // every 1 millisecond, and read data from the GPS for you. that makes the
 // loop code a heck of a lot easier!
 useInterrupt(true);
 
 delay(1000);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect)
{
 char c = GPS.read();
 // if you want to debug, this is a good time to do it!
 if (GPSECHO)
   if (c) UDR0 = c;  
   // writing direct to UDR0 is much much faster than Serial.print
   // but only one character can be written at a time.
}

void useInterrupt(boolean v)
{
 if (v)
 {
   // Timer0 is already used for millis() - we'll just interrupt somewhere
   // in the middle and call the "Compare A" function above
   OCR0A = 0xAF;
   TIMSK0 |= _BV(OCIE0A);
   usingInterrupt = true;
 }
 else
 {
   // do not call the interrupt function COMPA anymore
   TIMSK0 &= ~_BV(OCIE0A);
   usingInterrupt = false;
 }
}


void loop()                     // run over and over again
{
  // do nothing! all reading and printing is done in the interrupt
}
