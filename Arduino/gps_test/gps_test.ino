// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code just echos whatever is coming from the GPS unit to the
// serial monitor, handy for debugging!
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground

// You can change the pin numbers to match your wiring:
SoftwareSerial mySerial(9, 6); // (TX, RX)

#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_Q_RELEASE "$PMTK605*31"

void setup() {
  while (!Serial); // wait for Serial to be ready

  Serial.begin(57600); // this baud rate doesn't actually matter!
  Serial1.begin(9600);
  mySerial.begin(9600);
  delay(2000);
  Serial.println("Get version!");
  mySerial.println(PMTK_Q_RELEASE);

  // you can send various commands to get it started
//  mySerial.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  mySerial.println(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  mySerial.println(PMTK_SET_NMEA_UPDATE_10HZ);
 }


void loop() {
//  if (Serial.available()) {
//   char c = Serial.read();
//   Serial.write(c);
//   mySerial.write(c);
//  }
  if (mySerial.available()) {
    char c = mySerial.read();
    Serial.write(c);
  }
}
