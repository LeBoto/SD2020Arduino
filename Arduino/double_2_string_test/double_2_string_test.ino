#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BMP_SCK A0
#define BMP_MISO A1
#define BMP_MOSI A2
#define BMP_CS A3

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
char dat[10];
char dat_string[50];
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("BMP388 test");

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
}

void loop() {
  strcpy(dat_string, "$TEMPALT,");
  dtostrf(bmp.temperature * 9.0/5.0 + 32.0, 3, 3, dat);
  strcat(dat_string, dat);
  strcat(dat_string, ",");
  dtostrf(bmp.pressure / 100.0, 3, 3, dat);
  strcat(dat_string, dat);
  strcat(dat_string, ",");
  dtostrf(bmp.readAltitude(SEALEVELPRESSURE_HPA), 3, 3, dat);
  strcat(dat_string, dat);
  Serial.println(dat_string);
}
