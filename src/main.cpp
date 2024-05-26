#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define BMP2_CS 0
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;
Adafruit_BMP3XX bmp2;

void setup() {
  Serial.begin(115200);
  while(!Serial); // Burn clock cycles until serial starts
  Serial.println("Test started");
  if(!bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {
    Serial.println("Could not find a valid BMP3 sensor.");
    while(1); // Burn clock cycles if failure. Program should be terminated.
  }

  if(!bmp2.begin_SPI(BMP2_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {
    Serial.println("Could not find a valid second BMP3 sensor.");
    while(1); // Burn clock cycles if failure. Program should be terminated.
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  bmp2.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp2.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp2.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp2.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop() {
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");
  Serial.print(bmp2.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");
  Serial.print(bmp2.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  Serial.print(bmp2.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  delay(500);
}