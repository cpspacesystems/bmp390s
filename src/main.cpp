/***************************************************************************
  This is a library for the BMP3XX temperature & pressure sensor

  Designed specifically to work with the Adafruit BMP388 Breakout
  ----> http://www.adafruit.com/products/3966

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define BMP2_CS 37

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;
Adafruit_BMP3XX bmp2;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Adafruit BMP388 / BMP390 test");

  // Set up BMP1 via SPI
  if (!bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI))
  { 
    Serial.println("Could not find a valid FIRST BMP3 sensor, check wiring!");
    while (1)
      ;
  }
  
  // Set up BMP2 via SPI (the only thing that changes is the CS pin)
  if (!bmp2.begin_SPI(BMP2_CS, BMP_SCK, BMP_MISO, BMP_MOSI))
  {   
      Serial.println("Could not find a valid SECOND BMP3 sensor, check wiring!");
      while (1)
        ;
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

  void loop()
  {
    if (!bmp.performReading())
    {
      Serial.println("Failed to perform reading :(");
      return;
    }
    Serial.print("Temperature 1 = ");
    Serial.print(bmp.temperature);
    Serial.println(" *C");

    Serial.print("Temperature 2 = ");
    Serial.print(bmp2.temperature);
    Serial.println(" *C");

    Serial.print("Pressure 1 = ");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print("Pressure 2 = ");
    Serial.print(bmp2.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print(bmp2.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.println();
    delay(2000);
  }
