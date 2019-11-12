#define programname "I2C_Scanner"
#define programversion "V1.2"
#define dateproduced "15/11/2017"
#define aurthorname "Stuart Robinson"

/*
********************************************************************************************************************************

Easy Build LoRaTracker Programs for Arduino

Copyright of the author Stuart Robinson - 15/11/2017

http://www.LoRaTracker.uk

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit permission
of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
free from errors.

This program scans each address on the I2C bus and reports if a device was found.

********************************************************************************************************************************
*/


#include <Arduino.h>
#include <Wire.h>
#include "I2C_Scanner.h"


void loop()
{
  Serial.println();
  Serial.println();
  setup_I2CScan();
  run_I2CScan();
  delay(1500);
}



void setup()
{

  Serial.begin(9600);                                   //setup Serial console ouput
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));
  Serial.println();
  Wire.begin();
 }


