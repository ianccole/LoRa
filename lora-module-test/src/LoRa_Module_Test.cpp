#define programname "LoRa_Module_Test"
#define programversion "V1.0"
#define dateproduced "15/11/2017"
#define aurthorname "Stuart Robinson"

/*
**************************************************************************************************

Easy Build LoRaTracker Programs for Arduino ATMEGA328

Copyright of the author Stuart Robinson - 15/11/2017

http://www.LoRaTracker.uk

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit permission
of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
free from errors.

**************************************************************************************************
*/

/*
********************************************************************************************************************************
The test program has been written to check out that the hardware and connectors for the tracker boards have been assembled correctly
such that there are funtional, with no open or short circuits. 

Whilst some of the pins and connectors have multiple purposes, the I2C pins on the Pro Mini for instance (A4 and A5) its really
only necessary to check that the pins are functional for one purpose. So whilst A4 is in one mode of operation used to read a
RC Servo pulse input, if the connector works for with an I2C device, that pin has been proven to be functional. 

Changes:

********************************************************************************************************************************
*/


                     
#include <Arduino.h>
#include "Locator2_Board_Definitions.h"          //specify PCB type
#include "Program_Definitions.h"

#include <SPI.h>
const byte lora_RXBUFF_Size = 32;                 
const byte lora_TXBUFF_Size = 64;
byte keypress;
byte TXPower;
const unsigned long TrackerMode_Frequency = 434400000;
const byte TrackerMode_Power = 10;
const int CalibrationOffset = 0;
const byte Deviation = 0x52;                     //typical deviation for tones, approx 5khz


#include "LoRa3.h"

void led_Flash(unsigned int flashes, unsigned int delaymS);
void  error_LoRaDevice();

void loop()
{
  Serial.println(F("LED Flash"));
  Serial.println();
  led_Flash(5,100);
  
  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
   
  Serial.print(F("Checking LoRa Device"));

  if (lora_CheckDevice() == true)
  {
    Serial.println(F(" - Present"));
    lora_Setup();                                      //Do the initial LoRa Setup
    lora_Print();
    lora_SetFreq(TrackerMode_Frequency, CalibrationOffset);
    Serial.print(F("Transmit FM Tone"));
    digitalWrite(LED1, HIGH);
    lora_Tone(1000, 2500, TXPower);                  //Transmit an FM tone, 1000hz, 2500ms, 10dBm
    digitalWrite(LED1, LOW);
    Serial.println(F(" - Done"));
    Serial.println();
    lora_Print();
  }
  else
  {
    Serial.println(F(" - LoRa Device Not Found"));
    lora_Print();
    Serial.println();
    error_LoRaDevice();
  }
 
  SPI.end();
  Serial.println();
  Serial.println();
  delay(1500);
}


void led_Flash(unsigned int flashes, unsigned int delaymS)
{
  //flash LED to show tracker is alive
  unsigned int index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}


void  error_LoRaDevice()
{
  int i;
  SPI.end();
  pinMode(13, OUTPUT);	                               //for Pro Mini PCB LED
  for (i = 0; i <= 49; i++)
  {
    digitalWrite(LED1, HIGH);
    digitalWrite(13, HIGH);
    delay(25);
    digitalWrite(LED1, LOW);
    digitalWrite(13, LOW);
    delay(25);
  }
}


void setup()
{
  pinMode(LED1, OUTPUT); 			               //for PCB LED
  pinMode(13, OUTPUT); 			               //for Pro Mini LED, Pin13
  pinMode(lora_TonePin, INPUT_PULLUP);		   //ensure tone out pin is input
  pinMode(lora_NReset, OUTPUT);			         //LoRa Device reset line
  pinMode (lora_NSS, OUTPUT);			           //LoRa Device select line
  digitalWrite(lora_NSS, HIGH);
  digitalWrite(lora_NReset, HIGH);

  Serial.begin(38400);                       //setup Serial console ouput
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));
  Serial.println();
 
}
