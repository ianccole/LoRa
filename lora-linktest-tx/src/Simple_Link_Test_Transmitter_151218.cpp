//Simple_Link_Test_Transmitter_151218.ino
#define programversion "V1.0"
#define authorname "Stuart Robinson"

/*
**************************************************************************************************

  LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson
  
  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without
  the explicit permission of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
  intended purpose and free from errors.

  Changes:
  091218 - New version, simplified, only sends test packets 

  To do:

  
******************************************************************************************************
*/

#include <Arduino.h>
#include "Pin_Definitions.h"                   //this file has the pin definitions used to connect to the LoRa device

uint16_t batch = 65535;
uint16_t testloop = 0;
int8_t lora_TestPower = 10;                      

#include <SPI.h>
#include "Settings.h"                          //program test settings here
#include "LoRa5.h"                             //LoRa library routines     

void init_LoRaTest1();
void Send_Test1Mode_Packet();
void lora_Custom1();
void Send_Test_Packet(char lmode);

void loop()
{
  testloop++;

  Serial.print(F("Test Loop "));
  Serial.println(testloop);
  Serial.print(F("Batch "));
  Serial.println(batch);
  Serial.println();
  Serial.flush();

  Serial.println(F("Loop Start Tone "));
  Serial.flush();

  lora_Tone(500, LoopStartTone_lengthmS, start_power, lora_DIO2);      //Transmit a pseudo FM tone and carrier
  delay(1000);

  init_LoRaTest1();                                         //setup LoRa modem

  Send_Test1Mode_Packet();                                  //send initial packet, contains batch number 
  delay(mode_delaymS);

  Serial.println("Start Mode1 Packets");
  Serial.println();
  for (lora_TestPower = start_power; lora_TestPower >= end_power; lora_TestPower--)
  {
    char strBuf[10];
    sprintf(strBuf, "%02d dBm ", lora_TestPower);
    Serial.print(strBuf);

    lora_Custom1();
    Serial.print("Test Packet ");
    Serial.flush();
    Send_Test_Packet('1');
    Serial.println();
    delay(packet_delay);
  }

  Serial.println("Finished Mode1 Packets");

  Serial.println();
  delay(mode_delaymS);

}


void PrintPacket()
{
  uint8_t index;
  for (index = 0; index <= lora_TXEnd; index++)
  {
    Serial.write(lora_TXBUFF[index]);
  }
}


void Send_Test1Mode_Packet()
{
  //causes RX to switch mode and print totals
  Serial.print(F("Transmitted batch "));
  Serial.println(batch);
  lora_TXBUFF[0] = '1';
  lora_TXBUFF[1] = lowByte(batch);
  lora_TXBUFF[2] = highByte(batch);
  digitalWrite(LED1, HIGH);
  lora_Send(0, 2, TestMode1, Broadcast, ThisNode, 10, 17, NoStrip);
  digitalWrite(LED1, LOW);
  delay(1000);                                                //leave enough time for receiver to print running totals
}


void Send_Test_Packet(char lmode)
{
  //build and send the test packet

  if (lora_TestPower > 9)
  {
    lora_TXBUFF[0] = '1';
    lora_TXBUFF[1] = ((lora_TestPower - 10) + 0x30);
  }
  else
  {
    lora_TXBUFF[0] = '0';
    lora_TXBUFF[1] = (lora_TestPower + 0x30);
    
  }

  lora_TXBUFF[2] = lmode;

  lora_TXEnd = 2;
  PrintPacket();
  
  Serial.print(F(" "));
  digitalWrite(LED1, HIGH);
  lora_Send(0, lora_TXEnd, Testpacket, Broadcast, ThisNode, 10, lora_TestPower, NoStrip);
  digitalWrite(LED1, LOW);
  Serial.print(F("TX Time "));
  Serial.print(lora_TXTime);
  Serial.print("mS ");
}


void Led_FlashStart()
{
  uint8_t index;
  for (index = 0; index <= 4; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    delay(100);
  }
}


void System_Error()
{
  while (1)
  {
    digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    delay(100);
  }
}


void init_LoRaTest1()
{
  //setup for LoRa modem testmode1
  float freq_temp;
  lora_Setup(lora_NSS, lora_NReset, lora_DIO0);                                                  //base LoRa device configuration
  lora_SetFreqInt(Frequency, CalibrationOffset);
  freq_temp = lora_GetFreqFloat();
  Serial.print(F("Set to Frequency "));
  Serial.print(freq_temp, 3);
  Serial.println(F("Mhz"));
  lora_SetModem2(Bandwidth, SpreadingFactor, CodeRate, Explicit);	//setup the LoRa modem parameters for test
}


void lora_Custom1()
{
  //used to have a custom set of LoRa register settings for test mode
  //leave empty if not used
}


void generate_batch()
{
  //generate the batch number
  uint16_t tempint = 0;
  uint16_t index;
  
  for (index = 0; index <= 1000; index++)
  {
    tempint =  tempint + analogRead(ADC_Channel);
  }

  Serial.print(F("Random Seed "));
  Serial.println(tempint);
  randomSeed((uint16_t) tempint);
  batch = random(1, 32767);                    //generate random batch number
}


void setup()
{
  pinMode(LED1, OUTPUT);		                   //for PCB LED
  Led_FlashStart();

  Serial.begin(115200);                        //setup Serial console ouput
  Serial.println();
  Serial.println(__FILE__);
  Serial.print(F("Compiled "));
  Serial.print(__TIME__);
  Serial.print(F(" "));
  Serial.println(__DATE__);
  Serial.println(F(authorname));
  Serial.println();

  SPI.begin();
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));

  pinMode(lora_NReset, INPUT);			           //LoRa Device reset line, normally left floating
  pinMode (lora_NSS, OUTPUT);			             //LoRa Device select line
  digitalWrite(lora_NSS, HIGH);

  if (lora_CheckDevice() == true)
  {
    init_LoRaTest1();
    digitalWrite(LED1, HIGH);
    Serial.print(F("Test Tone "));
    lora_Tone(1000, 1000, 2, lora_DIO2);              //Transmit an FM tone, 1000hz, 1000ms, 2dBm
    lora_TXOFFDirect();                               //ensure TXOFF runs so packet time is recorded correctly
    digitalWrite(LED1, LOW);
    Serial.println();
  }
  else
  {
    Serial.println(F("LoRa Device Error"));
    System_Error();
  }

  lora_Print();                                 //print all the LoRa registes
  Serial.println();

  generate_batch();                             //generate the batch number to use

}

