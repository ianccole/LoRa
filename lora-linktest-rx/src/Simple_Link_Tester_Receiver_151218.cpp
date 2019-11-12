//Simple_Link_Tester_Receiver_151218.ino
#define programversion "V1.0"
#define authorname "Stuart Robinson"

/*
**************************************************************************************************

  LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson - 15/12/18

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without
  the explicit permission of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
  intended purpose and free from errors.

  Changes:
  101218 Changed to LoRa5 library file

  To do:

******************************************************************************************************
*/

#include <Arduino.h>
#include "Pin_Definitions.h"

boolean SD_Found = false;                   //set if SD card found at program startup
uint32_t Mode1_Packet_Count = 0;
uint8_t Mode1_Max_Power = 0;
uint32_t Mode1_Cycles = 0;

uint16_t lora_Test1Count[20];	              //buffer where counts of received packets are stored
uint16_t batch = 0;                         //value read from packet
uint16_t current_batch = 0;
uint8_t current_lora_RXSource = 0;


#include "Settings.h"
#include <SPI.h>
#include "LoRa5.h"

#ifdef USESD
#include <SdFat.h>                          //https://github.com/greiman/SdFat
SdFat SD;
File logFile;
#endif

#ifdef UseI2CLCD
#include <LiquidCrystal_I2C.h>              //https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/  - download LiquidCrystal_V1.2.1.zip
LiquidCrystal_I2C disp(LCDI2C_Address, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  //Set the LCD I2C address and pins used
#endif

#ifdef UseSD1306
#include "SSD1306Ascii.h"                   //https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h"
SSD1306AsciiWire disp;
#endif

void checkforpacket();
void processPacket();
void clear_Counts();
void writescreen_TestPacket();
void writescreen_SNR();
void writescreen_RSSI();
void writescreen_PacketsBatch();
uint16_t Read_UInt(uint8_t addr, uint8_t localbuff[]);
void print_Test1Count();

void loop()
{
//   if (!(digitalRead(Switch1)))              //if switch is pressed, close log and halt
//   {
//     Serial.print(F("Switch Pressed - Closing log "));
//     Serial.println(batch);

// #ifdef USESD
//     logFile.print(F("Switch Pressed - Closing log "));
//     logFile.println(batch);
//     logFile.close();
// #endif

// #ifdef UseDisplay
//     disp.clear();
//     disp.setCursor(0, 0);
//     disp.print("Closed Log");
//     disp.setCursor(0, 1);
//     disp.print(batch);
// #endif

//     while (1);                  //loop forever

//   }

  checkforpacket();
}


void checkforpacket()
{
  //check if a packet has been received

  if (lora_readRXreadyDIO0())
  {
    lora_RXOFF();                                    //stop more packets comming in
    lora_ReadPacketDetails();                        //read packet details
    PacketOK = lora_IsPacketValid();                 //checks if packet is OK, updates counts, updates HeaderOK, PayloadhasCRC, CRCError

    digitalWrite(LED1, HIGH);
    Serial.print(F("RX "));
    lora_ReadPacket();

    lora_PrintReceptionInfo();
    Serial.print("  ");
    lora_PrintAddressInfo();

#ifdef USESD
    lora_ReceptionInfoSD();
    lora_AddressInfoSD();
#endif

    digitalWrite(LED1, LOW);

    if (PacketOK)
    {
      processPacket();
    }
    else
    {
      Serial.println(F("  Packet Error"));

#ifdef USESD
      lora_ReceptionInfoSD();
      lora_AddressInfoSD();
      logFile.println(F("  Packet Error"));
#endif

#ifdef UseDisplay
      disp.setCursor(0, 0);
      disp.print("Pkt  Error");
#endif

    }
  lora_RXONLoRa();                                 //ready for next
  }
}


void processPacket()
{
  uint8_t lTXpower;
  uint16_t i;

  if ((current_lora_RXSource != 0) && (current_lora_RXSource != lora_RXSource))     //detect if transmitte source has changed
  {
    Serial.println(F("Source has changed"));

    clear_Counts();

#ifdef USESD
    Serial.println(F("Close log"));
    logFile.close();
#endif

    current_lora_RXSource = lora_RXSource;
    current_batch = 0;
    return;
  }

  if (lora_RXPacketType == Testpacket)
  {
    lTXpower = ((lora_RXBUFF[0] - 48) * 10) +  (lora_RXBUFF[1] - 48);

#ifdef UseDisplay
    writescreen_TestPacket();
    writescreen_SNR();
    writescreen_RSSI();
    writescreen_PacketsBatch();
#endif

    if (lora_RXBUFF[2] == '1')
    {
      if (Mode1_Cycles > 0)
      {
        Mode1_Packet_Count++;
        i = lora_Test1Count[lTXpower];
        i++;
        lora_Test1Count[lTXpower] = i;
      }

      if (lTXpower > Mode1_Max_Power)
      {
        Mode1_Max_Power = lTXpower;
      }

    }

    Serial.print(F(" ("));
    Serial.print(lTXpower);
    Serial.print(F("dBm) "));
    Serial.println();

#ifdef USESD
    logFile.print(F(" ("));
    logFile.print(lTXpower);
    logFile.print(F("dBm) "));
    logFile.println();
    logFile.flush();
#endif
  }

  if (lora_RXPacketType == TestMode1)
  {
    //this is a command to switch to TestMode1 also updates totals and logs

    Serial.println();
    Serial.println(F("Switch Mode1 "));

    batch = Read_UInt(1, lora_RXBUFF);

    Serial.print(F("Transmitted Batch "));
    Serial.println(batch);
    Serial.println();

#ifdef USESD
    setup_SDLOG(batch);                             //setup SD, again, checks for SD failure
#endif


#ifdef USESD
    if ((current_batch == 0) || (current_batch != batch))           //check for first testmode1 packet or batch change
    {
      current_batch = batch;
      start_log(current_batch);
    }

    logFile.println();
    logFile.print(F("Switch Mode1 - "));
    logFile.println(current_batch);
    logFile.println();
    logFile.flush();
#endif

    if (Mode1_Cycles > 0)
    {
      print_Test1Count();
    }

    Serial.println();
    Mode1_Cycles++;
  }

  lora_RXONLoRa();
}


uint16_t Read_UInt(uint8_t addr, uint8_t localbuff[])
{
  uint8_t lowbyte, highbyte;
  lowbyte = localbuff[addr];
  highbyte = localbuff[addr + 1];
  return (lowbyte + (highbyte * 256));
}


void start_log(uint16_t lbatch)
{
  current_batch = lbatch;
  clear_Counts();

#ifdef USESD
  Serial.println(F("Close current file"));
  logFile.close();                           //close the current logfile
  setup_SDLOG(current_batch);                //open SD card file with number of batch
#endif
}


void clear_Counts()
{
  uint8_t i;
  Mode1_Cycles = 0;
  Mode1_Packet_Count = 0;
  Mode1_Max_Power = 0;

  Serial.println(F("Clear packet counts"));
  for (i = 17; i >= 2; i--)
  {
    lora_Test1Count[i] = 0;
  }
}


void print_Test1Count()
{
  //prints running totals of the power of received packets in CSV format
  uint8_t i;
  uint32_t j;

#ifdef USESD
  logFile.print(F("Mode1 Test Packets "));
  logFile.println(Mode1_Packet_Count);
  logFile.print(F("Mode1 Test Cycles "));
  logFile.println(Mode1_Cycles);
  logFile.print(F("Mode1 Max Power "));

  if (Mode1_Max_Power == 0)
  {
    logFile.println(F("NA"));
  }
  else
  {
    logFile.print(Mode1_Max_Power);
    logFile.println(F("dBm"));
  }

  logFile.println();
  for (i = 17; i >= 2; i--)
  {
    logFile.print(i);
    logFile.print("dBm,");
    j = lora_Test1Count[i];
    logFile.print(j);
    logFile.print("  ");
  }
  logFile.println();

  logFile.print("CSV");
  for (i = 17; i >= 2; i--)
  {
    logFile.print(",");
    j = lora_Test1Count[i];
    logFile.print(j);
  }
  logFile.println();
  logFile.println();
  logFile.flush();
#endif

  Serial.print(F("Mode1 Test Packets "));
  Serial.println(Mode1_Packet_Count);
  Serial.print(F("Mode1 Test Cycles "));
  Serial.println(Mode1_Cycles);
  Serial.print(F("Mode1 Max Power "));

  if (Mode1_Max_Power == 0)
  {
    Serial.println(F("NA"));
  }
  else
  {
    Serial.print(Mode1_Max_Power);
    Serial.println(F("dBm"));
  }

  Serial.println();
  for (i = 17; i >= 2; i--)
  {
    Serial.print(i);
    Serial.print("dBm,");
    j = lora_Test1Count[i];
    Serial.print(j);
    Serial.print("  ");
  }
  Serial.println();

  Serial.print("CSV");
  for (i = 17; i >= 2; i--)
  {
    Serial.print(",");
    j = lora_Test1Count[i];
    Serial.print(j);
  }
  Serial.println();
}


void systemerror()
{
  while (1)
  {
    digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    delay(100);
  }
}


void led_FlashStart()
{
  uint8_t i;
  for (i = 0; i <= 4; i++)
  {
    digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    delay(100);
  }
}


void lora_Custom1()
{
  //used to have a custom set of LoRa register settings for each test mode
  //leave blank if not used
}

void init_LoRaTest1()
{
  lora_SetFreqInt(Frequency, CalibrationOffset);                      //Setup the LoRa frequency
  lora_SetModem2(Bandwidth, SpreadingFactor, CodeRate, Explicit);  //Setup the LoRa modem parameters
}


/*****************************************************************************/
//  Start Display routines
/*****************************************************************************/

#ifdef UseDisplay
void writescreen_TestPacket()
{
  uint8_t tempbyte, tempbyte1, power;
  disp.clear();
  disp.setCursor(0, 0);
  disp.print("Test ");
  tempbyte = lora_RXBUFF[0] - 48;             //convert ASCII to number
  tempbyte1 = lora_RXBUFF[1] - 48;
  power = (tempbyte * 10) + tempbyte1;
  disp.print(power);
  disp.print("dBm");
}


void writescreen_SNR()
{
  disp.setCursor(0, 2);
  disp.print(F("SNR "));
  disp.print(lora_PacketSNR);                  //now print the SNR
  disp.print(F("dB "));
}


void writescreen_RSSI()
{
  disp.setCursor(0, 1);
  disp.print(F("RSSI "));
  disp.print(lora_PacketRSSI);                  //now print the SNR
  disp.print(F("dBm"));

}

void writescreen_PacketsBatch()
{
  disp.setCursor(0, 3);
  disp.print(F("Pk "));                       //send count to LCD
  disp.print(lora_RXpacketCount);               //send count to LCD
  disp.print(F("  "));
  disp.print(batch);

  if (!SD_Found)
  {
    disp.print(F("  NoSD"));
  }

}
#endif

/*****************************************************************************/
//  End Display routines
/*****************************************************************************/


/*****************************************************************************/
//  Start SD Card routines
/*****************************************************************************/

#ifdef USESD

void lora_AddressInfoSD()
{
  //print the information for packet last received
  logFile.print(F("RXType,"));
  logFile.write(lora_RXPacketType);
  logFile.print(F("  Destination,"));
  logFile.write(lora_RXDestination);
  logFile.print(F("  Source,"));
  logFile.write(lora_RXSource);

  logFile.print(F("  Length,"));
  logFile.print(lora_RXPacketL);
  logFile.print(F("  "));
  logFile.flush();
}


void lora_ReceptionInfoSD()
{
  //print the information for packet last received
  //note, lora_PacketSNR has already been converted into a signed value
  //lora_PacketRSSI is a signed value also

  logFile.print(F("SNR,"));
  logFile.print(lora_PacketSNR);
  logFile.print(F("dB"));

  logFile.print(F("  RSSI,"));
  logFile.print(lora_PacketRSSI);
  logFile.print(F("dB   "));
  logFile.flush();
}


boolean setup_SDLOG(uint16_t lbatch)
{
  //checks if the SD card is present and can be initialised
  Serial.println();
  Serial.print(F("SD card..."));

  if (!SD.begin(SD_CS))
  {
    Serial.println(F("Failed, or not present"));
    SD_Found = false;
    return false;                                 //don't do anything more:
  }

  Serial.print(F("Initialized OK"));
  SD_Found = true;

  char file_name[] = "xxxxx.txt";                 //prototype file name
  char extension[] = ".txt";                      //sometimes the extension gets modified

  sprintf(file_name, "%d%s", lbatch, extension);
  logFile = SD.open(file_name, FILE_WRITE);

  Serial.print(F("...Writing to "));
  Serial.write(file_name[0]);

  if (lbatch > 9)
  {
    Serial.write(file_name[1]);
  }
  if (lbatch > 99)
  {
    Serial.write(file_name[2]);
  }
  if (lbatch > 999)
  {
    Serial.write(file_name[3]);
  }
  if (lbatch > 9999)
  {
    Serial.write(file_name[4]);
  }

  Serial.println(F(".txt"));
  return true;
}



void lora_RXBuffPrintSD(uint8_t PrintType)
{
  //Print contents of RX buffer as ASCII, decimal or HEX
  //PrintType = 0 = ASCII
  //PrintType = 1 = Decimal
  //PrintType = 2 = HEX

  uint8_t bufferData;

  logFile.write(lora_RXPacketType);
  logFile.write(lora_RXDestination);
  logFile.write(lora_RXSource);

  for (uint8_t index = lora_RXStart; index <= lora_RXEnd; index++)
  {
    if (PrintType == 0)
    {
      logFile.write(lora_RXBUFF[index]);
    }

    if (PrintType == 1)
    {
      logFile.print(lora_RXBUFF[index]);
      logFile.print(F(" "));
    }

    if (PrintType == 2)
    {
      bufferData = lora_RXBUFF[index];
      if (bufferData < 0x10)
      {
        logFile.print(F("0"));
      }
      logFile.print(bufferData, HEX);
      logFile.print(F(" "));
    }
  }
  logFile.flush();
}

#endif

/*****************************************************************************/
//  End SD Card routines
/*****************************************************************************/


void setup()
{
  pinMode(LED1, OUTPUT);		                      //setup pin for PCB LED
  led_FlashStart();

  Serial.begin(115200);                           //setup Serial console ouput
  Serial.println();
  Serial.println(__FILE__);
  Serial.print(F("Compiled "));
  Serial.print(__TIME__);
  Serial.print(F(" "));
  Serial.println(__DATE__);
  Serial.println(F(authorname));
  Serial.println();
  Serial.println();

  SPI.begin();                                   //initialize SPI
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
  
  pinMode (lora_NSS, OUTPUT);                     //setup pin for LoRa device slave select
  digitalWrite(lora_NSS, HIGH);
    
  pinMode(lora_NReset, INPUT);		                //setup pin for LoRa device reset line
  pinMode (lora_DIO0, INPUT);                     //setup pin for LoRa device slave select
  pinMode(Switch1, INPUT_PULLUP);                 //setup pin for switch

#ifdef UseDisplay

#ifdef UseSD1306
  disp.begin(&Adafruit128x64, SD1306_Address);
  disp.setFont(Adafruit5x7);
  disp.set1X();
#endif

#ifdef UseI2CLCD
  disp.begin(20, 4);                              //initialize the lcd for 20 chars 4 lines
  disp.backlight();                               //turn on backlight, if option available
#endif

  Serial.println(F("Display Option Selected"));
  disp.clear();
  disp.setCursor(0, 0);
  disp.print(F("Starting"));
#endif


#ifdef USESD
  Serial.println(F("SD Logging Option Selected"));
  setup_SDLOG(batch);                             //setup SD
#else
  Serial.println(F("No SD card logging enabled"));
  SD_Found = false;
#endif

  if (lora_CheckDevice() == true)
  {
#ifdef EnableTone
    init_LoRaTest1();
    Serial.println(F("LoRa Tone"));
    digitalWrite(LED1, HIGH);
    lora_Tone(1000, 1000, 2, lora_DIO2);                      //Transmit an FM tone, 1000hz, 1000ms
    digitalWrite(LED1, LOW);
    Serial.println();
#endif
  }
  else
  {
    Serial.println(F("LoRa Device Error"));
    systemerror();
  }
  
  lora_Setup(lora_NSS, lora_NReset, lora_DIO0); 
  init_LoRaTest1();
  lora_RXONLoRa();

  Serial.println();
  lora_Print();
  Serial.println();

  lora_PrintLoRaSettings();

  Serial.println();
  Serial.println(F("Receiver ready"));

#ifdef UseDisplay
  float freq_temp = lora_GetFreqFloat();
  disp.setCursor(0, 0);
  disp.print(F("Ready    "));
  disp.setCursor(0, 1);
  disp.print(freq_temp, 3);
  disp.print(F("MHz"));
#endif

}


