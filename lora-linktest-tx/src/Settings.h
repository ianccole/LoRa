//Settings.h for transmitter
/*
******************************************************************************************************

LoRaTracker Programs for Arduino

Copyright of the author Stuart Robinson

http://www.LoRaTracker.uk

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.

Changes:

To Do:

******************************************************************************************************
*/

//*******  Setup Test Parameters Here ! ***************

//LoRa Modem Parameters
const uint32_t  Frequency = 434400000;           //frequency of transmissions
const uint16_t  CalibrationOffset = 0;           //adjustment for frequency in Hz, assumed at room temp

#define Bandwidth BW125000                       //LoRa bandwidth
#define SpreadingFactor SF7                      //LoRa spreading factor
#define CodeRate CR45                            //LoRa coding rate

const int8_t start_power = 10;                   //Start power for transmissions
const int8_t end_power = 0;                      //End power for transmissions

const uint16_t LoopStartTone_lengthmS = 500;     //length of LoopStartTone in mS
const uint16_t ModeStartDelaymS = 2000;          //delay in mS after sending mode change at start of loop, gives RX time to print and log packet totals.
const uint16_t mode_delaymS = 200;               //mS delay between modes
const uint16_t packet_delay = 1000;              //mS delay between packets, minimum 50mS for BW125000, SF12 test packet

const uint8_t ThisNode = 'T';                     //node number goes out as part of packet addressing 

#define lora_RXBUFF_Size 1                        //RXBUFF is not used
#define lora_TXBUFF_Size 16                       //TXBUFF only needed for short test packet 


