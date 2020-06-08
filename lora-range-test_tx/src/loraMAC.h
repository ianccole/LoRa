/*
   LoRa PTP MAC 
*/


#ifndef LORAMAC_H
#define LORAMAC_H

#include <Arduino.h>
#include <LoRa.h>

class LoRaMAC {
public:
   LoRaMAC();
   
   int begin(long frequency);

private:
   uint8_t srcNode;
   uint8_t dstNode;
   uint32_t messageCount;

};

#endif
