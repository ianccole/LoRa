#include <Arduino.h>
#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <MeshNet.h>
#include <MemoryFree.h>

#define FLASHHDRLEN (10)

#ifdef UseSD1306
#define SD1306_Address 0x3C                       //define I2C address for SD1306
#include "SSD1306Ascii.h"                   //https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h"
SSD1306AsciiWire disp;
#endif

#ifdef FOTA_CLIENT
#include <avr/wdt.h>
#include <SPIFlash.h>
SPIFlash flash(SS_FLASHMEM);
#endif

// RH_RF95 MeshNet::rf95;

static char _tmpMessage[MESH_NET_MAX_MESSAGE_LEN];
static char buffer[50];

int freeMem()
{
//   extern int __heap_start, *__brkval;
//   int v;
//   return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
  return freeMemory();
}

void printMsg(const char * msg, bool clear=false)
{
    // disp.setCursor(0, row);
#ifdef UseSD1306
    if(clear)
        disp.clear();
    disp.print(msg);
#endif
    Serial.print(msg);
}

#ifdef FOTA_CLIENT
void hexConv (const uint8_t * (& pStr), byte & b)
{
    b = *pStr++ - '0';
    if (b > 9)
        b -= 7;

    // high-order nybble
    b <<= 4;

    byte b1 = *pStr++ - '0';
    if (b1 > 9)
        b1 -= 7;

    b |= b1;
}

void resetUsingWatchdog()
{
#ifdef __AVR__
  //wdt_disable();
  Serial.print(F("REBOOTING"));
  wdt_enable(WDTO_15MS);
  while(1) 
    Serial.print(F("."));
#elif defined(MOTEINO_M0)
  //WDT->CTRL.reg = 0; // disable watchdog
  //while (WDT->STATUS.bit.SYNCBUSY == 1); // sync is required
  //WDT->CONFIG.reg = 0; // see Table 18.8.2 Timeout Period (valid values 0-11)
  //WDT->CTRL.reg = WDT_CTRL_ENABLE; //enable WDT
  //while (WDT->STATUS.bit.SYNCBUSY == 1);
  //WDT->CLEAR.reg= 0x00; // system reset via WDT
  //while (WDT->STATUS.bit.SYNCBUSY == 1);
  *((volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4)) = 0xF1A507AF;
  NVIC_SystemReset();
#endif
}
#endif

////////////////////////////////////////////////////////////////////
// Constructors
MeshNet::MeshNet(RH_RF95& rf95)
    : fotaTimeout(0), fotaActive(false), rf95(rf95)
{
}

void MeshNet::setup(uint8_t thisAddress, float freqMHz, int8_t power, uint16_t cad_timeout)
{
	MeshNet::power = power;
	manager = new RHMesh(rf95, thisAddress);
    ping=0;

#ifdef UseSD1306
    disp.begin(&Adafruit128x64, SD1306_Address);
    disp.setFont(Adafruit5x7);
    disp.set1X();
#endif

	if (!manager->init())
	{
        Serial.println(F("init failed"));
        return;
	}

    sprintf(buffer, "RF95 ready(mem = %d)\n",freeMem());
    printMsg(buffer, true);

#ifdef LED_BUILTIN
	pinMode(LED_BUILTIN, OUTPUT);
#endif

	rf95.setTxPower(power);
	rf95.setFrequency(freqMHz);
	rf95.setCADTimeout(cad_timeout);

//   // long range configuration requires for on-air time
//   boolean longRange = false;
//   if (longRange)
//   {
//     RH_RF95::ModemConfig modem_config = {
//         0x78, // Reg 0x1D: BW=125kHz, Coding=4/8, Header=explicit
//         0xC4, // Reg 0x1E: Spread=4096chips/symbol, CRC=enable
//         0x08  // Reg 0x26: LowDataRate=On, Agc=Off.  0x0C is LowDataRate=ON, ACG=ON
//     };
//     rf95.setModemRegisters(&modem_config);
//     if (!rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096))
//     {
//       Serial.println(F("set config failed"));
//     }
//   }
}

void MeshNet::loop(uint16_t wait_ms)
{
	uint8_t len = sizeof(_tmpMessage);
	uint8_t from;
    uint8_t currentSeconds = seconds();

    if(ping && uint8_t(currentSeconds - pingTimeout) > pingInterval)
    {
        pingTimeout = currentSeconds;
        pingNode(ping);
    }
#ifdef FOTA_CLIENT
    if(fotaActive && uint8_t(currentSeconds - fotaTimeout) > fotaInterval)
    {
        fotaActive = false;
        Serial.println("FOTA timeout");
    }
#endif
	uint8_t dest;
    uint8_t id;
    uint8_t flags;

	if (manager->recvfromAckTimeout((uint8_t *)&_tmpMessage, &len, wait_ms, &from, &dest, &id, &flags))
	{
		blinkLed();

        MeshMessageHeader *p = (MeshMessageHeader *)&_tmpMessage;

        if (len >= 1)
        {
            RHRouter::RoutingTableEntry *route = manager->getRouteTo(from);
            sprintf(buffer, "Rx:%d RSSI:%d\nSNR:%d Hop:%d Id:%d len:%d\n", from, rf95.lastRssi(), rf95.lastSNR(), route->next_hop, id, len);
            // Serial.print(buffer);
            printMsg(buffer, true);

            switch(p->msgType)
            {
                case MESH_NET_MESSAGE_TYPE_PING_RESPONSE:
                {
                    MeshNetPingMessage *a = (MeshNetPingMessage *)p;
                    sprintf(buffer, "%d dBm RSSI:%d\nSNR:%d\n",a->power, a->rssi, a->snr);
                    printMsg(buffer);
                    break;
                }
                case MESH_NET_MESSAGE_TYPE_PING_REQUEST:
                {
                    MeshNetPingMessage *a;
                    
                    a = (MeshNetPingMessage *)p;
                    sprintf(buffer, "%d dBm RSSI:%d\nSNR:%d\n", a->power, rf95.lastRssi(), rf95.lastSNR());
                    printMsg(buffer);

                    a = (MeshNetPingMessage *)_tmpMessage;
                    a->header.msgType = MESH_NET_MESSAGE_TYPE_PING_RESPONSE;
                    a->power = power;
                    a->rssi = rf95.lastRssi();
                    a->snr = rf95.lastSNR();

                    sendtoWaitStats((uint8_t*)&_tmpMessage, sizeof(MeshNet::MeshNetPingMessage), from);
                    break;
                }

#ifdef FOTA_CLIENT
                case MESH_NET_MESSAGE_TYPE_FOTA_REQUEST:
                {
                    MeshNetFOTAMessageReq *a = (MeshNetFOTAMessageReq *)p;
                    a->data[len - (sizeof(MeshMessageHeader) + sizeof(MeshFOTAHeader))] = '\0';
                    handleFOTA(a, from);
                    break;
                }
#endif
#ifdef FOTA_SERVER
                case MESH_NET_MESSAGE_TYPE_FOTA_RESPONSE:
                {
                    MeshNetFOTAMessageRsp *a = (MeshNetFOTAMessageRsp *)p;
                    sprintf(buffer, "%s for SEQ:%d,%d\n", flags?"NAK":"ACK", a->headerFOTA.sequence,flags);
                    printMsg(buffer);
                    break;
                }
#endif
                case MESH_NET_MESSAGE_TYPE_APP_REQUEST:
                {
                    MeshNetAppMessage *a = (MeshNetAppMessage *)p;
                    a->data[len - sizeof(MeshMessageHeader)] = '\0';
                    Serial.println(a->data);

                    MeshNetPingMessage *r;
                    r = (MeshNetPingMessage *)_tmpMessage;
                    r->header.msgType = MESH_NET_MESSAGE_TYPE_PING_RESPONSE;
                    r->power = power;
                    r->rssi = rf95.lastRssi();
                    r->snr = rf95.lastSNR();

                    sendtoWaitStats((uint8_t*)&_tmpMessage, sizeof(MeshNet::MeshNetPingMessage), from);

                    break;
                }

                default:
                    Serial.println("Unhandled: message");
                    break;
            }
        }
	}
}

uint8_t MeshNet::sendtoWaitStats(uint8_t *buf, uint8_t len, uint8_t address, uint8_t flags)
{
	blinkLed();

	uint8_t error = manager->sendtoWait(buf, len, address, flags);

    switch(error)
    {
        case RH_ROUTER_ERROR_NO_ROUTE:
            Serial.println("no route");
            break;

        case RH_ROUTER_ERROR_NONE:
        {
            RHRouter::RoutingTableEntry *route = manager->getRouteTo(address);
            sprintf(buffer, "TxAck:%d RSSI:%d\nSNR:%d Hop:%d\n", address, rf95.lastRssi(), rf95.lastSNR(), route->next_hop);
            printMsg(buffer);
        }
        break;
    }

    return error;
}

void MeshNet::pingNode(uint8_t address, uint8_t flags)
{
    MeshNetPingMessage *a = (MeshNetPingMessage *)_tmpMessage;
    a->header.msgType = MESH_NET_MESSAGE_TYPE_PING_REQUEST;
	a->power = power;
    
	sendtoWaitStats((uint8_t*)_tmpMessage, sizeof(MeshNet::MeshNetPingMessage), address, flags);
}

void MeshNet::appMessage(uint8_t address, char * buf, uint8_t flags)
{
    MeshNetAppMessage *a = (MeshNetAppMessage *)_tmpMessage;
    a->header.msgType = MESH_NET_MESSAGE_TYPE_APP_REQUEST;
    uint8_t len = strlen(buf) + sizeof(MeshMessageHeader);
    memcpy(a->data, buf, strlen(buf));       
	sendtoWaitStats((uint8_t*)_tmpMessage, len, address, flags);
}

#ifdef FOTA_SERVER
void MeshNet::sendFOTAREQ(uint8_t address, uint16_t seqnum, char *buf)
{
    MeshNetFOTAMessageReq *a = (MeshNetFOTAMessageReq *)_tmpMessage;
    a->header.msgType = MESH_NET_MESSAGE_TYPE_FOTA_REQUEST;
	a->headerFOTA.sequence = seqnum;
    uint8_t len = strlen(buf) + sizeof(MeshMessageHeader) + sizeof(MeshFOTAHeader);
    memcpy(a->data, buf, strlen(buf));   
	manager->sendtoWait((uint8_t*)&_tmpMessage, len, address);
}
#endif

#ifdef FOTA_CLIENT
void MeshNet::sendFOTARSP(uint8_t address, uint16_t seqnum, uint8_t flags)
{
    MeshNetFOTAMessageReq *a = (MeshNetFOTAMessageReq *)_tmpMessage;
    a->header.msgType = MESH_NET_MESSAGE_TYPE_FOTA_RESPONSE;
	a->headerFOTA.sequence = seqnum;
    uint8_t len = sizeof(MeshMessageHeader) + sizeof(MeshFOTAHeader);
	manager->sendtoWait((uint8_t*)&_tmpMessage, len, address, flags);
}
#endif

#ifdef FOTA_CLIENT
void MeshNet::handleFOTA(MeshNetFOTAMessageReq *msg, uint8_t from)
{
    if (msg->headerFOTA.sequence == 0)
    {
        Serial.print("Erasing Flash chip ... ");
        flash.blockErase32K(0);
        while(flash.busy());
        Serial.println("DONE");        
        fotaActive = true;
        flashIndex = FLASHHDRLEN; // reserve space for flash header
    }
    if(fotaActive)
    {
        if(burnHexLine(msg->data))
        {
            // ack
            sendFOTARSP(from, msg->headerFOTA.sequence, 0x00);
            fotaTimeout = seconds();

            if(fotaActive == false)
            {
                resetUsingWatchdog();
            }
        }
        else
        {
            // nak
            fotaActive = false;
            sendFOTARSP(from, msg->headerFOTA.sequence, 0x01);
        }
    }
    else
    {
        // nak not started
        sendFOTARSP(from, msg->headerFOTA.sequence, 0x02);
    }
}

bool MeshNet::burnHexLine(const uint8_t *pLine)
{
    Serial.println((char *)pLine);
    if(*pLine++ != ':')
    {
        return false;
    }

    const int maxHexData = 30;
    byte hexBuffer [maxHexData];
    int bytesInLine = 0;

    // convert entire line from ASCII into binary
    while (isxdigit (*pLine) && isxdigit (*(pLine+1)) )
    {
        // can't fit?
        if (bytesInLine >= maxHexData)
        {
            Serial.println (F("Line too long to process."));
            return false;
        }

        hexConv(pLine, hexBuffer[bytesInLine++]);
    }

    Serial.print("Bytes: ");
    Serial.println(bytesInLine);

    byte checksum = 0;
    for (int i = 0; i < (bytesInLine); i++)
    {
        checksum += hexBuffer [i]; // sum them all, including the csum byte
    }

    // checksum should be zero
    if (checksum == 0)
    {
        Serial.println (F("checksum OK"));

        byte len = hexBuffer [0];
        // unsigned long addrH = hexBuffer [1];
        // unsigned long addrL = hexBuffer [2];
        // unsigned long addr = addrL | (addrH << 8);
        byte recType = hexBuffer [3];

        // Serial.println(len);
        // Serial.println(addr, HEX);
        // Serial.println(recType);

        switch(recType)
        {
            case 0:
                flash.writeBytes(flashIndex, (void*)(&hexBuffer[4]), len);
                flashIndex += len;
                break;

            case 1:
                /*
                +0-------------------1--------------
                |0|1|2|3|4|5|6|7|8|9|0|1..
                +------------------------------------
                |F|L|X|I|M|G|:|N|N|:|x|x|x|...
                +------------------------------------
                */

                // write header
                flash.writeBytes(0,"FLXIMG:", 7);

                // write length
                flashIndex -= FLASHHDRLEN;
                flash.writeByte(7, flashIndex >> 8);
                flash.writeByte(8, flashIndex & 0xff);
                flash.writeByte(9, ':');

                // indicate we are ready to reboot
                fotaActive = false;

            break;
        }
        return true;
    }

    return false;
}
#endif