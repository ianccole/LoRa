#include <Arduino.h>
#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <MeshNet.h>
#include <MemoryFree.h>
#include <gps.h>
#include <payload.h>
#if defined(LOW_POWER_NODE)
#include <ArduinoLowPower.h>
#endif

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

MeshNet::MeshNetApplicationMessage MeshNet::_tmpMessage;
char MeshNet::buffer[80];

int freeMem()
{
//   extern int __heap_start, *__brkval;
//   int v;
//   return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
  return freeMemory();
}

void printMsg(const char * msg)
{
    // disp.setCursor(0, row);
#ifdef UseSD1306
    disp.print(msg);
#endif
    Serial.print(msg);
}

// void displayStats(int16_t rssi, int8_t snr)
// {
// #ifdef UseSD1306
//     // disp.set1X();
//     // disp.setCursor(0,0);
//     disp.print(F("RSSI "));
//     disp.print(rssi);

//     // disp.setCursor(0,2);
//     disp.print(F(" SNR  "));
//     disp.println(snr);
// #endif
// }

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
    : pingNodeId(0), fotaTimeout(0), nodeIdx(0), fotaActive(false), rf95(rf95)
{
}

void MeshNet::setModemConfig(uint8_t mode)
{
    rf95.setModemConfig((RH_RF95::ModemConfigChoice) mode);
}

void MeshNet::setup(uint8_t thisAddress, uint8_t nodeType, float freqMHz, int8_t power, uint16_t cad_timeout)
{
	MeshNet::power = power;
    MeshNet::nodeType = (meshNodeType)nodeType;
	manager = new RHMesh(rf95, thisAddress);
    
#if defined(NODE_HAVE_GPS)
    gpsModule.setup();
#endif

#ifdef UseSD1306
    Wire.begin();
    Wire.beginTransmission(SD1306_Address);
    Wire.endTransmission();

    disp.begin(&Adafruit128x64, SD1306_Address);
    disp.setFont(System5x7);
    disp.set1X();
    disp.setScrollMode(SCROLL_MODE_AUTO);
#endif

	if (!manager->init())
	{
        Serial.println(F("init failed"));
        return;
	}

    sprintf(buffer, "Ready (mem = %d)\n",freeMem());
    printMsg(buffer);

#ifdef LED_BUILTIN
	pinMode(LED_BUILTIN, OUTPUT);
#endif
    _freqMHz = freqMHz;
	rf95.setTxPower(power);
	rf95.setFrequency(_freqMHz);
	rf95.setCADTimeout(cad_timeout);
    manager->setTimeout(1500);
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

#if defined(NODE_HAVE_GPS)
    gpsModule.checkGPS();
#endif
    // if(pingNodeId && uint8_t(currentSeconds - pingTimeout) > pingInterval)
    // {
    //     pingTimeout = currentSeconds;
    //     pingNode(pingNodeId);
    // }
    if (nodeIdx && uint8_t(currentSeconds - pingTimeout) > pingInterval)
    {
        uint8_t ii;

        pingTimeout = currentSeconds;
        for(ii=0;ii<nodeIdx;ii++)
        {
            pingNode(nodes[ii]);    
        }
    }
#ifdef FOTA_CLIENT
    if(fotaActive && uint8_t(currentSeconds - fotaTimeout) > fotaInterval)
    {
        fotaActive = false;
        Serial.println(F("FOTA timeout"));
    }
#endif
	uint8_t dest;
    uint8_t id;
    uint8_t flags;


	if (manager->recvfromAck((uint8_t *)&_tmpMessage, &len, &from, &dest, &id, &flags))
	// if (manager->recvfromAckTimeout((uint8_t *)&_tmpMessage, &len, wait_ms, &from, &dest, &id, &flags))
	{
        // uint8_t snr = rf95.lastSNR();
        // int16_t rssi = rf95.lastRssi();

		blinkLed();
        // displayStats(rssi, snr);

        MeshMessageHeader *p = (MeshMessageHeader *)&_tmpMessage;


        if (len >= 1)
        {
            RHRouter::RoutingTableEntry *route = manager->getRouteTo(from);
            // sprintf(buffer, "Rx:%d RSSI:%d SNR:%d Hop:%d Id:%d len:%d\n", from, rf95.lastRssi(), rf95.lastSNR(), route->next_hop, id, len);
            sprintf(buffer, "%x: %d %d %d %d\n", _tmpMessage.header.msgType, rf95.lastRssi(), rf95.lastSNR(), route->next_hop, id);
            printMsg(buffer);

            switch(p->msgType)
            {
                case MESH_NET_MESSAGE_TYPE_PING_RESPONSE:
                {
                    MeshNetPingRsp *a = (MeshNetPingRsp *)p;
                    sprintf(buffer, "%x: %d dBm RSSI:%d SNR:%d ppm:%d\n", _tmpMessage.header.msgType, a->power, a->rssi - 50, a->snr, a->ppm);
                    printMsg(buffer);
                    break;
                }
                case MESH_NET_MESSAGE_TYPE_PING_REQUEST:
                {
                    sendPingRsp(from);
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
                    sendPingRsp(from);
                    break;
                }

#if defined(NODE_HAVE_GPS)
                case MESH_NET_MESSAGE_TYPE_FIX_REQUEST:
                    sendFixRsp(from);
                    break;
#endif
                case MESH_NET_MESSAGE_TYPE_FIX_RESPONSE:
                {

                    if ( flags & 0x01 )
                    {
                        // uint8_t channel;
                        // uint8_t type;
                        // uint8_t size;

                        Payload pl(_tmpMessage.data, MESH_NET_MAX_MESSAGE_LEN);
                        int32_t latitude = pl.getValue32();
                        int32_t longitude = pl.getValue32();
                        int32_t fix_age = pl.getValue32();
                        int32_t ttff = pl.getValue32();
                        uint32_t date = pl.getValue32();
                        uint32_t time = pl.getValue32();
                        uint32_t time_age = pl.getValue32();;

                        sprintf(buffer, "Date: %lu Time: %lu timeage: %ld\n" , date, time, time_age);
                        printMsg(buffer);

                        // sprintf(buffer, "LAT: %ld LON: %ld fixage: %ld TTFF %ld\n", latitude, longitude, fix_age, ttff);                        
                        sprintf(buffer, "LAT: %ld LON: %ld TTFF %ld fixage: %ld\n", latitude, longitude, ttff, fix_age);                        
                        printMsg(buffer);
                    }
                    break;
                }

                case MESH_NET_MESSAGE_TYPE_MOD_REQUEST:
                {
                    MeshNetModReq *a = (MeshNetModReq *)p;
                    handleModReq(a, flags, from);
                    break;
                }

                case MESH_NET_MESSAGE_TYPE_MOD_RESPONSE:
                {
                    Serial.println(F("Mod response"));
                    if (flags & modreq_mode)
                    {
                        sprintf(buffer, "Set Mode: %d\n", _mode);                        
                        printMsg(buffer);
                        setModemConfig(_mode);
                    }
                    break;
                }

                default:
                    Serial.println(F("Unhandled: message"));
                    break;
            }
        }
	}

#if defined(LOW_POWER_NODE)
    if (nodeType == meshNodeType::node_track)
    {
        if (gpsModule.gpsFix)
        {
            Serial.println(F("Sending Fix"));
            sendFixRsp(GATEWAY_NODE_ID);

            Serial.println(F("Sleeping"));
            gpsModule.powerOff();
            rf95.sleep(); // put radio into sleep mode
            LowPower.sleep(FIX_INTERVAL_MS);

            Serial.println(F("Waking"));
            // now awake. power up peripherals
            gpsModule.powerOn();

            rf95.setModeIdle(); // back to idle
        }
        else
        {
            Serial.println(F("No Fix"));
        }
    }

#endif
}

void MeshNet::sendPingRsp(uint8_t address)
{
    MeshNetPingRsp *r;
    r = (MeshNetPingRsp *)&_tmpMessage;
    r->header.msgType = MESH_NET_MESSAGE_TYPE_PING_RESPONSE;
    r->power = power;
    r->rssi = rf95.lastRssi() + 50;
    r->snr = rf95.lastSNR();

    int ferror = rf95.frequencyError();
    r->ppm = (ferror / _freqMHz);
    // Serial.println(r->ppm);

    uint8_t flags = 0;
    sendtoWaitStats(_tmpMessage, sizeof(MeshNet::MeshNetPingRsp), address, flags);
}

void MeshNet::sendFixReq(uint8_t address,uint8_t flags)
{
    // MeshNetFixReq *a = (MeshNetFixReq *)&_tmpMessage;
    _tmpMessage.header.msgType = MESH_NET_MESSAGE_TYPE_FIX_REQUEST;    
	sendtoWaitStats(_tmpMessage, MESH_NET_MESSAGE_HDR_LEN, address, flags);
}

#if defined(NODE_HAVE_GPS)
void MeshNet::sendFixRsp(uint8_t address)
{
    _tmpMessage.header.msgType = MESH_NET_MESSAGE_TYPE_FIX_RESPONSE;

    uint8_t flags = gpsModule.gpsFix ? 1 : 0;


    int32_t latitude; //= gpsModule.getLatitude();
    int32_t longitude; //= gpsModule.getLongitude();
    uint32_t fix_age;
    uint32_t time_age;
    uint32_t date;
    uint32_t time; 

    gpsModule.getPosition(&latitude, &longitude, &fix_age);
    gpsModule.getDateTime(&date, &time, &time_age);

    Payload pl(_tmpMessage.data, MESH_NET_MAX_MESSAGE_LEN);
    pl.addValue32(latitude);
    pl.addValue32(longitude);
    pl.addValue32(fix_age);
    pl.addValue32(gpsModule.ttff);

    pl.addValue32(date);
    pl.addValue32(time);
    pl.addValue32(time_age);

    sprintf(buffer, "Date: %lu Time: %lu timeage: %ld " , date, time, time_age);
    printMsg(buffer);

    sprintf(buffer, "LAT: %ld LON: %ld fixage: %ld TTFF %ld\n", latitude, longitude, fix_age, gpsModule.ttff);                        
    printMsg(buffer);


    sendtoWaitStats(_tmpMessage, MESH_NET_MESSAGE_HDR_LEN + pl.getSize(), address, flags);
}
#endif

void MeshNet::sendModReq(uint8_t address, uint8_t mode, uint8_t power, uint8_t flags)
{
    if (flags & modreq_mode)
    {
        _mode = mode;
    }

    MeshNetModReq *a = (MeshNetModReq *)&_tmpMessage;
    a->header.msgType = MESH_NET_MESSAGE_TYPE_MOD_REQUEST;    
    a->mode = mode;
    a->power = power;
	sendtoWaitStats(_tmpMessage, sizeof(MeshNet::MeshNetModReq), address, flags);
}

void MeshNet::sendModRsp(uint8_t address, uint8_t flags)
{
    MeshNetModRsp *a = (MeshNetModRsp *)&_tmpMessage;
    a->header.msgType = MESH_NET_MESSAGE_TYPE_MOD_RESPONSE;    
	sendtoWaitStats(_tmpMessage, sizeof(MeshNet::MeshNetModRsp), address, flags);
}

void MeshNet::handleModReq(MeshNetModReq *a, uint8_t flags, uint8_t from)
{
    Serial.println(F("handleModReq"));
    if (flags & modreq_power)
    {
        setPower(a->power);
        sprintf(buffer, "Set power: %d\n", a->power);                        
        Serial.print(buffer);
    }
    if (flags & modreq_mode)
    {
        sendModRsp(from, flags);
        setModemConfig(a->mode);
        sprintf(buffer, "Set Mode: %d\n", a->mode);                        
        Serial.print(buffer);
    }
}

uint8_t MeshNet::sendtoWaitStats(MeshNetApplicationMessage &msg, uint8_t len, uint8_t address, uint8_t flags)
{
	blinkLed();

	uint8_t error = manager->sendtoWait((uint8_t*)&msg, len, address, flags);

    switch(error)
    {
        case RH_ROUTER_ERROR_NO_ROUTE:
            Serial.println(F("no route"));
            break;

        case RH_ROUTER_ERROR_NONE:
        {
            RHRouter::RoutingTableEntry *route = manager->getRouteTo(address);
            sprintf(buffer, "%x: %d %d %d\n", msg.header.msgType, rf95.lastRssi(), rf95.lastSNR(), route->next_hop);
            printMsg(buffer);
        }
        break;
    }

    return error;
}

void MeshNet::pingNode(uint8_t address, uint8_t flags)
{
    MeshNetPingReq *a = (MeshNetPingReq *)&_tmpMessage;
    a->header.msgType = MESH_NET_MESSAGE_TYPE_PING_REQUEST;
	a->power = power;
    
	sendtoWaitStats(_tmpMessage, sizeof(MeshNet::MeshNetPingReq), address, flags);
}

void MeshNet::appMessage(uint8_t address, char * buf, uint8_t flags)
{
    MeshNetApplicationMessage *a = (MeshNetApplicationMessage *)&_tmpMessage;
    a->header.msgType = MESH_NET_MESSAGE_TYPE_APP_REQUEST;
    uint8_t len = strlen(buf) + sizeof(MeshMessageHeader);
    memcpy(a->data, buf, strlen(buf));       
	sendtoWaitStats(_tmpMessage, len, address, flags);
}

void MeshNet::sendFix(uint8_t address)
{
#if defined(NODE_HAVE_GPS)
    if (gpsModule.gpsFix)
    {
        gpsModule.getFixStr(buffer);
        Serial.print(buffer);
        appMessage(address, buffer);
    }
    else
    {
        Serial.println(F("No Fix"));
    }
#endif
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
        Serial.print(F("Erasing Flash chip ... "));
        flash.blockErase32K(0);
        while(flash.busy());
        Serial.println(F("DONE"));        
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

    Serial.print(F("Bytes: "));
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