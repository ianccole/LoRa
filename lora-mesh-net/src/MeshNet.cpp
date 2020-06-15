#include <Arduino.h>
#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <MeshNet.h>

#define UseSD1306 

#ifdef UseSD1306
#define SD1306_Address 0x3C                       //define I2C address foe SD1306
#define LCDI2C_Address 0x3F                       //define I2C address for PCF8574 LCD backpack, usually 0x27 or 0x3F
#include "SSD1306Ascii.h"                   //https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h"
SSD1306AsciiWire disp;
#endif

RH_RF95 MeshNet::rf95;

MeshNet::MeshNetPingMessage msg;
static char buffer[50];

int freeMem()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void printMsg(const char * msg, bool clear=false)
{
    // disp.setCursor(0, row);
    if(clear)
        disp.clear();
    disp.print(msg);
    Serial.print(msg);
}

void MeshNet::setup(uint8_t thisAddress, float freqMHz, int8_t power, uint16_t cad_timeout)
{
	MeshNet::power = power;
	manager = new RHMesh(rf95, thisAddress);

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
	uint8_t len = sizeof(msg);
	uint8_t from;
	uint8_t dest;
    uint8_t id;


	if (manager->recvfromAckTimeout((uint8_t *)&msg, &len, wait_ms, &from, &dest, &id))
	{
		blinkLed();

        MeshMessageHeader *p = (MeshMessageHeader *)&msg;

        if (len >= 1)
        {
            RHRouter::RoutingTableEntry *route = manager->getRouteTo(from);
            sprintf(buffer, "Rx:%d RSSI:%d\nSNR:%d Hop:%d Id:%d\n", from, rf95.lastRssi(), rf95.lastSNR(), route->next_hop, id);
            // Serial.print(buffer);
            printMsg(buffer, true);

            switch(p->msgType)
            {
                case MESH_NET_MESSAGE_TYPE_PING_RESPONSE:
                {
                    MeshNetPingMessage *a = (MeshNetPingMessage *)p;
                    sprintf(buffer, "%d dBm RSSI:%d\nSNR:%d\n",a->power, a->rssi, a->snr);
                    // Serial.print(buffer);
                    printMsg(buffer);
                    break;
                }
                case MESH_NET_MESSAGE_TYPE_PING_REQUEST:
                {
                    MeshNetPingMessage *a = (MeshNetPingMessage *)p;
                    msg.header.msgType = MESH_NET_MESSAGE_TYPE_PING_RESPONSE;
                    msg.power = power;
                    msg.rssi = rf95.lastRssi();
                    msg.snr = rf95.lastSNR();

                    sprintf(buffer, "%d dBm RSSI:%d\nSNR:%d\n", a->power, rf95.lastRssi(), rf95.lastSNR());
                    // Serial.print(buffer);
                    printMsg(buffer);

                    sendtoWaitStats((uint8_t*)&msg, sizeof(MeshNet::MeshNetPingMessage), from);
                    break;
                }
                default:
                    sprintf(buffer, "Unhandled:%3d\n", p->msgType);
                    Serial.print(buffer);
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
            // Serial.print(buffer);
            printMsg(buffer);

        }
        break;
    }

    return error;
}

void MeshNet::pingNode(uint8_t address, uint8_t flags)
{
	// uint8_t error;

	memset(&msg, 0, sizeof(MeshNet::MeshNetPingMessage));

    msg.header.msgType = MESH_NET_MESSAGE_TYPE_PING_REQUEST;
	msg.power = power;
    
	sendtoWaitStats((uint8_t*)&msg, sizeof(MeshNet::MeshNetPingMessage), address, flags);
}

void MeshNet::arpNode(uint8_t address)
{
    manager->doArp(address);
}