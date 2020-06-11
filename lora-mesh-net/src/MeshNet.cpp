#include <Arduino.h>
#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <MeshNet.h>


#define FREQMHZ 434.4
#define POWER 20
#define CAD_TIMEOUT 500

RH_RF95 MeshNet::rf95;

MeshNet::MeshNetPingMessage msg;
static char buffer[40];

void MeshNet::setup(uint8_t thisAddress, float freqMHz, int8_t power, uint16_t cad_timeout)
{
	MeshNet::power = power;
	manager = new RHMesh(rf95, thisAddress);

	if (!manager->init())
	{
	Serial.println(F("init failed"));
	return;
	}

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
	if (manager->recvfromAckTimeout((uint8_t *)&msg, &len, wait_ms, &from))
	{
		blinkLed();
	    Serial.println("incoming");
	}
}

void MeshNet::pingNode(uint8_t address, uint8_t flags)
{
	uint8_t error;

	memset(&msg, 0, sizeof(MeshNet::MeshNetPingMessage));

    msg.header.msgType = MESH_NET_MESSAGE_TYPE_PING_REQUEST;
	msg.power = power;
    
	blinkLed();
	Serial.println("outgoing");
	error = manager->sendtoWait((uint8_t*)&msg, sizeof(MeshNet::MeshNetPingMessage), address, flags);

    switch(error)
    {
        case RH_ROUTER_ERROR_NO_ROUTE:
            Serial.println("no route");
            break;

        case RH_ROUTER_ERROR_NONE:
        {
            RHRouter::RoutingTableEntry *route = manager->getRouteTo(address);

            sprintf(buffer, "Sent to:%3d ACKed from:%3d RSSI:%3d SNR:%3d\n", 
                address, route->next_hop, rf95.lastRssi(), rf95.lastSNR());
            Serial.print(buffer);
        }
        break;
    }
}