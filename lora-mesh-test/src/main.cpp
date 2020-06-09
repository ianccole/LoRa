
/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include <SPIFlash.h>

#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#define RH_HAVE_SERIAL
#define FREQMHZ 434.4
#define POWER 23
#define CAD_TIMEOUT 500

uint8_t nodes[] = {0x23, 0x8a, 0xe3, 0x52, 0xfe};

#define N_NODES (sizeof(nodes) / sizeof(nodes[0]))

uint8_t nodeId = 0;
uint8_t routes[N_NODES] = {}; // full routing table for mesh
int16_t rssi[N_NODES] = {};   // signal strength info

// mesh mymesh(nodes);

// Singleton instance of the radio driver
RH_RF95 rf95;

// Class to manage message delivery and receipt, using the driver declared above
RHMesh *manager;

// message buffer
char buf[RH_MESH_MAX_MESSAGE_LEN];

// SPIFlash flash(FLASH_SS, 0xEF30);
SPIFlash flash(SS_FLASHMEM);

int freeMem()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void setup()
{
  Serial.begin(9600);

  if (flash.initialize())
  {
    Serial.println("Init OK!");
  }
  else
  {
    Serial.println("Init FAIL!");
  }

#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
#endif

  Serial.print("DeviceID: ");
  Serial.println(flash.readDeviceId(), HEX);

  flash.readUniqueId();

  for (uint8_t i = 0; i < 8; i++)
  {

    Serial.print(flash.UNIQUEID[i], HEX);
    Serial.print(' ');
    nodeId += flash.UNIQUEID[i];
  }
  if (nodeId == 0xff)
  {
    nodeId -= 1;
  }
  Serial.print("\nnodeId Id: ");
  Serial.println(nodeId, HEX);

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  manager = new RHMesh(rf95, nodeId);

  if (!manager->init())
  {
    Serial.println(F("init failed"));
  }
  else
  {
    Serial.println("done");
  }
  rf95.setTxPower(POWER, false);
  rf95.setFrequency(FREQMHZ);
  rf95.setCADTimeout(CAD_TIMEOUT);

  // long range configuration requires for on-air time
  boolean longRange = false;
  if (longRange)
  {
    RH_RF95::ModemConfig modem_config = {
        0x78, // Reg 0x1D: BW=125kHz, Coding=4/8, Header=explicit
        0xC4, // Reg 0x1E: Spread=4096chips/symbol, CRC=enable
        0x08  // Reg 0x26: LowDataRate=On, Agc=Off.  0x0C is LowDataRate=ON, ACG=ON
    };
    rf95.setModemRegisters(&modem_config);
    if (!rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096))
    {
      Serial.println(F("set config failed"));
    }
  }

  Serial.println("RF95 ready");
  Serial.print(F("mem = "));
  Serial.println(freeMem());
}

const __FlashStringHelper *getErrorString(uint8_t error)
{
  switch (error)
  {
  case RH_ROUTER_ERROR_NONE:
    return F(" OK");
    break;
  case RH_ROUTER_ERROR_INVALID_LENGTH:
    return F(" INV");
    break;
  case RH_ROUTER_ERROR_NO_ROUTE:
    return F(" NR");
    break;
  case RH_ROUTER_ERROR_TIMEOUT:
    return F(" TO");
    break;
  case RH_ROUTER_ERROR_NO_REPLY:
    return F(" NAK");
    break;
  case RH_ROUTER_ERROR_UNABLE_TO_DELIVER:
    return F(" DEL");
    break;
  }
  return F(" unknown");
}

void updateRoutingTable()
{
  for (uint8_t n = 0; n < N_NODES; n++)
  {
    uint8_t node = nodes[n];
    RHRouter::RoutingTableEntry *route = manager->getRouteTo(node);
    if (node == nodeId)
    {
      routes[n] = 255; // self
    }
    else
    {
      routes[n] = route->next_hop;
      if (routes[n] == 0)
      {
        // if we have no route to the node, reset the received signal strength
        rssi[n] = 0;
      }
    }
  }
}

// Create a JSON string with the routing info to each node
void getRouteInfoString(char *p, size_t len)
{
  p[0] = '\0';
  strcat(p, "[");
  for (uint8_t n = 1; n <= N_NODES; n++)
  {
    strcat(p, "{\"n\":");
    sprintf(p + strlen(p), "%02X", routes[n - 1]);
    strcat(p, ",");
    strcat(p, "\"r\":");
    sprintf(p + strlen(p), "%4d", rssi[n - 1]);
    strcat(p, "}");
    if (n < N_NODES)
    {
      strcat(p, ",");
    }
  }
  strcat(p, "]");
}

void printNodeInfo(uint8_t node, char *s)
{
  Serial.print(F("node: "));
  Serial.print(F("{"));
  Serial.print(F("\""));
  Serial.print(node, HEX);
  Serial.print(F("\""));
  Serial.print(F(": "));
  Serial.print(s);
  Serial.println(F("}"));
}

void setRssiNextHop(uint8_t node, int16_t rssi_val)
{
  for (uint8_t n = 0; n < N_NODES; n++)
  {
    if (nodes[n] == node)
    {
      rssi[n] = rssi_val;
      return;
    }
  }
}

void blinkLed()
{
#ifdef LED_BUILTIN
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
#endif
}

void loop()
{
  for (uint8_t n = 0; n < N_NODES; n++)
  {
    uint8_t node = nodes[n];
    if (node == nodeId)
      continue; // self

    updateRoutingTable();
    getRouteInfoString(buf, RH_MESH_MAX_MESSAGE_LEN);

    Serial.print(F("->"));
    Serial.print(node, HEX);
    Serial.print(F(" :"));
    Serial.print(buf);

    // send an acknowledged message to the target node
    uint8_t error = manager->sendtoWait((uint8_t *)buf, strlen(buf), node);
    Serial.println(getErrorString(error));
    if (error == RH_ROUTER_ERROR_NONE)
    {
      // we received an acknowledgement from the next hop for the node we tried to send to.
      RHRouter::RoutingTableEntry *route = manager->getRouteTo(node);
      // Serial.println(n);
      // Serial.println(route->next_hop);
      // Serial.println(node);
      if (route->next_hop != 0)
      {
        setRssiNextHop(route->next_hop, rf95.lastRssi());
        rssi[route->next_hop - 1] = rf95.lastRssi();
      }
    }
    // if (nodeId == nodes[0]) printNodeInfo(nodeId, buf); // debugging

    // listen for incoming messages. Wait a random amount of time before we transmit
    // again to the next node
    unsigned long nextTransmit = millis() + random(3000, 5000);
    while (nextTransmit > millis())
    {
      int waitTime = nextTransmit - millis();
      uint8_t len = sizeof(buf);
      uint8_t from;
      if (manager->recvfromAckTimeout((uint8_t *)buf, &len, waitTime, &from))
      {
        buf[len] = '\0'; // null terminate string
        Serial.print(from, HEX);
        Serial.print(F("->"));
        Serial.print(F(" :"));
        Serial.println(buf);
        // if (nodeId == nodes[0]) printNodeInfo(from, buf); // debugging
        // we received data from node 'from', but it may have actually come from an intermediate node
        RHRouter::RoutingTableEntry *route = manager->getRouteTo(from);
        if (route->next_hop != 0)
        {
          setRssiNextHop(route->next_hop, rf95.lastRssi());
          //   rssi[route->next_hop-1] = rf95.lastRssi();
          blinkLed();
        }
      }
    }
  }
}
