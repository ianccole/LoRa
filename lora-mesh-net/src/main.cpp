
/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#define FREQMHZ 434.4
#define POWER 20
#define CAD_TIMEOUT 500

#include <MeshNet.h>

struct nodeInfo
{
  uint8_t nodeId;
  uint8_t nodeType;
  uint8_t RFU;
  uint8_t RFU2;
} nodeInfo;

const byte numChars = 32;
char buffer[numChars];   // an array to store the received data
bool newData = false;
bool serialData = false;

// uint8_t nodes[] = {0x23, 0x8a, 0xe3, 0x52, 0xfe};

// #define N_NODES (sizeof(nodes) / sizeof(nodes[0]))

uint8_t nodeId = 0;
// uint8_t routes[N_NODES] = {}; // full routing table for mesh
// int16_t rssi[N_NODES] = {};   // signal strength info

// Singleton instance of the radio driver
// RH_RF95 rf95;

// Class to manage message delivery and receipt, using the driver declared above
RHMesh *manager;

MeshNet mesh;

// message buffer
// const uint8_t buflen = RH_MESH_MAX_MESSAGE_LEN;
// char buf[buflen];

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
   
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        Serial.print(rc);
        if (rc != endMarker) {
            buffer[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            buffer[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void handleData() {
  if (newData == true) 
  {
    newData = false;

    switch(buffer[0])
    {
      case 'N':
        nodeInfo.nodeId = atoi(&buffer[1]);
        sprintf(buffer, "Node id: 0x%02X\n", nodeInfo.nodeId);
        Serial.print(buffer);
        break;

      case 'T':
        nodeInfo.nodeType = atoi(&buffer[1]);
        sprintf(buffer, "Node type: 0x%02X\n", nodeInfo.nodeType);
        Serial.print(buffer);
        break;

      case 'W':
        EEPROM.put(0, nodeInfo);
        break;

      case 'P':
        nodeId = atoi(&buffer[1]);
        sprintf(buffer, "Ping Node id: 0x%02X\n", nodeId);
        Serial.print(buffer);
        // mesh.pingNode(nodeId);
        mesh.ping = nodeId;
        break;

      case 'A':
        nodeId = atoi(&buffer[1]);
        sprintf(buffer, "ARP Node id: 0x%02X\n", nodeId);
        Serial.print(buffer);
        mesh.arpNode(nodeId);
        break;

      case 'R':
        mesh.printRoutingTable();
        break;
    }
  }
}

int freeMem()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void setup()
{
  Serial.begin(9600);

// #ifdef LED_BUILTIN
//   pinMode(LED_BUILTIN, OUTPUT);
// #endif

  EEPROM.get(0, nodeInfo);
  sprintf(buffer, "Node id: 0x%02X, Node type: 0x%02X\n", nodeInfo.nodeId, nodeInfo.nodeType);
  Serial.print(buffer);

  // manager = new RHMesh(rf95, nodeId);

  // if (!manager->init())
  // {
  //   Serial.println(F("init failed"));
  // }
  // else
  // {
  //   Serial.println("done");
  // }
  // rf95.setTxPower(POWER, false);
  // rf95.setFrequency(FREQMHZ);
  // rf95.setCADTimeout(CAD_TIMEOUT);

  // // long range configuration requires for on-air time
  // boolean longRange = false;
  // if (longRange)
  // {
  //   RH_RF95::ModemConfig modem_config = {
  //       0x78, // Reg 0x1D: BW=125kHz, Coding=4/8, Header=explicit
  //       0xC4, // Reg 0x1E: Spread=4096chips/symbol, CRC=enable
  //       0x08  // Reg 0x26: LowDataRate=On, Agc=Off.  0x0C is LowDataRate=ON, ACG=ON
  //   };
  //   rf95.setModemRegisters(&modem_config);
  //   if (!rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096))
  //   {
  //     Serial.println(F("set config failed"));
  //   }
  // }
  nodeId = nodeInfo.nodeId; //XXXX
  mesh.setup(nodeInfo.nodeId, FREQMHZ, POWER, CAD_TIMEOUT);

  Serial.println("RF95 ready");
  Serial.print(F("mem = "));
  Serial.println(freeMem());
}

// const __FlashStringHelper *getErrorString(uint8_t error)
// {
//   switch (error)
//   {
//   case RH_ROUTER_ERROR_NONE:
//     return F(" OK");
//     break;
//   case RH_ROUTER_ERROR_INVALID_LENGTH:
//     return F(" INV");
//     break;
//   case RH_ROUTER_ERROR_NO_ROUTE:
//     return F(" NR");
//     break;
//   case RH_ROUTER_ERROR_TIMEOUT:
//     return F(" TO");
//     break;
//   case RH_ROUTER_ERROR_NO_REPLY:
//     return F(" NAK");
//     break;
//   case RH_ROUTER_ERROR_UNABLE_TO_DELIVER:
//     return F(" DEL");
//     break;
//   }
//   return F(" unknown");
// }

// void updateRoutingTable()
// {
//   for (uint8_t n = 0; n < N_NODES; n++)
//   {
//     uint8_t node = nodes[n];
//     RHRouter::RoutingTableEntry *route = manager->getRouteTo(node);
//     if (node == nodeId)
//     {
//       routes[n] = 255; // self
//     }
//     else
//     {
//       routes[n] = route->next_hop;
//       if (routes[n] == 0)
//       {
//         // if we have no route to the node, reset the received signal strength
//         rssi[n] = 0;
//       }
//     }
//   }
// }

// // Create a JSON string with the routing info to each node
// void getRouteInfoString(char *p, size_t len)
// {
//   p[0] = '\0';
//   strcat(p, "[");
//   for (uint8_t n = 1; n <= N_NODES; n++)
//   {
//     strcat(p, "{\"n\":");
//     sprintf(p + strlen(p), "%02X", routes[n - 1]);
//     strcat(p, ",");
//     strcat(p, "\"r\":");
//     sprintf(p + strlen(p), "%4d", rssi[n - 1]);
//     strcat(p, "}");
//     if (n < N_NODES)
//     {
//       strcat(p, ",");
//     }
//   }
//   strcat(p, "]");
// }

// void printNodeInfo(uint8_t node, char *s)
// {
//   Serial.print(F("node: "));
//   Serial.print(F("{"));
//   Serial.print(F("\""));
//   Serial.print(node, HEX);
//   Serial.print(F("\""));
//   Serial.print(F(": "));
//   Serial.print(s);
//   Serial.println(F("}"));
// }

// void setRssiNextHop(uint8_t node, int16_t rssi_val)
// {
//   for (uint8_t n = 0; n < N_NODES; n++)
//   {
//     if (nodes[n] == node)
//     {
//       rssi[n] = rssi_val;
//       return;
//     }
//   }
// }

// void blinkLed()
// {
// #ifdef LED_BUILTIN
//   digitalWrite(LED_BUILTIN, HIGH);
//   delay(100);
//   digitalWrite(LED_BUILTIN, LOW);
// #endif
// }

void loop()
{
    recvWithEndMarker();
    handleData();

    mesh.loop(200);

//   for (uint8_t n = 0; n < N_NODES; n++)
//   {
//     uint8_t node = nodes[n];
//     if (node == nodeId)
//       continue; // self

//     updateRoutingTable();
//     getRouteInfoString(buf, RH_MESH_MAX_MESSAGE_LEN);

//     Serial.print(F("->"));
//     Serial.print(node, HEX);
//     Serial.print(F(" :"));
//     Serial.print(buf);

//     // send an acknowledged message to the target node
//     uint8_t error = manager->sendtoWait((uint8_t *)buf, strlen(buf), node);
//     Serial.println(getErrorString(error));
//     if (error == RH_ROUTER_ERROR_NONE)
//     {
//       // we received an acknowledgement from the next hop for the node we tried to send to.
//       RHRouter::RoutingTableEntry *route = manager->getRouteTo(node);
//       // Serial.println(n);
//       // Serial.println(route->next_hop);
//       // Serial.println(node);
//       if (route->next_hop != 0)
//       {
//         setRssiNextHop(route->next_hop, mesh.lastRssi());
//         rssi[route->next_hop - 1] = mesh.lastRssi();
//       }
//     }

    // Serial.println("check serial");
    // recvWithEndMarker();
    // handleData();

    // if (nodeId == nodes[0]) printNodeInfo(nodeId, buf); // debugging

    // listen for incoming messages. Wait a random amount of time before we transmit
    // again to the next node
//     unsigned long nextTransmit = millis() + random(3000, 5000);
//     while (nextTransmit > millis())
//     {
//       int waitTime = nextTransmit - millis();
//       uint8_t len = sizeof(buf);
//       uint8_t from;
//       if (manager->recvfromAckTimeout((uint8_t *)buf, &len, waitTime, &from))
//       {
//         buf[len] = '\0'; // null terminate string
//         Serial.print(from, HEX);
//         Serial.print(F("->"));
//         Serial.print(F(" :"));
//         Serial.println(buf);
//         // if (nodeId == nodes[0]) printNodeInfo(from, buf); // debugging
//         // we received data from node 'from', but it may have actually come from an intermediate node
//         RHRouter::RoutingTableEntry *route = manager->getRouteTo(from);
//         if (route->next_hop != 0)
//         {
//           setRssiNextHop(route->next_hop, mesh.lastRssi());
//           //   rssi[route->next_hop-1] = rf95.lastRssi();
//           mesh.blinkLed();
//         }
//       }
//     }
//   }
}
