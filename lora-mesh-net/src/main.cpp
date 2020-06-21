
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

uint8_t nodeId = 0;

MeshNet mesh;

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

void setup()
{
  Serial.begin(9600);

  EEPROM.get(0, nodeInfo);
  sprintf(buffer, "Node id: %d, Node type: %d\n", nodeInfo.nodeId, nodeInfo.nodeType);
  Serial.print(buffer);

  nodeId = nodeInfo.nodeId; //XXXX
  mesh.setup(nodeInfo.nodeId, FREQMHZ, POWER, CAD_TIMEOUT);
}

void loop()
{
    recvWithEndMarker();
    handleData();

    mesh.loop(200);
}
