
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

const byte numChars = 64;
char buffer[numChars];   // an array to store the received data
bool newData = false;
bool serialData = false;

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
            sprintf(buffer, "Node id: %d\n", nodeInfo.nodeId);
            Serial.print(buffer);
            break;

        case 'T':
            nodeInfo.nodeType = atoi(&buffer[1]);
            sprintf(buffer, "Node type: %d\n", nodeInfo.nodeType);
            Serial.print(buffer);
            break;

        case 'W':
            EEPROM.put(0, nodeInfo);
            break;

        case 'P':
        {
            uint8_t nodeId = atoi(&buffer[1]);
            sprintf(buffer, "Ping Node id: %d\n", nodeId);
            Serial.print(buffer);
            // mesh.pingNode(nodeId);
            mesh.ping = nodeId;
            break;
        }

        case 'R':
            mesh.printRoutingTable();
            break;

        // F node seq :100030000C94AD000C94AD000C94AD000C94AD008C
        case 'F':
        {   
            char *s;
            uint8_t nodeId;
            uint8_t seqnum;
            s = strtok(&buffer[1], " ");
            nodeId = atoi(s);
            // Serial.println(nodeId, DEC);
            s = strtok(NULL, " ");
            seqnum = atoi(s);
            // Serial.println(seqnum, DEC);
            s = strtok(NULL, " ");
            // Serial.println(s);
            mesh.sendFOTAREQ(nodeId, seqnum, s);
            break;
        }
    }
  }
}

void setup()
{
  Serial.begin(57600);

  EEPROM.get(0, nodeInfo);
  sprintf(buffer, "Node id: %d, Node type: %d\n", nodeInfo.nodeId, nodeInfo.nodeType);
  Serial.print(buffer);

  mesh.setup(nodeInfo.nodeId, FREQMHZ, POWER, CAD_TIMEOUT);
}

void loop()
{
    recvWithEndMarker();
    handleData();

    mesh.loop(200);
}
