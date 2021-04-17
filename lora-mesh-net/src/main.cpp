
/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#define FREQMHZ 434.4
#define POWER 20
#define CAD_TIMEOUT 500

#include <MeshNet.h>
#include <NVStorage.h>
 
#if defined(ARDUINO_ARCH_SAMD)
    #define Serial SerialUSB
#else
    #define DIO0 2
#endif

static RH_RF95 rf95(SS,DIO0);

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

MeshNet mesh(rf95);

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
            Serial.println(EEPROM.read(0));
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
            EEPROM.write(0, nodeInfo.nodeId);
            EEPROM.write(1, nodeInfo.nodeType);

#if defined(_SAMD21_)
            EEPROM.commit();
#endif
            break;

        case 'P':
        {
            uint8_t nodeId = atoi(&buffer[1]);
            sprintf(buffer, "Ping Node id: %d\n", nodeId);
            Serial.print(buffer);
            // mesh.pingNodeId = nodeId;
            mesh.addNode(nodeId);
            break;
        }

        case 'D':
        {
            int8_t powerdBm = atoi(&buffer[1]);
            sprintf(buffer, "Power: %d dBm\n", powerdBm);
            Serial.print(buffer);
            mesh.setPower(powerdBm);
            break;
        }

        case 'R':
            mesh.printRoutingTable();
            break;
#ifdef FOTA_SERVER
        // F node seq :100030000C94AD000C94AD000C94AD000C94AD008C
        case 'F':
        {   
            char *s;
            uint8_t nodeId;
            uint16_t seqnum;
            s = strtok(&buffer[1], " ");
            nodeId = atoi(s);
            // Serial.println(nodeId, DEC);
            s = strtok(NULL, " ");
            seqnum = atoi(s);
            // Serial.println(seqnum, DEC);
            s = strtok(NULL, " ");
            // Serial.println(s);
            s[strlen(s)] = '\0';
            mesh.sendFOTAREQ(nodeId, seqnum, s);
            break;
        }
#endif
        case 'G':
        {
            uint8_t nodeId = atoi(&buffer[1]);
            mesh.sendFix(nodeId);
            break;
        }
    }
  }
}

void setup()
{
    Serial.begin(57600);

    nodeInfo.nodeId = EEPROM.read(0);
    nodeInfo.nodeType = EEPROM.read(1);

    sprintf(buffer, "Node id: %d, Node type: %d\n", nodeInfo.nodeId, nodeInfo.nodeType);
    Serial.print(buffer);

    mesh.setup(nodeInfo.nodeId, nodeInfo.nodeType, FREQMHZ, POWER, CAD_TIMEOUT);
}

void loop()
{

    recvWithEndMarker();
    handleData();

    mesh.loop(20);

}
