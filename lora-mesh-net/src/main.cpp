
/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>

#include <RHMesh.h>
#include <RH_RF95.h>
#include <MeshNet.h>
#include <NVStorage.h>
#define FREQMHZ 434.4
#define POWER 20
#define MODE 0
#define CAD_TIMEOUT 500
 
static RH_RF95 rf95(SS,DIO0);

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
    char * s;
    newData = false;
    s = strtok(buffer, " ");

    switch(*s)
    {
        case 'N':
            Serial.println(EEPROM.read(0), HEX);
            s = strtok(NULL, " ");
            nodeInfo.nodeId = atoi(s);
            sprintf(buffer, "Node id: %d\n", nodeInfo.nodeId);
            Serial.print(buffer);
            break;

        case 'T':
            s = strtok(NULL, " ");
            nodeInfo.nodeType = atoi(s);
            sprintf(buffer, "Node type: %d\n", nodeInfo.nodeType);
            Serial.print(buffer);
            break;

        case 'W':
            EEPROM.write(0, nodeInfo.nodeId);
            EEPROM.write(1, nodeInfo.nodeType);

#if defined(_SAMD21_)
            EEPROM.commit();
            // NVIC_SystemReset();             
#endif
            break;

        case 'P':
        {
            s = strtok(NULL, " ");
            uint8_t nodeId = atoi(s);
            sprintf(buffer, "Ping Node id: %d\n", nodeId);
            Serial.print(buffer);
            mesh.addNode(nodeId, MeshNet::meshModeType::mode_ping);
            break;
        }

        case 'D':
        {
            s = strtok(NULL, " ");
            int8_t powerdBm = atoi(s);
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
            uint8_t nodeId;
            uint16_t seqnum;
            s = strtok(NULL, " ");
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
            s = strtok(NULL, " ");
            uint8_t nodeId = atoi(s);
            mesh.sendFix(nodeId);
            break;
        }

        case 'L':
        {

            s = strtok(NULL, " ");
            uint8_t mode = atoi(s);
            mesh.setModemConfig(mode);
            break;
        }

        case 'M':
        {
            // broadcast mode change
            // M <mode>            
            // s = strtok(NULL, " ");
            // uint8_t nodeId = atoi(s);
            // Serial.println(nodeId, DEC);
            
            s = strtok(NULL, " ");
            uint8_t mode = atoi(s);
            Serial.println(mode, DEC);
            
            mesh.sendModReq(0xff, mode, 0, MeshNet::modreq_mode);
            break;
        }

        case 'K':
        {
            // W <node> <power>            
            s = strtok(NULL, " ");
            uint8_t nodeId = atoi(s);
            Serial.println(nodeId, DEC);
            
            s = strtok(NULL, " ");
            uint8_t power = atoi(s);
            Serial.println(power, DEC);
            
            mesh.sendModReq(nodeId, 0, power, MeshNet::modreq_power);
            break;
        }

        case 'F':
        {
            // F <node>
            s = strtok(NULL, " ");
            uint8_t nodeId = atoi(s);
            Serial.println(nodeId, DEC);
            mesh.sendFixReq(nodeId);
            break;
        }

        case 'H':
        {
            // H <node>
            s = strtok(NULL, " ");
            uint8_t nodeId = atoi(s);
            sprintf(buffer, "FixReq Node id: %d\n", nodeId);
            Serial.print(buffer);
            mesh.addNode(nodeId, MeshNet::meshModeType::mode_fix);
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

    sprintf(buffer, "Node id: 0x%x Node type: %d\n", nodeInfo.nodeId, nodeInfo.nodeType);
    Serial.print(buffer);

    mesh.setup(nodeInfo.nodeId, nodeInfo.nodeType, FREQMHZ, POWER, CAD_TIMEOUT, MODE);
}

void loop()
{

    recvWithEndMarker();
    handleData();

    mesh.loop(200);
}
