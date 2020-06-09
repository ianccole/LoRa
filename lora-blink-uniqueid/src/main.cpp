
/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
// #include <SPIFlash.h>

#include <SPI.h>
#include <EEPROM.h>

// #ifdef LED_BUILTIN
//   #undef LED_BUILTIN
//   #define LED_BUILTIN 4
// #endif

// SPIFlash flash(FLASH_SS, 0xEF30);
// SPIFlash flash(SS_FLASHMEM);
// uint8_t nodeId = 0;

struct nodeInfo
{
  uint8_t nodeId;
  uint8_t nodeType;
  uint8_t RFU;
  uint8_t RFU2;
} nodeInfo;

const byte numChars = 32;
char buffer[numChars];   // an array to store the received data
boolean newData = false;

void setup()
{
  Serial.begin(9600);

  EEPROM.get(0, nodeInfo);
  sprintf(buffer, "Node id: 0x%02X, Node type: 0x%02X\n", nodeInfo.nodeId, nodeInfo.nodeType);
  Serial.print(buffer);

  // if (flash.initialize())
  // {
  //   Serial.println("Init OK!");
  // }
  // else
  // {
  //   Serial.println("Init FAIL!");
  // }

  // Serial.print("DeviceID: ");
  // Serial.println(flash.readDeviceId(), HEX);

  // flash.readUniqueId(); 

  // for (uint8_t i=0;i<8;i++) 
  // { 
    
  //   Serial.print(flash.UNIQUEID[i], HEX); 
  //   Serial.print(' '); 
  //   uniqueId += flash.UNIQUEID[i];
  // }
  // Serial.print("\nUnique Id: ");
  // Serial.println(uniqueId, HEX); 

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

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
    // Serial.print("This just in ... ");
    // Serial.println(buffer);
    newData = false;

    switch(buffer[0])
    {
      case '+':
        Serial.println("going up");
        break;

      case '-':
        Serial.println("going down");
        break;

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
    }
  }
}

void oldserial(int &delaymsoff)
{
  char incomingByte;
  if(Serial.available()>0)
  {
    incomingByte = Serial.read();
    Serial.print(incomingByte);

    switch(incomingByte)
    {
      case '+':
        Serial.println("going up");
        delaymsoff += 100;
        break;

      case '-':
        Serial.println("going down");
        delaymsoff -= 100;
        break;

      case 'n':
        // node
        incomingByte = Serial.parseInt();
        // print
    }
    Serial.println(delaymsoff);
  }
}

void loop()
{
  static int delayms = 100;
  static int delaymsoff = 1000/2;

  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  // Serial.println("LED ON");

  // wait for a second
  delay(delayms);

  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
  // Serial.println("LED OFF");

   // wait for a second
  delay(delaymsoff);

  //oldserial(delaymsoff);

  recvWithEndMarker();
  handleData();
}
