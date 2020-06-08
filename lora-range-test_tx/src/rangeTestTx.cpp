#include <SPI.h>
#include <LoRa.h>

const uint32_t frequency = 434400000;
const uint32_t baud = 115200;
const uint8_t destination, localAddress = 0xff;


int msgCount = 0;

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

void setup() {
  Serial.begin(baud);
  while (!Serial);

  Serial.println("LoRa Sender");

  if (!LoRa.begin(frequency)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(msgCount);

  // send packet
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(msgCount);
  LoRa.endPacket();

  msgCount++;

  delay(5000);
}