#include <Arduino.h>

/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include <SPIMemory.h>

#ifdef LED_BUILTIN
  #undef LED_BUILTIN
  #define LED_BUILTIN 4
#endif

SPIFlash flash(8);

void printUniqueID(void) {
  long long _uniqueID = flash.getUniqueID();
  if (_uniqueID) {
    Serial.print("Unique ID: ");
    Serial.print(uint32_t(_uniqueID / 1000000L));
    Serial.print(uint32_t(_uniqueID % 1000000L));
    Serial.print(", ");
    Serial.print("0x");
    Serial.print(uint32_t(_uniqueID >> 32), HEX);
    Serial.println(uint32_t(_uniqueID), HEX);
  }
}

void setup()
{
  Serial.begin(9600);

  flash.begin();

  uint32_t JEDEC = flash.getJEDECID();

  Serial.print("JEDEC ID: 0x");
  Serial.println(JEDEC, HEX);

  Serial.println(flash.getCapacity());

  printUniqueID();

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  char incomingByte;
  static int delayms = 100;
  static int delaymsoff = 1000;

  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("LED ON");
  // wait for a second
  delay(delayms);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("LED OFF");
   // wait for a second
  delay(delaymsoff);
  Serial.println(flash.getCapacity());
  if(Serial.available()>0)
  {
    incomingByte = Serial.read();

    Serial.print("IN:");
    Serial.println(incomingByte);

    if(incomingByte == '+'){
      Serial.println("going up");
      delaymsoff += 100;
    }
    if(incomingByte == '-'){
      Serial.println("going down");
      delaymsoff -= 100;
    }    
    Serial.println(delayms);
  }
}
