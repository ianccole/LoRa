#ifndef NVSTORAGE_h
#define NVSTORAGE_h

#if defined(_SAMD21_)
    #define EEPROM_EMULATION_SIZE 64
    #include <FlashAsEEPROM.h>
    // EEPROM.init();
#else
    #include <EEPROM.h>
#endif



#endif
