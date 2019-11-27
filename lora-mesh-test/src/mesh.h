// mesh.h
#ifndef mesh_h
#define mesh_h

#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#define RH_HAVE_SERIAL

class mesh
{
// Singleton instance of the radio driver
    static RH_RF95 rf95;
    static RHMesh *manager;
    static char buf[RH_MESH_MAX_MESSAGE_LEN];
    // SPIFlash flash(FLASH_SS, 0xEF30);
    static SPIFlash &flash;
    // static uint8_t nodes[] = {0x23, 0x8a, 0xe3, 0x52, 0xfe};
    // #define N_NODES (sizeof(nodes)/sizeof(nodes[0]))

    public:
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

            for (uint8_t i=0;i<8;i++) 
            {     
                Serial.print(flash.UNIQUEID[i], HEX); 
                Serial.print(' '); 
                nodeId += flash.UNIQUEID[i];
            }
            if(nodeId == 0xff)
            {
                nodeId -= 1;
            }
            Serial.print("\nnodeId Id: ");
            Serial.println(nodeId, HEX); 

            // initialize LED digital pin as an output.
            pinMode(LED_BUILTIN, OUTPUT);

              manager = new RHMesh(rf95, nodeId);

            if (!manager->init()) {
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
            if (longRange) {
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

            for(uint8_t n=1;n<=N_NODES;n++) 
            {
                routes[n-1] = 0;
                rssi[n-1] = 0;
            }

            Serial.print(F("mem = "));
            Serial.println(freeMem());
        };

    private:
        uint8_t nodeId = 0;
        uint8_t routes[N_NODES]; // full routing table for mesh
        int16_t rssi[N_NODES]; // signal strength info

        int freeMem() 
        {
            extern int __heap_start, *__brkval;
            int v;
            return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
        };

};

#endif
