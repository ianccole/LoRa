
#ifndef MeshNet_h
#define MeshNet_h

// Types of MeshNet message, used to set msgType in the MeshNetHeader
#define MESH_NET_MESSAGE_TYPE_ROUTE_FAILURE                  0
#define MESH_NET_MESSAGE_TYPE_PING_REQUEST                   1
#define MESH_NET_MESSAGE_TYPE_PING_RESPONSE                  2

class MeshNet
{
public:
    #define MESH_NET_MAX_MESSAGE_LEN (RH_MESH_MAX_MESSAGE_LEN - sizeof(MeshNet::MeshMessageHeader))

    /// Structure of the basic MeshNet header.
    typedef struct
    {
        uint8_t             msgType;  ///< Type of MeshNet message, one of MESH_NET_MESSAGE_TYPE_*
    } MeshMessageHeader;

    /// Signals an application layer message for the caller of RHMesh
    typedef struct
    {
        MeshMessageHeader   header; ///< msgType = RH_MESH_MESSAGE_TYPE_APPLICATION 
        uint8_t             data[MESH_NET_MAX_MESSAGE_LEN]; ///< Application layer payload data
    } MesNetApplicationMessage;

    typedef struct
    {
        MeshMessageHeader   header;  ///< msgType = RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_*
        uint8_t             power;   ///< Transmit power of this message
        uint8_t             rssi;    ///< RSSI in Ping response
        uint8_t             snr;     ///< SNR in Ping response
    } MeshNetPingMessage;

    void setup(uint8_t thisAddress, float freqMHz, int8_t power, uint16_t cad_timeout);

    void loop(uint16_t wait_ms);

    void pingNode(uint8_t address, uint8_t flags = 0);

    int16_t lastRssi()
    {
        return rf95.lastRssi();
    }

    void printRoutingTable()
    {
        manager->printRoutingTable();
    }

    void blinkLed()
    {
        #ifdef LED_BUILTIN
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        #endif
    };

private:
    int8_t power;
    // Singleton instance of the radio driver
    static RH_RF95 rf95;

    // Class to manage message delivery and receipt, using the driver declared above
    RHMesh *manager;


};
#endif