
#ifndef MeshNet_h
#define MeshNet_h

// Types of MeshNet message, used to set msgType in the MeshNetHeader
#define MESH_NET_MESSAGE_TYPE_ROUTE_FAILURE                  0
#define MESH_NET_MESSAGE_TYPE_PING_REQUEST                   0x50
#define MESH_NET_MESSAGE_TYPE_PING_RESPONSE                  0x51
#define MESH_NET_MESSAGE_TYPE_FOTA_REQUEST                   0x52
#define MESH_NET_MESSAGE_TYPE_FOTA_RESPONSE                  0x53

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
        int8_t              rssi;    ///< RSSI in Ping response
        int8_t              snr;     ///< SNR in Ping response
    } MeshNetPingMessage;

    typedef struct
    {
        MeshMessageHeader   header; ///< msgType = RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_*
        uint8_t             flags;  ///< flags
        uint8_t             data[MESH_NET_MAX_MESSAGE_LEN - 1]; ///< Intel Hex string
    } MeshNetFOTAMessageReq;

    typedef struct
    {
        MeshMessageHeader   header; ///< msgType = RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_*
        uint8_t             count;  ///< line count
    } MeshNetFOTAMessageRsp;

    void setup(uint8_t thisAddress, float freqMHz, int8_t power, uint16_t cad_timeout);

    void loop(uint16_t wait_ms);

    void pingNode(uint8_t address, uint8_t flags = 0);

    void sendFOTA(uint8_t address, char *buf);

    void arpNode(uint8_t address);

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

    uint8_t ping;

private:
    int8_t power;
    // Singleton instance of the radio driver
    static RH_RF95 rf95;

    // Class to manage message delivery and receipt, using the driver declared above
    RHMesh *manager;

    uint8_t sendtoWaitStats(uint8_t *buf, uint8_t len, uint8_t dest, uint8_t flags = 0);

    void handleFOTA(MeshNetFOTAMessageReq *msg);
};
#endif
