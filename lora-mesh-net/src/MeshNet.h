
#ifndef MeshNet_h
#define MeshNet_h

// Types of MeshNet message, used to set msgType in the MeshNetHeader
#define MESH_NET_MESSAGE_TYPE_ROUTE_FAILURE                  0
#define MESH_NET_MESSAGE_TYPE_PING_REQUEST                   0x50
#define MESH_NET_MESSAGE_TYPE_PING_RESPONSE                  0x51
#define MESH_NET_MESSAGE_TYPE_FOTA_REQUEST                   0x52
#define MESH_NET_MESSAGE_TYPE_FOTA_RESPONSE                  0x53
#define MESH_NET_MESSAGE_TYPE_APP_REQUEST                    0x54
#define MESH_NET_MESSAGE_TYPE_APP_RESPONSE                   0x55

#define seconds() (millis()/1000)

class MeshNet
{
public:
    #define MESH_NET_MAX_MESSAGE_LEN (RH_MESH_MAX_MESSAGE_LEN - sizeof(MeshNet::MeshMessageHeader))

    enum class meshNodeType
    {
        node_gateway    = 0,
        node_relay      = 1,
        node_track      = 2,
        node_test       = 3,
        node_default    = 255
    };

    /// Structure of the basic MeshNet header.
    typedef struct
    {
        uint8_t             msgType;  ///< Type of MeshNet message, one of MESH_NET_MESSAGE_TYPE_*
    } MeshMessageHeader;

    /// Signals an application layer message for the caller of RHMesh
    typedef struct
    {
        MeshMessageHeader   header; ///< msgType = RH_MESH_MESSAGE_TYPE_APPLICATION 
        char                data[MESH_NET_MAX_MESSAGE_LEN - sizeof(MeshMessageHeader)]; ///< Application layer payload data
    } MesNetApplicationMessage;

    /*-----------------------------
    | flags:
    |0 1 2 3 4 5 6 7 
    +-+-+-+-+-+-+-+-+
    |A|A|
    |P|D|
    |C|R|
    +-+-+-+-+-+-+-+-+
    | APC Adaptive Power Control
    | ADC Adaptive Data Rate
    +------------------------------*/
    typedef struct
    {
        MeshMessageHeader   header;  ///< msgType = RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_*
        uint8_t             power;   ///< Transmit power of this message
    } MeshNetPingReq;

    typedef struct
    {
        MeshMessageHeader   header;  ///< msgType = RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_*
        uint8_t             power;   ///< Transmit power of this message
        int8_t              rssi;    ///< RSSI in Ping response
        int8_t              snr;     ///< SNR in Ping response
        // int8_t              gpsFix;
        unsigned long       date;
        unsigned long       time;
        long                lat;
        long                lon;
        unsigned long       fix_age;
        unsigned long       time_age;
    } MeshNetPingRsp;

    typedef struct 
    {
        uint16_t            sequence;
    } MeshFOTAHeader;

    typedef struct
    {
        MeshMessageHeader   header; ///< msgType = RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_*
        MeshFOTAHeader      headerFOTA;
        uint8_t             data[MESH_NET_MAX_MESSAGE_LEN - sizeof(MeshMessageHeader) - sizeof(MeshFOTAHeader)]; ///< Intel Hex string
    } MeshNetFOTAMessageReq;

    /*-----------------------------
    | flags:
    |0 1 2 3 4 5 6 7 
    +-+-+-+-+-+-+-+-+
    | ACK value     |
    +-+-+-+-+-+-+-+-+
    | 0 ACK
    | 1 NAK csum error
    | 2 NAK not active
    +------------------------------*/
    typedef struct
    {
        MeshMessageHeader   header; ///< msgType = RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_*
        MeshFOTAHeader      headerFOTA;
        uint16_t            sequence;
    } MeshNetFOTAMessageRsp;

    typedef struct
    {
        MeshMessageHeader   header;  ///< msgType = RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_*
        char                data[MESH_NET_MAX_MESSAGE_LEN - sizeof(MeshMessageHeader)]; ///< message text
    } MeshNetAppMessage;

    /// Constructor.
    /// \param[in] driver The RadioHead driver to use to transport messages.
    /// \param[in] thisAddress The address to assign to this node. Defaults to 0
    MeshNet(RH_RF95&);

    void setup(uint8_t thisAddress, uint8_t nodeType, float freqMHz, int8_t power, uint16_t cad_timeout);

    void loop(uint16_t wait_ms);

    void pingNode(uint8_t address, uint8_t flags = 0);

    void appMessage(uint8_t address, char * buffer, uint8_t flags = 0);

    void sendFix(uint8_t address);
    void sendFOTAREQ(uint8_t address, uint16_t seqnum, char *buf);
    void sendFOTARSP(uint8_t address, uint16_t seqnum, uint8_t flags);

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

    void setPower(int8_t powerdBm)
    {   
        power = powerdBm;
        rf95.setTxPower(power);
    };

    void addNode(uint8_t node)
    {
        if (node == 0)
        {
            memset(nodes, 0, sizeof(nodes));
            nodeIdx = 0;
        }
        else
        {
            if(nodeIdx < sizeof(nodes)-1)
            {
                nodes[nodeIdx++] = node;
            }
        }
    }

    void MeshNet::setModemConfig(uint8_t mode);

    uint8_t pingNodeId;

private:
    const uint8_t pingInterval = 5;
    const uint8_t fotaInterval = 15;
    uint8_t pingTimeout;
    uint8_t fotaTimeout;
    uint32_t flashIndex;
    int8_t power;
    uint8_t nodes[4];
    uint8_t nodeIdx;

    // node type
    meshNodeType nodeType;
    
    bool fotaActive;
    // Singleton instance of the radio driver
    RH_RF95& rf95;

    // Class to manage message delivery and receipt, using the driver declared above
    RHMesh *manager;

    void sendPingRsp(uint8_t address);
    uint8_t sendtoWaitStats(uint8_t *buf, uint8_t len, uint8_t dest, uint8_t flags = 0);
    bool burnHexLine(const uint8_t *pLine);
    void handleFOTA(MeshNetFOTAMessageReq *msg, uint8_t from);
};
#endif
