
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
#define MESH_NET_MESSAGE_TYPE_FIX_REQUEST                    0x56
#define MESH_NET_MESSAGE_TYPE_FIX_RESPONSE                   0x57
#define MESH_NET_MESSAGE_TYPE_MOD_REQUEST                    0x58
#define MESH_NET_MESSAGE_TYPE_MOD_RESPONSE                   0x59

#define GATEWAY_NODE_ID                                      100
#define FIX_INTERVAL_MS                                      10000
#define seconds() (millis()/1000)

class MeshNet
{
public:
    #define MESH_NET_MESSAGE_HDR_LEN sizeof(MeshNet::MeshMessageHeader)
    #define MESH_NET_MAX_MESSAGE_LEN (RH_MESH_MAX_MESSAGE_LEN - sizeof(MeshNet::MeshMessageHeader))
    

    enum class meshNodeType
    {
        node_track      = 0,
        node_relay      = 1,
        node_gateway    = 2,
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
        uint8_t             data[MESH_NET_MAX_MESSAGE_LEN]; ///< Application layer payload data
    } MeshNetApplicationMessage;

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
        int8_t              ppm;     ///< Fc ppm error
    } MeshNetPingRsp;

    // typedef struct
    // {
    //     MeshMessageHeader   header;  ///< msgType = RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_*
    // } MeshNetFixReq;

    // typedef struct
    // {
    //     MeshMessageHeader   header;  ///< msgType = RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_*
    //     int8_t              pad1;
    //     int8_t              pad2;
    //     int8_t              pad3;
    //     uint32_t            date;
    //     uint32_t            time;
    //     int32_t             lat;
    //     int32_t             lon;
    //     uint32_t            fix_age;
    //     uint32_t            time_age;
    //     int8_t              gpsFix;
    // } MeshNetFixRsp;

    /*-----------------------------
    | flags:
    |0 1 2 3 4 5 6 7 
    +-+-+-+-+-+-+-+-+
    |P|M|
    |W|O|
    |R|D|
    +-+-+-+-+-+-+-+-+
    | PWR Action power request
    | MOD Action mode request
    +------------------------------*/
    enum MeshNetModReqFlags
    {
        modreq_none        = 0,
        modreq_power       = 1,
        modreq_mode        = 2
    };
    typedef struct
    {
        MeshMessageHeader   header;  ///< msgType = RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_*
        uint8_t             mode;    ///< requested mode
        uint8_t             power;   ///< requested Transmit power
    } MeshNetModReq;

    typedef struct
    {
        MeshMessageHeader   header;  ///< msgType = RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_*
    } MeshNetModRsp;

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
    void sendFixReq(uint8_t address,uint8_t flags=0);
    void sendFixRsp(uint8_t address);

    void sendModReq(uint8_t address, uint8_t mode, uint8_t power, uint8_t flags=0);
    void sendModRsp(uint8_t address, uint8_t flags=0);
    void handleModReq(MeshNetModReq *a, uint8_t flags, uint8_t from);

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

    void setModemConfig(uint8_t mode);

    uint8_t pingNodeId;

private:
    /// Temporary mesage buffer
    static MeshNetApplicationMessage _tmpMessage;
    static char buffer[50];

    const uint8_t pingInterval = 5;
    const uint8_t fotaInterval = 15;
    uint8_t pingTimeout;
    uint8_t fotaTimeout;
    uint32_t flashIndex;
    int8_t power;
    float _freqMHz;
    uint8_t _mode;
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
    uint8_t sendtoWaitStats(MeshNetApplicationMessage &msg, uint8_t len, uint8_t dest, uint8_t flags = 0);
    bool burnHexLine(const uint8_t *pLine);
    void handleFOTA(MeshNetFOTAMessageReq *msg, uint8_t from);
};
#endif
