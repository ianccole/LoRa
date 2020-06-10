
#ifndef MeshNet_h
#define MeshNet_h

// Types of MeshNet message, used to set msgType in the MeshNetHeader
#define MESH_NET_MESSAGE_TYPE_APPLICATION                    0
#define MESH_NET_MESSAGE_TYPE_PING_REQUEST                   1
#define MESH_NET_MESSAGE_TYPE_PING_RESPONSE                  2
#define MESH_NET_MESSAGE_TYPE_ROUTE_FAILURE                  3

class MeshNet : public RHMesh
{
public:
    #define MESH_NET_MAX_MESSAGE_LEN (RH_ROUTER_MAX_MESSAGE_LEN - sizeof(MeshNet::MeshMessageHeader))

    /// Structure of the basic MeshNet header.
    typedef struct
    {
        uint8_t             msgType;  ///< Type of MeshNet message, one of MESH_NET_MESSAGE_TYPE_*
    } MeshNetMessageHeader;

    /// Signals an application layer message for the caller of RHMesh
    typedef struct
    {
        MeshMessageHeader   header; ///< msgType = RH_MESH_MESSAGE_TYPE_APPLICATION 
        uint8_t             data[MESH_NET_MAX_MESSAGE_LEN]; ///< Application layer payload data
    } MesNetApplicationMessage;

    typedef struct
    {
        MeshMessageHeader   header;  ///< msgType = RH_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_*
        uint8_t             destlen; ///< Reserved. Must be 1
        uint8_t             dest;    ///< The address of the destination node being pinged
        uint8_t             power;   ///< Transmit power of this message
        uint8_t             rssi;    ///< RSSI in Ping response
        uint8_t             snr;     ///< SNR in Ping response
    } MeshNetPingMessage;


};
#endif
