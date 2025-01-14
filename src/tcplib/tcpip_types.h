/**
 * TCP/IP Stack User Types Header File
 *
 * @file tcpip_types.h
 *
 * @ingroup tcpiplite
 *
 * @brief This file provides the TCP/IP Stack type definitions.
 *
 * @version TCP/IP Lite Driver Version 5.0.0
 */
/*
    © [2023] Microchip Technology Inc. and its subsidiaries

    Subject to your compliance with these terms, you may use Microchip software and any derivatives 
    exclusively with Microchip products. You are responsible for complying with 3rd party license terms 
    applicable to your use of 3rd party software (including open source software) that may accompany 
    Microchip software. SOFTWARE IS “AS IS.” NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR 
    STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-
    INFRINGEMENT, MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR 
    CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED 
    TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
    POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY 
    LAW, MICROCHIP’S TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
    */

#ifndef TCPIP_TYPES_H
#define TCPIP_TYPES_H

// Included Files
#include <stdint.h>

typedef enum
{
    TCB_ERROR = -1,
    TCB_NO_ERROR = 0
} tcbError_t;

/**
 * @ingroup tcpiplite
 * @struct ethernetFrame_t
 * @brief Ethernet frame information.
 */
typedef struct
{
    uint8_t destinationMAC[6];
    uint8_t sourceMAC[6];
    union
    {
        uint16_t type; /**< Ethernet 2 frame type, 802.3 length, 802.1Q TPID */
        uint16_t length;
        uint16_t tpid;
    } id;
    // if tpid == 0x8100 then TCI structure goes here
    // if tpid != 0x8100, then ethertype/length goes here
    // UP to 1500 Bytes of payload goes here
    // 32 bit checksum goes here
} ethernetFrame_t;

#define ETHERTYPE_IPV4  0x0800
#define ETHERTYPE_ARP   0x0806
#define ETHERTYPE_IPV6  0x86DD
#define ETHERTYPE_VLAN  0x8100
#define ETHERTYPE_LLDP  0x88CC
#define ETHERTYPE_EAPoL 0x888E

// From RFC 2851
#define INETADDRESSTYPE_IPV4 1
#define INETADDRESSTYPE_IPV6 2
#define INETADDRESSTYPE_DNS 16

#define ETHERNET_ADDR_LEN 6
#define IP_ADDR_LEN 4

// From RFC 3493
// Supported address families
#ifndef AF_INET6
/**
 * @ingroup tcpiplite
 * @def AF_INET
 * @brief Internet IP Protocol
 */
#define AF_INET 2
#endif

#ifndef AF_INET6
/**
 * @ingroup tcpiplite
 * @def AF_INET6
 * @brief IP version 6
 */
#define AF_INET6 10
#endif

#ifndef PF_INET
/**
 * @ingroup tcpiplite
 * @def PF_INET
 * @brief Protocol families, same as address families
 */
#define PF_INET AF_INET
#endif

#ifndef PF_INET6
#define PF_INET6 AF_INET6
#endif

#ifndef IN6ADDR_ANY_INIT
#define IN6ADDR_ANY_INIT { { { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 } } }
#endif

#ifndef IN6ADDR_LOOPBACK_INIT
#define IN6ADDR_LOOPBACK_INIT { { { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1 } } }
#endif

#ifndef INET_ADDRSTRLEN
/**
 * @ingroup tcpiplite
 * @def INET_ADDRSTRLEN
 * @brief Length of the string form for IP
 */
#define INET_ADDRSTRLEN 16
#endif

#ifndef INET6_ADDRSTRLEN
/**
 * @ingroup tcpiplite
 * @def INET6_ADDRSTRLEN
 * @brief Length of the string form for IPv6
 */
#define INET6_ADDRSTRLEN 46
#endif

/**
 * @ingroup tcpiplite
 * @struct ipv4Header_t
 * @brief IPv4 header information
 */
typedef struct
{
    unsigned ihl : 4;                /**< Internet header length in 32-bit words */
    unsigned version : 4;            /**< 4 for IPV4 */
    unsigned ecn : 2;                /**< Explicit Congestion Notification RFC3168 */
    unsigned dscp : 6;               /**< Differentiated Service Code Point RFC3260 */
    uint16_t length;                 /**< Total length including header and data (no more than 576 octets) */
    uint16_t identifcation;          /**< ID for packet fragments */
    unsigned fragmentOffsetHigh : 5; /**< Offset for a fragment, needed for reassembly */
    unsigned : 1;                    /**< Leave this bit as zero */
    unsigned dontFragment : 1;       /**< Drop if fragmentation is required to route */
    unsigned moreFragments : 1;      /**< Fragments have this bit set (except for the final packet) */
    uint8_t fragmentOffsetLow;       /**< Low byte for the fragment offset */
    uint8_t timeToLive;              /**< Decrement at each hop and discard when zero */
    uint8_t protocol;                /**< IP Protocol (from RFC790) */
    uint16_t headerCksm;             /**< RFC1071 defines this calculation */
    uint32_t srcIpAddress;           /**< Source IP Address */
    uint32_t dstIpAddress;           /**< Destination IP Address */
    /* options could go here if IHL > 5 */
    /* payload goes here */
} ipv4Header_t;

/**
 * @ingroup tcpiplite
 * @struct ipv4_pseudo_header_t
 * @brief Pseudo header used for checksum calculation on UDP and TCP
 */
typedef struct
{
    uint32_t srcIpAddress; /**< Source IP Address */
    uint32_t dstIpAddress; /**< Destination IP Address */
    uint8_t protocol;      /**< Protocol */
    uint8_t z;             /**< Used for memory cleaning */
    uint16_t length;       /**< Length */
} ipv4_pseudo_header_t;

/**
 * @ingroup tcpiplite
 * @struct icmpHeader_t
 * @brief ICMP header information
 */
typedef struct
{
    union
    {
        uint16_t typeCode;
        struct
        {
            uint8_t code;
            uint8_t type;
        };
    };
    uint16_t checksum;
} icmpHeader_t;


/**
 * @ingroup tcpiplite
 * @enum icmpTypeCodes_t
 * @brief ICMP Types and Codes
 */
typedef enum 
{
    ECHO_REPLY                                  = 0x0000,   /**< Echo reply */
    DEST_NETWORK_UNREACHABLE                    = 0x0300,   /**< Destination network unreachable  */
    DEST_HOST_UNREACHABLE                       = 0x0301,   /**< Destination host unreachable */
    DEST_PROTOCOL_UNREACHABLE                   = 0x0302,   /**< Destination protocol unreachable */
    DEST_PORT_UNREACHABLE                       = 0x0303,   /**< Destination port unreachable */
    FRAGMENTATION_REQUIRED                      = 0x0304,   /**< Fragmentation required */
    SOURCE_ROUTE_FAILED                         = 0x0305,   /**< Fragmentation failed */
    DESTINATION_NETWORK_UNKNOWN                 = 0x0306,   /**< Unknown destination network */
    SOURCE_HOST_ISOLATED                        = 0x0307,   /**< Source host isolated */
    NETWORK_ADMINISTRATIVELY_PROHIBITED         = 0x0308,   /**< Network administratively prohibited */
    HOST_ADMINISTRATIVELY_PROHIBITED            = 0x0309,   /**< Host administratively prohibited */
    NETWORK_UNREACHABLE_FOR_TOS                 = 0x030A,   /**< Network unreachable for TOS */
    HOST_UNREACHABLE_FOR_TOS                    = 0x030B,   /**< Host unreachable for TOS */
    COMMUNICATION_ADMINISTRATIVELY_PROHIBITED   = 0x030C,   /**< Communication administratively prohibited */
    HOST_PRECEDENCE_VIOLATION                   = 0x030D,   /**< Host precedence violation */
    PRECEDENCE_CUTOFF_IN_EFFECT                 = 0x030E,   /**< Precedence cutoff in effect */
    SOURCE_QUENCH                               = 0x0400,   /**< Source quench */
    REDIRECT_DATAGRAM_FOR_THE_NETWORK           = 0x0500,   /**< Redirect message for the network*/
    REDIRECT_DATAGRAM_FOR_THE_HOST              = 0x0501,   /**< Redirect message for the host*/
    REDIRECT_DATAGRAM_FOR_THE_TOS_AND_NETWORK   = 0x0502,   /**< Redirect message for the TOS and network*/
    REDIRECT_DATAGRAM_FOR_THE_TOS_AND_HOST      = 0x0503,   /**< Redirect message for the TOS and host*/
    ALTERNATE_HOST_ADDRESS                      = 0x0600,   /**< Alternate host address */
    ECHO_REQUEST                                = 0x0800,   /**< Echo Request: Ask for a ping! */
    UNASSIGNED_ECHO_TYPE_CODE_REQUEST_1         = 0x082A,   /**< Unassigned codes */
    UNASSIGNED_ECHO_TYPE_CODE_REQUEST_2         = 0x08FC,   /**< Unassigned codes */
    ROUTER_ADVERTISEMENT                        = 0x0900,   /**< Router advertisement */
    ROUTER_SOLICITATION                         = 0x0A00,   /**< Router solicitaion */
    TRACEROUTE                                  = 0x3000    /**< Trace route */
} icmpTypeCodes_t;

/**
 * @ingroup tcpiplite
 * @struct udpHeader_t
 * @brief UDP header information
 */
typedef struct
{
    uint16_t srcPort;  /**< Source Port */
    uint16_t dstPort;  /**< Destination Port */
    uint16_t length;   /**< Length */
    uint16_t checksum; /**< Checksum */
} udpHeader_t;

/**
 * @ingroup tcpiplite
 * @struct tcpHeader_t
 * @brief TCP header information
 */
typedef struct
{
    uint16_t sourcePort;     /**< Source port */
    uint16_t destPort;       /**< Destination port */
    uint32_t sequenceNumber; /**< Sequence number */
    uint32_t ackNumber;      /**< ACK Number */
    union
    {
        uint8_t byte13;
        struct
        {
            uint8_t ns          : 1;    /**< ECN-nonce concealment protection (added to header by RFC 3540). */
            uint8_t reserved    : 3;    /**< For future use and needs to be set to zero. */
            uint8_t dataOffset  : 4;    /**< Specifies the size of the TCP header in 32-bit words. */
        };
    };

    union
    {
        uint8_t flags;          /**< Flags */
        struct
        {
            uint8_t fin : 1;    /**< No more data from sender. */
            uint8_t syn : 1;    /**< Synchronizes sequence numbers. Only the first packet sent from each end must have this flag set. */
            uint8_t rst : 1;    /**< Resets the connection. */
            uint8_t psh : 1;    /**< Asks to push the buffered data to the receiving application. */
            uint8_t ack : 1;    /**< Indicates that the ACKfield is significant. */
            uint8_t urg : 1;    /**< Indicates that the Urgent pointer field is significant. */
            uint8_t ece : 1;    /**< ECN-Echo. Depends on SYN flag set or clear. */
            uint8_t cwr : 1;    /**< Congestion Window Reduced (CWR) (added to header by RFC 3168). */
        };
    };

    uint16_t windowSize; /**< Window size. */
    uint16_t checksum;   /**< TCP Header Checksum. */
    uint16_t urgentPtr;  /**< Urgent pointer. */
    // Options follow here
    // Pad the header so the total header is a multiple of 4 bytes
    // Data follows
} tcpHeader_t;


/**
 * @ingroup tcpiplite
 * @enum ipProtocolNumbers
 * @brief IP Protocol Numbers. 
 * List from RFC5237 http://www.iana.org/assignments/protocol-numbers/protocol-numbers.txt
 */
typedef enum {
//     Keyword        Decimal               Protocol                                                            Reference
    HOPOPT_TCPIP        = 0,    /**< IPv6 Hop-by-Hop Option                                             [RFC2460] */
    ICMP_TCPIP          = 1,    /**< Internet Control Message                                           [RFC792] */
    IGMP_TCPIP          = 2,    /**< Internet Group Management                                          [RFC1112] */
    GGP_TCPIP           = 3,    /**< Gateway-to-Gateway                                                 [RFC823] */
    IPV4_TCPIP          = 4,    /**< IPv4 encapsulation                                                 [RFC2003] */
    ST_TCPIP            = 5,    /**< Stream                                                             [RFC1190][RFC1819] */
    TCP_TCPIP           = 6,    /**< Transmission Control                                               [RFC793] */
    CBT_TCPIP           = 7,    /**< CBT                                                                [Tony_Ballardie] */
    EGP_TCPIP           = 8,    /**< Exterior Gateway Protocol                                          [RFC888][David_Mills] */
    IGP_TCPIP           = 9,    /**< Any private interior gateway (used by Cisco for their IGRP)        [Internet_Assigned_Numbers_Authority] */
    BBN_RCC_MON_TCPIP   = 10,   /**< BBN RCC Monitoring                                                 [Steve_Chipman] */
    NVP_II_TCPIP        = 11,   /**< Network Voice Protocol                                             [RFC741][Steve_Casner] */
    PUP_TCPIP           = 12,   /**< PUP                                                                [Boggs, D., J. Shoch, E. Taft, and R. Metcalfe, "PUP: An Internetwork Architecture", XEROX Palo Alto Research Center, CSL-79-10, July 1979; also in IEEE Transactions on Communication, Volume COM-28, Number 4, April 1980.][[XEROX]] */
    ARGUS_TCPIP         = 13,   /**< ARGUS                                                              [Robert_W_Scheifler] */
    EMCON_TCPIP         = 14,   /**< EMCON                                                              [<mystery contact>] */
    XNET_TCPIP          = 15,   /**< Cross Net Debugger                                                 [Haverty, J., "XNET Formats for Internet Protocol Version 4", IEN 158, October 1980.][Jack_Haverty] */
    CHAOS_TCPIP         = 16,   /**< Chaos                                                              [J_Noel_Chiappa] */
    UDP_TCPIP           = 17,   /**< User Datagram                                                      [RFC768][Jon_Postel] */
    MUX_TCPIP           = 18,   /**< Multiplexing                                                       [Cohen, D. and J. Postel, "Multiplexing Protocol", IEN 90, USC/Information Sciences Institute, May 1979.][Jon_Postel] */
    DCN_MEAS_TCPIP      = 19,   /**< DCN Measurement Subsystems                                         [David_Mills] */
    HMP_TCPIP           = 20,   /**< Host Monitoring                                                    [RFC869][Robert_Hinden] */
    PRM_TCPIP           = 21,   /**< Packet Radio Measurement                                           [Zaw_Sing_Su] */
    XNS_IDP_TCPIP       = 22,   /**< XEROX NS IDP                                                       ["The Ethernet, A Local Area Network: Data Link Layer and Physical Layer Specification", AA-K759B-TK, Digital Equipment Corporation, Maynard, MA. Also as: "The Ethernet - A Local Area Network", Version 1.0, Digital Equipment Corporation, Intel Corporation, Xerox Corporation, September 1980. And: "The Ethernet, A Local Area Network: Data Link Layer and Physical Layer Specifications", Digital, Intel and Xerox, November 1982. And: XEROX, "The Ethernet, A Local Area Network: Data Link Layer and Physical Layer Specification", X3T51/80-50, Xerox Corporation, Stamford, CT., October 1980.][[XEROX]] */
    TRUNK_1_TCPIP       = 23,   /**< Trunk-1                                                            [Barry_Boehm] */
    TRUNK_2_TCPIP       = 24,   /**< Trunk-2                                                            [Barry_Boehm] */
    LEAF_1_TCPIP        = 25,   /**< Leaf-1                                                             [Barry_Boehm] */
    LEAF_2_TCPIP        = 26,   /**< Leaf-2                                                             [Barry_Boehm] */
    RDP_TCPIP           = 27,   /**< Reliable Data Protocol                                             [RFC908][Robert_Hinden] */
    IRTP_TCPIP          = 28,   /**< Internet Reliable Transaction                                      [RFC938][Trudy_Miller] */
    ISO_TP4_TCPIP       = 29,   /**< ISO Transport Protocol Class 4                                     [RFC905][<mystery contact>] */
    NETBLT_TCPIP        = 30,   /**< Bulk Data Transfer Protocol                                        [RFC969][David_Clark] */
    MFE_NSP_TCPIP       = 31,   /**< MFE Network Services Protocol                                      [Shuttleworth, B., "A Documentary of MFENet, a National Computer Network", UCRL-52317, Lawrence Livermore Labs, Livermore, California, June 1977.][Barry_Howard] */
    MERIT_INP_TCPIP     = 32,   /**< MERIT Internodal Protocol                                          [Hans_Werner_Braun] */
    DCCP_TCPIP          = 33,   /**< Datagram Congestion Control Protocol                               [RFC4340] */
    THREEPC_TCPIP       = 34,   /**< Third Party Connect Protocol                                       [Stuart_A_Friedberg] */
    IDPR_TCPIP          = 35,   /**< Inter-Domain Policy Routing Protocol                               [Martha_Steenstrup] */
    XTP_TCPIP           = 36,   /**< XTP                                                                [Greg_Chesson] */
    DDP_TCPIP           = 37,   /**< Datagram Delivery Protocol                                         [Wesley_Craig] */
    IDPR_CMTP_TCPIP     = 38,   /**< IDPR Control Message Transport Proto                               [Martha_Steenstrup] */
    TPpp_TCPIP          = 39,   /**< TP++ Transport Protocol                                            [Dirk_Fromhein] */
    IL_TCPIP            = 40,   /**< IL Transport Protocol                                              [Dave_Presotto] */
    IPV6_TUNNEL_TCPIP   = 41,   /**< IPv6 encapsulation                                                 [RFC2473] */
    SDRP_TCPIP          = 42,   /**< Source Demand Routing Protocol                                     [Deborah_Estrin] */
    IPV6_Route_TCPIP    = 43,   /**< Routing Header for IPv6                                            [Steve_Deering] */
    IPV6_Frag_TCPIP     = 44,   /**< Fragment Header for IPv6                                           [Steve_Deering] */
    IDRP_TCPIP          = 45,   /**< Inter-Domain Routing Protocol                                      [Sue_Hares] */
    RSVP_TCPIP          = 46,   /**< Reservation Protocol                                               [RFC2205][RFC3209][Bob_Braden] */
    GRE_TCPIP           = 47,   /**< Generic Routing Encapsulation                                      [RFC1701][Tony_Li] */
    DSR_TCPIP           = 48,   /**< Dynamic Source Routing Protocol                                    [RFC4728] */
    BNA_TCPIP           = 49,   /**< BNA                                                                [Gary Salamon] */
    ESP_TCPIP           = 50,   /**< Encap Security Payload                                             RFC4303] */
    AH_TCPIP            = 51,   /**< Authentication Header                                              [RFC4302] */
    I_NLSP_TCPIP        = 52,   /**< Integrated Net Layer Security TUBA                                 [K_Robert_Glenn] */
    SWIPE_TCPIP         = 53,   /**< IP with Encryption                                                 [John_Ioannidis] */
    NARP_TCPIP          = 54,   /**< NBMA Address Resolution Protocol                                   [RFC1735] */
    MOBILE_TCPIP        = 55,   /**< IP Mobility                                                        [Charlie_Perkins] */
    TLSP_TCPIP          = 56,   /**< Transport Layer Security Protocol using Kryptonet key management   [Christer_Oberg] */
    SKIP_TCPIP          = 57,   /**< SKIP                                                               [Tom_Markson] */
    IPV6_ICMP_TCPIP     = 58,   /**< ICMP for IPv6                                                      [RFC2460] */
    IPV6_NoNxt_TCPIP    = 59,   /**< No Next Header for IPv6                                            [RFC2460] */
    IPV6_Opts_TCPIP     = 60,   /**< Destination Options for IPv6                                       [RFC2460] */
    CFTP_TCPIP          = 62,   /**< CFTP                                                               [Forsdick, H., "CFTP", Network Message, Bolt Beranek and Newman, January 1982.][Harry_Forsdick] */
    SAT_EXPAK_TCPIP     = 64,   /**< SATNET and Backroom EXPAK                                          [Steven_Blumenthal] */
    KRYPTOLAN_TCPIP     = 65,   /**< Kryptolan                                                          [Paul Liu] */
    RVD_TCPIP           = 66,   /**< MIT Remote Virtual Disk Protocol                                   [Michael_Greenwald] */
    IPPC_TCPIP          = 67,   /**< Internet Pluribus Packet Core                                      [Steven_Blumenthal] */
    SAT_MON_TCPIP       = 69,   /**< SATNET Monitoring                                                  [Steven_Blumenthal] */
    VISA_TCPIP          = 70,   /**< VISA Protocol                                                      [Gene_Tsudik] */
    IPCV_TCPIP          = 71,   /**< Internet Packet Core Utility                                       [Steven_Blumenthal] */
    CPNX_TCPIP          = 72,   /**< Computer Protocol Network Executive                                [David Mittnacht] */
    CPHB_TCPIP          = 73,   /**< Computer Protocol Heart Beat                                       [David Mittnacht] */
    WSN_TCPIP           = 74,   /**< Wang Span Network                                                  [Victor Dafoulas] */
    PVP_TCPIP           = 75,   /**< Packet Video Protocol                                              [Steve_Casner] */
    BR_SAT_MON_TCPIP    = 76,   /**< Backroom SATNET Monitoring                                         [Steven_Blumenthal] */
    SUN_ND_TCPIP        = 77,   /**< SUN ND PROTOCOL-Temporary                                          [William_Melohn] */
    WB_MON_TCPIP        = 78,   /**< WIDEBAND Monitoring                                                [Steven_Blumenthal] */
    WB_EXPAK_TCPIP      = 79,   /**< WIDEBAND EXPAK                                                     [Steven_Blumenthal] */
    ISO_IP_TCPIP        = 80,   /**< ISO Internet Protocol                                              [Marshall_T_Rose] */
    VMTP_TCPIP          = 81,   /**< VMTP                                                               [Dave_Cheriton] */
    SECURE_VMTP_TCPIP   = 82,   /**< SECURE-VMTP                                                        [Dave_Cheriton] */
    VINES_TCPIP         = 83,   /**< VINES                                                              [Brian Horn] */
    TTP_TCPIP           = 84,   /**< TTP                                                                [Jim_Stevens] */
    IPTM_TCPIP          = 84,   /**< Protocol Internet Protocol Traffic Manager                         [Jim_Stevens] */
    NSFNET_IGP_TCPIP    = 85,   /**< NSFNET-IGP                                                         [Hans_Werner_Braun] */
    DGP_TCPIP           = 86,   /**< Dissimilar Gateway Protocol                                        [M/A-COM Government Systems, "Dissimilar Gateway Protocol Specification, Draft Version", Contract no. CS901145, November 16, 1987.][Mike_Little] */
    TCF_TCPIP           = 87,   /**< TCF                                                                [Guillermo_A_Loyola] */
    EIGRP_TCPIP         = 88,   /**< EIGRP                                                              [Cisco Systems, "Gateway Server Reference Manual", Manual Revision B, January 10, 1988.][Guenther_Schreiner] */
    OSPFIGP_TCPIP       = 89,   /**< OSPFIGP                                                            [RFC1583][RFC2328][RFC5340][John_Moy] */
    Sprite_RPC_TCPIP    = 90,   /**< Sprite RPC Protocol                                                [Welch, B., "The Sprite Remote Procedure Call System", Technical Report, UCB/Computer Science Dept., 86/302, University of California at Berkeley, June 1986.][Bruce Willins] */
    LARP_TCPIP          = 91,   /**< Locus Address Resolution Protocol                                  [Brian Horn] */
    MTP_TCPIP           = 92,   /**< Multicast Transport Protocol                                       [Susie_Armstrong] */
    AX25_TCPIP          = 93,   /**< AX.25 Frames                                                       [Brian_Kantor] */
    IPIP_TCPIP          = 94,   /**< IP-within-IP Encapsulation Protocol                                [John_Ioannidis] */
    MICP_TCPIP          = 95,   /**< Mobile Internetworking Control Pro.                                [John_Ioannidis] */
    SCC_SP_TCPIP        = 96,   /**< Semaphore Communications Sec. Pro.                                 [Howard_Hart] */
    ETHERIP_TCPIP       = 97,   /**< Ethernet-within-IP Encapsulation                                   [RFC3378] */
    ENCAP_TCPIP         = 98,   /**< Encapsulation Header                                               [RFC1241][Robert_Woodburn] */
    GMTP_TCPIP          = 100,  /**< GMTP                                                               [[RXB5]] */
    IFMP_TCPIP          = 101,  /**< Ipsilon Flow Management Protocol                                   [Bob_Hinden][November 1995, 1997.] */
    PNNI_TCPIP          = 102,  /**< PNNI over IP                                                       [Ross_Callon] */
    PIM_TCPIP           = 103,  /**< Protocol Independent Multicast                                     [RFC4601][Dino_Farinacci] */
    ARIS_TCPIP          = 104,  /**< ARIS                                                               [Nancy_Feldman] */
    SCPS_TCPIP          = 105,  /**< SCPS                                                               [Robert_Durst] */
    QNX_TCPIP           = 106,  /**< QNX                                                                [Michael_Hunter] */
    A_N_TCPIP           = 107,  /**< Active Networks                                                    [Bob_Braden] */
    IPComp_TCPIP        = 108,  /**< IP Payload Compression Protocol                                    [RFC2393] */
    SNP_TCPIP           = 109,  /**< Sitara Networks Protocol                                           [Manickam_R_Sridhar] */
    Compaq_Peer_TCPIP   = 110,  /**< Compaq Peer Protocol                                               [Victor_Volpe] */
    IPX_in_IP_TCPIP     = 111,  /**< IPX in IP                                                          [CJ_Lee] */
    VRRP_TCPIP          = 112,  /**< Virtual Router Redundancy Protocol                                 [RFC5798] */
    PGM_TCPIP           = 113,  /**< PGM Reliable Transport Protocol                                    [Tony_Speakman] */
    L2TP_TCPIP          = 115,  /**< Layer Two Tunneling Protocol                                       [RFC3931][Bernard_Aboba] */
    DDX_TCPIP           = 116,  /**< D-II Data Exchange (DDX)                                           [John_Worley] */
    IATP_TCPIP          = 117,  /**< Interactive Agent Transfer Protocol                                [John_Murphy] */
    STP_TCPIP           = 118,  /**< Schedule Transfer Protocol                                         [Jean_Michel_Pittet] */
    SRP_TCPIP           = 119,  /**< SpectraLink Radio Protocol                                         [Mark_Hamilton] */
    UTI_TCPIP           = 120,  /**< UTI                                                                [Peter_Lothberg] */
    SMP_TCPIP           = 121,  /**< Simple Message Protocol                                            [Leif_Ekblad] */
    SM_TCPIP            = 122,  /**< SM                                                                 [Jon_Crowcroft] */
    PTP_TCPIP           = 123,  /**< Performance Transparency Protocol                                  [Michael_Welzl] */
    ISIS_TCPIP          = 124,  /**< Over IPv4                                                          [Tony_Przygienda] */
    FIRE_TCPIP          = 125,  /**<                                                                    [Criag_Partridge] */
    CRTP_TCPIP          = 126,  /**< Combat Radio Transport Protocol                                    [Robert_Sautter] */
    CRUDP_TCPIP         = 127,  /**< Combat Radio User Datagram                                         [Robert_Sautter] */
    SSCOPMCE_TCPIP      = 128,  /**<                                                                    [Kurt_Waber] */
    IPLT_TCPIP          = 129,  /**<                                                                    [[Hollbach]] */
    SPS_TCPIP           = 130,  /**< Secure Packet Shield                                               [Bill_McIntosh] */
    PIPE_TCPIP          = 131,  /**< Private IP Encapsulation within IP                                 [Bernhard_Petri] */
    SCTP_TCPIP          = 132,  /**< Stream Control Transmission Protocol                               [Randall_R_Stewart] */
    FC_TCPIP            = 133   /**< Fibre Channel                                                      [Murali_Rajagopal][RFC6172] */
} ipProtocolNumbers;

/**
 * @ingroup tcpiplite
 * @struct inAddr_t
 * @brief INET address
 */
typedef struct
{
    union
    {
        uint32_t s_addr;
        uint8_t s_addr_byte[4];
    };
} inAddr_t;


/**
 * @ingroup tcpiplite
 * @struct sockaddr_in4_t
 * @brief INET4 socket address
 */
typedef struct
{
    uint16_t port;
    inAddr_t addr;
} sockaddr_in4_t;


extern const char *network_errors[];

/**
 * @ingroup tcpiplite
 * @enum error_msg
 * @brief Error message
 */
typedef enum
{
    ERROR = 0,
    SUCCESS,
    LINK_NOT_FOUND,
    BUFFER_BUSY,
    TX_LOGIC_NOT_IDLE,
    TX_QUEUED,
    DMA_TIMEOUT,
    MAC_NOT_FOUND,
    IP_WRONG_VERSION,
    IPV4_CHECKSUM_FAILS,
    DEST_IP_NOT_MATCHED,
    ICMP_CHECKSUM_FAILS,
    UDP_CHECKSUM_FAILS,
    TCP_CHECKSUM_FAILS,
    PORT_NOT_AVAILABLE,
    ARP_IP_NOT_MATCHED,
    EAPoL_PACKET_FAILURE,
    INCORRECT_IPV4_HLEN,
    IPV4_NO_OPTIONS,
    IPV6_CHECKSUM_FAILS,
    IPV6_LOCAL_ADDR_RESOLVE,
    IPV6_LOCAL_ADDR_INVALID,
    NO_GATEWAY,
    ADDRESS_RESOLUTION,
    GLOBAL_DESTINATION,
    ARP_WRONG_HARDWARE_ADDR_TYPE,
    ARP_WRONG_PROTOCOL_TYPE,
    ARP_WRONG_HARDWARE_ADDR_LEN,
    ARP_WRONG_PROTOCOL_LEN
} error_msg;

/**
 * @ingroup tcpiplite
 * @enum destIP_t
 * @brief Destination IP Address
 */
typedef struct
{
    inAddr_t dest_addr;
} destIP_t;

/**
 * @ingroup tcpiplite
 * @typedef int8_t
 * @brief Counts up to 256 sockets numbers.
 */
typedef int8_t socklistsize_t;

/**
 * @ingroup tcpiplite
 * @typedef ip_receive_function_ptr
 * @brief Function pointer to the function that receives the payload
 * @param int16_t Available bytes for the payload
 * @return None.
 */
typedef void (*ip_receive_function_ptr)(int16_t);

#endif /* TCPIP_TYPES_H */
