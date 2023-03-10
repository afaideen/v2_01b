/*******************************************************************************
  TCP Module private definitions for the Microchip TCP/IP Stack

  Company:
    Microchip Technology Inc.
    
  File Name:
    tcp_private.h

  Summary:
    
  Description:
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright ? 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef _TCP_PRIVATE_H_
#define _TCP_PRIVATE_H_

/****************************************************************************
  Section:
	Debug Definitions
  ***************************************************************************/
#define TCPIP_TCP_DEBUG_MASK_BASIC          (0x0001)
#define TCPIP_TCP_DEBUG_MASK_TRACE_STATE    (0x0002)

// enable TCP debugging levels
#define TCPIP_TCP_DEBUG_LEVEL  (TCPIP_TCP_DEBUG_MASK_BASIC)



/****************************************************************************
  Section:
	Type Definitions
  ***************************************************************************/


#define TCP_CHECKSUM_OFFSET (16u)


// The minimum value a RX or TX TCP buffer can have
#define TCP_MIN_RX_BUFF_SIZE    (25)
#define TCP_MIN_TX_BUFF_SIZE    (25)

// for efficiency reasons, 
// any request to change a TX/RX buffer size
// that results in a difference less than this limit
// will be ignored
#define TCP_MIN_BUFF_CHANGE     (32)

/****************************************************************************
  Section:
	State Machine Variables
  ***************************************************************************/


typedef struct
{
    IPV4_PACKET             v4Pkt;      // safe cast to IPV4_PACKET
    TCPIP_MAC_DATA_SEGMENT  tcpSeg[2];  // always zero copy data for TCP state machine
}TCP_V4_PACKET;


typedef struct
{
    IPV6_PACKET             v6Pkt;      // safe cast to IPV6_PACKET
    TCPIP_MAC_DATA_SEGMENT  tcp6Seg;     
}TCP_V6_PACKET;


/****************************************************************************
  Section:
	TCB Definitions
  ***************************************************************************/

// TCP Control Block (TCB) stub data storage. 
typedef struct
{
	uint8_t*            txStart;		            // First byte of TX buffer
	uint8_t*            txEnd;			            // Last byte of TX buffer
	uint8_t*            txHead;			            // Head pointer for TX - user write pointer
	uint8_t*            txTail;			            // Tail pointer for TX - socket read pointer
	uint8_t*	        txUnackedTail;	            // TX tail pointer for data that is not yet acked
	uint8_t*            rxStart;		            // First byte of RX buffer.
	uint8_t*            rxEnd;			            // Last byte of RX buffer
	uint8_t*            rxHead;			            // Head pointer for RX - socket write pointer
	uint8_t*            rxTail;			            // Tail pointer for RX - user read pointer
    uint32_t            eventTime;			        // Packet retransmissions, state changes
	uint32_t            eventTime2;		            // Window updates, automatic transmission
    uint32_t            delayedACKTime;             // Delayed Acknowledgment timer
    uint32_t            closeWaitTime;		        // TCP_CLOSE_WAIT, TCP_FIN_WAIT_2, TCP_TIME_WAIT timeout

    TCP_SOCKET   sktIx;                             // socket number
    struct
    {
        uint16_t bServer : 1;					    // Socket should return to listening state when closed
		uint16_t bTimerEnabled	: 1;			    // Timer is enabled
		uint16_t bTimer2Enabled : 1;			    // Second timer is enabled
		uint16_t bDelayedACKTimerEnabled : 1;	    // DelayedACK timer is enabled
		uint16_t bOneSegmentReceived : 1;		    // A segment has been received
		uint16_t bHalfFullFlush : 1;			    // Flush is for being half full
		uint16_t bTXASAP : 1;					    // Transmit as soon as possible (for Flush)
		uint16_t bTXASAPWithoutTimerReset : 1;	    // Transmit as soon as possible (for Flush), but do not reset retransmission timers
		uint16_t bTXFIN : 1;					    // FIN needs to be transmitted
		uint16_t bSocketReset : 1;				    // Socket has been reset (self-clearing semaphore)
		uint16_t bInitialized : 1;				    // Future expansion
		uint16_t failedDisconnect : 1;			    // Failed to send a FIN
		uint16_t halfThresType : 2; 		        // a TCP_OPTION_THRES_FLUSH_TYPE value
        uint16_t keepAlive : 1;  		            // keep alive option enabled
		uint16_t delayAckSend : 1;                  // delay sending all acknowledged data
    } Flags;

    IPV4_ADDR           destAddress;                // socket destination address
    IPV4_ADDR           srcAddress;                 // socket source address
    TCPIP_NET_IF*       pSktNet;                    // which interface this socket is bound to
    union
    {
        IPV4_PACKET*  pV4Pkt;                       // IPv4 use; TCP_V4_PACKET type
        IPV6_PACKET*  pV6Pkt;                       // IPv6 use; TCP_V6_PACKET type
        void*         pTxPkt;                       // generic
    };
    // 
	uint32_t            retryInterval;			    // How long to wait before retrying transmission
	uint32_t		    MySEQ;					    // Local sequence number
	uint32_t		    RemoteSEQ;				    // Remote sequence number
    TCP_PORT            remotePort;			    	// Remote port number
    TCP_PORT        	localPort;				    // Local port number
	uint16_t		    remoteWindow;			    // Remote window size
	uint16_t		    localWindow;			    // last advertised window size
	uint16_t		    wFutureDataSize;		    // How much out-of-order data has been received
	int16_t		        sHoleSize;				    // Size of the hole, or -1 for none exists.  (0 indicates hole has just been filled)
	uint16_t		    wRemoteMSS;				    // Maximum Segment Size option advertised by the remote node during initial handshaking
	uint16_t		    localMSS;				    // our advertised MSS
	uint16_t		    maxRemoteWindow;	        // max advertised remote window size
    uint16_t            keepAliveTmo;               // timeout, ms
    uint16_t            remoteHash;	                // Consists of remoteIP, remotePort, localPort for connected sockets.
    struct
    {
		uint16_t openAddType    : 2;		        // the address type used at open
        uint16_t bFINSent       : 1;		        // A FIN has been sent
		uint16_t bSYNSent       : 1;		        // A SYN has been sent
		uint16_t bRXNoneACKed1  : 1;	            // A duplicate ACK was likely received
		uint16_t bRXNoneACKed2  : 1;	            // A second duplicate ACK was likely received
		uint16_t nonLinger      : 1; 		        // linger option
		uint16_t nonGraceful    : 1; 		        // graceful close
        uint16_t ackSent        : 1;                // acknowledge sent in this pass
        uint16_t seqInc         : 1;                // sequence number incremented after FIN ack 
        uint16_t forceKill      : 1;                // socket should be killed, even if it's server socket 
        uint16_t forceFlush     : 1;                // flush data any time caller writes to the socket
                                                    // i.e., disable Nagle
        uint16_t srcSet         : 1;                // source address is set 
        uint16_t openBindIf     : 1;                // socket is bound to interface when opened 
        uint16_t openBindAdd    : 1;                // socket is bound to address when opened 
        uint16_t halfThresFlush : 1;                // when set, socket will flush at half TX buffer threshold
    } flags;
    uint8_t             smState;			        // TCPIP_TCP_STATE: State of this socket
    uint8_t             addType;                    // IPV4/6 socket type; IP_ADDRESS_TYPE enum type
	uint8_t		        retryCount;				    // Counter for transmission retries
    uint8_t             keepAliveCount;             // current counter
    TCPIP_TCP_SIGNAL_FUNCTION sigHandler;              // socket signal handler
    uint16_t            sigMask;                     // TCPIP_TCP_SIGNAL_TYPE: mask of active events
    uint8_t             keepAliveLim;               // current limit
    union
    {
        uint8_t         val;
        struct
        {
            uint8_t tracePrevState  : 4;            // socket previous traced state
            uint8_t traceStateFlag  : 1;            // socket state is traced
            uint8_t reserved        : 3;            // padding; not used
        };
    }dbgFlags;
} TCB_STUB;

#endif  // _TCP_PRIVATE_H_
