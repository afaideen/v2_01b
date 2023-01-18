/* 
 * File:   xmodem.h
 * Author: m91145
 *
 * Created on 22 December 2014, 11:19
 */

#ifndef XMODEM_H
#define	XMODEM_H

#ifdef	__cplusplus
extern "C" {
#endif

#define XMODEM_MAX_OBJ_INSTANCES 1

#define XMODEM_INDEX_0  0

/* xmodem transfer symbols */
#define XMODEM_SOH  0x01
#define XMODEM_STX  0x02
#define XMODEM_EOT  0x04
#define XMODEM_ACK  0x06
#define XMODEM_NAK  0x15
#define XMODEM_CAN  0x18


/* max size of the usb input data */
#define XMODEM_READ_BUFFER_SIZE 512
/* max size of the received assembly buffer */
#define XMODEM_ASSEMBLY_BUFFER_SIZE 1060

/* Return values from the XMODEM state machine
 * The return values indicate what additional processing is required by the calling function
 * This includes aborting the sequence, doing nothing or processing a received packet of data
 */
typedef enum {
    XMODEM_RETURN_ABORT = -2,   /* abort xmodem process and discard data */
    XMODEM_RETURN_ERROR = -1,   /* general error (chosen to match SYS_STATUS_ERROR) */
    XMODEM_RETURN_IDLE = 0,     /* xmodem state idle, no processing to do */
    XMODEM_RETURN_PROCESS,      /* return with a block of data to process */
    XMODEM_RETURN_COMPLETE      /* xmodem processing complete */
} XMODEM_RETURN;

/* XMODEM state */
typedef enum {
    XMODEM_STATE_IDLE = 0,
    XMODEM_STATE_INIT,
    XMODEM_STATE_SEND_INVITE,
    XMODEM_STATE_WAIT_SOH,
    XMODEM_STATE_READ_DATA,
    XMODEM_STATE_CRC_CHECK,
    XMODEM_STATE_SEND_NAK,
    XMODEM_STATE_SEND_ACK,
    XMODEM_STATE_ABORT
} APP_XMODEM_STATE;

/**/
typedef struct
{
    SYS_STATUS  xmodemStatus;

    /* xmodem receive mode */
    APP_XMODEM_STATE xmState;
    uint8_t xmBuffer[XMODEM_READ_BUFFER_SIZE];  /* the buffer of xmodem data, a ringbuffer */
    uint32_t xmTailIndex;                       /* index of the tail or read element */
    uint32_t xmHeadIndex;                       /* index of the head or write element */
    SYS_TMR_HANDLE xmTimer;                     /* timer used during xmodem transfers */
    uint32_t xmAbortcount;                      /* number of CAN chars received */
    uint32_t xmTimeoutCounter;                  /* timeout counter */
    uint8_t xmSOHchar;                          /* SOH char */
    uint32_t xmRepeatCounter;                   /* counter for repeats of chars */
    uint32_t xmPacketSize;                      /* expected packet size */

    /* temporary storage for the command/console handlers */
    USB_DEVICE_CDC_EVENT_HANDLER defaultEventHandler;
    uintptr_t defaultUserData;

    /* Read transfer handle */
    USB_DEVICE_CDC_TRANSFER_HANDLE readTransferHandle;

    /* Write transfer handle */
    USB_DEVICE_CDC_TRANSFER_HANDLE writeTransferHandle;

    /* True if a character was read */
    bool isReadComplete;

    /* True if a character was written*/
    bool isWriteComplete;

    size_t rxLength;                            /* length of just received packet */
    
    /* buffer to hold a packet of data as it is being assembled */
    uint8_t xmAssembledData[XMODEM_ASSEMBLY_BUFFER_SIZE];
    uint32_t xmAssembledIndex;
} XMODEM_OBJECT_INSTANCE;


/* XMODEM state machine */
/* count, pointer to value used to store size of return data
 * pData, pointer to xmodem data array containing received xmodem data
 * return, xmodem return code as above 
 */
XMODEM_RETURN DRV_XMODEM_Process(SYS_MODULE_OBJ object, uint32_t* count, uint8_t** pData);
SYS_MODULE_OBJ DRV_XMODEM_Initialize(const SYS_MODULE_INDEX index, const SYS_MODULE_INIT * const init);

void DRV_XMODEM_Start(SYS_MODULE_OBJ object);
void DRV_XMODEM_Finish(SYS_MODULE_OBJ object);
SYS_STATUS DRV_XMODEM_Status(SYS_MODULE_OBJ object);
void DRV_XMODEM_READ_CALLBACK(void* handle);

#ifdef	__cplusplus
}
#endif

#endif	/* XMODEM_H */

