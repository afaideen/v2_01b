/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include "system_config.h"
#include "sqi.h"
#include "system_definitions.h"
#include "peripheral/sqi/plib_sqi.h"
#include "xmodem.h"

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

/* Size of screen in memory */
#define APP_SCREEN_MEMORY_SIZE  (480 * 272 * 2)


// *****************************************************************************

/* Application States

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
 */

typedef enum {
    /* Application's state machine's initial state. */
    APP_STATE_FLASH_INIT = 0,

    /* read the FLASH ID to verify it is working */
    APP_STATE_FLASH_ID_READ,

    /* restart display sequence */
    APP_STATE_INIT,

    /* write test data to the SQI FLASH */
    APP_STATE_FLASH_PAGE_WRITE,

    /* display screen */
    APP_STATE_DISPLAY_IDLE,

    /* wait until screen updated */
    APP_STATE_DISPLAY_AUTO,

    /* write serial data */
    APP_STATE_DISPLAY_WAIT,

    /* pause before next screen */
    APP_STATE_DISPLAY_PAUSE,

    /* downloading image data */
    APP_STATE_IMAGE_UPDATE,

    /* erase the flash */
    APP_STATE_FLASH_ERASE,
    APP_STATE_FLASH_ERASE_COMPLETE_MSG,
    APP_STATE_FLASH_ERASE_COMPLETE,

    /* program the flash with a test pattern */
    APP_STATE_FLASH_PTEST_START,
    APP_STATE_FLASH_PTEST,
    APP_STATE_FLASH_PTEST_COMPLETE,

    /* copy the display to the FLASH*/
    APP_STATE_FLASH_COPY_START,
    APP_STATE_FLASH_COPY,
    APP_STATE_FLASH_COPY_COMPLETE,

    /* xmodem transfer main app states */
    APP_STATE_XMODEM_INIT,
    APP_STATE_XMODEM_START,
    APP_STATE_XMODEM_WAIT_FOR_SEND,
    APP_STATE_XMODEM_PROCESS,
    APP_STATE_XMODEM_COMPLETE,
    APP_STATE_XMODEM_ABORT,
    APP_STATE_XMODEM_CLEANUP,

    /* sqi data dump states */
    APP_STATE_START_DUMP,
    APP_STATE_DUMP_LINE,
    APP_STATE_DUMP_LINE_COMPLETE,
    APP_STATE_DUMP_COMPLETE,

    /* Application Error state*/
    APP_STATE_ERROR

} APP_STATES;

// *****************************************************************************

/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* Device layer handle returned by device layer open function */
    USB_DEVICE_HANDLE deviceHandle;

    /* Application's current state*/
    APP_STATES state;

    /* flash page programming data */
    uint32_t pageProgramCount;      /* number of pages left to program */
    uint32_t pageProgramAddress;    /* current address in FLASH to program */
    uint16_t pageProgramValue;      /* value to put into memory */
    uint32_t screenCopyOffset;      /* screen offset address to copy from */

    /* flash page display data */
    uint32_t sqiDisplayAddress;
    uint32_t linesToDisplay;
    uint32_t dumpIndex;

    /* variables used in copying to the screen */
    uint8_t* screenDest;
    uint32_t screenLeftToCopy;
    uint32_t screenBytesToIgnore;
    
    /* Timer driver handle */
    DRV_HANDLE handleTmr;

    /* system timer for screen updates */
    SYS_TMR_HANDLE screenUpdateTimer;
    
    SYS_MODULE_OBJ  xmodemObject0;
} APP_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
 */
USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler(USB_DEVICE_CDC_INDEX index,
    USB_DEVICE_CDC_EVENT event, void * pData, uintptr_t userData);

void APP_USBDeviceEventHandler (USB_DEVICE_EVENT event, void * eventData, uintptr_t context);

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
 */

void APP_Initialize(void);


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks(void);

/* process xmodem incoming data */
void APP_USB_XMODEM_Receive(uint32_t len, uint8_t* pData);

#endif /* _APP_H */
/*******************************************************************************
 End of File
 */

