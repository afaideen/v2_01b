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

#ifndef _APPUSB_H
#define _APPUSB_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <app.h>
#include "driver/usb/usbhs/drv_usbhs.h"

#include "usb/usb_host_hub.h"
#include "usb/usb_host_hub_interface.h"
#include "usb/usb_host_msd.h"
#include "usb/usb_host_scsi.h"


// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
/*******************************************************
 * Application definitions.
 *******************************************************/

#define APP_HOST_CDC_BAUDRATE_SUPPORTED 9600UL
#define APP_HOST_CDC_PARITY_TYPE        0
#define APP_HOST_CDC_STOP_BITS          0
#define APP_HOST_CDC_NO_OF_DATA_BITS    8



//USB_HOST_CDC_EVENT_RESPONSE APP_USBHostCDCEventHandler
//(
//    USB_HOST_CDC_HANDLE cdcHandle,
//    USB_HOST_CDC_EVENT event,
//    void * eventData,
//    uintptr_t context
//);




extern USB_DATA   usbData;
extern FLH_DATA   flashData;
extern bool       diseble_operation;
//extern uint16_t SYSTEM_START_INDEX;
extern uint8_t    boot_sequence;
extern volatile uint32_t tick_timer; // tick counter

//void GoBootStage(void);
//void GoResStage(void);
void APP_TMR_DelayMS ( unsigned int delayMs );
//void ScreenForward(uint16_t newScreen);
//void ScreenBackward(void);

#endif /* _APPUSB_H */
/*******************************************************************************
 End of File
 */

