/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
USB_DATA  usbData;
FLH_DATA  flashData;

volatile uint32_t tick = 0; // tick counter
volatile uint32_t tick_timer = 0; // tick counter
volatile uint32_t activity = 0; // tick counter
bool     coreupdate    = false;
void USB_MSD_Initialize (void);

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.State = APP_STATE_INIT;
    
    flashData.State = FLH_STATE_INIT;
    flashData.LastError = 0;

    usbData.State = USB_STATE_WFIX;
    usbData.LastError = 0;
 
    strcpy(usbData.mntPath, SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0);  
    strcpy(usbData.externalResourceFile, BOOTLOADER_IMAGE_FILE_NAME);
    strcat(usbData.mntPath, "/");
    strcat(usbData.mntPath, usbData.externalResourceFile);
    strcpy(flashData.flashSrc, usbData.mntPath);
    
    /****************************************************************/

    USB_MSD_Initialize();
//    USB_CDC_Initialize();

    /****************************************************************/
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appData.State )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
            appData.State = APP_STATE_USB_TIMER_SET;
            break;

        case APP_STATE_USB_TIMER_SET:         
//                GFX_GOL_MessageCallbackSet(GFX_INDEX_0, &GFX_GOL_MessageCallback);
//                GFX_GOL_DrawCallbackSet(GFX_INDEX_0, &GFX_GOL_DrawCallback);
//                appData.StartTime = tick;
                usbData.TimeFix =tick_timer;
                appData.State =  APP_STATE_DONE;
                break;
        case APP_STATE_DONE:   
                break;

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
    
    APP_FLASH_Task();

    APP_USB_Task();
    
   // Blink the LED
    if ((_CP0_GET_COUNT() & 0x2000000) != 0)
    {
        BSP_LEDStateSet(BSP_LED_1,0);
    }
    else
    {
       BSP_LEDStateSet(BSP_LED_1,1);
    }
}
 



/*******************************************************************************
 End of File
 */
