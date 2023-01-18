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

#include "system_definitions.h"

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

extern GFX_COLOR frameBuffer[1][DISP_HOR_RESOLUTION][DISP_VER_RESOLUTION];

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

static void _delayMS ( unsigned int delayMs );

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
    appData.state = APP_STATE_INIT;
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    uint8_t reg12;
    
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            appData.drvHandle = DRV_CAMERA_OVM7690_Open( DRV_CAMERA_OVM7690_INDEX_0,
                                                         DRV_IO_INTENT_BLOCKING );
            if( appData.drvHandle == DRV_HANDLE_INVALID )
            {
                return;
            }
            
            appData.state = APP_STATE_CAMERA_INIT;
                
            break;
        }
        
        case APP_STATE_CAMERA_INIT:
        {   
            /* Set Frame buffer address */
            DRV_CAMERA_OVM7690_FrameBufferAddressSet(appData.drvHandle,
                                                     (void *)frameBuffer);
            
            /*Software Reset*/
            reg12 = DRV_CAMERA_OVM7690_REG12_SOFT_RESET;
            while(DRV_CAMERA_OVM7690_RegisterSet( DRV_CAMERA_OVM7690_REG12_REG_ADDR, 
                                                  reg12 ) == false);
            _delayMS(100);

            /*Horizontal Mirror Enable HSYNC/VSYCNC/PCLK and Color*/
            while(DRV_CAMERA_OVM7690_RegisterSet( DRV_CAMERA_OVM7690_REG0C_REG_ADDR, 
                                                  0x06) == false);
            _delayMS(100);
            
            /* HREF Reverse */
            while(DRV_CAMERA_OVM7690_RegisterSet( DRV_CAMERA_OVM7690_REG28_REG_ADDR, 
                                                  0x10) == false);
            _delayMS(100);
            
            /* PCLK output gated */
            while(DRV_CAMERA_OVM7690_RegisterSet( DRV_CAMERA_OVM7690_REG3E_REG_ADDR, 
                                                  0x70 ) == false);
            _delayMS(100);

            /*Slow down Pixel Clock*/
            while(DRV_CAMERA_OVM7690_RegisterSet( DRV_CAMERA_OVM7690_CLKRC_REG_ADDR, 
                                                  0x01) == false);
            _delayMS(100);

            /*SubSample and RGB color*/
            while(DRV_CAMERA_OVM7690_RegisterSet( DRV_CAMERA_OVM7690_REG12_REG_ADDR, 
                                                  0x16) == false);
            _delayMS(100);

            DRV_CAMERA_OVM7690_FrameRectSet( appData.drvHandle,
                                             0x69, 0x0E,
                                             DISP_HOR_RESOLUTION + 0x69,
                                             DISP_VER_RESOLUTION + 0x0E);
            
            appData.state = APP_STATE_CAMERA_START;
            
            break;
        }
        
        case APP_STATE_CAMERA_START:
        {
            DRV_CAMERA_OVM7690_Start(appData.drvHandle);
            appData.state = APP_STATE_DONE;
        
            break;
        }

        /* Application Done */
        case APP_STATE_DONE:
        {
            break;
        }
    }
}
 
static void _delayMS ( unsigned int delayMs )
{
    if(delayMs)
    {
        uint32_t sysClk = SYS_CLK_FREQ;
        uint32_t t0;
        t0 = _CP0_GET_COUNT();
        while (_CP0_GET_COUNT() - t0 < (sysClk/2000)*delayMs);
    }
}

/*******************************************************************************
 End of File
 */

