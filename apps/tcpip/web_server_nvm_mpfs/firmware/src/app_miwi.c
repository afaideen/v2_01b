/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_miwi.c

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

#include "app_miwi.h"
#include "GenericTypeDefs.h"
#include "mla_v13_june_2013/LCD_ST7032.h"

void putsUART2(unsigned int *buffer);
void DelayMs(uint16_t ms);
BOOL MiApp_ProtocolInit(BOOL bNetworkFreezer);

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
BYTE LCDText[16*2+1];

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

APP_MIWI_DATA app_miwiData;

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
    void APP_MIWI_Initialize ( void )

  Remarks:
    See prototype in app_miwi.h.
 */

void APP_MIWI_Initialize ( void )
{
        /* Place the App state machine in its initial state. */
        app_miwiData.state = APP_MIWI_STATE_INIT;
        LEDS_OFF();

        ConfigureLCD_SPI();
        LCDInit();
        /*******************************************************************/
        // Display Start-up Splash Screen
        /*******************************************************************/
        LCDBacklightON();
        LED2_ON();
        LCDErase();
        sprintf((char *) LCDText, (char*) "  MiWi - WiFi  ");
        sprintf((char *) &(LCDText[16]), (char*) " Gateway  Demo");
        LCDUpdate();
        
        DelayMs(1500);
        LCDErase();
        strcpy((char *) LCDText, DRV_WIFI_DEFAULT_SSID);
        strcpy((char *) LCDText+16, TCPIP_NETWORK_DEFAULT_IP_ADDRESS);
        LCDUpdate();
        
        DelayMs(3000);
        
        LCDBacklightOFF();
//        while(1)
//        {
//                LED1_INV();
////                DelayMs(4000);
//                BSP_DelayMs(4000);
//        }
        MiApp_ProtocolInit(FALSE);
        putsUART( (unsigned int*) "Running MiWi...\r\n" );
}


/******************************************************************************
  Function:
    void APP_MIWI_Tasks ( void )

  Remarks:
    See prototype in app_miwi.h.
 */

void APP_MIWI_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_miwiData.state )
    {
        /* Application's initial state. */
        case APP_MIWI_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                app_miwiData.state = APP_MIWI_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_MIWI_STATE_SERVICE_TASKS:
        {
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
