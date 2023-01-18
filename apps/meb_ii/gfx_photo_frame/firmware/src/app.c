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
#include "driver/driver_common.h"
#include "driver/tmr/drv_tmr.h"
#include "system/ports/sys_ports.h"
#include "system/debug/sys_debug.h"
#include <ctype.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

extern volatile GFX_COLOR frameBuffer[1][(DISP_VER_RESOLUTION)][(DISP_HOR_RESOLUTION)];
static uint8_t __attribute__((coherent, aligned(4))) pageCopy[256];
static uint8_t dumpLineBuffer[100];

/******************************************************************************/
/* Application command handlers */
static int _APP_Command_erase(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _APP_Command_ptest(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _APP_Command_pcopy(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _APP_Command_dump(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _APP_Command_download(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _APP_Command_display(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);

static const SYS_CMD_DESCRIPTOR appCmdTbl[] =
{
    {"erase",   _APP_Command_erase,     ": Erase the entire FLASH"},
    {"ptest",   _APP_Command_ptest,     ": Program a test pattern into the flash, ptest 0xADDRESS [ VALUE PAGECOUNT ]"},
    {"pcopy",   _APP_Command_pcopy,     ": Copy the display to the FLASH, pcopy ADDRESS"},
    {"dump",    _APP_Command_dump,      ": Dump a single page of memory"},
    {"download", _APP_Command_download, ": Download a new image to the display using XMODEM, download [ BYTES TO IGNORE ] (108 default)"},
    {"display", _APP_Command_display,   ": Display a page stored at address, display ADDRESS"}
};


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

void APP_Initialize(void)
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;

    /* configure memory region in flash to display */
    appData.sqiDisplayAddress = 0x00000000;

    /****************************************************************/
    /* configure the sqi */
    /* Set up SQI Configuration (SQI1CFG) Register */
    PLIB_SQI_ConfigWordSet(SQI_ID_0,
                           1,
                           SQI_CS_OEN_1,
                           SQI_DATA_OEN_QUAD,
                           1, // Resets control, transmit, receive buffers and state machines
                           1, // Burst Enable (always set to '1')
                           0, // SQID2 doesn?t act as HOLD# signal in single and dual lane modes
                           0, // SQID3 doesn?t act as WP# signal in single and dual lane modes
                           0, // Receive latch is not active in transmit mode
                           SQI_DATA_FORMAT_MSBF,
                           SQI_DATA_MODE_3,
                           SQI_XFER_MODE_PIO
                           );

    /* configure SQI1clkcon */
    PLIB_SQI_ClockDividerSet(SQI_ID_0, CLK_DIV_2);
    PLIB_SQI_ClockEnable(SQI_ID_0);
    while (!PLIB_SQI_ClockIsStable(SQI_ID_0));

    /* clear the interrupt signal enable bits, Errata fix */
//    SQI1INTSEN = 0x00000000;

    appData.state = APP_STATE_FLASH_INIT;

    /****************************************************************/
    /* add the commands to the command processor */
    SYS_CMD_ADDGRP(appCmdTbl, sizeof(appCmdTbl) / sizeof(SYS_CMD_DESCRIPTOR), "app", ": application commands");

    /* prepare the xmodem driver */
    appData.xmodemObject0 = DRV_XMODEM_Initialize(XMODEM_INDEX_0, NULL);
    
}

/****************************************************************************/
/* Application's Timer Callback Function */

/* This is where we run the model        */
//static void APP_TimerCallback(uintptr_t context, uint32_t alarmCount)
//{
//    /* turn on the LED to signal start of image transfer */
//    BSP_LEDOn(BSP_LED_1);
//}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

uint8_t __attribute__((coherent)) sqiBuffer[4096];

void APP_Tasks(void)
{
    SQI_STATUS sqiStatus;
    uint32_t i;
    uint8_t tempPrintBuffer[16];
    uint8_t* displayAddress;
    uint32_t screenByteCount;
    uint16_t *destPixel;
    XMODEM_RETURN xmReturn;
    uint32_t xmDataSize;
    uint8_t* xmData;

    /* Check the application's current state. */
    switch (appData.state) {
        case APP_STATE_FLASH_INIT:
            /* init the SQI */
            SQI_Flash_Setup();
            appData.state = APP_STATE_FLASH_ID_READ;
            break;

        case APP_STATE_FLASH_ID_READ:
            sqiStatus = SQI_FlashID_Read();
            if (sqiStatus == SQI_STATUS_SUCCESS)
            {
                appData.state = APP_STATE_INIT;
            }

            break;

        case APP_STATE_INIT:
            SYS_CONSOLE_MESSAGE("Application started.\r\n");
            GFX_ColorSet(0, BLACK);
            GFX_ScreenClear(0);

            /* Initialize the operation */
            appData.state = APP_STATE_DISPLAY_IDLE;
            break;

        case APP_STATE_DISPLAY_IDLE:
            /* do nothing state */
            SYS_CMD_READY_TO_READ();
                    
            if (PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_S1) == BSP_SWITCH_STATE_PRESSED)
            {
                /* start automatic cycling of images */
                appData.sqiDisplayAddress = 0x00000000;
                appData.state = APP_STATE_DISPLAY_AUTO;
            }
            break;

        case APP_STATE_DISPLAY_AUTO:
            /* display new image on the screen */
            SYS_CMD_READY_TO_READ();
            
            screenByteCount = APP_SCREEN_MEMORY_SIZE;
            displayAddress = (uint8_t*) & frameBuffer[0][0][0];
            /* copy as much as possible each time */
            while (screenByteCount >= 65536)
            {
                SQI_DMA_PageRead(appData.sqiDisplayAddress, 65536, displayAddress);
                appData.sqiDisplayAddress += 65536;
                displayAddress += 65536;
                screenByteCount -= 65536;
            }
            /* copy the remainder */
            SQI_DMA_PageRead(appData.sqiDisplayAddress, 64512, displayAddress);
            appData.sqiDisplayAddress += 64512;

            /* start a timer running */
            appData.screenUpdateTimer = SYS_TMR_DelayMS(5000);
            appData.state = APP_STATE_DISPLAY_WAIT;
            break;

        case APP_STATE_DISPLAY_WAIT:
            /* pause before displaying a new image */
            SYS_CMD_READY_TO_READ();
            
            if (SYS_TMR_DelayStatusGet(appData.screenUpdateTimer) == true)
            {
                if (appData.sqiDisplayAddress >= (4 * APP_SCREEN_MEMORY_SIZE))
                {
                    /* wraparound to the beginning */
                    appData.sqiDisplayAddress = 0x00000000;
                }
                appData.state = APP_STATE_DISPLAY_AUTO;
            }
            break;

        case APP_STATE_DISPLAY_PAUSE:
            /* pause for a while so that the numbers can be seen */
            break;

        case APP_STATE_IMAGE_UPDATE:
            /* display data from requested address */
            screenByteCount = APP_SCREEN_MEMORY_SIZE;
            displayAddress = (uint8_t*) & frameBuffer[0][0][0];
            /* copy as much as possible each time */
            while (screenByteCount >= 65536)
            {
                SQI_DMA_PageRead(appData.sqiDisplayAddress, 65536, displayAddress);
                appData.sqiDisplayAddress += 65536;
                displayAddress += 65536;
                screenByteCount -= 65536;
            }
            
            /* the remainder is a multiple of 256 so we can copy this in one call */
            /* 480x272x2=261120, 261120-(3x65536)=64512   (64512/256=252)*/
            SQI_DMA_PageRead(appData.sqiDisplayAddress, 64512, displayAddress);

            /* go back to app idle state */
            appData.state = APP_STATE_DISPLAY_IDLE;
            break;

        /****************************************************************************/
        /* perform erase of flash */
        case APP_STATE_FLASH_ERASE:
            SYS_CONSOLE_MESSAGE("Erasing FLASH\r\n");
            SQI_Flash_Erase();
            appData.state = APP_STATE_FLASH_ERASE_COMPLETE_MSG;
            break;

        case APP_STATE_FLASH_ERASE_COMPLETE_MSG:
            SYS_CONSOLE_MESSAGE("Erase complete");
            appData.state = APP_STATE_FLASH_ERASE_COMPLETE;
            break;

        case APP_STATE_FLASH_ERASE_COMPLETE:
            /* restore the command interpreter */
            if (SYS_CMD_READY_TO_READ() != true)
            {
                break;
            }           
            /* for now loop back to the display state */
            appData.state = APP_STATE_DISPLAY_IDLE;
            break;

        /****************************************************************************/
        /* perform program of fixed region of memory with test pattern */
        case APP_STATE_FLASH_PTEST_START:
            //SYS_CONSOLE_PRINT("Programming page: %08X\r\n", appData.pageProgramAddress);

            /* fill the SQI buffer with the data */
            destPixel = (uint16_t*) sqiBuffer;
            for (i = 0; i < 128; i++)
            {
                *destPixel = appData.pageProgramValue;
                destPixel++;
            }
            BSP_LEDOn(BSP_LED_1);
            SQI_PIO_PageWrite(appData.pageProgramAddress, sqiBuffer);
            BSP_LEDOff(BSP_LED_1);
            appData.state = APP_STATE_FLASH_PTEST;
            break;

        case APP_STATE_FLASH_PTEST:
            /* increment to the next address */
            appData.pageProgramCount--;
            if (appData.pageProgramCount > 0)
            {

                appData.pageProgramAddress += 0x100;
                appData.pageProgramAddress &= 0x3FFF00; /* allow wraparound */
                appData.state = APP_STATE_FLASH_PTEST_START;
            }
            else
            {
                SYS_CONSOLE_MESSAGE("Programming test pattern complete\r\n");
                appData.state = APP_STATE_FLASH_PTEST_COMPLETE;
            }
            break;

        case APP_STATE_FLASH_PTEST_COMPLETE:
            /* restore the command interpreter */
            if (SYS_CMD_READY_TO_READ() != true)
            {
                break;
            }       
            /* for now loop back to the display state */
            appData.state = APP_STATE_DISPLAY_IDLE;
            break;

        /****************************************************************************/
        /* program the flash with a copy of the screen */
        case APP_STATE_FLASH_COPY_START:
            //SYS_CONSOLE_PRINT("Programming page: %08X\r\n", appData.pageProgramAddress);
            SYS_CONSOLE_PRINT(".");

            /* calculate the address in the display buffer to copy data from */
            displayAddress = (uint8_t*) &frameBuffer[0][0][0];
            displayAddress += appData.screenCopyOffset;

            /* fill the SQI buffer with the data */
            for (i = 0; i < 256; i++)
            {
                sqiBuffer[i] = *displayAddress;
                displayAddress++;
            }
            BSP_LEDOn(BSP_LED_1);
            SQI_PIO_PageWrite(appData.pageProgramAddress, sqiBuffer);
            BSP_LEDOff(BSP_LED_1);
            appData.state = APP_STATE_FLASH_COPY;
            break;

        case APP_STATE_FLASH_COPY:
            /* increment to the next page */
            appData.screenCopyOffset += 256;
            appData.pageProgramAddress += 256;
            if (appData.screenCopyOffset < (APP_SCREEN_MEMORY_SIZE))
            {
                /* copy the next page */
                appData.state = APP_STATE_FLASH_COPY_START;
            }
            else
            {
                SYS_CONSOLE_MESSAGE("\r\nCopy complete\r\n");
                appData.state = APP_STATE_FLASH_COPY_COMPLETE;
            }

            break;

        case APP_STATE_FLASH_COPY_COMPLETE:
            /* restore the command interpreter */
            if (SYS_CMD_READY_TO_READ() != true)
            {
                break;
            }   
            /* for now loop back to the display state */
            appData.state = APP_STATE_DISPLAY_IDLE;
            break;

        /****************************************************************************/
        /* handle main application states during XMODEM download */
        case APP_STATE_XMODEM_INIT:
            /* finalize printing help messages */
            if (DRV_XMODEM_Status(appData.xmodemObject0) != SYS_STATUS_READY)
            {
                SYS_CONSOLE_MESSAGE("XMODEM driver not ready\r\n");
                appData.state = APP_STATE_DISPLAY_IDLE;
                break;
            }

            SYS_CONSOLE_MESSAGE("Start XMODEM download\r\n");
            appData.state = APP_STATE_XMODEM_START;
            break;

        case APP_STATE_XMODEM_START:
            /* ensure printing has finished */
            if (!SYS_CMD_READY_TO_WRITE())
            {
                break;
            }
            
            /* init the transfer variables */
            appData.screenDest = (uint8_t*) &frameBuffer[0][0][0];
            appData.screenLeftToCopy = DISP_HOR_RESOLUTION * DISP_VER_RESOLUTION * 2;

            /* start the xmodem state machine */
            DRV_XMODEM_Start(appData.xmodemObject0);
            appData.state = APP_STATE_XMODEM_WAIT_FOR_SEND;
            break;

        case APP_STATE_XMODEM_WAIT_FOR_SEND:
            if (PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_S1) == BSP_SWITCH_STATE_PRESSED)
            {
                    appData.state = APP_STATE_XMODEM_PROCESS;
            }
            break;

        case APP_STATE_XMODEM_PROCESS:
            /* call the xmodem state machine */
            xmReturn = DRV_XMODEM_Process(appData.xmodemObject0, &xmDataSize, &xmData);
            
            switch (xmReturn)
            {
                case XMODEM_RETURN_ABORT:
                    appData.state = APP_STATE_XMODEM_ABORT;
                    break;

                case XMODEM_RETURN_IDLE:
                    /* idle state, no further processing required */
                    break;

                case XMODEM_RETURN_PROCESS:
                    /* block of data ready to process */
                    /* copy the data whilst we have data to copy and still in the screen area */
                    while ((appData.screenLeftToCopy > 0) && (xmDataSize > 0))
                    {
                        if (appData.screenBytesToIgnore > 0)
                        {
                            /* discard this many bytes from the beginning */
                            appData.screenBytesToIgnore--;
                        }
                        else
                        {
                            /* copy the data */
                            *appData.screenDest = *xmData;
                            appData.screenDest++;
                            appData.screenLeftToCopy--;
                        }
                        xmData++;
                        xmDataSize--;

                    }
                    break;

                case XMODEM_RETURN_COMPLETE:
                    appData.state = APP_STATE_XMODEM_COMPLETE;
                    break;
                    
                case XMODEM_RETURN_ERROR:
                    break;

            }
            break;

        case APP_STATE_XMODEM_COMPLETE:
            DRV_XMODEM_Finish(appData.xmodemObject0);
            SYS_CONSOLE_MESSAGE("\r\nXMODEM download complete\r\n");
            appData.state = APP_STATE_XMODEM_CLEANUP;
            break;

        case APP_STATE_XMODEM_ABORT:
            DRV_XMODEM_Finish(appData.xmodemObject0);
            SYS_CONSOLE_MESSAGE("\x18\x18\x18\r\nTransfer aborted\r\n");
            /* return to idle state */
            appData.state = APP_STATE_XMODEM_CLEANUP;
            break;

        case APP_STATE_XMODEM_CLEANUP:
            /* tidy up after transfer completion */
            if (!SYS_CMD_READY_TO_READ())
            {
                break;
            }
            appData.state = APP_STATE_DISPLAY_IDLE;
            break;

        /****************************************************************************/
        /* handle main application states during sqi memory dump */
        case APP_STATE_START_DUMP:
            SYS_CONSOLE_MESSAGE(" Address   00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F    0123456789ABCDEF\r\n");
            SYS_CONSOLE_MESSAGE("--------   -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --    ----------------\r\n");

            /* read the page of flash data */
            SQI_DMA_PageRead(appData.sqiDisplayAddress, 256, pageCopy);
            appData.dumpIndex = 0;
            appData.state = APP_STATE_DUMP_LINE;
            break;

        case APP_STATE_DUMP_LINE:
            sprintf((char*)dumpLineBuffer, "%08X   ", appData.sqiDisplayAddress);
            for(i = 0; i < 16; i++)
            {
                sprintf((char*)tempPrintBuffer, "%02X ", pageCopy[appData.dumpIndex]);
                strcat((char*)dumpLineBuffer, (const char*)tempPrintBuffer);
                appData.dumpIndex++;
            }
            strcat((char*)dumpLineBuffer, "   ");
            /* rewind to print the ASCII equivalents */
            appData.dumpIndex -= 16;
            for(i = 0; i < 16; i++)
            {
                if (isprint(pageCopy[appData.dumpIndex] & 0x7F))
                {
                    sprintf((char*)tempPrintBuffer, "%c", pageCopy[appData.dumpIndex]);
                }
                else
                {
                    sprintf((char*)tempPrintBuffer, ".");
                }
                strcat((char*)dumpLineBuffer, (const char*)tempPrintBuffer);
                appData.dumpIndex++;
            }

            strcat((char*)dumpLineBuffer, "\r\n");
            SYS_CONSOLE_PRINT((const char*)dumpLineBuffer);
            appData.sqiDisplayAddress += 16;
            appData.state = APP_STATE_DUMP_LINE_COMPLETE;
            break;

        case APP_STATE_DUMP_LINE_COMPLETE:
            if (!SYS_CMD_READY_TO_WRITE())
            {
                break;
            }
            
            appData.linesToDisplay--;
            if (appData.linesToDisplay != 0)
            {
                appData.state = APP_STATE_DUMP_LINE;
            }
            else
            {
                appData.state = APP_STATE_DUMP_COMPLETE;
            }
            break;

        case APP_STATE_DUMP_COMPLETE:
            /* tidy up after transfer completion */
            if (!SYS_CMD_READY_TO_READ())
            {
                break;
            }
            appData.state = APP_STATE_DISPLAY_IDLE;
            break;

        case APP_STATE_ERROR:
            /* error state */
            break;

            /* The default state should never be executed. */
        default:
            /* TODO: Handle error in application's state machine. */
            break;
    }
}

/**************************************************************************************************/
/* Application commands */

int _APP_Command_erase(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    /* erase the flash */
    appData.state = APP_STATE_FLASH_ERASE;
    return false;
}

/* program a number of pages with a pattern */
/* ptest [address] [value] [pagecount] */
int _APP_Command_ptest(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    uint32_t address, pageCount;
    uint32_t value;
    char* endChar;

    if (argc == 1)
    {
        SYS_CONSOLE_MESSAGE("Illegal number of parameters/r/n");
        return true;
    }

    value = 0x0000; // default color is black
    pageCount = 1; // program 1 page by default

    /* read the address */
    errno = 0;
    address = strtoul(argv[1], &endChar, 0);
    if (errno != 0)
    {
        SYS_CONSOLE_MESSAGE("Illegal address parameter/r/n");
        return true;
    }

    if (argc > 2)
    {
        /* read the value */
        errno = 0;
        value = strtoul(argv[2], &endChar, 0);
        if (errno != 0)
        {
            SYS_CONSOLE_MESSAGE("Illegal fill value/r/n");
            return true;
        }
    }

    if (argc > 3)
    {
        /* read the page count */
        errno = 0;
        pageCount = strtoul(argv[3], &endChar, 0);
        if (errno != 0)
        {
            SYS_CONSOLE_MESSAGE("Illegal page count value/r/n");
            return true;
        }
    }

    /* perform programming of page(s) with the value */
    appData.pageProgramAddress = address;
    appData.pageProgramValue = (uint16_t) value;
    appData.pageProgramCount = pageCount;
    appData.state = APP_STATE_FLASH_PTEST_START;

    return false;
}

/* copy the display to the flash at specificed address */
/* pcopy [address] */
int _APP_Command_pcopy(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    uint32_t address;
    char* endChar;

    /* read the address in pData */
    if (argc != 2)
    {
        SYS_CONSOLE_MESSAGE("Illegal number of parameters\r\n");
        return true;
    }

    errno = 0;
    address = strtoul(argv[1], &endChar, 0);
    if (errno != 0)
    {
        SYS_CONSOLE_MESSAGE("Illegal parameter\r\n");
        return true;
    }

    /* perform programming of page(s) with the value */
    appData.pageProgramAddress = address & 0x3FFF00;
    appData.screenCopyOffset = 0;
    appData.state = APP_STATE_FLASH_COPY_START;
    
    return true;
}

int _APP_Command_dump(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    uint32_t address;
    char* endChar;

    if (argc != 2)
    {
        SYS_CONSOLE_MESSAGE("Illegal number of parameters\r\n");
        return true;
    }

    errno = 0;
    address = strtoul(argv[1], &endChar, 0);
    if (errno != 0)
    {
        SYS_CONSOLE_MESSAGE("Illegal parameter\r\n");
        return true;
    }

    appData.sqiDisplayAddress = address & 0x003FFF00;
    appData.linesToDisplay = 16; /* 16 lines of 16 bytes */
    appData.state = APP_STATE_START_DUMP;
    return false;
}

int _APP_Command_download(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    uint32_t downloadIgnore = 108;  /* bmp V4 header default value = 108, bmp v5 = 124*/
    char* endChar;

    /* read the number of bytes to ignore in pData */
    if (argc > 1)
    {
        errno = 0;
        downloadIgnore = strtoul(argv[1], &endChar, 0);
        if (errno != 0)
        {
            SYS_CONSOLE_MESSAGE("Illegal parameter\r\n");
            return true;
        }
    }

    appData.screenBytesToIgnore = downloadIgnore;
    appData.state = APP_STATE_XMODEM_INIT;
    appData.screenCopyOffset = 0;
    return false;
}

int _APP_Command_display(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    uint32_t address = 0x00000000;
    char* endChar;

    /* read the address*/
    if (argc > 1)
    {
        errno = 0;
        address = strtoul(argv[1], &endChar, 0);
        if (errno != 0)
        {
            SYS_CONSOLE_MESSAGE("Illegal parameter\r\n");
            return true;
        }
    }

    address &= 0x003FFFFF;
    appData.sqiDisplayAddress = address;
    appData.state = APP_STATE_IMAGE_UPDATE;
    return true;
}


/*******************************************************************************
 End of File
 */
