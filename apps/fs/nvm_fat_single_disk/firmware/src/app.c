/*******************************************************************************
  MPLAB Harmony Application Source File
  NVM FAT Single Disk Demo Application
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.
    NVM FAT Single Disk Demo
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

#include <string.h>
#include "app.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
/* This is the string that will written to the file */
const uint8_t writeData[13] = "Hello World";

/* This string contins the original value of FILE.txt (before being written by the demo */
const uint8_t originalData[4] = "Data";
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
    /* Intialize the app state to wait for
     * media attach. */
    appData.state = APP_MOUNT_DISK;
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    /* The application task state machine */

    switch(appData.state)
    {
        case APP_MOUNT_DISK:
            if(SYS_FS_Mount("/dev/nvma1", "/mnt/myDrive", FAT, 0, NULL) != 0)
            {
                /* The disk could not be mounted. Try
                 * mounting again untill success. */

                appData.state = APP_MOUNT_DISK;
            }
            else
            {
                /* Mount was successful. Open a file.
                 * Let the switch case fall through. */
                appData.state = APP_OPEN_FILE;
            }
            break;

        case APP_OPEN_FILE:

            appData.fileHandle = SYS_FS_FileOpen("/mnt/myDrive/FILE.TXT", SYS_FS_FILE_OPEN_READ_PLUS);
            if(appData.fileHandle == SYS_FS_HANDLE_INVALID)
            {
                /* Could not open the file. Error out*/
                appData.state = APP_ERROR;
            }
            else
            {
                /* Check the file status */
                appData.state = APP_READ_FILE_STAT;
            }
            break;

        case APP_READ_FILE_STAT:
            if( SYS_FS_FileStat("/mnt/myDrive/FILE.TXT", &appData.fileStatus ) == SYS_FS_RES_FAILURE )
            {
                /* Reading file status was a failure */
                appData.state = APP_ERROR;
            }
            else
            {
                /* Read file size */
                appData.state = APP_READ_FILE_SIZE;
            }
            break;

        case APP_READ_FILE_SIZE:
            appData.fileSize = SYS_FS_FileSize( appData.fileHandle );

            if(appData.fileSize == -1)
            {
                /* Reading file size was a failure */
                appData.state = APP_ERROR;
            }
            else
            {
                if(appData.fileSize == appData.fileStatus.fsize)
                {
                    appData.state = APP_DO_FILE_SEEK;
                }
                else
                    appData.state = APP_ERROR;
            }
            break;

        case APP_DO_FILE_SEEK:
            if(SYS_FS_FileSeek( appData.fileHandle, appData.fileSize, SYS_FS_SEEK_SET ) == -1)
            {
                /* File seek caused an error */
                appData.state = APP_ERROR;
            }
            else
            {
                /* Check for End of file */
                appData.state = APP_CHECK_EOF;
            }
            break;

        case APP_CHECK_EOF:
            if(SYS_FS_FileEOF( appData.fileHandle ) == false )
            {
                /* Either, EOF is not reached or there was an error
                 In any case, for the application, its an error condition */
                appData.state = APP_ERROR;
            }
            else
            {
                appData.state = APP_DO_ANOTHER_FILE_SEEK;
            }
            break;

        case APP_DO_ANOTHER_FILE_SEEK:
            /* Move file pointer to begining of file */
            if(SYS_FS_FileSeek( appData.fileHandle, 0, SYS_FS_SEEK_SET ) == -1)
            {
                /* File seek caused an error */
                appData.state = APP_ERROR;
            }
            else
            {
                /* Check for original file content */
                appData.state = APP_READ_ORIGINAL_FILE_CONTENT;
            }
            break;

        case APP_READ_ORIGINAL_FILE_CONTENT:
            if(SYS_FS_FileRead(appData.fileHandle, (void *)appData.data, 4) == -1)
            {
                /* There was an error while reading the file.
                 * Close the file and error out. */

                SYS_FS_FileClose(appData.fileHandle);
                appData.state = APP_ERROR;
            }
            else
            {
                if(memcmp(appData.data, originalData, 4) != 0)
                {
                    /* The written and the read data dont
                     * match. */
                    appData.state = APP_ERROR;
                }
                else
                {
                    /* The test was successful. Lets do a file seek to move to begin of file . */
                    appData.state = APP_FINAL_FILE_SEEK;
                }

                break;
            }

        case APP_FINAL_FILE_SEEK:
            /* Move file pointer to begining of file */
            if(SYS_FS_FileSeek( appData.fileHandle, 4, SYS_FS_SEEK_SET) == -1)
            {
                /* File seek caused an error */
                appData.state = APP_ERROR;
            }
            else
            {
                /* Do a file write now */
                appData.state = APP_WRITE_TO_FILE;
            }
            break;

        case APP_WRITE_TO_FILE:
            if(SYS_FS_FileWrite(appData.fileHandle, (const void *)writeData, 13) == -1)
            {
                /* Write was not successful. Close the file
                 * and error out.*/
                SYS_FS_FileClose(appData.fileHandle);
                appData.state = APP_ERROR;
            }
            else
            {
                /* Flush the data to NVM*/
                SYS_FS_FileSync(appData.fileHandle);
                /* Write was successful. Read the file content. */
                appData.state = APP_SEEK_TO_START;
            }
            break;

        case APP_SEEK_TO_START:

            if(SYS_FS_FileSeek(appData.fileHandle, 4, SYS_FS_SEEK_SET) == -1)
            {
                /* Could not seek the file. Error out*/
                appData.state = APP_ERROR;
            }
            else
            {
                /* Read the file content */
                appData.state = APP_READ_VERIFY_FILE;
            }
            break;

        case APP_READ_VERIFY_FILE:

            if(SYS_FS_FileRead(appData.fileHandle, (void *)appData.data, 13) == -1)
            {
                /* There was an error while reading the file.
                 * Close the file and error out. */

                SYS_FS_FileClose(appData.fileHandle);
                appData.state = APP_ERROR;
            }
            else
            {
                if(strcmp((const char *)appData.data, (const char *)writeData) != 0)
                {
                    /* The written and the read data dont
                     * match. */
                    appData.state = APP_ERROR;
                }
                else
                {
                    /* The test was successful. Lets idle. */
                    appData.state = APP_IDLE;
                }

                SYS_FS_FileClose(appData.fileHandle);
                break;
            }
        case APP_IDLE:
            /* The application comes here when the demo
             * has completed successfully. Switch on
             * green LED. */
                BSP_LEDOn(APP_SUCCESS_LED);
            break;
        case APP_ERROR:
            /* The application comes here when the demo
             * has failed. Switch on the red LED.*/
               BSP_LEDOn(APP_FAILURE_LED);
            break;
        default:
            break;



    }

} //End of APP_Tasks

 

/*******************************************************************************
 End of File
 */

