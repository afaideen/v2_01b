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


#include "app_flash.h"

#define PIC32_KVA0_TO_KVA1_VAR(v)   (*(typeof(v)*)((unsigned long)&(v) | 0x20000000u))
#define PIC32_KVA0_TO_KVA1_PTR(v)   ((typeof(v)*)((unsigned long)(v) | 0x20000000u))
#define PIC32_UNCACHED_VAR(v)       PIC32_KVA0_TO_KVA1_VAR(v)
#define PIC32_UNCACHED_PTR(v)       PIC32_KVA0_TO_KVA1_PTR(v)

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

uint8_t  FLH_asciiBuffer[1024];
uint32_t FLH_pointer = 0;
uint8_t  FLH_hexRec[100];
T_REC    FLH_record;
size_t   FLH_numBytes;
uint32_t FLH_i;
uint32_t FLH_i_tot;
uint32_t FLH_recCount = 0;

static bool sync=false;
static bool syncErr=false;
static bool syncCheck=false;
static uint8_t syncWidx;
//static uint8_t syncW[258];
static uint8_t syncFW[258];//={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static uint8_t syncR[258];
static uint32_t syncAddr;
static uint8_t syncRem;
static bool sync1page;
static uint8_t syncWx[258];
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

void APP_FLASH_Task ( void )
{
    //SQI_STATUS sqiStatus;

    
    /* Check the application's current state. */
    switch ( flashData.State )
    {
         case FLH_STATE_INIT:
            sqiInitData.clkDivider = CLK_DIV_2;
            sqiInitData.csPins = SQI_CS_OEN_1;
            sqiInitData.dataMode = SQI_DATA_MODE_3;
            SST26Init((DRV_SQI_INIT_DATA*)&sqiInitData);
            flashData.State = FLH_STATE_SETUP;
            break;

        case FLH_STATE_SETUP:
            flashData.Boot = FALSE;
            flashData.State = FLH_STATE_CHECK_DRVR_STATE;
            break;

        case FLH_STATE_CHECK_DRVR_STATE:
            if (SST26_SQI_FlashID_Read())
            {
//                LoadCfg();
                //flashData.State =FLH_STATE_DONE;
                flashData.State = FLH_STATE_LOADRES;
            }
            else {
                flashData.LastError = 1;
                flashData.State =FLH_STATE_CHECK_DRVR_STATE;
            }
            break;

        case FLH_STATE_LOADRES_BOOT:
             flashData.Boot = TRUE;
             flashData.State = FLH_STATE_LOADRES;
            break;

        case FLH_STATE_LOADRES:
             FLH_OpenResFile();
             FLH_i_tot = 0;             
            break;

        case FLH_STATE_ERASE:    //Lettura Analogiche
             SST26ChipErase();   //Verify if only part of itù
             syncErr=false;
             flashData.State = FLH_STATE_READ_FILE;
            break;

        case FLH_STATE_READ_FILE:
             coreupdate=true;
             FLH_ReadFile();
             break;

        case FLH_STATE_PROCESS_RECORD:
            processRecord();
              //if (processRecord())
              //{
              //      SYS_FS_FileClose(flashData.fileHandle);
              //      flashData.LoadPercent = 0;
              //      flashData.State = FLH_STATE_LOADRES;
              //      
              //}
            break;

        case FLH_STATE_PROCESS_BUFFER:
             processSize();
             flashData.State = FLH_STATE_READ_FILE;
             break;

        case FLH_STATE_LOADRES_COMPLETE:
             if (flashData.Boot==TRUE)
             {
              //App State Boot
             };
             FLH_i_tot = 0;
//             GreenLedOff();
//             coreupdate=true;
//             LoadCfg();
             flashData.State = FLH_STATE_WAIT_REMOVE;
#if defined(GFX_USE_DISPLAY_CONTROLLER_LCC)
             GFX_HGC_SetScreenState(HGC_SCREEN_STATE_SETUP_SCREEN_screen2);
#endif
             BSP_LEDOff(BSP_LED_2);
             BSP_LEDOn(BSP_LED_3);
             //GFX_HGC_DrawItem(StaticText3);
             //GFX_STATICTEXT_STATE_0  GFX_GOL_STATICTEXT_DRAW_STATE | GFX_GOL_STATICTEXT_NOBACKGROUND_STATE
             //GFX_HGC_DrawItem(StaticText1);
             break;

        case FLH_STATE_WAIT_REMOVE:   //Lettura Analogiche
             break;

        case FLH_STATE_LOADRES_EFL:

   
             flashData.fileHandle = SYS_FS_FileOpen(flashData.flashSrc, (SYS_FS_FILE_OPEN_READ));
             if(flashData.fileHandle == SYS_FS_HANDLE_INVALID)
             {
                    /* Could not open the file. Error out*/
                    flashData.State = FLH_STATE_ERROR;
                    flashData.LastError = 2;
             }
             else
             {
                    /* File opened successfully. Write to file */
                    flashData.fileSize = SYS_FS_FileSize(flashData.fileHandle);
                    flashData.State = FLH_STATE_ERASE_EFL;
             }
            break;

        case FLH_STATE_ERASE_EFL:
            SST26ChipErase();
            syncErr=false;
            flashData.State = FLH_STATE_READ_FILE_EFL;
            break;               
        
        case FLH_STATE_READ_FILE_EFL:
            /* Try reading the file */
            FLH_numBytes = SYS_FS_FileRead( flashData.fileHandle, (void *)flashData.fileBuffer, 512 );
            if (FLH_numBytes <= 0)
            {
                SYS_FS_FileClose(flashData.fileHandle);
                flashData.State = FLH_STATE_LOADRES_COMPLETE;

            }
            else
            {
                BSP_LEDToggle(BSP_LED_2);                
                memcpy(&FLH_asciiBuffer[FLH_pointer], flashData.fileBuffer, FLH_numBytes);
                /* Process the buffer that we read. */
                FLH_processBuffer(FLH_numBytes);
//                appData.state = APP_STATE_CLOSE_FILE;
            }
            break;

        case FLH_STATE_DONE:   //Lettura Analogiche
             break;
             
        case FLH_STATE_ERROR:
             break;
    }

} //End of APP_Flash_Tasks

void FLH_OpenResFile(void)
{
    flashData.fileHandle = SYS_FS_FileOpen(flashData.flashSrc, (SYS_FS_FILE_OPEN_READ));
     if(flashData.fileHandle == SYS_FS_HANDLE_INVALID)
    {
        /* Could not open the file. Error out*/
        flashData.State = FLH_STATE_ERROR;
        flashData.LastError = 2;
    }
    else
    {
        /* File opened successfully. Write to file */
        flashData.fileSize = SYS_FS_FileSize(flashData.fileHandle);
        flashData.State = FLH_STATE_ERASE;

    }
}

void FLH_ReadFile(void)
{
    /* Try reading the file */
    FLH_numBytes = SYS_FS_FileRead( flashData.fileHandle, (void *)flashData.fileBuffer, 512 );
    if (FLH_numBytes <= 0)
    {
        SYS_FS_FileClose(flashData.fileHandle);
        if (flashData.LoadPercent>0) {
          flashData.State = FLH_STATE_LOADRES_COMPLETE;
        }
        else {
          flashData.State = FLH_STATE_ERROR;
          flashData.LastError = 3;
        }

    }
    else
    {
//        GreenLedToggle();
        memcpy(&FLH_asciiBuffer[FLH_pointer], flashData.fileBuffer, FLH_numBytes);
        FLH_i = 0;
        flashData.State = FLH_STATE_PROCESS_RECORD;
    }
}

/********************************************************************
* Function: 	ConvertAsciiToHex()
*
* Precondition:
*
* Input: 		Ascii buffer and hex buffer.
*
* Output:
*
* Side Effects:	No return from here.
*
* Overview: 	Converts ASCII to Hex.
*
*
* Note:		 	None.
********************************************************************/
void ConvertAsciiToHex(uint8_t* asciiRec, uint8_t* hexRec)
{
	uint8_t i = 0;
	uint8_t k = 0;
	uint8_t hex;


	while((asciiRec[i] >= 0x30) && (asciiRec[i] <= 0x66))
	{
		// Check if the ascci values are in alpha numeric range.

		if(asciiRec[i] < 0x3A)
		{
			// Numerical reperesentation in ASCII found.
			hex = asciiRec[i] & 0x0F;
		}
		else
		{
			// Alphabetical value.
			hex = 0x09 + (asciiRec[i] & 0x0F);
		}

		// Following logic converts 2 bytes of ASCII to 1 byte of hex.
		k = i%2;

		if(k)
		{
			hexRec[i/2] |= hex;

		}
		else
		{
			hexRec[i/2] = (hex << 4) & 0xF0;
		}
		i++;
	}
}

void processRecord(void)
{   
    if (FLH_i < (FLH_numBytes + FLH_pointer))
    //for(i = 0; i < (numBytes + pointer); i ++)
    {
            // This state machine seperates-out the valid hex records from the read 512 bytes.
        switch(FLH_record.status)
        {
            case REC_FLASHED:
            case REC_NOT_FOUND:
                if(FLH_asciiBuffer[FLH_i] == ':')
                {
                        // We have a record found in the 512 bytes of data in the buffer.
                    FLH_record.start = &FLH_asciiBuffer[FLH_i];
                    FLH_record.len = 0;
                    FLH_record.status = REC_FOUND_BUT_NOT_FLASHED;
                }
                break;
            case REC_FOUND_BUT_NOT_FLASHED:
                if((FLH_asciiBuffer[FLH_i] == 0x0A) || (FLH_asciiBuffer[FLH_i] == 0xFF))
                {
                        // We have got a complete record. (0x0A is new line feed and 0xFF is End of file)
                    // Start the hex conversion from element
                    // 1. This will discard the ':' which is
                    // the start of the hex record.
                    ConvertAsciiToHex(&FLH_record.start[1],FLH_hexRec);
                    FLH_ProgramHexRecord(FLH_hexRec, FLH_record.len >> 1);
                    FLH_recCount++;
                    FLH_record.status = REC_FLASHED;
                }
                break;
        }
        // Move to next byte in the buffer.
        FLH_record.len ++;
        flashData.LoadPercent =  (int)((FLH_i_tot*100)/(flashData.fileSize));
        FLH_i_tot++;
        FLH_i++;
    }
    else {
       flashData.State = FLH_STATE_PROCESS_BUFFER;
    }


}

void processSize(void)
{

    if(FLH_record.status == REC_FOUND_BUT_NOT_FLASHED)
    {
            // We still have a half read record in the buffer. The next half part of the record is read
            // when we read 512 bytes of data from the next file read.
        memcpy(FLH_asciiBuffer, FLH_record.start, FLH_record.len);
        FLH_pointer = FLH_record.len;
        FLH_record.status = REC_NOT_FOUND;
    }
    else
    {
        FLH_pointer = 0;
    }
}

void FLH_processBuffer(size_t numBytes)
{
    uint32_t i;
    uint32_t recCount = 0;

    for(i = 0; i < (numBytes + FLH_pointer); i ++)
    {
            // This state machine seperates-out the valid hex records from the read 512 bytes.
        switch(FLH_record.status)
        {
            case REC_FLASHED:
            case REC_NOT_FOUND:
                if(FLH_asciiBuffer[i] == ':')
                {
                        // We have a record found in the 512 bytes of data in the buffer.
                    FLH_record.start = &FLH_asciiBuffer[i];
                    FLH_record.len = 0;
                    FLH_record.status = REC_FOUND_BUT_NOT_FLASHED;
                }
                break;
            case REC_FOUND_BUT_NOT_FLASHED:
                if((FLH_asciiBuffer[i] == 0x0A) || (FLH_asciiBuffer[i] == 0xFF))
                {
                    // We have got a complete record. (0x0A is new line feed and 0xFF is End of file)
                    // Start the hex conversion from element
                    // 1. This will discard the ':' which is
                    // the start of the hex record.
                    ConvertAsciiToHex(&FLH_record.start[1],FLH_hexRec);
                    FLH_ProgramHexRecord(FLH_hexRec, FLH_record.len >> 1);
                    recCount++;
                    FLH_record.status = REC_FLASHED;
                }
                break;
        }
        // Move to next byte in the buffer.
        FLH_record.len ++;
    }

    if(FLH_record.status == REC_FOUND_BUT_NOT_FLASHED)
    {
            // We still have a half read record in the buffer. The next half part of the record is read
            // when we read 512 bytes of data from the next file read.
        memcpy(FLH_asciiBuffer, FLH_record.start, FLH_record.len);
        FLH_pointer = FLH_record.len;
        FLH_record.status = REC_NOT_FOUND;
    }
    else
    {
        FLH_pointer = 0;
    }
}

void FLH_ProgramHexRecord(uint8_t* HexRecord, int32_t totalLen)
{
    static T_HEX_RECORD HexRecordSt;
    uint8_t Checksum = 0;
    static uint32_t i;
    uint32_t nextRecStartPt = 0;

    while(totalLen>=5) // A hex record must be at least 5 bytes. (1 Data Len byte + 1 rec type byte+ 2 address bytes + 1 crc)
    {
        HexRecord = &HexRecord[nextRecStartPt];
        HexRecordSt.RecDataLen = HexRecord[0];
        HexRecordSt.RecType = HexRecord[3];
        HexRecordSt.Data = &HexRecord[4];

        //Determine next record starting point.
        nextRecStartPt = HexRecordSt.RecDataLen + 5;

        // Decrement total hex record length by length of current record.
        totalLen = totalLen - nextRecStartPt;

        // Hex Record checksum check.
        Checksum = 0;
        for(i = 0; i < HexRecordSt.RecDataLen + 5; i++)
        {
            Checksum += HexRecord[i];
        }

        if(Checksum != 0)
        {
            Nop(); // for breakpoint
            //Error. Hex record Checksum mismatch.
        }
        else
        {
            // Hex record checksum OK.
            switch(HexRecordSt.RecType)
            {
                case DATA_RECORD:  //Record Type 00, data record.
                    HexRecordSt.Address.byte.MB = 0;
                    HexRecordSt.Address.byte.UB = 0;
                    HexRecordSt.Address.byte.HB = HexRecord[1];
                    HexRecordSt.Address.byte.LB = HexRecord[2];

                    // Compute the address.
                    HexRecordSt.Address.Val = HexRecordSt.Address.Val + HexRecordSt.ExtLinAddress.Val + HexRecordSt.ExtSegAddress.Val;

                    if(syncErr==false)//first entering after flash erase or POR
                    {
                        syncErr=true;    
                        syncWidx=0;
                        /*ignore first data string since the address is the end of the file*/
                        //SST26WriteArray(HexRecordSt.Address.Val, HexRecordSt.Data, HexRecordSt.RecDataLen);
                        //syncAddr=HexRecordSt.Address.Val;
                        for(i=0;i<0x100;i++){syncWx[i]=0xFF;}//clear the buffer
                        sync=false;
                        for(i=0;i<0x100;i++){syncFW[i]=0xA5;}
                    }
                    else
                    {
                        if (sync==false)
                        {
                        sync=true;
                        /*sync to the first valid address*/
                        syncAddr=HexRecordSt.Address.Val;
                        sync1page=false;
                        }
                            
                    
                        //fill the buffer
                        if(HexRecordSt.RecDataLen<(0xFF-syncWidx+1))
                        {
                            memcpy(&syncWx[syncWidx], HexRecordSt.Data, HexRecordSt.RecDataLen);
                            syncWidx+=HexRecordSt.RecDataLen;
                        }
                        else if(HexRecordSt.RecDataLen==(0xFF-syncWidx+1))
                        {
                            memcpy(&syncWx[syncWidx], HexRecordSt.Data, HexRecordSt.RecDataLen);
                            syncWidx=0xFF;                   }
                        else
                        {
                            syncRem=(0xFF-syncWidx);
                            memcpy(&syncWx[syncWidx], HexRecordSt.Data, syncRem);  
                            syncWidx=0xFF;                       
                        }

                        if(syncWidx==0xFF)//write 256 buffer
                        {
                            syncWidx=0;
                            syncCheck=false;
                            while(!syncCheck){//loop if write is not verified
                                //write 256 array page
                                SST26WriteArray(syncAddr, &syncWx[0], 0x100);
                                //clear syncR
                                for (i=0;i<0x100;i++){
                                syncR[i]=0x0;
                                }
                            
                                //read back 256 array page
                                SST26ReadArray(syncAddr, syncR, 0x100);
                                syncCheck=true;
                                // verify
                            for (i=0;i<0x100;i++){
                                if(syncWx[i]!=syncR[i]){
                                    Nop();
                                    Nop();
                                    syncCheck=false;
//                                    ErrorBeep();
                                    //SYS_RESET_SoftwareReset();
//                                    return true;
                                    break;
                                }
                            }
                            
   
                        }
                            //update address for the next page
                            syncAddr+=0x100; 
                            //load the buffer with the reminder the next page
                            for(i=0;i<0x100;i++){syncWx[i]=0xFF;}//clear the buffer
                            if(syncRem)
                            {                     
                                syncWidx=HexRecordSt.RecDataLen-syncRem;
                                memcpy(&syncWx[0], (HexRecordSt.Data+syncRem), syncWidx);                 
                                syncRem=0;
                            }
                        }
                      }
//                    memcpy(&syncWx[syncWidx], HexRecordSt.Data, HexRecordSt.RecDataLen);
//                    SST26WriteArray(HexRecordSt.Address.Val, HexRecordSt.Data, HexRecordSt.RecDataLen);
//                    //read back HexRecordSt.RecDataLen array 
//                    SST26ReadArray(HexRecordSt.Address.Val, syncR, HexRecordSt.RecDataLen);
//                    syncCheck=true;
//                    // verify
//                    for (i=0;i<HexRecordSt.RecDataLen;i++){
//                        if(syncWx[i]!=syncR[i]){
//                            Nop();
//                            Nop();
//                            syncCheck=false;
//                            break;
//                        }
//                    }
                    HexRecordSt.Data += HexRecordSt.RecDataLen;
                    HexRecordSt.Address.Val += HexRecordSt.RecDataLen;
                    HexRecordSt.RecDataLen -= HexRecordSt.RecDataLen;

#ifdef __DEBUG
                    Nop(); // for breakpoint
#endif
                    break;

                case EXT_SEG_ADRS_RECORD:  // Record Type 02, defines 4th to 19th bits of the data address.
                    HexRecordSt.ExtSegAddress.byte.MB = 0;
                    HexRecordSt.ExtSegAddress.byte.UB = HexRecordSt.Data[0];
                    HexRecordSt.ExtSegAddress.byte.HB = HexRecordSt.Data[1];
                    HexRecordSt.ExtSegAddress.byte.LB = 0;
                    // Reset linear address.
                    HexRecordSt.ExtLinAddress.Val = 0;
                    break;

                case EXT_LIN_ADRS_RECORD:   // Record Type 04, defines 16th to 31st bits of the data address.
                    HexRecordSt.ExtLinAddress.byte.MB = HexRecordSt.Data[0];
                    HexRecordSt.ExtLinAddress.byte.UB = HexRecordSt.Data[1];
                    HexRecordSt.ExtLinAddress.byte.HB = 0;
                    HexRecordSt.ExtLinAddress.byte.LB = 0;
                    // Reset segment address.
                    HexRecordSt.ExtSegAddress.Val = 0;
                    break;

                case END_OF_FILE_RECORD:  //Record Type 01, defines the end of file record.
                    //check all data have been written
                    if(syncWidx)
                    {
                        SST26WriteArray(syncAddr, syncWx, 0x100);
                        syncWidx=0;
                    }
                    HexRecordSt.ExtSegAddress.Val = 0;
                    HexRecordSt.ExtLinAddress.Val = 0;
                    break;
                default:

                    HexRecordSt.ExtSegAddress.Val = 0;
                    HexRecordSt.ExtLinAddress.Val = 0;
                    break;
            }
        }
    }//while(1)
}

void CheckSync(void)
{
////
}
