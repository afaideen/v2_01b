/*******************************************************************************
  MPLAB Harmony Bootloader Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    nvm.c

  Summary:
    This file contains the source code for handling NVM controllers.

  Description:
    This file contains the source code for the NVM handling functions for PIC32MX
	and MZ devices.
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

#include "app.h"
//#include "system_config.h"
#include "system/devcon/sys_devcon.h"
#include "peripheral/int/plib_int.h"
#include "peripheral/nvm/plib_nvm.h"
#include "bootloader/src/nvm.h"
#include "sqi.h"
#include "system/debug/sys_debug.h"

#define KVAx_TO_PA(v)    ((v) & 0x3FFFFFFF)

typedef struct
{
    uint8_t RecDataLen;
    uint32_t Address;
    uint8_t RecType;
    uint8_t* Data;
    uint8_t CheckSum;
    uint32_t ExtSegAddress;
    uint32_t ExtLinAddress;
}T_HEX_RECORD;

typedef struct T_MEM_BUFFER
{
    uint32_t devAddress;                // Address in the device memory
    uint8_t  data[SQI_FLASH_PAGE_SIZE]; // Data to be written out
    uint32_t count;                     // Number of bytes in buffer
    struct T_MEM_BUFFER *nextBuf;       // Next element in the linked list
    struct T_MEM_BUFFER *prevBuf;       // Previous element in the linked list
}T_MEM_BUFFER;

T_MEM_BUFFER    *memRoot = NULL;        // Root of the linked list

extern uint32_t sqiOps;
uint32_t nvmOps = 0;
uint32_t recordCount = 0;
uint32_t bufferCount = 0;

void APP_NVMOperation(uint32_t nvmop)
{
    
    uint32_t processorStatus;
    
    processorStatus = PLIB_INT_GetStateAndDisable( INT_ID_0 );
    
    // Disable flash write/erase operations
    PLIB_NVM_MemoryModifyInhibit(NVM_ID_0);

    PLIB_NVM_MemoryOperationSelect(NVM_ID_0, nvmop);

    // Allow memory modifications
    PLIB_NVM_MemoryModifyEnable(NVM_ID_0);

    /* Unlock the Flash */
    PLIB_NVM_FlashWriteKeySequence(NVM_ID_0, 0);
    PLIB_NVM_FlashWriteKeySequence(NVM_ID_0, NVM_PROGRAM_UNLOCK_KEY1);
    PLIB_NVM_FlashWriteKeySequence(NVM_ID_0, NVM_PROGRAM_UNLOCK_KEY2);

    PLIB_NVM_FlashWriteStart(NVM_ID_0);
    
    PLIB_INT_SetState(INT_ID_0, processorStatus);
    
//    PLIB_NVM_MemoryModifyInhibit(NVM_ID_0);
    
    nvmOps++;

}

void APP_PFMFlashErase( void )
{
#if (USE_PAGE_ERASE)
    unsigned int flashAddr = BOOTLOADER_FLASH_BASE_ADDRESS;

    while (flashAddr < BOOTLOADER_FLASH_END_ADDRESS)
    {
        while (!PLIB_NVM_FlashWriteCycleHasCompleted(NVM_ID_0));
        PLIB_NVM_FlashAddressToModify(NVM_ID_0, KVAx_TO_PA(flashAddr));
        APP_NVMOperation(PAGE_ERASE_OPERATION);
        flashAddr += NVM_PAGE_SIZE;
    }
#else
    #if defined(BOOTLOADER_STATE_SAVE)
    APP_NVMOperation(UPPER_FLASH_REGION_ERASE_OPERATION);
    #else
    APP_NVMOperation(FLASH_ERASE_OPERATION);
    #endif
#endif
}

void APP_NVMWordWrite(void* address, uint32_t data)
{
     unsigned int addr = KVAx_TO_PA((unsigned int) address);
#if defined(BOOTLOADER_STATE_SAVE)
    // Ensure we write to the other program flash
    if (addr < 0x1FC00000)
    {
         addr += 0x100000;
    }
#endif
    PLIB_NVM_FlashAddressToModify(NVM_ID_0, addr);

    PLIB_NVM_FlashProvideData(NVM_ID_0, data);

    APP_NVMOperation(WORD_PROGRAM_OPERATION);
}

void APP_NVMRowWrite(void* address, void* data)
{
    unsigned int addr = KVAx_TO_PA((unsigned int) address);
    #if defined(BOOTLOADER_STATE_SAVE)
    // Ensure we write to the other program flash    
    if (addr < 0x1FC00000)
    {
        addr += 0x100000;
    }
    #endif

    PLIB_NVM_FlashAddressToModify(NVM_ID_0, addr);

    PLIB_NVM_DataBlockSourceAddress(NVM_ID_0, KVAx_TO_PA((unsigned int)data));

    APP_NVMOperation(ROW_PROGRAM_OPERATION);
}

void APP_NVMQuadWordWrite(void* address, uint32_t* data)
{
 #if (USE_QUAD_WORD_WRITE)
    unsigned int addr = KVAx_TO_PA((unsigned int) address);

    #if defined(BOOTLOADER_STATE_SAVE)
    // Ensure we write to the other program flash
    if (addr < 0x1FC00000)
    {
        addr += 0x100000;
    }
    #endif

    if (PLIB_NVM_ExistsProvideQuadData(NVM_ID_0))
    {
        PLIB_NVM_FlashAddressToModify(NVM_ID_0, addr);

        PLIB_NVM_FlashProvideQuadData(NVM_ID_0, data);

        APP_NVMOperation(QUAD_WORD_PROGRAM_OPERATION);
    }
#endif
}

void APP_NVMClearError(void)
{
    APP_NVMOperation(NO_OPERATION);
}

void APP_BufferInit(void)
{
    memRoot = NULL;
}

void APP_BufferDataAdd(uint32_t Address, uint8_t *Data, uint32_t dataSize)
{
    T_MEM_BUFFER    *memPtr;
    uint32_t        pageAddr;
    uint16_t        addrHigh;
    uint8_t        pageOffset;
    
    //Address = KVAx_TO_PA(Address);
    pageAddr = Address & 0xFFFFFF00;
    addrHigh = KVAx_TO_PA(pageAddr) >> 16;
    
    // Ignore any page address outside of the SQI and Program Flash areas
    if ((addrHigh < 0x1D00) || // Below Program Flash
            ((addrHigh >= 0x1D20) && (addrHigh < 0x3000)) || // Between Program Flash and SQI Flash
            (addrHigh >= 0x3400) )  // Above SQI memory
        return;
    
    if (memRoot == NULL)
    {
        memRoot = malloc(sizeof(T_MEM_BUFFER));
        if (NULL == memRoot)
        {
            // Error out. Breakpoint for now
            SYS_DEBUG_BreakPoint();
        }
        memset(memRoot->data, 0xFF, SQI_FLASH_PAGE_SIZE);
        memRoot->nextBuf = memRoot;
        memRoot->prevBuf = memRoot;
        memRoot->count = 0;
        memRoot->devAddress = pageAddr;
        bufferCount = 1;
    }
    memPtr = memRoot;
    do
    {
        if (memPtr->devAddress == pageAddr)
            break;
        memPtr = memPtr->nextBuf;
    }
    while (memPtr != memRoot);
    
    if (memPtr->devAddress != pageAddr)
    {
        T_MEM_BUFFER    *newBuf;
        
        // Create the new node
        newBuf = malloc(sizeof(T_MEM_BUFFER));
        if (NULL == newBuf)
        {
            SYS_DEBUG_BreakPoint();
        }
        memset(newBuf->data, 0xFF, SQI_FLASH_PAGE_SIZE);
        newBuf->count = 0;
        newBuf->devAddress = pageAddr;
        newBuf->prevBuf = memPtr;
        newBuf->nextBuf = memPtr->nextBuf;
        memPtr->nextBuf = newBuf;
        newBuf->nextBuf->prevBuf = newBuf;
        memPtr = newBuf;
        bufferCount++;
    }
    
    pageOffset = Address & SQI_FLASH_OFFSET_MASK;
    
    // Now copy the data into the buffer
    // NOTE: This needs to be based on address, not on count! We don't know
    //       what order the data will come in.
    if (memPtr->count + dataSize > SQI_FLASH_PAGE_SIZE)
    {
        uint32_t memOffset = SQI_FLASH_PAGE_SIZE - memPtr->count;
        // Copy only what we need, then move on to the next one.
        memcpy(&memPtr->data[pageOffset], Data, memOffset);
        APP_BufferDataAdd(Address + memOffset, Data + memOffset, dataSize - memOffset);
        memPtr->count = SQI_FLASH_PAGE_SIZE;
//        SYS_DEBUG_BreakPoint();
    }
    else
    {
        memcpy(&memPtr->data[pageOffset], Data, dataSize);
        memPtr->count += dataSize;
    }
    
    // Determine if we need to write this buffer to the device
#if (1)
    if (memPtr->count == SQI_FLASH_PAGE_SIZE)
    {
        // Program NVM memory if the address is appropriate
        if ((memPtr->devAddress >= 0x1D000000) && (memPtr->devAddress < 0x1D200000))
        {
            while (memPtr->count)
            {
                APP_NVMQuadWordWrite((void*)memPtr->devAddress, 
                        (uint32_t*)&memPtr->data[SQI_FLASH_PAGE_SIZE - memPtr->count]);
                while (!PLIB_NVM_FlashWriteCycleHasCompleted(NVM_ID_0));
                if (PLIB_NVM_WriteOperationHasTerminated(NVM_ID_0))
                {
                    SYS_DEBUG_BreakPoint();
                    APP_NVMClearError();
                }
                memPtr->count -= 16;
                memPtr->devAddress += 16;
            }
        }
        else
        {
            // Copy the contents to the transfer buffer
            memcpy((void*)WRITE_BUF_ADDR, memPtr->data, SQI_FLASH_PAGE_SIZE);
            // Go program it
            SQI_PIO_PageWrite(memPtr->devAddress);
            SQI_PIO_Read(memPtr->devAddress);
            if (memcmp((void*)PIO_READ_BUF_ADDR, memPtr->data, SQI_FLASH_PAGE_SIZE) != 0)
            {
                SYS_DEBUG_BreakPoint();
            }
        }

        // Remove the buffer from the list
        memPtr->nextBuf->prevBuf = memPtr->prevBuf;
        memPtr->prevBuf->nextBuf = memPtr->nextBuf;
        
        if (memPtr == memRoot)
        {
            memRoot = memRoot->nextBuf;
            if (memRoot == memPtr)  // There was no other element
            {
                memRoot = NULL;
            }
        }
        free(memPtr);
        bufferCount--;
    }
#endif
}

void APP_BuffersFinish( void )
{
    T_MEM_BUFFER    *memPtr;
    
    if (memRoot != NULL)
    {
        memRoot->prevBuf->nextBuf = NULL;   // Cut off the loop so we can do this faster
    }

//    SQI_DMA_Mode();
    while (memRoot != NULL)
    {
        // Program NVM memory if the address is appropriate
        if ((memRoot->devAddress >= 0x1D000000) && (memRoot->devAddress < 0x1D200000))
        {
            while (memRoot->count)
            {
                APP_NVMQuadWordWrite((void*)memRoot->devAddress, 
                        (uint32_t*)&memRoot->data[SQI_FLASH_PAGE_SIZE - memRoot->count]);
                while (!PLIB_NVM_FlashWriteCycleHasCompleted(NVM_ID_0));
                if (PLIB_NVM_WriteOperationHasTerminated(NVM_ID_0))
                {
                    SYS_DEBUG_BreakPoint();
                    APP_NVMClearError();
                }
                memRoot->count -= 16;
                memRoot->devAddress += 16;
            }
        }
        else if ((memRoot->devAddress >= BOOTLOADER_FLASH_BASE_ADDRESS) && (memRoot->devAddress <= BOOTLOADER_FLASH_END_ADDRESS))
        {
            memcpy((void*)WRITE_BUF_ADDR, memRoot->data, SQI_FLASH_PAGE_SIZE);
            // Go program it
            SQI_PIO_PageWrite(memRoot->devAddress);
            SQI_PIO_Read(memRoot->devAddress);
            if (memcmp((void*)PIO_READ_BUF_ADDR, memRoot->data, SQI_FLASH_PAGE_SIZE) != 0)
            {
                SYS_DEBUG_BreakPoint();
            }
        }
        
        memPtr = memRoot->nextBuf;
        free(memRoot);
        bufferCount--;
        memRoot = memPtr;
    }
    //SQI_XIP_Mode();
}

void APP_FlashErase( void )
{
    sqiOps = 0;
    nvmOps = 0;
    bufferCount = 0;
    SQI_Flash_Erase();
    APP_PFMFlashErase();
    APP_BufferInit();
}

char APP_ProgramHexRecord(uint8_t* HexRecord, int32_t totalLen)
{
    static T_HEX_RECORD HexRecordSt;
    static uint32_t _Len = 0;
    uint8_t Checksum = 0;
    uint32_t i;
    static uint32_t nextRecStartPt = 0;
    static uint8_t state =0;
    
    if(state == 0)
    {    
        nextRecStartPt = 0;
        _Len = totalLen;
        state = 1;        
    }
    
    while(_Len >= 5) // A hex record must be atleast 5 bytes. (1 Data Len byte + 1 rec type byte+ 2 address bytes + 1 crc)
    {
       
        //HexRecord = &HexRecord[nextRecStartPt];
        HexRecordSt.RecDataLen = HexRecord[nextRecStartPt];
        HexRecordSt.RecType = HexRecord[nextRecStartPt+3];
        HexRecordSt.Data = &HexRecord[nextRecStartPt+4];

        // Hex Record checksum check.
        Checksum = 0;
        for(i = 0; i < HexRecordSt.RecDataLen + 5; i++)
        {
            Checksum += HexRecord[nextRecStartPt+i];
        }

        if(Checksum != 0)
        {
            return (0);//HEX_REC_CRC_ERROR;
        }
        else
        {
            // Hex record checksum OK.
            switch(HexRecordSt.RecType)
            {
                case DATA_RECORD:  //Record Type 00, data record.
                    HexRecordSt.Address = (HexRecord[nextRecStartPt+1]<<8) + HexRecord[nextRecStartPt+2];

                    // Derive the address.
                    HexRecordSt.Address = HexRecordSt.Address + HexRecordSt.ExtLinAddress + HexRecordSt.ExtSegAddress;
                    
//                    if ((HexRecordSt.Address >= BOOTLOADER_FLASH_BASE_ADDRESS) && (HexRecordSt.Address <= BOOTLOADER_FLASH_END_ADDRESS))
                    {
                        APP_BufferDataAdd(HexRecordSt.Address, HexRecordSt.Data, HexRecordSt.RecDataLen);
                    }
//                    else while (HexRecordSt.RecDataLen)
//                    {
//                        void* ProgAddress = (void *)HexRecordSt.Address;
//                        uint32_t QuadData[4];   /* For Quad-write operations */
//                        uint32_t WrData;
//                        
//                        // Convert the Physical address to Virtual address.
////                        ProgAddress = PA_TO_KVA0(HexRecordSt.Address);
//
//                        // Make sure we are not writing boot area and device configuration bits.
//                        if(((ProgAddress >= (void *)0x1D000000) && (ProgAddress < (void *)0x1D200000)))
//                        {
//#if (USE_QUAD_WORD_WRITE)
//                            /* Determine if we can do this with a quad-word write */
//                            if ((HexRecordSt.RecDataLen >= 16) &&
//                                    (0 == ((unsigned int)ProgAddress & 0xF)))
//                            {
//                                memcpy(QuadData, HexRecordSt.Data, 16); // Copy 4 words of data in.
//                                APP_NVMQuadWordWrite(ProgAddress, QuadData);
//                                // Increment the address.
//                                HexRecordSt.Address += 16;
//                                // Increment the data pointer.
//                                HexRecordSt.Data += 16;
//                                // Decrement data len.
//                                if(HexRecordSt.RecDataLen > 15)
//                                {
//                                    HexRecordSt.RecDataLen -= 16;
//                                }
//                                else
//                                {
//                                    HexRecordSt.RecDataLen = 0;
//                                }
//                            }
//                            else
//#endif
//                            {
//                                if(HexRecordSt.RecDataLen < 4)
//                                {
//                                    // Sometimes record data length will not be in multiples of 4. Appending 0xFF will make sure that..
//                                    // we don't write junk data in such cases.
//                                    WrData = 0xFFFFFFFF;
//                                    memcpy(&WrData, HexRecordSt.Data, HexRecordSt.RecDataLen);
//                                }
//                                else
//                                {
//                                    memcpy(&WrData, HexRecordSt.Data, 4);
//                                }
//                                // Write the data into flash.
//                                APP_NVMWordWrite(ProgAddress, WrData);
//                                // Increment the address.
//                                HexRecordSt.Address += 4;
//                                // Increment the data pointer.
//                                HexRecordSt.Data += 4;
//                                // Decrement data len.
//                                if(HexRecordSt.RecDataLen > 3)
//                                {
//                                    HexRecordSt.RecDataLen -= 4;
//                                }
//                                else
//                                {
//                                    HexRecordSt.RecDataLen = 0;
//                                }
//                            }
//                            while (!PLIB_NVM_FlashWriteCycleHasCompleted(NVM_ID_0));
//                            if (PLIB_NVM_WriteOperationHasTerminated(NVM_ID_0))
//                            {
//                                SYS_DEBUG_BreakPoint();
//                                APP_NVMClearError();
//                                return HEX_REC_PGM_ERROR;
//                            }
//                                // Assert on error. This must be caught during debug phase.
//    //                            ASSERT(Result==0);
//                        }
//                        else    // Out of boundaries. Adjust and move on.
//                        {
//                            // Increment the address.
//                            HexRecordSt.Address += 4;
//                            // Increment the data pointer.
//                            HexRecordSt.Data += 4;
//                            // Decrement data len.
//                            if(HexRecordSt.RecDataLen > 3)
//                            {
//                                HexRecordSt.RecDataLen -= 4;
//                            }
//                            else
//                            {
//                                HexRecordSt.RecDataLen = 0;
//                            }
//                        }
//                    }
                    break;

                case EXT_SEG_ADRS_RECORD:  // Record Type 02, defines 4th to 19th bits of the data address.
                    HexRecordSt.ExtSegAddress >>= (HexRecordSt.Data[0]<<12) + (HexRecordSt.Data[1]<<4);
                    
                    // Reset linear address.
                    HexRecordSt.ExtLinAddress = 0;
                    break;

                case EXT_LIN_ADRS_RECORD:   // Record Type 04, defines 16th to 31st bits of the data address.
                    HexRecordSt.ExtLinAddress = (HexRecordSt.Data[0]<<24) + (HexRecordSt.Data[1]<<16);

                    // Reset segment address.
                    HexRecordSt.ExtSegAddress = 0;
                    break;

                case END_OF_FILE_RECORD:  //Record Type 01, defines the end of file record.
                    APP_BuffersFinish();
                    BSP_LEDOn(BSP_LED_3);
                    HexRecordSt.ExtSegAddress = 0;
                    HexRecordSt.ExtLinAddress = 0;
                    break;
                default:
                    HexRecordSt.ExtSegAddress = 0;
                    HexRecordSt.ExtLinAddress = 0;
                    break;
            }
            
        //Determine next record starting point.
        nextRecStartPt += HexRecordSt.RecDataLen + 5;

        // Decrement total hex record length by length of current record.
        _Len -= HexRecordSt.RecDataLen + 5;// _Len - nextRecStartPt;

        }
    }//while(1)

    state = 0;

    if ( (HexRecordSt.RecType == DATA_RECORD) || (HexRecordSt.RecType == EXT_SEG_ADRS_RECORD)
            || (HexRecordSt.RecType == EXT_LIN_ADRS_RECORD) || (HexRecordSt.RecType == END_OF_FILE_RECORD) )
    {
        return (HEX_REC_NORMAL);
    }
    else
    {
        return (HEX_REC_UNKNOW_TYPE);
    }

}
