/*****************************************************************************
* Microchip Graphics Library
* Graphics Display Designer (GDD) Template
*****************************************************************************

* Dependencies:    See INCLUDES section below
* Processor:       PIC24F, PIC24H, dsPIC, PIC32
* Compiler:        MPLAB C30 V3.22, MPLAB C32 V1.05b
* Linker:          MPLAB LINK30, MPLAB LINK32
* Company:         Microchip Technology Incorporated
*
* Software License Agreement
*
* Copyright (c) 2010 Microchip Technology Inc.  All rights reserved.
* Microchip licenses to you the right to use, modify, copy and distribute
* Software only when embedded on a Microchip microcontroller or digital
* signal controller, which is integrated into your product or third party
* product (pursuant to the sublicense terms in the accompanying license
* agreement).
*
* You should refer to the license agreement accompanying this Software
* for additional information regarding your rights and obligations.
*
* SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
* OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
* PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
* OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,

* BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
* DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
* INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
* COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
* CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
* OR OTHER SIMILAR COSTS.
*
* Author               Date        Comment
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*
*****************************************************************************/
#ifndef FLASH_CLIENT_H_
#define FLASH_CLIENT_H_

#include "app.h" //
#include "btl_types.h"

typedef enum {
    REC_FLASHED = 0,
    REC_NOT_FOUND,
    REC_FOUND_BUT_NOT_FLASHED
} T_REC_STATUS;

typedef struct
{
    uint8_t *start;
    uint8_t len;
    T_REC_STATUS status;
}T_REC;

typedef struct
{
	uint8_t RecDataLen;
	BTL_DWORD_VAL Address;
	uint8_t RecType;
	uint8_t* Data;
	uint8_t CheckSum;
	BTL_DWORD_VAL ExtSegAddress;
	BTL_DWORD_VAL ExtLinAddress;
}T_HEX_RECORD;


#define DATA_RECORD         0
#define END_OF_FILE_RECORD  1
#define EXT_SEG_ADRS_RECORD 2
#define EXT_LIN_ADRS_RECORD 4
#define APP_FLASH_BASE_ADDRESS 0x0
#define APP_FLASH_END_ADDRESS  0x1FFFF

void FLH_ProgramHexRecord(uint8_t* HexRecord, int32_t totalLen);
void ConvertAsciiToHex(uint8_t* asciiRec, uint8_t* hexRec);
void processRecord(void);
void processSize(void);
void FLH_OpenResFile(void);
void FLH_ReadFile(void);
void FLH_processBuffer(size_t numBytes);

DRV_SQI_INIT_DATA sqiInitData;


extern FLH_DATA  flashData;
extern bool     coreupdate;

#endif /* FLASH_CLIENT_H_ */


