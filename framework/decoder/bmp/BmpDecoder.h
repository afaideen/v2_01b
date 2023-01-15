#ifndef __BMPDECODER_H__
 #define __BMPDECODER_H__

/******************************************************************************
* File Name:        BmpDecoder.h
* Dependencies:    Image decoding library; project requires File System library
* Processor:       PIC24/dsPIC30/dsPIC33/PIC32MX
* Compiler:        C30 v2.01/C32 v0.00.18
* Company:         Microchip Technology Inc.

 * Software License Agreement
 *
 * Copyright © 2008 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).  
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY
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

Author                 Date           Comments
--------------------------------------------------------------------------------
Pradeep Budagutta    03-Mar-2008    First release
*******************************************************************************/

/*************************/
/**** DATA STRUCTURES ****/
/*************************/
typedef struct _BMPDECODER
{
        IMG_FILE *pImageFile;                       /* Image file pointer */
        uint32_t lWidth;
        uint32_t lHeight;
        uint32_t lImageOffset;
        uint16_t wPaletteEntries;
        uint8_t bBitsPerPixel;
        uint8_t bHeaderType;
        uint8_t blBmMarkerFlag : 1;
        uint8_t blCompressionType : 3;
        uint8_t bNumOfPlanes : 3;
        uint8_t b16bit565flag : 1;
        uint8_t aPalette[256][3]; /* Each palette entry has RGB */
} BMPDECODER;

/* Function prototype */
/* This function must be called after setting proper values in the global variables of ImageDecoder.c */
uint8_t BMP_bDecode(IMG_FILE *pFile);
int8_t BDEC_bInterpretResourceHeader(GFX_RESOURCE_HDR* header);
void BDEC_vResetData(BMPDECODER *pBmpDec);
uint8_t BDEC_bReadHeader(BMPDECODER *pBmpDec);

#endif
