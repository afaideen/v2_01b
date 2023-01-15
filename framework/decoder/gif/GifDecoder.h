#ifndef __GIFDECODER_H__
 #define __GIFDECODER_H__
/******************************************************************************
* File Name:        GifDecoder.h
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
Pradeep Budagutta    14-Mar-2008    First release
*******************************************************************************/

/*************************/
/**** DATA STRUCTURES ****/
/*************************/
typedef struct _GIFDECODER
{
	IMG_FILE *pImageFile;                       /* Image file pointer */
	uint16_t wImageWidth;
	uint16_t wImageHeight;
	uint16_t wImageX;
	uint16_t wImageY;
	uint16_t wScreenWidth;
	uint16_t wScreenHeight;
	uint16_t wGlobalPaletteEntries;
	uint16_t wLocalPaletteEntries;
    bool hasTransparentColor;
    uint8_t bTransparentColorIndex;
	uint8_t bBgColorIndex;
	uint8_t bPixelAspectRatio;
	uint8_t blGifMarkerFlag : 1;
	uint8_t blGloabalColorTableFlag : 1;
	uint8_t blLocalColorTableFlag : 1;
	uint8_t blInterlacedFlag : 1;
	uint8_t blFirstcodeFlag : 1;
	uint8_t bInterlacePass : 3;
#if GIF_USE_16_BITS_PER_PIXEL == 0
	uint8_t aPalette[256][3]; /* Each palette entry has RGB */
#else
	uint16_t awPalette[256]; /* Each palette entry has RGB */
#endif
	/* For decoding */
	uint8_t abSymbol[4096];

#if GIF_CRUSH_PREV_SYMBOL_PTR_TABLE == 0        
	uint16_t awPrevSymbolPtr[4096];
#else
	uint16_t awPrevSymbolPtr[(4096 * 3) / 4];
#endif

	uint16_t wInitialSymbols;
	uint16_t wMaxSymbol;
	uint8_t bInitialSymbolBits;
	uint8_t bMaxSymbolBits;
	uint32_t lGlobalColorTablePos;
	/* Work memory */
	uint8_t bWorkBits;
	uint8_t bRemainingDataInBlock;
	uint8_t bRemainingBits;
	uint16_t wCurrentX;
	uint16_t wCurrentY;
} GIFDECODER;


/* User configuration */
#define GIF_CRUSH_PREV_SYMBOL_PTR_TABLE     1 /* If 1, this saves 2KB of RAM but requires more time to decode */

#ifdef IMG_USE_ONLY_565_GRAPHICS_DRIVER_FOR_OUTPUT
    #define GIF_USE_16_BITS_PER_PIXEL           1 /* If this is 1, then 16 bits/pixel is used and hence requires 256 Bytes less RAM */
#else
    #define GIF_USE_16_BITS_PER_PIXEL           0
#endif

/* User configuration */

/* Function prototype */
/* This function must be called after setting proper values in the global variables of ImageDecoder.c */
uint8_t GIF_bDecode(IMG_FILE *pFile);
int8_t GIF_bInterpretResourceHeader(GFX_RESOURCE_HDR* header);
void GIF_vResetData(GIFDECODER *pGifDec);
void GIF_vPutPrevCode(GIFDECODER *pGifDec, uint16_t wAddress, uint16_t wCode);
uint16_t GIF_wGetPrevCode(GIFDECODER *pGifDec, uint16_t wAddress);
void GIF_vInitializeTable(GIFDECODER *pGifDec);
void GIF_vReadColorTable(GIFDECODER *pGifDec, uint16_t wNumberOfEntries);
uint8_t GIF_bReadHeader(GIFDECODER *pGifDec);
uint8_t GIF_bReadNextImageDescriptor(GIFDECODER *pGifDec);
uint16_t GIF_wGetNextByte(GIFDECODER *pGifDec);
uint16_t GIF_wGetNextSymbol(GIFDECODER *pGifDec);
void GIF_vPaintData(GIFDECODER *pGifDec, uint8_t bData);
void GIF_vDisplayCode(GIFDECODER *pGifDec, uint16_t wCode);
uint8_t GIF_bTraceFirstData(GIFDECODER *pGifDec, uint16_t wCode);
void GIF_vExecuteClearCode(GIFDECODER *pGifDec);
uint8_t GIF_bDecodeNextImage(GIFDECODER *pGifDec);

#endif
