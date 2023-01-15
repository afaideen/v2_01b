#define __BMPDECODER_C__
/******************************************************************************

* File Name:        BmpDecoder.c
* Dependencies:    Image decoding library; project requires File System library
* Processor:       PIC24/dsPIC30/dsPIC33/PIC32MX
* Compiler:        C30 v2.01/C32 v0.00.18
* Company:         Microchip Technology, Inc.

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

#include "gfx/gfx_image_decoder.h"
#include "gfx/bmp_image.h"

#ifdef IMG_SUPPORT_BMP

/**************************/
/******* FUNCTIONS  *******/
/**************************/

int8_t BDEC_bInterpretResourceHeader(GFX_RESOURCE_HDR* header)
{
	BMPDECODER BMP_BmpDecoder;

	if (header == NULL)
		return -1;

	BMPfopen(header);
	BDEC_vResetData(&BMP_BmpDecoder);
	BMP_BmpDecoder.pImageFile = header;

	if (BDEC_bReadHeader(&BMP_BmpDecoder) != 0)
	{
		return(1);
	}

	header->resource.image.height = BMP_BmpDecoder.lHeight;
	header->resource.image.width = BMP_BmpDecoder.lWidth;

	return 0;
}

/*******************************************************************************
Function:       void BDEC_vResetData(BMPDECODER *pBmpDec)

Precondition:   None

Overview:       This function resets the variables so that new Bitmap image
                can be decoded

Input:          Bitmap decoder's data structure

Output:         None
*******************************************************************************/
void BDEC_vResetData(BMPDECODER *pBmpDec)
{
    pBmpDec->pImageFile = NULL;
    pBmpDec->lWidth = 0;
    pBmpDec->lHeight = 0;
    pBmpDec->lImageOffset = 0;
    pBmpDec->wPaletteEntries = 0;
    pBmpDec->bBitsPerPixel = 0;
    pBmpDec->bHeaderType = 0;
    pBmpDec->blBmMarkerFlag = 0;
    pBmpDec->blCompressionType = 0;
    pBmpDec->bNumOfPlanes = 0;
    pBmpDec->b16bit565flag = 0;    
}

/*******************************************************************************
Function:       uint8_t BDEC_bReadHeader(BMPDECODER *pBmpDec)

Precondition:   None

Overview:       This function reads the Bitmap file header and 
                fills the data structure

Input:          Bitmap decoder's data structure

Output:         Error code - '0' means no error
*******************************************************************************/
uint8_t BDEC_bReadHeader(BMPDECODER *pBmpDec)
{
        uint8_t bByte1, bByte2;
        uint16_t wWord;
        uint32_t lLong;

        IMG_FREAD(&bByte1, sizeof(bByte1), 1, pBmpDec->pImageFile);  /* Marker */
        IMG_FREAD(&bByte2, sizeof(bByte2), 1, pBmpDec->pImageFile);  /* Marker */
        
        if(bByte1 == 'B' && bByte2 == 'M')
        {
                  pBmpDec->blBmMarkerFlag = 1;
        }
        else
        {
                  return(100);
        }

        IMG_FREAD(&lLong, sizeof(lLong), 1, pBmpDec->pImageFile);  /* File length */
        IMG_FREAD(&wWord, sizeof(wWord), 1, pBmpDec->pImageFile);  /* Reserved */
        IMG_FREAD(&wWord, sizeof(wWord), 1, pBmpDec->pImageFile);  /* Reserved */

        IMG_FREAD(&lLong, sizeof(lLong), 1, pBmpDec->pImageFile);  /* Image offset */
        pBmpDec->lImageOffset = lLong;

        IMG_FREAD(&lLong, sizeof(lLong), 1, pBmpDec->pImageFile);  /* Header length */
        pBmpDec->bHeaderType = (uint8_t)lLong;

        if(pBmpDec->bHeaderType >= 40)
        {
                  IMG_FREAD(&lLong, sizeof(lLong), 1, pBmpDec->pImageFile);  /* Image Width */
                  pBmpDec->lWidth = lLong;

                  IMG_FREAD(&lLong, sizeof(lLong), 1, pBmpDec->pImageFile);  /* Image Height */
                  pBmpDec->lHeight = lLong;

                  IMG_FREAD(&wWord, sizeof(wWord), 1, pBmpDec->pImageFile);  /* Number of Planes */
                  pBmpDec->bNumOfPlanes = (uint8_t)wWord;

                  IMG_FREAD(&wWord, sizeof(wWord), 1, pBmpDec->pImageFile);  /* Bits per Pixel */
                  pBmpDec->bBitsPerPixel = (uint8_t)wWord;

                  IMG_FREAD(&lLong, sizeof(lLong), 1, pBmpDec->pImageFile);  /* Compression info */
                  pBmpDec->blCompressionType = (uint8_t)lLong;

                  IMG_FREAD(&lLong, sizeof(lLong), 1, pBmpDec->pImageFile);  /* Image length */
                  IMG_FREAD(&lLong, sizeof(lLong), 1, pBmpDec->pImageFile);  /* xPixels per metre */
                  IMG_FREAD(&lLong, sizeof(lLong), 1, pBmpDec->pImageFile);  /* yPixels per metre */

                  IMG_FREAD(&lLong, sizeof(lLong), 1, pBmpDec->pImageFile);  /* Palette entries */
                  pBmpDec->wPaletteEntries = (uint16_t)lLong;

                  if(pBmpDec->wPaletteEntries == 0)
                  {
                          uint16_t wTemp = (uint16_t)(pBmpDec->lImageOffset - 14 - 40)/4;
                          if(wTemp > 0)
                          {
                                   pBmpDec->wPaletteEntries = wTemp; /* This is because of a bug in MSPAINT */
                          }
                  }

                  IMG_FREAD(&lLong, sizeof(lLong), 1, pBmpDec->pImageFile);  /* Important colors */
                  if(pBmpDec->bBitsPerPixel == 16 && pBmpDec->bHeaderType > 40)
                  {
                          IMG_FREAD(&lLong, sizeof(lLong), 1, pBmpDec->pImageFile);  /* Red mask */
                          if((uint16_t)lLong == 0xF800)
                          {
                                     pBmpDec->b16bit565flag = 1;
                          }
                  }

                  IMG_FSEEK(pBmpDec->pImageFile, pBmpDec->bHeaderType + 14, 0);

                  if(pBmpDec->wPaletteEntries <= 256)
                  {
                          uint16_t wCounter;
                          for(wCounter = 0; wCounter < pBmpDec->wPaletteEntries; wCounter++)
                          {
                                     IMG_FREAD(&pBmpDec->aPalette[wCounter][2], sizeof(uint8_t), 1, pBmpDec->pImageFile); /* R */
                                     IMG_FREAD(&pBmpDec->aPalette[wCounter][1], sizeof(uint8_t), 1, pBmpDec->pImageFile); /* G */
                                     IMG_FREAD(&pBmpDec->aPalette[wCounter][0], sizeof(uint8_t), 1, pBmpDec->pImageFile); /* B */
                                     IMG_FREAD(&wWord, sizeof(uint8_t), 1, pBmpDec->pImageFile); /* Dummy */
                          }
                  }
        }
        return(0);
}

/*******************************************************************************
Function:       uint8_t BMP_bDecode(IMG_FILE *pFile)

Precondition:   None

Overview:       This function decodes and displays a Bitmap image

Input:          Image file

Output:         Error code - '0' means no error
*******************************************************************************/
uint8_t BMP_bDecode(IMG_FILE *pFile)
{
        BMPDECODER BmpDec;
        uint16_t wX, wY;
        uint8_t bPadding;

    	BMPfopen(pFile);
        BDEC_vResetData(&BmpDec);
        BmpDec.pImageFile = pFile;
        BDEC_bReadHeader(&BmpDec);
        if(BmpDec.blBmMarkerFlag == 0 || BmpDec.bHeaderType < 40 || (BmpDec.blCompressionType != 0 && BmpDec.blCompressionType != 3))
        {
            IMG_vImageFail();
            return 100;
        }    
        IMG_wImageWidth = (uint16_t)BmpDec.lWidth;
        IMG_wImageHeight = (uint16_t)BmpDec.lHeight;
        IMG_vSetboundaries();

        IMG_FSEEK(pFile, BmpDec.lImageOffset, 0);
        if(BmpDec.wPaletteEntries == 0 && BmpDec.bBitsPerPixel == 8) /* Grayscale Image */
        {
                bPadding = (4 - (BmpDec.lWidth % 4))%4;
                for(wY = 0; wY < BmpDec.lHeight; wY++)
                {
                         IMG_vLoopCallback();
                         IMG_vCheckAndAbort();
                         for(wX = 0; wX < BmpDec.lWidth; wX++)
                         {
                                   uint8_t bY;
                                   IMG_FREAD(&bY, sizeof(bY), 1, BmpDec.pImageFile); /* Y */
                                   IMG_vSetColor(bY, bY, bY);
                                   IMG_vPutPixel(wX, BmpDec.lHeight - wY - 1);
                         }
                         for(wX = 0; wX < bPadding; wX++)
                         {
                                   uint8_t bValue;
                                   IMG_FREAD(&bValue, sizeof(bValue), 1, BmpDec.pImageFile);
                         }
                }
        }
        else if(BmpDec.bBitsPerPixel == 16) /* 16-bit Color Image */
        {
                bPadding = (4 - ((BmpDec.lWidth*2) % 4))%4;
                for(wY = 0; wY < BmpDec.lHeight; wY++)
                {
                         IMG_vLoopCallback();
                         IMG_vCheckAndAbort();
                         for(wX = 0; wX < BmpDec.lWidth; wX++)
                         {
                                   uint16_t wColor;
                                   uint8_t bR, bG, bB;
                                   IMG_FREAD(&wColor, sizeof(wColor), 1, BmpDec.pImageFile); /* RGB */
                                   if(BmpDec.b16bit565flag == 1)
                                   {
                                              bR = (wColor >> 11) << 3;
                                              bG = ((wColor & 0x07E0) >> 5) << 2;
                                              bB = (wColor & 0x001F) << 3;
                                   }
                                   else
                                   {
                                              bR = ((wColor & 0x7FFF) >> 10) << 3;
                                              bG = ((wColor & 0x03E0) >> 5) << 3;
                                              bB = (wColor & 0x001F) << 3;
                                   }
                                   IMG_vSetColor(bR, bG, bB);
                                   IMG_vPutPixel(wX, BmpDec.lHeight - wY - 1);
                         }
                         for(wX = 0; wX < bPadding; wX++)
                         {
                                   uint8_t bValue;
                                   IMG_FREAD(&bValue, sizeof(bValue), 1, BmpDec.pImageFile);
                         }
                }
        }
        else if(BmpDec.bBitsPerPixel == 24) /* True color Image */
        {
                bPadding = (4 - ((BmpDec.lWidth*3) % 4))%4;
                for(wY = 0; wY < BmpDec.lHeight; wY++)
                {
                         IMG_vLoopCallback();
                         IMG_vCheckAndAbort();
                         for(wX = 0; wX < BmpDec.lWidth; wX++)
                         {
                                   uint8_t bR, bG, bB;
                                   IMG_FREAD(&bB, sizeof(bB), 1, BmpDec.pImageFile); /* B */
                                   IMG_FREAD(&bG, sizeof(bG), 1, BmpDec.pImageFile); /* G */
                                   IMG_FREAD(&bR, sizeof(bR), 1, BmpDec.pImageFile); /* R */
                                   IMG_vSetColor(bR, bG, bB);
                                   IMG_vPutPixel(wX, BmpDec.lHeight - wY - 1);
                         }
                         for(wX = 0; wX < bPadding; wX++)
                         {
                                   uint8_t bValue;
                                   IMG_FREAD(&bValue, sizeof(bValue), 1, BmpDec.pImageFile);
                         }
                }
        }
        else if(BmpDec.wPaletteEntries != 0 && BmpDec.bBitsPerPixel == 1) /* B/W Image */
        {
                uint16_t wBytesPerRow = BmpDec.lWidth/8;
                uint8_t bAdditionalBitsPerRow = BmpDec.lWidth % 8;
                bPadding = (4 - ((wBytesPerRow + (bAdditionalBitsPerRow?1:0)) % 4))%4;
                for(wY = 0; wY < BmpDec.lHeight; wY++)
                {
                         uint8_t bBits, bValue;
                         IMG_vLoopCallback();
                         IMG_vCheckAndAbort();
                         for(wX = 0; wX < wBytesPerRow; wX++)
                         {
                                   IMG_FREAD(&bValue, sizeof(bValue), 1, BmpDec.pImageFile);
                                   
                                   for(bBits = 0; bBits < 8; bBits++)
                                   {
                                             uint8_t bIndex = (bValue & (0x80 >> bBits))?1:0;
                                             IMG_vSetColor(BmpDec.aPalette[bIndex][0], BmpDec.aPalette[bIndex][1], BmpDec.aPalette[bIndex][2]);
                                             IMG_vPutPixel(wX*8+bBits, BmpDec.lHeight - wY - 1);
                                   }
                         }
                         if(bAdditionalBitsPerRow > 0)
                         {
                                   IMG_FREAD(&bValue, sizeof(bValue), 1, BmpDec.pImageFile);
                                   
                                   for(bBits = 0; bBits < bAdditionalBitsPerRow; bBits++)
                                   {
                                             uint8_t bIndex = (bValue & (0x80 >> bBits))?1:0;
                                             IMG_vSetColor(BmpDec.aPalette[bIndex][0], BmpDec.aPalette[bIndex][1], BmpDec.aPalette[bIndex][2]);
                                             IMG_vPutPixel(wX*8+bBits, BmpDec.lHeight - wY - 1);
                                   }
                         }
                         for(wX = 0; wX < bPadding; wX++)
                         {
                                   uint8_t bValue;
                                   IMG_FREAD(&bValue, sizeof(bValue), 1, BmpDec.pImageFile);
                         }
                }
        }
        else if(BmpDec.wPaletteEntries != 0 && BmpDec.bBitsPerPixel == 4) /* 16 colors Image */
        {
                uint16_t wBytesPerRow = BmpDec.lWidth/2;
                uint8_t bAdditionalNibblePerRow = BmpDec.lWidth % 2;
                bPadding = (4 - ((wBytesPerRow + bAdditionalNibblePerRow) % 4))%4;
                for(wY = 0; wY < BmpDec.lHeight; wY++)
                {
                         IMG_vLoopCallback();
                         IMG_vCheckAndAbort();
                         for(wX = 0; wX < wBytesPerRow; wX++)
                         {
                                   uint8_t bIndex, bValue;
                                   IMG_FREAD(&bValue, sizeof(bValue), 1, BmpDec.pImageFile);
                                   bIndex = bValue >> 4;
                                   IMG_vSetColor(BmpDec.aPalette[bIndex][0], BmpDec.aPalette[bIndex][1], BmpDec.aPalette[bIndex][2]);
                                   IMG_vPutPixel(wX*2, BmpDec.lHeight - wY - 1);
                                   bIndex = bValue & 0x0F;
                                   IMG_vSetColor(BmpDec.aPalette[bIndex][0], BmpDec.aPalette[bIndex][1], BmpDec.aPalette[bIndex][2]);
                                   IMG_vPutPixel(wX*2+1, BmpDec.lHeight - wY - 1);
                         }
                         if(bAdditionalNibblePerRow)
                         {
                                   uint8_t bIndex, bValue;
                                   IMG_FREAD(&bValue, sizeof(bValue), 1, BmpDec.pImageFile); /* Bits8 */
                                   bIndex = bValue >> 4;
                                   IMG_vSetColor(BmpDec.aPalette[bIndex][0], BmpDec.aPalette[bIndex][1], BmpDec.aPalette[bIndex][2]);
                                   IMG_vPutPixel(wX*2, BmpDec.lHeight - wY - 1);
                         }
                         for(wX = 0; wX < bPadding; wX++)
                         {
                                   uint8_t bValue;
                                   IMG_FREAD(&bValue, sizeof(bValue), 1, BmpDec.pImageFile);
                         }
                }
        }
        else if(BmpDec.wPaletteEntries != 0 && BmpDec.bBitsPerPixel == 8) /* 256 colors Image */
        {
                bPadding = (4 - (BmpDec.lWidth % 4))%4;
                for(wY = 0; wY < BmpDec.lHeight; wY++)
                {
                         IMG_vLoopCallback();
                         IMG_vCheckAndAbort();
                         for(wX = 0; wX < BmpDec.lWidth; wX++)
                         {
                                   uint8_t bIndex;
                                   IMG_FREAD(&bIndex, sizeof(bIndex), 1, BmpDec.pImageFile);
                                   IMG_vSetColor(BmpDec.aPalette[bIndex][0], BmpDec.aPalette[bIndex][1], BmpDec.aPalette[bIndex][2]);
                                   IMG_vPutPixel(wX, BmpDec.lHeight - wY - 1);
                         }
                         for(wX = 0; wX < bPadding; wX++)
                         {
                                   uint8_t bValue;
                                   IMG_FREAD(&bValue, sizeof(bValue), 1, BmpDec.pImageFile);
                         }
                }
        }

        IMG_vImageComplete();
        return 0;
}

#endif /* #ifdef IMG_SUPPORT_BMP */

#undef __BMPDECODER_C__
