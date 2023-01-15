/*******************************************************************************
  Graphics GIF Image Decoder

  Company:
    Microchip Technology Inc.

  File Name:
    gif_image.h

  Summary:
    This implements the gif image decoding of the primitive layer.

  Description:
    Refer to Microchip Graphics Library for complete documentation of the
    GIF Image Decoder.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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
//DOM-IGNORE-END

#ifndef _GIFIMAGE_H
#define _GIFIMAGE_H

/************************************************************************
 * Section:  Includes
 *******************************************************************************/
#include "system_config.h"
#include "gfx/gfx.h"
#include "gfx/gfx_image_decoder.h"

#ifdef __cplusplus
    extern "C" {
#endif
        
// One function must be implemented in the driver:

/************************************************************************
* Function: void FunctionToReadArrayFromMemory(DWORD address, BYTE* pData, nCount)                                                                     
*
* Overview: this function reads data into buffer specified
*                                                                       
* Input: memory address, pointer to the data buffer, data count
*                                                                       
************************************************************************/ 

#ifdef IMG_SUPPORT_GIF

    #ifdef SYS_FS_MEDIA_NUMBER
        #define USE_GIF_FS_EXTERNAL
    #endif

#endif        

/*******************************************************************************
Function:       void* GIFfopen(void* gifImage)

Precondition:   None

Overview:       This function sets up the variables to emulate a file
                using constant data in internal flash or external memory

Input:          pointer to GIF file image

Output:         pointer to GIF file image
*******************************************************************************/
void                        *GIFfopen(void *gifImage);

/*******************************************************************************
Function:       void GIFInit()

Precondition:   None

Overview:       This function initializes GIF decoder

Input:          None

Output:         None
*******************************************************************************/
void                        GIFInit(void);

#ifdef __cplusplus
    }
#endif
    
#endif // _GIFIMAGE
