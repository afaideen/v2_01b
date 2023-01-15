/*******************************************************************************
  Graphics BMP Image Decoder

  Company:
    Microchip Technology Inc.

  File Name:
    bmp_image.c

  Summary:
    This implements the bmp image decoding of the primitive layer.

  Description:
    Refer to Microchip Graphics Library for complete documentation of the
    BMP Image Decoder.
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

#include "system_config.h"
#include "system_definitions.h"
#include "gfx/gfx_types_resource.h"
#include "gfx/bmp_image.h"
#ifdef USE_BMP_FS_EXTERNAL
#include "system/fs/sys_fs.h"
#endif
/*******************************************************************************
    GLOBAL VARIABLES
*******************************************************************************/
uint32_t               _bmpImageIndex;    // pointer to current location in image
IMG_FILE_SYSTEM_API _bmpFileApi;       // structure with data access functions pointers

/*******************************************************************************
    LOCAL FUNCTIONS PROTOTYPES
*******************************************************************************/
size_t              BMPfread(void *ptr, size_t size, size_t n, void *stream);  // read form image
bool                 BMPfseek(void *stream, long offset, int whence);           // seek in image
bool                 BMPfeof(void *stream); // eof of image

/*******************************************************************************
Function:       void BMPInit()

Precondition:   None

Overview:       This function initializes BMP decoder

Input:          None

Output:         None
*******************************************************************************/
void BMPInit(void)
{
    _bmpFileApi.pFread = BMPfread;
    _bmpFileApi.pFseek = BMPfseek;
    _bmpFileApi.pFtell = NULL; // not used by BMP decoder;
    _bmpFileApi.pFeof = BMPfeof;
}

/*******************************************************************************
Function:       size_t BMPfread(void *ptr, size_t size, size_t n, void* file)

Precondition:   None

Overview:       This function emulates file system's fread() on constant data
                in internal flash or external memory

Input:          Data buffer, size of data chunk, size of data type, file pointer

Output:         Number of bytes copied
*******************************************************************************/
size_t BMPfread(void *ptr, size_t size, size_t n, void *file)
{
    size_t count = 0;
    int type;
#ifdef USE_BMP_FS_EXTERNAL
    uint32_t address = 0;
#endif
    type = *((GFX_RESOURCE *)file);

    switch(type & GFX_MEM_MASK)
    {
    #ifdef USE_BMP_FS_EXTERNAL
        case GFX_RESOURCE_MEMORY_FS:
            
            count = size * n;            
            address = ((GFX_RESOURCE_HDR *)file)->resource.image.location.extAddress;
            
            if (SYS_FS_FileSeek(address, _bmpImageIndex, SYS_FS_SEEK_SET) == -1)
            {
                return 0;
            }

            if (SYS_FS_FileRead(address, ptr, count) == -1)
            {
                return 0;
            }        

            _bmpImageIndex += count;
            break;
    #endif
        default:
            return 0;
    }

    return count;
}

/*******************************************************************************
Function:       int BMPfseek(void* stream, long offset, int whence)

Precondition:   None

Overview:       This function emulates file system's fseek() on constant data
                in internal flash or external memory. BMP Decoder uses ONLY absolute and relative offsets.

Input:          File pointer, offset and the referance position

Output:         TRUE if success
*******************************************************************************/
bool BMPfseek(void *file, long offset, int whence)
{
    if(whence == 0)
    {
        _bmpImageIndex = offset;        
    }
    else if(whence == 1)
    {
        _bmpImageIndex += offset;        
    }
    else
    {
        return false; // BMP Decoder uses ONLY absolute and relative offsets
    }

    return true;
}

/*******************************************************************************
Function:       int BMPfeof(void *stream)

Precondition:   None

Overview:       This function emulates file system's feof() on constant data
                in Flash. BMP Decoder doesn't use EOF.

Input:          File pointer

Output:         True if end of file is reached. BMP Decoder doesn't use EOF.
                FALSE is returned.
*******************************************************************************/
bool BMPfeof(void *file)
{
#ifdef USE_BMP_FS_EXTERNAL
    uint32_t address = ((GFX_RESOURCE_HDR *)file)->resource.image.location.extAddress;

    if (SYS_FS_FileEOF(address) == 1)
    {
        return true;
    }
#endif
    return false; // BMP Decoder doesn't use File System
}

/*******************************************************************************
Function:       void* BMPfopen(void* bmpImage)

Precondition:   None

Overview:       This function sets up the variables to emulate a file
                using constant data in internal flash or external memory

Input:          pointer to BMP file image

Output:         pointer to BMP file image
*******************************************************************************/
void *BMPfopen(void *file)
{
    _bmpImageIndex = 0;
    return file;
}
