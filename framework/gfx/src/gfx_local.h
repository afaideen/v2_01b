/*******************************************************************************
 Module for Microchip Graphics Library

  Company:
    Microchip Technology Inc.

  File Name:
    gfx_local.h

  Summary:
    Internal interface header for the Graphics Library Object.

  Description:
    This header file establishes the graphics object combining the object
    and primitive layers.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _GFX_LOCAL_H
// DOM-IGNORE-BEGIN
    #define _GFX_LOCAL_H
// DOM-IGNORE-END

#include "system_config.h"
#include "gfx/src/gfx_primitive_local.h"
//#include "gfx/src/gfx_gol_local.h"

#ifdef __cplusplus
    extern "C" {
#endif
        
//// *****************************************************************************
///* GFX Library Instance Object
//
//  Summary:
//    Defines the object required for interaction with the GFX Library.
//
//  Description:
//    This defines the object required for interaction with the GFX Library.
//    This object exists once per application instance of the library.
//
//  Remarks:
//    None.
//*/
//
//typedef struct _GFX_OBJ
//{
//
//   /* Required : The status of the driver */
//    SYS_STATUS                                              status;
//
//    // /* Object Index */
//    SYS_MODULE_INDEX                                        objIndex;
//
//    /* Required : Flag to indicate in use  */
//    bool                                                    inUse;
//
//    /* Flag to indicate that PMP is used in exclusive access mode */
//    bool                                                    IsExclusive;
//
//    /* Optional: number of clients possible with the softwsare instance */
//    uint8_t                                                 numClients;
//
//    GFX_Primitive_DATA                                      primData;
//
//    GFX_GOL_OBJ                                             golObj;
//
//} GFX_OBJ;
//
//
//GFX_OBJ                                          gGfxObj[2];

#ifdef __cplusplus
    }
#endif
    
#endif // _GFX_LOCAL_H
