/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    drv_gfx_ssd1926.h

  Summary:
    Interface for the graphics library where the primitives are rendered and sent 
	to the graphics controller either external or internal

  Description:
    None
*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
 
#ifndef _DRV_GFX_SSD1926_H
    #define _DRV_GFX_SSD1926_H
// DOM-IGNORE-END

#include "driver/gfx/controller/drv_gfx_controller.h"

#ifdef __cplusplus
    extern "C" {
#endif
        
// *****************************************************************************
// *****************************************************************************
// Section: Data Types and Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* SSD1926 Driver Module Index Count

  Summary:
    Number of valid SSD1926 driver indices.

  Description:
    This constant identifies SSD1926 driver index definitions.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is device-specific.
*/

#define DRV_GFX_SSD1926_INDEX_COUNT     DRV_GFX_SSD1926_NUMBER_OF_MODULES


// *****************************************************************************
/*
  Structure: DRV_GFX_SSD1926_COMMAND

  Summary:
        Structure for the commands in the driver queue.

  Description:
        Structure for the commands in the driver queue.

  Input:
        address     - pixel address
        array       - pointer to array of pixel data
        data        - pixel color
*/
// *****************************************************************************

typedef struct
{
   uint32_t                     address;                 //whether or not the task is complete
   uint16_t                      *array;
   uint16_t                        data;
} DRV_GFX_SSD1926_COMMAND;

// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function: uint8_t DRV_GFX_SSD1926_SetReg(uint16_t index, uint8_t value)

  Summary:
    updates graphics controller register value (byte access)

  Description:
    none

  Input:
    index - register number 
    value - value to write to register

  Output:
    1 - call was not passed
    0 - call was passed
*/
uint16_t DRV_GFX_SSD1926_SetReg(uint16_t index, uint8_t value);

// *****************************************************************************
/*
  Function: uint8_t DRV_GFX_SSD1926_GetReg(uint16_t index, uint8_t *data)

  Summary:
    returns graphics controller register value (byte access)

  Description:
    none

  Input:
    index - register number 
    *data - array to store data

  Output:
    0 - when call was passed
*/
uint8_t  DRV_GFX_SSD1926_GetReg(uint16_t  index, uint8_t *data);

/*********************************************************************
  Function:
     DRV_GFX_SSD1926_Open(uint8_t instance)
    
  Summary:
    opens an instance of the graphics controller

  Description:
    none

  Return:
                  
  *********************************************************************/
DRV_HANDLE DRV_GFX_SSD1926_Open( const SYS_MODULE_INDEX index,
                                 const DRV_IO_INTENT intent );

// *****************************************************************************
/*
  Function: void DRV_GFX_SSD1926_Close( DRV_HANDLE handle )

  Summary:
    closes an instance of the graphics controller

  Description:
    none

  Input:
    instance of the driver

*/
void DRV_GFX_SSD1926_Close( DRV_HANDLE handle );

/*********************************************************************
  Function:
     void DRV_GFX_SSD1926_InterfaceSet( DRV_HANDLE handle, DRV_GFX_INTERFACE * interface )

  Summary:
    Returns the API of the graphics controller

  Description:
    none

  Return:

  *********************************************************************/
void DRV_GFX_SSD1926_InterfaceSet( DRV_HANDLE handle, DRV_GFX_INTERFACE * interface );

// *****************************************************************************
/*
  Function:
     void DRV_GFX_SSD1926_MaxXGet()

  Summary:
     Returns x extent of the display.

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    <code>

  Remarks:
*/
uint16_t DRV_GFX_SSD1926_MaxXGet();

// *****************************************************************************
/*
  Function:
     void GFX_MaxYGet()

  Summary:
     Returns y extent of the display.

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    <code>

  Remarks:
*/
uint16_t DRV_GFX_SSD1926_MaxYGet();


// *****************************************************************************
/*
  Function: void DRV_GFX_SSD1926_SetColor(GFX_COLOR color)

  Summary: Sets the color for the driver instance

  Description:
  
  Output: none

*/

void DRV_GFX_SSD1926_SetColor(GFX_COLOR color);

// *****************************************************************************
/*
  Function: void DRV_GFX_SSD1926_SetInstance(uint8_t instance)

  Summary: Sets the instance for the driver

  Description:
  
  Output: none

*/

void DRV_GFX_SSD1926_SetInstance(uint8_t instance);

// *****************************************************************************
/*
  Function: uint16_t DRV_GFX_SSD1926_PixelPut(short x, short y)

  Summary:
    outputs one pixel into the frame buffer at the x,y coordinate given

  Description:
    none

  Input:
        x,y - pixel coordinates
  Output:
    NULL - call not successful
    !NULL - address of the display driver queue command
*/
void DRV_GFX_SSD1926_PixelPut(uint16_t x, uint16_t y);

// *****************************************************************************
/*
  Function: void  DRV_GFX_SSD1926_PixelArrayPut(uint16_t *color, uint16_t x, uint16_t y, uint16_t count, uint16_t lineCount)

  Summary:
    outputs an array of pixels of length count starting at *color 

  Description:
    none

  Input:
          *color - start of the array
		  x - x coordinate of the start point.
		  y - y coordinate of the end point.
		  count - number of pixels
              lineCount - number of lines
  Output:
        NULL - call not successful
        !NULL - handle to the number of pixels remaining
*/
void DRV_GFX_SSD1926_PixelArrayPut(uint16_t *color, uint16_t x, uint16_t y, uint16_t count, uint16_t lineCount);

// *****************************************************************************
/*
  Function: uint16_t  DRV_GFX_SSD1926_PixelArrayGet(uint16_t *color, short x, short y, uint16_t count)

  Summary:
    gets an array of pixels of length count starting at *color 

  Description:
    none

  Input:
          instance - driver instance
          *color - start of the array
		  x - x coordinate of the start point.
		  y - y coordinate of the end point.
		  count - number of pixels
  Output:
         NULL - call not successful
         !NULL - address of the display driver queue command
*/ 
uint16_t* DRV_GFX_SSD1926_PixelArrayGet(uint16_t *color, uint16_t x, uint16_t y, uint16_t count);

// *****************************************************************************
/*
  Function: void DRV_GFX_SSD1926_BarFill(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)

  Summary:
    Hardware accelerated barfill function

  Description:
    see primitive BarFill
  
  Output:
    1 - call not successful (PMP driver busy)
    0 - call successful
*/
void DRV_GFX_SSD1926_BarFill(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom);

// *****************************************************************************
/*
  Function: SYS_MODULE_OBJ DRV_GFX_SSD1926_Initialize(const SYS_MODULE_INDEX   moduleIndex,
                                          const SYS_MODULE_INIT    * const moduleInit)
  Summary:
    resets LCD, initializes PMP

  Description:
    none

  Input:
        instance - driver instance
  Output:
    1 - call not successful (PMP driver busy)
    0 - call successful
*/
SYS_MODULE_OBJ DRV_GFX_SSD1926_Initialize(const SYS_MODULE_INDEX   moduleIndex,
                                          const SYS_MODULE_INIT    * const moduleInit);

// *****************************************************************************
/*
  Function: uint16_t DRV_GFX_SSD1926_Busy(uint8_t instance)

  Summary:
    Returns non-zero if LCD controller is busy 
          (previous drawing operation is not completed).

  Description:
    none

  Input:
          instance - driver instance
  Output:
         1 - busy
         0 - not busy
*/ 
uint16_t DRV_GFX_SSD1926_Busy(uint8_t instance);

/*************************************************************************

  Function:
      void DRV_GFX_SSD1926_Tasks(void)
    
  Summary:
    Task machine that renders the driver calls for the graphics library it
    must be called periodically to output the contents of its circular
    buffer                                                                
  *************************************************************************/
void DRV_GFX_SSD1926_Tasks(SYS_MODULE_OBJ object);

/**************************************************************************
  Function:
       SYS_STATUS DRV_GFX_SSD1926_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the SSD1926 driver module.

  Description:
    This function provides the current status of the SSD1926 driver module.

  Conditions:
    The DRV_GFX_SSD1926_Initialize function must have been called before calling
    this function.

  Input:
    object -  Driver object handle, returned from DRV_GFX_SSD1926_Initialize

  Return:
    SYS_STATUS_READY - Indicates that the driver is busy with a previous
    system level operation and cannot start another

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_GFX_SSD1926_Initialize
    SYS_STATUS          status;

    status = DRV_GFX_SSD1926_Status( object );
    if( SYS_STATUS_READY != status )
    {
        // Handle error
    }
    </code>
*/
SYS_STATUS DRV_GFX_SSD1926_Status ( SYS_MODULE_OBJ object );

//DOM-IGNORE-BEGIN
/*********************************************************************
* Overview: SSD1926 registers definitions.
*********************************************************************/
#define SSD1926_REG_PLL_CONFIG_0                    0x126
#define SSD1926_REG_PLL_CONFIG_1                    0x127
#define SSD1926_REG_PLL_CONFIG_2                    0x12b
#define SSD1926_REG_DISP_BUFFER_SIZE                0x01
#define SSD1926_REG_CONFIG_READBACK                 0x02
#define SSD1926_REG_REVISION_CODE                   0x03
#define SSD1926_REG_MEMCLK_CONFIG                   0x04
#define SSD1926_REG_PCLK_CONFIG                     0x05
#define SSD1926_REG_VCLK_CONFIG_0                   0x06
#define SSD1926_REG_VCLK_CONFIG_1                   0x07
#define SSD1926_REG_LUT_BLUE_WRITE_DATA             0x08
#define SSD1926_REG_LUT_GREEN_WRITE_DATA            0x09
#define SSD1926_REG_LUT_RED_WRITE_DATA              0x0a
#define SSD1926_REG_LUT_WRITE_ADDR                  0x0b
#define SSD1926_REG_LUT_BLUE_READ_DATA              0x0c
#define SSD1926_REG_LUT_GREEN_READ_DATA             0x0d
#define SSD1926_REG_LUT_RED_READ_DATA               0x0e
#define SSD1926_REG_LUT_READ_ADDR                   0x0f
#define SSD1926_REG_PANEL_TYPE                      0x10
#define SSD1926_REG_MOD_RATE                        0x11
#define SSD1926_REG_HORIZ_TOTAL_0                   0x12
#define SSD1926_REG_HORIZ_TOTAL_1                   0x13
#define SSD1926_REG_HDP                             0x14
#define SSD1926_REG_HDP_START_POS0                  0x16
#define SSD1926_REG_HDP_START_POS1                  0x17
#define SSD1926_REG_VERT_TOTAL0                     0x18
#define SSD1926_REG_VERT_TOTAL1                     0x19
#define SSD1926_REG_VDP0                            0x1c
#define SSD1926_REG_VDP1                            0x1d
#define SSD1926_REG_VDP_START_POS0                  0x1e
#define SSD1926_REG_VDP_START_POS1                  0x1f
#define SSD1926_REG_HSYNC_PULSE_WIDTH               0x20
#define SSD1926_REG_LLINE_PULSE_START_SUBPIXEL_POS  0x21
#define SSD1926_REG_HSYNC_PULSE_START_POS0          0x22
#define SSD1926_REG_HSYNC_PULSE_START_POS1          0x23
#define SSD1926_REG_VSYNC_PULSE_WIDTH               0x24
#define SSD1926_REG_VSYNC_PULSE_START_POS0          0x26
#define SSD1926_REG_VSYNC_PULSE_START_POS1          0x27
#define SSD1926_REG_POST_PROCESSING_SATURATION      0x2c
#define SSD1926_REG_POST_PROCESSING_BRIGHTNESS      0x2d
#define SSD1926_REG_POST_PROCESSING_CONTRAST        0x2e
#define SSD1926_REG_POST_PROCESSING_CTRL            0x2f
#define SSD1926_REG_FPFRAME_START_OFFSET0           0x30
#define SSD1926_REG_FPFRAME_START_OFFSET1           0x31
#define SSD1926_REG_FPFRAME_STOP_OFFSET0            0x34
#define SSD1926_REG_FPFRAME_STOP_OFFSET1            0x35
#define SSD1926_REG_LSHIFT_POLARITY                 0x38
#define SSD1926_REG_GPIO1_PULSE_START               0x3c
#define SSD1926_REG_GPIO1_PULSE_STOP                0x3e
#define SSD1926_REG_GPIO2_PULSE_DELAY               0x40
#define SSD1926_REG_LCD_SUBPIXEL_ALIGNMENT          0x42
#define SSD1926_REG_STN_COLOR_DEPTH                 0x45
#define SSD1926_REG_INTERRUPT_FLAG                  0x48
#define SSD1926_REG_INTERRUPT_ENABLE                0x4A
#define SSD1926_REG_DYN_DITHER_CONTROL              0x50
#define SSD1926_REG_DISPLAY_MODE                    0x70
#define SSD1926_REG_SPECIAL_EFFECTS                 0x71
#define SSD1926_REG_RGB_SETTING                     0x1a4
#define SSD1926_REG_MAIN_WIN_DISP_START_ADDR0       0x74
#define SSD1926_REG_MAIN_WIN_DISP_START_ADDR1       0x75
#define SSD1926_REG_MAIN_WIN_DISP_START_ADDR2       0x76
#define SSD1926_REG_MAIN_WIN_ADDR_OFFSET0           0x78
#define SSD1926_REG_MAIN_WIN_ADDR_OFFSET1           0x79
#define SSD1926_REG_FLOAT_WIN_DISP_START_ADDR0      0x7c
#define SSD1926_REG_FLOAT_WIN_DISP_START_ADDR1      0x7d
#define SSD1926_REG_FLOAT_WIN_DISP_START_ADDR2      0x7e
#define SSD1926_REG_FLOAT_WIN_ADDR_OFFSET0          0x80
#define SSD1926_REG_FLOAT_WIN_ADDR_OFFSET1          0x81
#define SSD1926_REG_FLOAT_WIN_X_START_POS0          0x84
#define SSD1926_REG_FLOAT_WIN_X_START_POS1          0x85
#define SSD1926_REG_FLOAT_WIN_Y_START_POS0          0x88
#define SSD1926_REG_FLOAT_WIN_Y_START_POS1          0x89
#define SSD1926_REG_FLOAT_WIN_X_END_POS0            0x8c
#define SSD1926_REG_FLOAT_WIN_X_END_POS1            0x8d
#define SSD1926_REG_FLOAT_WIN_Y_END_POS0            0x90
#define SSD1926_REG_FLOAT_WIN_Y_END_POS1            0x91
#define SSD1926_REG_POWER_SAVE_CONFIG               0xa0
#define SSD1926_REG_SOFTWARE_RESET                  0xa2
#define SSD1926_REG_SCRATCH_PAD0                    0xa4
#define SSD1926_REG_SCRATCH_PAD1                    0xa5
#define SSD1926_REG_GPIO_CONFIG0                    0xa8
#define SSD1926_REG_GPIO_CONFIG1                    0xa9
#define SSD1926_REG_GPIO_STATUS_CONTROL0            0xac
#define SSD1926_REG_GPIO_STATUS_CONTROL1            0xad
#define SSD1926_REG_PWM_CV_CLOCK_CONTROL            0xb0
#define SSD1926_REG_PWM_CV_CLOCK_CONFIG             0xb1
#define SSD1926_REG_CV_CLOCK_BURST_LENGTH           0xb2
#define SSD1926_REG_PWM_CLOCK_DUTY_CYCLE            0xb3
#define SSD1926_REG_PWM1_CLOCK_CONTROL              0xb4
#define SSD1926_REG_PWM1_CLOCK_CONFIG               0xb5
#define SSD1926_REG_PWM1_CLOCK_DUTY_CYCLE           0xb7
#define SSD1926_REG_PWM2_CLOCK_CONTROL              0xb8
#define SSD1926_REG_PWM2_CLOCK_CONFIG               0xb9
#define SSD1926_REG_PWM2_CLOCK_DUTY_CYCLE           0xbb
#define SSD1926_REG_PWM3_CLOCK_CONTROL              0xbc
#define SSD1926_REG_PWM3_CLOCK_CONFIG               0xbd
#define SSD1926_REG_PWM3_CLOCK_DUTY_CYCLE           0xbf
#define SSD1926_REG_CURSOR_FEATURE                  0xc0
#define SSD1926_REG_CURSOR1_BLINK_TOTAL0            0xc4
#define SSD1926_REG_CURSOR1_BLINK_TOTAL1            0xc5
#define SSD1926_REG_CURSOR1_BLINK_ON0               0xc8
#define SSD1926_REG_CURSOR1_BLINK_ON1               0xc9
#define SSD1926_REG_CURSOR1_MEM_START0              0xcc
#define SSD1926_REG_CURSOR1_MEM_START1              0xcd
#define SSD1926_REG_CURSOR1_MEM_START2              0xce
#define SSD1926_REG_CURSOR1_POSX0                   0xd0
#define SSD1926_REG_CURSOR1_POSX1                   0xd1
#define SSD1926_REG_CURSOR1_POSY0                   0xd4
#define SSD1926_REG_CURSOR1_POSY1                   0xd5
#define SSD1926_REG_CURSOR1_HORIZ_SIZE_0            0xd8
#define SSD1926_REG_CURSOR1_HORIZ_SIZE_1            0xd9
#define SSD1926_REG_CURSOR1_VERT_SIZE_0             0xdc
#define SSD1926_REG_CURSOR1_VERT_SIZE_1             0xdd
#define SSD1926_REG_CURSOR1_COL_IND1_0              0xe0
#define SSD1926_REG_CURSOR1_COL_IND1_1              0xe1
#define SSD1926_REG_CURSOR1_COL_IND1_2              0xe2
#define SSD1926_REG_CURSOR1_COL_IND1_3              0xe3
#define SSD1926_REG_CURSOR1_COL_IND2_0              0xe4
#define SSD1926_REG_CURSOR1_COL_IND2_1              0xe5
#define SSD1926_REG_CURSOR1_COL_IND2_2              0xe6
#define SSD1926_REG_CURSOR1_COL_IND2_3              0xe7
#define SSD1926_REG_CURSOR1_COL_IND3_0              0xe8
#define SSD1926_REG_CURSOR1_COL_IND3_1              0xe9
#define SSD1926_REG_CURSOR1_COL_IND3_2              0xea
#define SSD1926_REG_CURSOR1_COL_IND3_3              0xeb
#define SSD1926_REG_CURSOR2_BLINK_TOTAL0            0xec
#define SSD1926_REG_CURSOR2_BLINK_TOTAL1            0xed
#define SSD1926_REG_CURSOR2_BLINK_ON0               0xf0
#define SSD1926_REG_CURSOR2_BLINK_ON1               0xf1
#define SSD1926_REG_CURSOR2_MEM_START0              0xf4
#define SSD1926_REG_CURSOR2_MEM_START1              0xf5
#define SSD1926_REG_CURSOR2_MEM_START2              0xf6
#define SSD1926_REG_CURSOR2_POSX0                   0xf8
#define SSD1926_REG_CURSOR2_POSX1                   0xf9
#define SSD1926_REG_CURSOR2_POSY0                   0xfc
#define SSD1926_REG_CURSOR2_POSY1                   0xfd
#define SSD1926_REG_CURSOR2_HORIZ_SIZE_0            0x100
#define SSD1926_REG_CURSOR2_HORIZ_SIZE_1            0x101
#define SSD1926_REG_CURSOR2_VERT_SIZE_0             0x104
#define SSD1926_REG_CURSOR2_VERT_SIZE_1             0x105
#define SSD1926_REG_CURSOR2_COL_IND1_0              0x108
#define SSD1926_REG_CURSOR2_COL_IND1_1              0x109
#define SSD1926_REG_CURSOR2_COL_IND1_2              0x10a
#define SSD1926_REG_CURSOR2_COL_IND1_3              0x10b
#define SSD1926_REG_CURSOR2_COL_IND2_0              0x10c
#define SSD1926_REG_CURSOR2_COL_IND2_1              0x10d
#define SSD1926_REG_CURSOR2_COL_IND2_2              0x10e
#define SSD1926_REG_CURSOR2_COL_IND2_3              0x10f
#define SSD1926_REG_CURSOR2_COL_IND3_0              0x110
#define SSD1926_REG_CURSOR2_COL_IND3_1              0x111
#define SSD1926_REG_CURSOR2_COL_IND3_2              0x112
#define SSD1926_REG_CURSOR2_COL_IND3_3              0x113
#define SSD1926_REG_MAIN_REFLESH                    0x12c
#define SSD1926_REG_PCLK_FREQ_RATIO_0               0x158
#define SSD1926_REG_PCLK_FREQ_RATIO_1               0x159
#define SSD1926_REG_PCLK_FREQ_RATIO_2               0x15a
#define SSD1926_REG_DV_OP_MODE                      0x160
#define SSD1926_REG_DV_FRAME_SAMPLING               0x161
#define SSD1926_REG_DV_NFRAME_POS_0                 0x162
#define SSD1926_REG_DV_NFRAME_POS_1                 0x163
#define SSD1926_REG_DV_JHORI_SIZE_0                 0x164
#define SSD1926_REG_DV_JHORI_SIZE_1                 0x165
#define SSD1926_REG_DV_JVERT_SIZE_0                 0x168
#define SSD1926_REG_DV_JVERT_SIZE_1                 0x169
#define SSD1926_REG_DV_JMEM_STR_0                   0x16c
#define SSD1926_REG_DV_JMEM_STR_1                   0x16d
#define SSD1926_REG_DV_JMEM_STR_2                   0x16e
#define SSD1926_REG_DV_JMEM_STR_3                   0x16f
#define SSD1926_REG_DV_VHDEC_RATIO                  0x170
#define SSD1926_REG_DV_VVDEC_RATIO                  0x171
#define SSD1926_REG_DV_JHDEC_RATIO                  0x172
#define SSD1926_REG_DV_JVDEC_RATIO                  0x173
#define SSD1926_REG_DV_HORI_PERIOD_0                0x174
#define SSD1926_REG_DV_HORI_PERIOD_1                0x175
#define SSD1926_REG_DV_HORI_MAX_0                   0x17c
#define SSD1926_REG_DV_HORI_MAX_1                   0x17d
#define SSD1926_REG_DV_VERT_MAX_0                   0x180
#define SSD1926_REG_DV_VERT_MAX_1                   0x181
#define SSD1926_REG_DV_HCROP_STR_0                  0x184
#define SSD1926_REG_DV_HCROP_STR_1                  0x185
#define SSD1926_REG_DV_VCROP_STR_0                  0x188
#define SSD1926_REG_DV_VCROP_STR_1                  0x189
#define SSD1926_REG_DV_HCROP_SIZE_0                 0x18c
#define SSD1926_REG_DV_HCROP_SIZE_1                 0x18d
#define SSD1926_REG_DV_VCROP_SIZE_0                 0x190
#define SSD1926_REG_DV_VCROP_SIZE_1                 0x191
#define SSD1926_REG_DV_FRAME_PULSE_WIDTH            0x194
#define SSD1926_REG_DV_VMEM_STR_ADDR1_0             0x19c
#define SSD1926_REG_DV_VMEM_STR_ADDR1_1             0x19d
#define SSD1926_REG_DV_VMEM_STR_ADDR1_2             0x19e
#define SSD1926_REG_DV_VMEM_STR_ADDR1_3             0x19f
#define SSD1926_REG_DV_VMEM_STR_ADDR2_0             0x1a0
#define SSD1926_REG_DV_VMEM_STR_ADDR2_1             0x1a1
#define SSD1926_REG_DV_VMEM_STR_ADDR2_2             0x1a2
#define SSD1926_REG_DV_VMEM_STR_ADDR2_3             0x1a3
#define SSD1926_REG_DV_OFORMAT                      0x1a4
#define SSD1926_REG_DV_ALPHA                        0x1a5
#define SSD1926_REG_DV_SUBPIXEL_MODE                0x1a6
#define SSD1926_REG_DV_CSC_MODE                     0x1a8
#define SSD1926_REG_DV_Y                            0x1a9
#define SSD1926_REG_DV_CB                           0x1aa
#define SSD1926_REG_DV_CR                           0x1ab
#define SSD1926_REG_DV_TV_0                         0x1ac
#define SSD1926_REG_DV_TV_1                         0x1ad
#define SSD1926_REG_DV_TV_2                         0x1ae
#define SSD1926_REG_DV_ENB                          0x1af
#define SSD1926_REG_DV_DV0_START_ADDR_0             0x1b0
#define SSD1926_REG_DV_DV0_START_ADDR_1             0x1b1
#define SSD1926_REG_DV_DV0_START_ADDR_2             0x1b2
#define SSD1926_REG_DV_DV1_START_ADDR_0             0x1b4
#define SSD1926_REG_DV_DV1_START_ADDR_1             0x1b5
#define SSD1926_REG_DV_DV1_START_ADDR_2             0x1b6
#define SSD1926_REG_2D_1d0                      0x1d0
#define SSD1926_REG_2D_1d1                      0x1d1
#define SSD1926_REG_2D_1d2                      0x1d2
#define SSD1926_REG_2D_1d4                      0x1d4
#define SSD1926_REG_2D_1d5                      0x1d5
#define SSD1926_REG_2D_1d6                      0x1d6
#define SSD1926_REG_2D_1d8                      0x1d8
#define SSD1926_REG_2D_1d9                      0x1d9
#define SSD1926_REG_2D_1dc                      0x1dc
#define SSD1926_REG_2D_1dd                      0x1dd
#define SSD1926_REG_2D_1de                      0x1de
#define SSD1926_REG_2D_1e4                      0x1e4
#define SSD1926_REG_2D_1e5                      0x1e5
#define SSD1926_REG_2D_1e8                      0x1e8
#define SSD1926_REG_2D_1e9                      0x1e9
#define SSD1926_REG_2D_1ec                      0x1ec
#define SSD1926_REG_2D_1ed                      0x1ed
#define SSD1926_REG_2D_1f0                      0x1f0
#define SSD1926_REG_2D_1f1                      0x1f1
#define SSD1926_REG_2D_1f4                      0x1f4
#define SSD1926_REG_2D_1f5                      0x1f5
#define SSD1926_REG_2D_1f6                      0x1f6
#define SSD1926_REG_2D_1f8                      0x1f8
#define SSD1926_REG_2D_1f9                      0x1f9
#define SSD1926_REG_2D_1fc                      0x1fc
#define SSD1926_REG_2D_1fd                      0x1fd
#define SSD1926_REG_2D_1fe                      0x1fe
#define SSD1926_REG_2D_204                      0x204
#define SSD1926_REG_2D_205                      0x205
#define SSD1926_REG_2D_206                      0x206
#define SSD1926_REG_2D_208                      0x208
#define SSD1926_REG_2D_209                      0x209
#define SSD1926_REG_2D_214                      0x214
#define SSD1926_REG_2D_215                      0x215
#define SSD1926_REG_2D_218                      0x218
#define SSD1926_REG_2D_219                      0x219
#define SSD1926_REG_2D_220                      0x220
#define SSD1926_REG_2D_CMD_1                    0x1d0
#define SSD1926_REG_2D_CMD_2                    0x1d1
#define SSD1926_REG_2D_CMD_FIFO_STATUS          0x1d2
#define SSD1926_REG_2D_SRC_WND_START_ADDR0      0x1d4
#define SSD1926_REG_2D_SRC_WND_START_ADDR1      0x1d5
#define SSD1926_REG_2D_SRC_WND_START_ADDR2      0x1d6
#define SSD1926_REG_2D_SRC_WND_ADDR_OFFSET0     0x1d8
#define SSD1926_REG_2D_SRC_WND_ADDR_OFFSET1     0x1d9
#define SSD1926_REG_2D_SRC_WND_COLOR_MODE       0x1dc
#define SSD1926_REG_2D_DEST_WND_COLOR_MODE      0x1dd
#define SSD1926_REG_2D_SRC_WND_WIDTH0           0x1e4
#define SSD1926_REG_2D_SRC_WND_WIDTH1           0x1e5
#define SSD1926_REG_2D_SRC_WND_HEIGHT0          0x1e8
#define SSD1926_REG_2D_SRC_WND_HEIGHT1          0x1e9
#define SSD1926_REG_2D_DEST_WND_WIDTH0          0x1ec
#define SSD1926_REG_2D_DEST_WND_WIDTH1          0x1ed
#define SSD1926_REG_2D_DEST_WND_HEIGHT0         0x1f0
#define SSD1926_REG_2D_DEST_WND_HEIGHT1         0x1f1
#define SSD1926_REG_2D_DEST_WND_START_ADDR0     0x1f4
#define SSD1926_REG_2D_DEST_WND_START_ADDR1     0x1f5
#define SSD1926_REG_2D_DEST_WND_START_ADDR2     0x1f6
#define SSD1926_REG_2D_DEST_WND_ADDR_OFFSET0    0x1f8
#define SSD1926_REG_2D_DEST_WND_ADDR_OFFSET1    0x1f9
#define SSD1926_REG_2D_WRTIE_PATTERN0           0x1fc
#define SSD1926_REG_2D_WRTIE_PATTERN1           0x1fd
#define SSD1926_REG_2D_WRTIE_PATTERN2           0x1fe
#define SSD1926_REG_2D_BRUSH_WND_START_ADDR0    0x204
#define SSD1926_REG_2D_BRUSH_WND_START_ADDR1    0x205
#define SSD1926_REG_2D_BRUSH_WND_START_ADDR2    0x206
#define SSD1926_REG_2D_BRUSH_WND_ADDR_OFFSET0   0x208
#define SSD1926_REG_2D_BRUSH_WND_ADDR_OFFSET1   0x209
#define SSD1926_REG_2D_BRUSH_WND_WIDTH0         0x214
#define SSD1926_REG_2D_BRUSH_WND_WIDTH1         0x215
#define SSD1926_REG_2D_BRUSH_WND_HEIGHT0        0x218
#define SSD1926_REG_2D_BRUSH_WND_HEIGHT1        0x219
#define SSD1926_REG_2D_CMD_FIFO_INT_EN          0x21c
#define SSD1926_REG_2D_CMD_FIFO_INT_STATUS      0x21e
#define SSD1926_REG_2D_CMD_FIFO_FLAG            0x220
#define SSD1926_REG_2D_CMD_FIFO_IND             0x222
#define SSD1926_REG_I2C_DATA_OUT                0x230
#define SSD1926_REG_I2C_CTL                     0x231
#define SSD1926_REG_I2C_EN_OUT                  0x232
#define SSD1926_REG_I2C_BAUD                    0x233
#define SSD1926_REG_I2C_STATUS                  0x234
#define SSD1926_REG_I2C_INTERRUPT               0x235
#define SSD1926_REG_I2C_DATA_READY              0x236
#define SSD1926_REG_I2C_DATA_IN                 0x237
#define SSD1926_REG_Sub_mode                    0x250
#define SSD1926_REG_Sub_CLK_Divide              0x252
#define SSD1926_REG_Sub_Hori_Size               0x254
#define SSD1926_REG_Sub_Vert_Size               0x25c
#define SSD1926_REG_Sub_Type                    0x260
#define SSD1926_REG_Sub_Bpp                     0x261
#define SSD1926_REG_Sub_TFT_Start               0x262
#define SSD1926_REG_Sub_SCLK_Divide             0x263
#define SSD1926_REG_Sub_Ctl_0                   0x270
#define SSD1926_REG_Sub_Ctl_1                   0x271
#define SSD1926_REG_Sub_YVU                     0x268
#define SSD1926_REG_Sub_Y_Offset                0x269
#define SSD1926_REG_Sub_Cb_Offset               0x26a
#define SSD1926_REG_Sub_Cr_Offset               0x26b
#define SSD1926_REG_Sub_Data_0                  0x26c
#define SSD1926_REG_Sub_Data_1                  0x26d
#define SSD1926_REG_Sub_RS                      0x26e
#define SSD1926_REG_Sub_Start_Adr_0             0x274
#define SSD1926_REG_Sub_Start_Adr_1             0x275
#define SSD1926_REG_Sub_Start_Adr_2             0x276
#define SSD1926_REG_Sub_Offset_0                0x278
#define SSD1926_REG_Sub_Offset_1                0x279
#define SSD1926_REG_Sub_Ready                   0x27d
#define SSD1926_REG_LCD0_Rise_0                 0x2b0
#define SSD1926_REG_LCD0_Rise_1                 0x2b1
#define SSD1926_REG_LCD0_Fall_0                 0x2b4
#define SSD1926_REG_LCD0_Fall_1                 0x2b5
#define SSD1926_REG_LCD0_Period_0               0x2b8
#define SSD1926_REG_LCD0_Period_1               0x2b9
#define SSD1926_REG_LCD0_Ctl_0                  0x2bc
#define SSD1926_REG_LCD0_Ctl_1                  0x2bd
#define SSD1926_REG_LCD1_Rise_0                 0x2c0
#define SSD1926_REG_LCD1_Rise_1                 0x2c1
#define SSD1926_REG_LCD1_Fall_0                 0x2c4
#define SSD1926_REG_LCD1_Fall_1                 0x2c5
#define SSD1926_REG_LCD1_Period_0               0x2c8
#define SSD1926_REG_LCD1_Period_1               0x2c9
#define SSD1926_REG_LCD1_Ctl_0                  0x2cc
#define SSD1926_REG_LCD1_Ctl_1                  0x2cd
#define SSD1926_REG_LCD2_Rise_0                 0x2d0
#define SSD1926_REG_LCD2_Rise_1                 0x2d1
#define SSD1926_REG_LCD2_Fall_0                 0x2d4
#define SSD1926_REG_LCD2_Fall_1                 0x2d5
#define SSD1926_REG_LCD2_Period_0               0x2d8
#define SSD1926_REG_LCD2_Period_1               0x2d9
#define SSD1926_REG_LCD2_Ctl_0                  0x2dc
#define SSD1926_REG_LCD2_Ctl_1                  0x2dd
#define SSD1926_REG_LCD3_Rise_0                 0x2e0
#define SSD1926_REG_LCD3_Rise_1                 0x2e1
#define SSD1926_REG_LCD3_Fall_0                 0x2e4
#define SSD1926_REG_LCD3_Fall_1                 0x2e5
#define SSD1926_REG_LCD3_Period_0               0x2e8
#define SSD1926_REG_LCD3_Period_1               0x2e9
#define SSD1926_REG_LCD3_Ctl_0                  0x2ec
#define SSD1926_REG_LCD3_Ctl_1                  0x2ed
#define SSD1926_REG_LCD4_Rise_0                 0x2f0
#define SSD1926_REG_LCD4_Rise_1                 0x2f1
#define SSD1926_REG_LCD4_Fall_0                 0x2f4
#define SSD1926_REG_LCD4_Fall_1                 0x2f5
#define SSD1926_REG_LCD4_Period_0               0x2f8
#define SSD1926_REG_LCD4_Period_1               0x2f9
#define SSD1926_REG_LCD4_Ctl_0                  0x2fc
#define SSD1926_REG_LCD4_Ctl_1                  0x2fd
#define SSD1926_REG_LCD5_Rise_0                 0x300
#define SSD1926_REG_LCD5_Rise_1                 0x301
#define SSD1926_REG_LCD5_Fall_0                 0x304
#define SSD1926_REG_LCD5_Fall_1                 0x305
#define SSD1926_REG_LCD5_Period_0               0x308
#define SSD1926_REG_LCD5_Period_1               0x309
#define SSD1926_REG_LCD5_Ctl_0                  0x30c
#define SSD1926_REG_LCD5_Ctl_1                  0x30d
#define SSD1926_REG_LCD6_Rise_0                 0x310
#define SSD1926_REG_LCD6_Rise_1                 0x311
#define SSD1926_REG_LCD6_Fall_0                 0x314
#define SSD1926_REG_LCD6_Fall_1                 0x315
#define SSD1926_REG_LCD6_Period_0               0x318
#define SSD1926_REG_LCD6_Period_1               0x319
#define SSD1926_REG_LCD6_Ctl_0                  0x31c
#define SSD1926_REG_LCD6_Ctl_1                  0x31d
#define SSD1926_REG_LCD7_Rise_0                 0x320
#define SSD1926_REG_LCD7_Rise_1                 0x321
#define SSD1926_REG_LCD7_Fall_0                 0x324
#define SSD1926_REG_LCD7_Fall_1                 0x325
#define SSD1926_REG_LCD7_Period_0               0x328
#define SSD1926_REG_LCD7_Period_1               0x329
#define SSD1926_REG_LCD7_Ctl_0                  0x32c
#define SSD1926_REG_LCD7_Ctl_1                  0x32d
#define SSD1926_REG_FRC_FRAME_CTL               0x334
#define SSD1926_REG_FRC_ENABLE                  0x336
#define SSD1926_REG_JPEG_RESIZER_CTL            0x360
#define SSD1926_REG_JPEG_RESIZER_STARTX_0       0x364
#define SSD1926_REG_JPEG_RESIZER_STARTX_1       0x365
#define SSD1926_REG_JPEG_RESIZER_STARTY_0       0x366
#define SSD1926_REG_JPEG_RESIZER_STARTY_1       0x367
#define SSD1926_REG_JPEG_RESIZER_ENDX_0         0x368
#define SSD1926_REG_JPEG_RESIZER_ENDX_1         0x369
#define SSD1926_REG_JPEG_RESIZER_ENDY_0         0x36a
#define SSD1926_REG_JPEG_RESIZER_ENDY_1         0x36b
#define SSD1926_REG_JPEG_RESIZER_OP_0           0x36c
#define SSD1926_REG_JPEG_RESIZER_OP_1           0x36e
#define SSD1926_REG_JPEG_CTRL                   0x380
#define SSD1926_REG_JPEG_STATUS                 0x382
#define SSD1926_REG_JPEG_STATUS1                0x383
#define SSD1926_REG_JPEG_RAW_STATUS             0x384
#define SSD1926_REG_JPEG_RAW_STATUS1            0x385
#define SSD1926_REG_JPEG_INTR_CTL0              0x386
#define SSD1926_REG_JPEG_INTR_CTL1              0x387
#define SSD1926_REG_JPEG_START_STOP             0x38a
#define SSD1926_REG_JPEG_FIFO_CTL               0x3a0
#define SSD1926_REG_JPEG_FIFO_STATUS            0x3a2
#define SSD1926_REG_JPEG_FIFO_SIZE              0x3a4
#define SSD1926_REG_JPEG_ENCODE_SIZE_LIMIT_0    0x3B0
#define SSD1926_REG_JPEG_ENCODE_SIZE_LIMIT_1    0x3B1
#define SSD1926_REG_JPEG_ENCODE_SIZE_LIMIT_2    0x3B2
#define SSD1926_REG_JPEG_ENCODE_SIZE_RESULT_0   0x3b4
#define SSD1926_REG_JPEG_ENCODE_SIZE_RESULT_1   0x3b5
#define SSD1926_REG_JPEG_ENCODE_SIZE_RESULT_2   0x3b6
#define SSD1926_REG_JPEG_FILE_SIZE_0            0x3B8
#define SSD1926_REG_JPEG_FILE_SIZE_1            0x3B9
#define SSD1926_REG_JPEG_FILE_SIZE_2            0x3BA
#define SSD1926_REG_JPEG_DECODE_X_SIZE          0x3d8
#define SSD1926_REG_JPEG_DECODE_Y_SIZE          0x3dc
#define SSD1926_REG_JPEG_OP_MODE_ENC            0x400
#define SSD1926_REG_JPEG_OP_MODE                0x401
#define SSD1926_REG_JPEG_CMD                    0x402
#define SSD1926_REG_DRI_SETTING                 0x40a
#define SSD1926_REG_JPEG_DECODE_VALUE           0x404
#define SSD1926_REG_JPEG_Y_PIXEL_SIZE_0         0x40c
#define SSD1926_REG_JPEG_Y_PIXEL_SIZE_1         0x40d
#define SSD1926_REG_JPEG_X_PIXEL_SIZE_0         0x40e
#define SSD1926_REG_JPEG_X_PIXEL_SIZE_1         0x40f
#define SSD1926_REG_JPEG_SRC_START_ADDR_0       0x410
#define SSD1926_REG_JPEG_SRC_START_ADDR_1       0x411
#define SSD1926_REG_JPEG_SRC_START_ADDR_2       0x412
#define SSD1926_REG_JPEG_SRC_START_ADDR_3       0x413
#define SSD1926_REG_JPEG_DEST_START_ADDR_0      0x414
#define SSD1926_REG_JPEG_DEST_START_ADDR_1      0x415
#define SSD1926_REG_JPEG_DEST_START_ADDR_2      0x416
#define SSD1926_REG_JPEG_DEST_START_ADDR_3      0x417
#define SSD1926_REG_JPEG_RST_MARKER             0x41c
#define SSD1926_REG_JPEG_RST_MARKER_STATUS      0x41e
#define SSD1926_REG_JPEG_INSERT_MARKER00        0x420
#define SSD1926_REG_JPEG_INSERT_MARKER01        0x422
#define SSD1926_REG_JPEG_MARKER_LENGTH00        0x424
#define SSD1926_REG_JPEG_MARKER_LENGTH01        0x426
#define SSD1926_REG_JPEG_MARKER_DATA_00         0x428
#define SSD1926_REG_JPEG_MARKER_DATA_01         0x42a
#define SSD1926_REG_JPEG_MARKER_DATA_02         0x42c
#define SSD1926_REG_JPEG_MARKER_DATA_03         0x42e
#define SSD1926_REG_JPEG_MARKER_DATA_04         0x430
#define SSD1926_REG_JPEG_MARKER_DATA_05         0x432
#define SSD1926_REG_JPEG_MARKER_DATA_06         0x434
#define SSD1926_REG_JPEG_MARKER_DATA_07         0x436
#define SSD1926_REG_JPEG_MARKER_DATA_08         0x438
#define SSD1926_REG_JPEG_MARKER_DATA_09         0x43a
#define SSD1926_REG_JPEG_MARKER_DATA_10         0x43c
#define SSD1926_REG_JPEG_MARKER_DATA_11         0x43e
#define SSD1926_REG_JPEG_MARKER_DATA_12         0x440
#define SSD1926_REG_JPEG_MARKER_DATA_13         0x442
#define SSD1926_REG_JPEG_MARKER_DATA_14         0x444
#define SSD1926_REG_JPEG_MARKER_DATA_15         0x446
#define SSD1926_REG_JPEG_MARKER_DATA_16         0x448
#define SSD1926_REG_JPEG_MARKER_DATA_17         0x44a
#define SSD1926_REG_JPEG_MARKER_DATA_18         0x44c
#define SSD1926_REG_JPEG_MARKER_DATA_19         0x44e
#define SSD1926_REG_JPEG_MARKER_DATA_20         0x450
#define SSD1926_REG_JPEG_MARKER_DATA_21         0x452
#define SSD1926_REG_JPEG_MARKER_DATA_22         0x454
#define SSD1926_REG_JPEG_MARKER_DATA_23         0x456
#define SSD1926_REG_JPEG_MARKER_DATA_24         0x458
#define SSD1926_REG_JPEG_MARKER_DATA_25         0x45a
#define SSD1926_REG_JPEG_MARKER_DATA_26         0x45c
#define SSD1926_REG_JPEG_MARKER_DATA_27         0x45e
#define SSD1926_REG_JPEG_MARKER_DATA_28         0x460
#define SSD1926_REG_JPEG_MARKER_DATA_29         0x462
#define SSD1926_REG_JPEG_MARKER_DATA_30         0x464
#define SSD1926_REG_JPEG_MARKER_DATA_31         0x466
#define SSD1926_REG_JPEG_SOI_CONST_00           0x468
#define SSD1926_REG_JPEG_SOI_CONST_01           0x46a
#define SSD1926_REG_JPEG_JFIF_CONST_00          0x46c
#define SSD1926_REG_JPEG_JFIF_CONST_01          0x46e
#define SSD1926_REG_JPEG_JFIF_CONST_02          0x470
#define SSD1926_REG_JPEG_JFIF_CONST_03          0x472
#define SSD1926_REG_JPEG_JFIF_CONST_04          0x474
#define SSD1926_REG_JPEG_JFIF_CONST_05          0x476
#define SSD1926_REG_JPEG_JFIF_CONST_06          0x478
#define SSD1926_REG_JPEG_JFIF_CONST_07          0x47a
#define SSD1926_REG_JPEG_JFIF_CONST_08          0x47c
#define SSD1926_REG_JPEG_JFIF_CONST_09          0x47e
#define SSD1926_REG_JPEG_JFIF_CONST_10          0x480
#define SSD1926_REG_JPEG_JFIF_CONST_11          0x482
#define SSD1926_REG_JPEG_JFIF_CONST_12          0x484
#define SSD1926_REG_JPEG_JFIF_CONST_13          0x486
#define SSD1926_REG_JPEG_JFIF_CONST_14          0x488
#define SSD1926_REG_JPEG_JFIF_CONST_15          0x48a
#define SSD1926_REG_JPEG_JFIF_CONST_16          0x48c
#define SSD1926_REG_JPEG_JFIF_CONST_17          0x48e
#define SSD1926_REG_JPEG_LUM_DC_HT_CONST_00     0x490
#define SSD1926_REG_JPEG_LUM_DC_HT_CONST_01     0x492
#define SSD1926_REG_JPEG_LUM_DC_HT_CONST_02     0x494
#define SSD1926_REG_JPEG_LUM_DC_HT_CONST_03     0x496
#define SSD1926_REG_JPEG_LUM_DC_HT_CONST_04     0x498
#define SSD1926_REG_JPEG_CHR_DC_HT_CONST_00     0x4a0
#define SSD1926_REG_JPEG_CHR_DC_HT_CONST_01     0x4a2
#define SSD1926_REG_JPEG_CHR_DC_HT_CONST_02     0x4a4
#define SSD1926_REG_JPEG_CHR_DC_HT_CONST_03     0x4a6
#define SSD1926_REG_JPEG_CHR_DC_HT_CONST_04     0x4a8
#define SSD1926_REG_JPEG_LUM_AC_HT_CONST_00     0x4b0
#define SSD1926_REG_JPEG_LUM_AC_HT_CONST_01     0x4b2
#define SSD1926_REG_JPEG_LUM_AC_HT_CONST_02     0x4b4
#define SSD1926_REG_JPEG_LUM_AC_HT_CONST_03     0x4b6
#define SSD1926_REG_JPEG_LUM_AC_HT_CONST_04     0x4b8
#define SSD1926_REG_JPEG_CHR_AC_HT_CONST_00     0x4c0
#define SSD1926_REG_JPEG_CHR_AC_HT_CONST_01     0x4c2
#define SSD1926_REG_JPEG_CHR_AC_HT_CONST_02     0x4c4
#define SSD1926_REG_JPEG_CHR_AC_HT_CONST_03     0x4c6
#define SSD1926_REG_JPEG_CHR_AC_HT_CONST_04     0x4c8
#define SSD1926_REG_JPEG_LUM_QT_CONST_00        0x4d0
#define SSD1926_REG_JPEG_LUM_QT_CONST_01        0x4d2
#define SSD1926_REG_JPEG_LUM_QT_CONST_02        0x4d4
#define SSD1926_REG_JPEG_LUM_QT_CONST_03        0x4d6
#define SSD1926_REG_JPEG_LUM_QT_CONST_04        0x4d8
#define SSD1926_REG_JPEG_CHR_QT_CONST_00        0x4e0
#define SSD1926_REG_JPEG_CHR_QT_CONST_01        0x4e2
#define SSD1926_REG_JPEG_CHR_QT_CONST_02        0x4e4
#define SSD1926_REG_JPEG_CHR_QT_CONST_03        0x4e6
#define SSD1926_REG_JPEG_CHR_QT_CONST_04        0x4e8
#define SSD1926_REG_JPEG_SOF_CONST_00           0x4f0
#define SSD1926_REG_JPEG_SOF_CONST_01           0x4f2
#define SSD1926_REG_JPEG_SOF_CONST_02           0x4f4
#define SSD1926_REG_JPEG_SOF_CONST_03           0x4f6
#define SSD1926_REG_JPEG_SOF_CONST_04           0x4f8
#define SSD1926_REG_JPEG_QUANT_T0_00            0x500
#define SSD1926_REG_JPEG_QUANT_T0_01            0x502
#define SSD1926_REG_JPEG_QUANT_T0_02            0x504
#define SSD1926_REG_JPEG_QUANT_T0_03            0x506
#define SSD1926_REG_JPEG_QUANT_T0_04            0x508
#define SSD1926_REG_JPEG_QUANT_T0_05            0x50a
#define SSD1926_REG_JPEG_QUANT_T0_06            0x50c
#define SSD1926_REG_JPEG_QUANT_T0_07            0x50e
#define SSD1926_REG_JPEG_QUANT_T0_08            0x510
#define SSD1926_REG_JPEG_QUANT_T0_09            0x512
#define SSD1926_REG_JPEG_QUANT_T0_10            0x514
#define SSD1926_REG_JPEG_QUANT_T0_11            0x516
#define SSD1926_REG_JPEG_QUANT_T0_12            0x518
#define SSD1926_REG_JPEG_QUANT_T0_13            0x51a
#define SSD1926_REG_JPEG_QUANT_T0_14            0x51c
#define SSD1926_REG_JPEG_QUANT_T0_15            0x51e
#define SSD1926_REG_JPEG_QUANT_T0_16            0x520
#define SSD1926_REG_JPEG_QUANT_T0_17            0x522
#define SSD1926_REG_JPEG_QUANT_T0_18            0x524
#define SSD1926_REG_JPEG_QUANT_T0_19            0x526
#define SSD1926_REG_JPEG_QUANT_T0_20            0x528
#define SSD1926_REG_JPEG_QUANT_T0_21            0x52a
#define SSD1926_REG_JPEG_QUANT_T0_22            0x52c
#define SSD1926_REG_JPEG_QUANT_T0_23            0x52e
#define SSD1926_REG_JPEG_QUANT_T0_24            0x530
#define SSD1926_REG_JPEG_QUANT_T0_25            0x532
#define SSD1926_REG_JPEG_QUANT_T0_26            0x534
#define SSD1926_REG_JPEG_QUANT_T0_27            0x536
#define SSD1926_REG_JPEG_QUANT_T0_28            0x538
#define SSD1926_REG_JPEG_QUANT_T0_29            0x53a
#define SSD1926_REG_JPEG_QUANT_T0_30            0x53c
#define SSD1926_REG_JPEG_QUANT_T0_31            0x53e
#define SSD1926_REG_JPEG_QUANT_T0_32            0x540
#define SSD1926_REG_JPEG_QUANT_T0_33            0x542
#define SSD1926_REG_JPEG_QUANT_T0_34            0x544
#define SSD1926_REG_JPEG_QUANT_T0_35            0x546
#define SSD1926_REG_JPEG_QUANT_T0_36            0x548
#define SSD1926_REG_JPEG_QUANT_T0_37            0x54a
#define SSD1926_REG_JPEG_QUANT_T0_38            0x54c
#define SSD1926_REG_JPEG_QUANT_T0_39            0x54e
#define SSD1926_REG_JPEG_QUANT_T0_40            0x550
#define SSD1926_REG_JPEG_QUANT_T0_41            0x552
#define SSD1926_REG_JPEG_QUANT_T0_42            0x554
#define SSD1926_REG_JPEG_QUANT_T0_43            0x556
#define SSD1926_REG_JPEG_QUANT_T0_44            0x558
#define SSD1926_REG_JPEG_QUANT_T0_45            0x55a
#define SSD1926_REG_JPEG_QUANT_T0_46            0x55c
#define SSD1926_REG_JPEG_QUANT_T0_47            0x55e
#define SSD1926_REG_JPEG_QUANT_T0_48            0x560
#define SSD1926_REG_JPEG_QUANT_T0_49            0x562
#define SSD1926_REG_JPEG_QUANT_T0_50            0x564
#define SSD1926_REG_JPEG_QUANT_T0_51            0x566
#define SSD1926_REG_JPEG_QUANT_T0_52            0x568
#define SSD1926_REG_JPEG_QUANT_T0_53            0x56a
#define SSD1926_REG_JPEG_QUANT_T0_54            0x56c
#define SSD1926_REG_JPEG_QUANT_T0_55            0x56e
#define SSD1926_REG_JPEG_QUANT_T0_56            0x570
#define SSD1926_REG_JPEG_QUANT_T0_57            0x572
#define SSD1926_REG_JPEG_QUANT_T0_58            0x574
#define SSD1926_REG_JPEG_QUANT_T0_59            0x576
#define SSD1926_REG_JPEG_QUANT_T0_60            0x578
#define SSD1926_REG_JPEG_QUANT_T0_61            0x57a
#define SSD1926_REG_JPEG_QUANT_T0_62            0x57c
#define SSD1926_REG_JPEG_QUANT_T0_63            0x57e
#define SSD1926_REG_JPEG_QUANT_T1_00            0x580
#define SSD1926_REG_JPEG_QUANT_T1_01            0x582
#define SSD1926_REG_JPEG_QUANT_T1_02            0x584
#define SSD1926_REG_JPEG_QUANT_T1_03            0x586
#define SSD1926_REG_JPEG_QUANT_T1_04            0x588
#define SSD1926_REG_JPEG_QUANT_T1_05            0x58a
#define SSD1926_REG_JPEG_QUANT_T1_06            0x58c
#define SSD1926_REG_JPEG_QUANT_T1_07            0x58e
#define SSD1926_REG_JPEG_QUANT_T1_08            0x590
#define SSD1926_REG_JPEG_QUANT_T1_09            0x592
#define SSD1926_REG_JPEG_QUANT_T1_10            0x594
#define SSD1926_REG_JPEG_QUANT_T1_11            0x596
#define SSD1926_REG_JPEG_QUANT_T1_12            0x598
#define SSD1926_REG_JPEG_QUANT_T1_13            0x59a
#define SSD1926_REG_JPEG_QUANT_T1_14            0x59c
#define SSD1926_REG_JPEG_QUANT_T1_15            0x59e
#define SSD1926_REG_JPEG_QUANT_T1_16            0x5a0
#define SSD1926_REG_JPEG_QUANT_T1_17            0x5a2
#define SSD1926_REG_JPEG_QUANT_T1_18            0x5a4
#define SSD1926_REG_JPEG_QUANT_T1_19            0x5a6
#define SSD1926_REG_JPEG_QUANT_T1_20            0x5a8
#define SSD1926_REG_JPEG_QUANT_T1_21            0x5aa
#define SSD1926_REG_JPEG_QUANT_T1_22            0x5ac
#define SSD1926_REG_JPEG_QUANT_T1_23            0x5ae
#define SSD1926_REG_JPEG_QUANT_T1_24            0x5b0
#define SSD1926_REG_JPEG_QUANT_T1_25            0x5b2
#define SSD1926_REG_JPEG_QUANT_T1_26            0x5b4
#define SSD1926_REG_JPEG_QUANT_T1_27            0x5b6
#define SSD1926_REG_JPEG_QUANT_T1_28            0x5b8
#define SSD1926_REG_JPEG_QUANT_T1_29            0x5ba
#define SSD1926_REG_JPEG_QUANT_T1_30            0x5bc
#define SSD1926_REG_JPEG_QUANT_T1_31            0x5be
#define SSD1926_REG_JPEG_QUANT_T1_32            0x5c0
#define SSD1926_REG_JPEG_QUANT_T1_33            0x5c2
#define SSD1926_REG_JPEG_QUANT_T1_34            0x5c4
#define SSD1926_REG_JPEG_QUANT_T1_35            0x5c6
#define SSD1926_REG_JPEG_QUANT_T1_36            0x5c8
#define SSD1926_REG_JPEG_QUANT_T1_37            0x5ca
#define SSD1926_REG_JPEG_QUANT_T1_38            0x5cc
#define SSD1926_REG_JPEG_QUANT_T1_39            0x5ce
#define SSD1926_REG_JPEG_QUANT_T1_40            0x5d0
#define SSD1926_REG_JPEG_QUANT_T1_41            0x5d2
#define SSD1926_REG_JPEG_QUANT_T1_42            0x5d4
#define SSD1926_REG_JPEG_QUANT_T1_43            0x5d6
#define SSD1926_REG_JPEG_QUANT_T1_44            0x5d8
#define SSD1926_REG_JPEG_QUANT_T1_45            0x5da
#define SSD1926_REG_JPEG_QUANT_T1_46            0x5dc
#define SSD1926_REG_JPEG_QUANT_T1_47            0x5de
#define SSD1926_REG_JPEG_QUANT_T1_48            0x5e0
#define SSD1926_REG_JPEG_QUANT_T1_49            0x5e2
#define SSD1926_REG_JPEG_QUANT_T1_50            0x5e4
#define SSD1926_REG_JPEG_QUANT_T1_51            0x5e6
#define SSD1926_REG_JPEG_QUANT_T1_52            0x5e8
#define SSD1926_REG_JPEG_QUANT_T1_53            0x5ea
#define SSD1926_REG_JPEG_QUANT_T1_54            0x5ec
#define SSD1926_REG_JPEG_QUANT_T1_55            0x5ee
#define SSD1926_REG_JPEG_QUANT_T1_56            0x5f0
#define SSD1926_REG_JPEG_QUANT_T1_57            0x5f2
#define SSD1926_REG_JPEG_QUANT_T1_58            0x5f4
#define SSD1926_REG_JPEG_QUANT_T1_59            0x5f6
#define SSD1926_REG_JPEG_QUANT_T1_60            0x5f8
#define SSD1926_REG_JPEG_QUANT_T1_61            0x5fa
#define SSD1926_REG_JPEG_QUANT_T1_62            0x5fc
#define SSD1926_REG_JPEG_QUANT_T1_63            0x5fe
#define SSD1926_REG_JPEG_DC_T0_R0_00            0x600
#define SSD1926_REG_JPEG_DC_T0_R0_01            0x602
#define SSD1926_REG_JPEG_DC_T0_R0_02            0x604
#define SSD1926_REG_JPEG_DC_T0_R0_03            0x606
#define SSD1926_REG_JPEG_DC_T0_R0_04            0x608
#define SSD1926_REG_JPEG_DC_T0_R0_05            0x60a
#define SSD1926_REG_JPEG_DC_T0_R0_06            0x60c
#define SSD1926_REG_JPEG_DC_T0_R0_07            0x60e
#define SSD1926_REG_JPEG_DC_T0_R0_08            0x610
#define SSD1926_REG_JPEG_DC_T0_R0_09            0x612
#define SSD1926_REG_JPEG_DC_T0_R0_10            0x614
#define SSD1926_REG_JPEG_DC_T0_R0_11            0x616
#define SSD1926_REG_JPEG_DC_T0_R0_12            0x618
#define SSD1926_REG_JPEG_DC_T0_R0_13            0x61a
#define SSD1926_REG_JPEG_DC_T0_R0_14            0x61c
#define SSD1926_REG_JPEG_DC_T0_R0_15            0x61e
#define SSD1926_REG_JPEG_DC_T0_R1_00            0x620
#define SSD1926_REG_JPEG_DC_T0_R1_01            0x622
#define SSD1926_REG_JPEG_DC_T0_R1_02            0x624
#define SSD1926_REG_JPEG_DC_T0_R1_03            0x626
#define SSD1926_REG_JPEG_DC_T0_R1_04            0x628
#define SSD1926_REG_JPEG_DC_T0_R1_05            0x62a
#define SSD1926_REG_JPEG_DC_T0_R1_06            0x62c
#define SSD1926_REG_JPEG_DC_T0_R1_07            0x62e
#define SSD1926_REG_JPEG_DC_T0_R1_08            0x630
#define SSD1926_REG_JPEG_DC_T0_R1_09            0x632
#define SSD1926_REG_JPEG_DC_T0_R1_10            0x634
#define SSD1926_REG_JPEG_DC_T0_R1_11            0x636
#define SSD1926_REG_JPEG_AC_T0_R0_00            0x640
#define SSD1926_REG_JPEG_AC_T0_R0_01            0x642
#define SSD1926_REG_JPEG_AC_T0_R0_02            0x644
#define SSD1926_REG_JPEG_AC_T0_R0_03            0x646
#define SSD1926_REG_JPEG_AC_T0_R0_04            0x648
#define SSD1926_REG_JPEG_AC_T0_R0_05            0x64a
#define SSD1926_REG_JPEG_AC_T0_R0_06            0x64c
#define SSD1926_REG_JPEG_AC_T0_R0_07            0x64e
#define SSD1926_REG_JPEG_AC_T0_R0_08            0x650
#define SSD1926_REG_JPEG_AC_T0_R0_09            0x652
#define SSD1926_REG_JPEG_AC_T0_R0_10            0x654
#define SSD1926_REG_JPEG_AC_T0_R0_11            0x656
#define SSD1926_REG_JPEG_AC_T0_R0_12            0x658
#define SSD1926_REG_JPEG_AC_T0_R0_13            0x65a
#define SSD1926_REG_JPEG_AC_T0_R0_14            0x65c
#define SSD1926_REG_JPEG_AC_T0_R0_15            0x65e
#define SSD1926_REG_JPEG_AC_T0_R1_00            0x660
#define SSD1926_REG_JPEG_AC_T0_R1_01            0x662
#define SSD1926_REG_JPEG_AC_T0_R1_02            0x664
#define SSD1926_REG_JPEG_AC_T0_R1_03            0x666
#define SSD1926_REG_JPEG_AC_T0_R1_04            0x668
#define SSD1926_REG_JPEG_AC_T0_R1_05            0x66a
#define SSD1926_REG_JPEG_AC_T0_R1_06            0x66c
#define SSD1926_REG_JPEG_AC_T0_R1_07            0x66e
#define SSD1926_REG_JPEG_AC_T0_R1_08            0x670
#define SSD1926_REG_JPEG_AC_T0_R1_09            0x672
#define SSD1926_REG_JPEG_AC_T0_R1_10            0x674
#define SSD1926_REG_JPEG_AC_T0_R1_11            0x676
#define SSD1926_REG_JPEG_AC_T0_R1_12            0x678
#define SSD1926_REG_JPEG_AC_T0_R1_13            0x67a
#define SSD1926_REG_JPEG_AC_T0_R1_14            0x67c
#define SSD1926_REG_JPEG_AC_T0_R1_15            0x67e
#define SSD1926_REG_JPEG_AC_T0_R1_16            0x680
#define SSD1926_REG_JPEG_AC_T0_R1_17            0x682
#define SSD1926_REG_JPEG_AC_T0_R1_18            0x684
#define SSD1926_REG_JPEG_AC_T0_R1_19            0x686
#define SSD1926_REG_JPEG_AC_T0_R1_20            0x688
#define SSD1926_REG_JPEG_AC_T0_R1_21            0x68a
#define SSD1926_REG_JPEG_AC_T0_R1_22            0x68c
#define SSD1926_REG_JPEG_AC_T0_R1_23            0x68e
#define SSD1926_REG_JPEG_AC_T0_R1_24            0x690
#define SSD1926_REG_JPEG_AC_T0_R1_25            0x692
#define SSD1926_REG_JPEG_AC_T0_R1_26            0x694
#define SSD1926_REG_JPEG_AC_T0_R1_27            0x696
#define SSD1926_REG_JPEG_AC_T0_R1_28            0x698
#define SSD1926_REG_JPEG_AC_T0_R1_29            0x69a
#define SSD1926_REG_JPEG_AC_T0_R1_30            0x69c
#define SSD1926_REG_JPEG_AC_T0_R1_31            0x69e
#define SSD1926_REG_JPEG_AC_T0_R1_32            0x6a0
#define SSD1926_REG_JPEG_AC_T0_R1_33            0x6a2
#define SSD1926_REG_JPEG_AC_T0_R1_34            0x6a4
#define SSD1926_REG_JPEG_AC_T0_R1_35            0x6a6
#define SSD1926_REG_JPEG_AC_T0_R1_36            0x6a8
#define SSD1926_REG_JPEG_AC_T0_R1_37            0x6aa
#define SSD1926_REG_JPEG_AC_T0_R1_38            0x6ac
#define SSD1926_REG_JPEG_AC_T0_R1_39            0x6ae
#define SSD1926_REG_JPEG_AC_T0_R1_40            0x6b0
#define SSD1926_REG_JPEG_AC_T0_R1_41            0x6b2
#define SSD1926_REG_JPEG_AC_T0_R1_42            0x6b4
#define SSD1926_REG_JPEG_AC_T0_R1_43            0x6b6
#define SSD1926_REG_JPEG_AC_T0_R1_44            0x6b8
#define SSD1926_REG_JPEG_AC_T0_R1_45            0x6ba
#define SSD1926_REG_JPEG_AC_T0_R1_46            0x6bc
#define SSD1926_REG_JPEG_AC_T0_R1_47            0x6be
#define SSD1926_REG_JPEG_AC_T0_R1_48            0x6c0
#define SSD1926_REG_JPEG_AC_T0_R1_49            0x6c2
#define SSD1926_REG_JPEG_AC_T0_R1_50            0x6c4
#define SSD1926_REG_JPEG_AC_T0_R1_51            0x6c6
#define SSD1926_REG_JPEG_AC_T0_R1_52            0x6c8
#define SSD1926_REG_JPEG_AC_T0_R1_53            0x6ca
#define SSD1926_REG_JPEG_AC_T0_R1_54            0x6cc
#define SSD1926_REG_JPEG_AC_T0_R1_55            0x6ce
#define SSD1926_REG_JPEG_AC_T0_R1_56            0x6d0
#define SSD1926_REG_JPEG_AC_T0_R1_57            0x6d2
#define SSD1926_REG_JPEG_AC_T0_R1_58            0x6d4
#define SSD1926_REG_JPEG_AC_T0_R1_59            0x6d6
#define SSD1926_REG_JPEG_AC_T0_R1_60            0x6d8
#define SSD1926_REG_JPEG_AC_T0_R1_61            0x6da
#define SSD1926_REG_JPEG_AC_T0_R1_62            0x6dc
#define SSD1926_REG_JPEG_AC_T0_R1_63            0x6de
#define SSD1926_REG_JPEG_AC_T0_R1_64            0x6e0
#define SSD1926_REG_JPEG_AC_T0_R1_65            0x6e2
#define SSD1926_REG_JPEG_AC_T0_R1_66            0x6e4
#define SSD1926_REG_JPEG_AC_T0_R1_67            0x6e6
#define SSD1926_REG_JPEG_AC_T0_R1_68            0x6e8
#define SSD1926_REG_JPEG_AC_T0_R1_69            0x6ea
#define SSD1926_REG_JPEG_AC_T0_R1_70            0x6ec
#define SSD1926_REG_JPEG_AC_T0_R1_71            0x6ee
#define SSD1926_REG_JPEG_AC_T0_R1_72            0x6f0
#define SSD1926_REG_JPEG_AC_T0_R1_73            0x6f2
#define SSD1926_REG_JPEG_AC_T0_R1_74            0x6f4
#define SSD1926_REG_JPEG_AC_T0_R1_75            0x6f6
#define SSD1926_REG_JPEG_AC_T0_R1_76            0x6f8
#define SSD1926_REG_JPEG_AC_T0_R1_77            0x6fa
#define SSD1926_REG_JPEG_AC_T0_R1_78            0x6fc
#define SSD1926_REG_JPEG_AC_T0_R1_79            0x6fe
#define SSD1926_REG_JPEG_AC_T0_R1_80            0x700
#define SSD1926_REG_JPEG_AC_T0_R1_81            0x702
#define SSD1926_REG_JPEG_AC_T0_R1_82            0x704
#define SSD1926_REG_JPEG_AC_T0_R1_83            0x706
#define SSD1926_REG_JPEG_AC_T0_R1_84            0x708
#define SSD1926_REG_JPEG_AC_T0_R1_85            0x70a
#define SSD1926_REG_JPEG_AC_T0_R1_86            0x70c
#define SSD1926_REG_JPEG_AC_T0_R1_87            0x70e
#define SSD1926_REG_JPEG_AC_T0_R1_88            0x710
#define SSD1926_REG_JPEG_AC_T0_R1_89            0x712
#define SSD1926_REG_JPEG_AC_T0_R1_90            0x714
#define SSD1926_REG_JPEG_AC_T0_R1_91            0x716
#define SSD1926_REG_JPEG_AC_T0_R1_92            0x718
#define SSD1926_REG_JPEG_AC_T0_R1_93            0x71a
#define SSD1926_REG_JPEG_AC_T0_R1_94            0x71c
#define SSD1926_REG_JPEG_AC_T0_R1_95            0x71e
#define SSD1926_REG_JPEG_AC_T0_R1_96            0x720
#define SSD1926_REG_JPEG_AC_T0_R1_97            0x722
#define SSD1926_REG_JPEG_AC_T0_R1_98            0x724
#define SSD1926_REG_JPEG_AC_T0_R1_99            0x726
#define SSD1926_REG_JPEG_AC_T0_R1_100           0x728
#define SSD1926_REG_JPEG_AC_T0_R1_101           0x72a
#define SSD1926_REG_JPEG_AC_T0_R1_102           0x72c
#define SSD1926_REG_JPEG_AC_T0_R1_103           0x72e
#define SSD1926_REG_JPEG_AC_T0_R1_104           0x730
#define SSD1926_REG_JPEG_AC_T0_R1_105           0x732
#define SSD1926_REG_JPEG_AC_T0_R1_106           0x734
#define SSD1926_REG_JPEG_AC_T0_R1_107           0x736
#define SSD1926_REG_JPEG_AC_T0_R1_108           0x738
#define SSD1926_REG_JPEG_AC_T0_R1_109           0x73a
#define SSD1926_REG_JPEG_AC_T0_R1_110           0x73c
#define SSD1926_REG_JPEG_AC_T0_R1_111           0x73e
#define SSD1926_REG_JPEG_AC_T0_R1_112           0x740
#define SSD1926_REG_JPEG_AC_T0_R1_113           0x742
#define SSD1926_REG_JPEG_AC_T0_R1_114           0x744
#define SSD1926_REG_JPEG_AC_T0_R1_115           0x746
#define SSD1926_REG_JPEG_AC_T0_R1_116           0x748
#define SSD1926_REG_JPEG_AC_T0_R1_117           0x74a
#define SSD1926_REG_JPEG_AC_T0_R1_118           0x74c
#define SSD1926_REG_JPEG_AC_T0_R1_119           0x74e
#define SSD1926_REG_JPEG_AC_T0_R1_120           0x750
#define SSD1926_REG_JPEG_AC_T0_R1_121           0x752
#define SSD1926_REG_JPEG_AC_T0_R1_122           0x754
#define SSD1926_REG_JPEG_AC_T0_R1_123           0x756
#define SSD1926_REG_JPEG_AC_T0_R1_124           0x758
#define SSD1926_REG_JPEG_AC_T0_R1_125           0x75a
#define SSD1926_REG_JPEG_AC_T0_R1_126           0x75c
#define SSD1926_REG_JPEG_AC_T0_R1_127           0x75e
#define SSD1926_REG_JPEG_AC_T0_R1_128           0x760
#define SSD1926_REG_JPEG_AC_T0_R1_129           0x762
#define SSD1926_REG_JPEG_AC_T0_R1_130           0x764
#define SSD1926_REG_JPEG_AC_T0_R1_131           0x766
#define SSD1926_REG_JPEG_AC_T0_R1_132           0x768
#define SSD1926_REG_JPEG_AC_T0_R1_133           0x76a
#define SSD1926_REG_JPEG_AC_T0_R1_134           0x76c
#define SSD1926_REG_JPEG_AC_T0_R1_135           0x76e
#define SSD1926_REG_JPEG_AC_T0_R1_136           0x770
#define SSD1926_REG_JPEG_AC_T0_R1_137           0x772
#define SSD1926_REG_JPEG_AC_T0_R1_138           0x774
#define SSD1926_REG_JPEG_AC_T0_R1_139           0x776
#define SSD1926_REG_JPEG_AC_T0_R1_140           0x778
#define SSD1926_REG_JPEG_AC_T0_R1_141           0x77a
#define SSD1926_REG_JPEG_AC_T0_R1_142           0x77c
#define SSD1926_REG_JPEG_AC_T0_R1_143           0x77e
#define SSD1926_REG_JPEG_AC_T0_R1_144           0x780
#define SSD1926_REG_JPEG_AC_T0_R1_145           0x782
#define SSD1926_REG_JPEG_AC_T0_R1_146           0x784
#define SSD1926_REG_JPEG_AC_T0_R1_147           0x786
#define SSD1926_REG_JPEG_AC_T0_R1_148           0x788
#define SSD1926_REG_JPEG_AC_T0_R1_149           0x78a
#define SSD1926_REG_JPEG_AC_T0_R1_150           0x78c
#define SSD1926_REG_JPEG_AC_T0_R1_151           0x78e
#define SSD1926_REG_JPEG_AC_T0_R1_152           0x790
#define SSD1926_REG_JPEG_AC_T0_R1_153           0x792
#define SSD1926_REG_JPEG_AC_T0_R1_154           0x794
#define SSD1926_REG_JPEG_AC_T0_R1_155           0x796
#define SSD1926_REG_JPEG_AC_T0_R1_156           0x798
#define SSD1926_REG_JPEG_AC_T0_R1_157           0x79a
#define SSD1926_REG_JPEG_AC_T0_R1_158           0x79c
#define SSD1926_REG_JPEG_AC_T0_R1_159           0x79e
#define SSD1926_REG_JPEG_AC_T0_R1_160           0x7a0
#define SSD1926_REG_JPEG_AC_T0_R1_161           0x7a2
#define SSD1926_REG_JPEG_DC_T1_R0_00            0x800
#define SSD1926_REG_JPEG_DC_T1_R0_01            0x802
#define SSD1926_REG_JPEG_DC_T1_R0_02            0x804
#define SSD1926_REG_JPEG_DC_T1_R0_03            0x806
#define SSD1926_REG_JPEG_DC_T1_R0_04            0x808
#define SSD1926_REG_JPEG_DC_T1_R0_05            0x80a
#define SSD1926_REG_JPEG_DC_T1_R0_06            0x80c
#define SSD1926_REG_JPEG_DC_T1_R0_07            0x80e
#define SSD1926_REG_JPEG_DC_T1_R0_08            0x810
#define SSD1926_REG_JPEG_DC_T1_R0_09            0x812
#define SSD1926_REG_JPEG_DC_T1_R0_10            0x814
#define SSD1926_REG_JPEG_DC_T1_R0_11            0x816
#define SSD1926_REG_JPEG_DC_T1_R0_12            0x818
#define SSD1926_REG_JPEG_DC_T1_R0_13            0x81a
#define SSD1926_REG_JPEG_DC_T1_R0_14            0x81c
#define SSD1926_REG_JPEG_DC_T1_R0_15            0x81e
#define SSD1926_REG_JPEG_DC_T1_R1_00            0x820
#define SSD1926_REG_JPEG_DC_T1_R1_01            0x822
#define SSD1926_REG_JPEG_DC_T1_R1_02            0x824
#define SSD1926_REG_JPEG_DC_T1_R1_03            0x826
#define SSD1926_REG_JPEG_DC_T1_R1_04            0x828
#define SSD1926_REG_JPEG_DC_T1_R1_05            0x82a
#define SSD1926_REG_JPEG_DC_T1_R1_06            0x82c
#define SSD1926_REG_JPEG_DC_T1_R1_07            0x82e
#define SSD1926_REG_JPEG_DC_T1_R1_08            0x830
#define SSD1926_REG_JPEG_DC_T1_R1_09            0x832
#define SSD1926_REG_JPEG_DC_T1_R1_10            0x834
#define SSD1926_REG_JPEG_DC_T1_R1_11            0x836
#define SSD1926_REG_JPEG_AC_T1_R0_00            0x840
#define SSD1926_REG_JPEG_AC_T1_R0_01            0x842
#define SSD1926_REG_JPEG_AC_T1_R0_02            0x844
#define SSD1926_REG_JPEG_AC_T1_R0_03            0x846
#define SSD1926_REG_JPEG_AC_T1_R0_04            0x848
#define SSD1926_REG_JPEG_AC_T1_R0_05            0x84a
#define SSD1926_REG_JPEG_AC_T1_R0_06            0x84c
#define SSD1926_REG_JPEG_AC_T1_R0_07            0x84e
#define SSD1926_REG_JPEG_AC_T1_R0_08            0x850
#define SSD1926_REG_JPEG_AC_T1_R0_09            0x852
#define SSD1926_REG_JPEG_AC_T1_R0_10            0x854
#define SSD1926_REG_JPEG_AC_T1_R0_11            0x856
#define SSD1926_REG_JPEG_AC_T1_R0_12            0x858
#define SSD1926_REG_JPEG_AC_T1_R0_13            0x85a
#define SSD1926_REG_JPEG_AC_T1_R0_14            0x85c
#define SSD1926_REG_JPEG_AC_T1_R0_15            0x85e
#define SSD1926_REG_JPEG_AC_T1_R1_00            0x860
#define SSD1926_REG_JPEG_AC_T1_R1_01            0x862
#define SSD1926_REG_JPEG_AC_T1_R1_02            0x864
#define SSD1926_REG_JPEG_AC_T1_R1_03            0x866
#define SSD1926_REG_JPEG_AC_T1_R1_04            0x868
#define SSD1926_REG_JPEG_AC_T1_R1_05            0x86a
#define SSD1926_REG_JPEG_AC_T1_R1_06            0x86c
#define SSD1926_REG_JPEG_AC_T1_R1_07            0x86e
#define SSD1926_REG_JPEG_AC_T1_R1_08            0x870
#define SSD1926_REG_JPEG_AC_T1_R1_09            0x872
#define SSD1926_REG_JPEG_AC_T1_R1_10            0x874
#define SSD1926_REG_JPEG_AC_T1_R1_11            0x876
#define SSD1926_REG_JPEG_AC_T1_R1_12            0x878
#define SSD1926_REG_JPEG_AC_T1_R1_13            0x87a
#define SSD1926_REG_JPEG_AC_T1_R1_14            0x87c
#define SSD1926_REG_JPEG_AC_T1_R1_15            0x87e
#define SSD1926_REG_JPEG_AC_T1_R1_16            0x880
#define SSD1926_REG_JPEG_AC_T1_R1_17            0x882
#define SSD1926_REG_JPEG_AC_T1_R1_18            0x884
#define SSD1926_REG_JPEG_AC_T1_R1_19            0x886
#define SSD1926_REG_JPEG_AC_T1_R1_20            0x888
#define SSD1926_REG_JPEG_AC_T1_R1_21            0x88a
#define SSD1926_REG_JPEG_AC_T1_R1_22            0x88c
#define SSD1926_REG_JPEG_AC_T1_R1_23            0x88e
#define SSD1926_REG_JPEG_AC_T1_R1_24            0x890
#define SSD1926_REG_JPEG_AC_T1_R1_25            0x892
#define SSD1926_REG_JPEG_AC_T1_R1_26            0x894
#define SSD1926_REG_JPEG_AC_T1_R1_27            0x896
#define SSD1926_REG_JPEG_AC_T1_R1_28            0x898
#define SSD1926_REG_JPEG_AC_T1_R1_29            0x89a
#define SSD1926_REG_JPEG_AC_T1_R1_30            0x89c
#define SSD1926_REG_JPEG_AC_T1_R1_31            0x89e
#define SSD1926_REG_JPEG_AC_T1_R1_32            0x8a0
#define SSD1926_REG_JPEG_AC_T1_R1_33            0x8a2
#define SSD1926_REG_JPEG_AC_T1_R1_34            0x8a4
#define SSD1926_REG_JPEG_AC_T1_R1_35            0x8a6
#define SSD1926_REG_JPEG_AC_T1_R1_36            0x8a8
#define SSD1926_REG_JPEG_AC_T1_R1_37            0x8aa
#define SSD1926_REG_JPEG_AC_T1_R1_38            0x8ac
#define SSD1926_REG_JPEG_AC_T1_R1_39            0x8ae
#define SSD1926_REG_JPEG_AC_T1_R1_40            0x8b0
#define SSD1926_REG_JPEG_AC_T1_R1_41            0x8b2
#define SSD1926_REG_JPEG_AC_T1_R1_42            0x8b4
#define SSD1926_REG_JPEG_AC_T1_R1_43            0x8b6
#define SSD1926_REG_JPEG_AC_T1_R1_44            0x8b8
#define SSD1926_REG_JPEG_AC_T1_R1_45            0x8ba
#define SSD1926_REG_JPEG_AC_T1_R1_46            0x8bc
#define SSD1926_REG_JPEG_AC_T1_R1_47            0x8be
#define SSD1926_REG_JPEG_AC_T1_R1_48            0x8c0
#define SSD1926_REG_JPEG_AC_T1_R1_49            0x8c2
#define SSD1926_REG_JPEG_AC_T1_R1_50            0x8c4
#define SSD1926_REG_JPEG_AC_T1_R1_51            0x8c6
#define SSD1926_REG_JPEG_AC_T1_R1_52            0x8c8
#define SSD1926_REG_JPEG_AC_T1_R1_53            0x8ca
#define SSD1926_REG_JPEG_AC_T1_R1_54            0x8cc
#define SSD1926_REG_JPEG_AC_T1_R1_55            0x8ce
#define SSD1926_REG_JPEG_AC_T1_R1_56            0x8d0
#define SSD1926_REG_JPEG_AC_T1_R1_57            0x8d2
#define SSD1926_REG_JPEG_AC_T1_R1_58            0x8d4
#define SSD1926_REG_JPEG_AC_T1_R1_59            0x8d6
#define SSD1926_REG_JPEG_AC_T1_R1_60            0x8d8
#define SSD1926_REG_JPEG_AC_T1_R1_61            0x8da
#define SSD1926_REG_JPEG_AC_T1_R1_62            0x8dc
#define SSD1926_REG_JPEG_AC_T1_R1_63            0x8de
#define SSD1926_REG_JPEG_AC_T1_R1_64            0x8e0
#define SSD1926_REG_JPEG_AC_T1_R1_65            0x8e2
#define SSD1926_REG_JPEG_AC_T1_R1_66            0x8e4
#define SSD1926_REG_JPEG_AC_T1_R1_67            0x8e6
#define SSD1926_REG_JPEG_AC_T1_R1_68            0x8e8
#define SSD1926_REG_JPEG_AC_T1_R1_69            0x8ea
#define SSD1926_REG_JPEG_AC_T1_R1_70            0x8ec
#define SSD1926_REG_JPEG_AC_T1_R1_71            0x8ee
#define SSD1926_REG_JPEG_AC_T1_R1_72            0x8f0
#define SSD1926_REG_JPEG_AC_T1_R1_73            0x8f2
#define SSD1926_REG_JPEG_AC_T1_R1_74            0x8f4
#define SSD1926_REG_JPEG_AC_T1_R1_75            0x8f6
#define SSD1926_REG_JPEG_AC_T1_R1_76            0x8f8
#define SSD1926_REG_JPEG_AC_T1_R1_77            0x8fa
#define SSD1926_REG_JPEG_AC_T1_R1_78            0x8fc
#define SSD1926_REG_JPEG_AC_T1_R1_79            0x8fe
#define SSD1926_REG_JPEG_AC_T1_R1_80            0x900
#define SSD1926_REG_JPEG_AC_T1_R1_81            0x902
#define SSD1926_REG_JPEG_AC_T1_R1_82            0x904
#define SSD1926_REG_JPEG_AC_T1_R1_83            0x906
#define SSD1926_REG_JPEG_AC_T1_R1_84            0x908
#define SSD1926_REG_JPEG_AC_T1_R1_85            0x90a
#define SSD1926_REG_JPEG_AC_T1_R1_86            0x90c
#define SSD1926_REG_JPEG_AC_T1_R1_87            0x90e
#define SSD1926_REG_JPEG_AC_T1_R1_88            0x910
#define SSD1926_REG_JPEG_AC_T1_R1_89            0x912
#define SSD1926_REG_JPEG_AC_T1_R1_90            0x914
#define SSD1926_REG_JPEG_AC_T1_R1_91            0x916
#define SSD1926_REG_JPEG_AC_T1_R1_92            0x918
#define SSD1926_REG_JPEG_AC_T1_R1_93            0x91a
#define SSD1926_REG_JPEG_AC_T1_R1_94            0x91c
#define SSD1926_REG_JPEG_AC_T1_R1_95            0x91e
#define SSD1926_REG_JPEG_AC_T1_R1_96            0x920
#define SSD1926_REG_JPEG_AC_T1_R1_97            0x922
#define SSD1926_REG_JPEG_AC_T1_R1_98            0x924
#define SSD1926_REG_JPEG_AC_T1_R1_99            0x926
#define SSD1926_REG_JPEG_AC_T1_R1_100           0x928
#define SSD1926_REG_JPEG_AC_T1_R1_101           0x92a
#define SSD1926_REG_JPEG_AC_T1_R1_102           0x92c
#define SSD1926_REG_JPEG_AC_T1_R1_103           0x92e
#define SSD1926_REG_JPEG_AC_T1_R1_104           0x930
#define SSD1926_REG_JPEG_AC_T1_R1_105           0x932
#define SSD1926_REG_JPEG_AC_T1_R1_106           0x934
#define SSD1926_REG_JPEG_AC_T1_R1_107           0x936
#define SSD1926_REG_JPEG_AC_T1_R1_108           0x938
#define SSD1926_REG_JPEG_AC_T1_R1_109           0x93a
#define SSD1926_REG_JPEG_AC_T1_R1_110           0x93c
#define SSD1926_REG_JPEG_AC_T1_R1_111           0x93e
#define SSD1926_REG_JPEG_AC_T1_R1_112           0x940
#define SSD1926_REG_JPEG_AC_T1_R1_113           0x942
#define SSD1926_REG_JPEG_AC_T1_R1_114           0x944
#define SSD1926_REG_JPEG_AC_T1_R1_115           0x946
#define SSD1926_REG_JPEG_AC_T1_R1_116           0x948
#define SSD1926_REG_JPEG_AC_T1_R1_117           0x94a
#define SSD1926_REG_JPEG_AC_T1_R1_118           0x94c
#define SSD1926_REG_JPEG_AC_T1_R1_119           0x94e
#define SSD1926_REG_JPEG_AC_T1_R1_120           0x950
#define SSD1926_REG_JPEG_AC_T1_R1_121           0x952
#define SSD1926_REG_JPEG_AC_T1_R1_122           0x954
#define SSD1926_REG_JPEG_AC_T1_R1_123           0x956
#define SSD1926_REG_JPEG_AC_T1_R1_124           0x958
#define SSD1926_REG_JPEG_AC_T1_R1_125           0x95a
#define SSD1926_REG_JPEG_AC_T1_R1_126           0x95c
#define SSD1926_REG_JPEG_AC_T1_R1_127           0x95e
#define SSD1926_REG_JPEG_AC_T1_R1_128           0x960
#define SSD1926_REG_JPEG_AC_T1_R1_129           0x962
#define SSD1926_REG_JPEG_AC_T1_R1_130           0x964
#define SSD1926_REG_JPEG_AC_T1_R1_131           0x966
#define SSD1926_REG_JPEG_AC_T1_R1_132           0x968
#define SSD1926_REG_JPEG_AC_T1_R1_133           0x96a
#define SSD1926_REG_JPEG_AC_T1_R1_134           0x96c
#define SSD1926_REG_JPEG_AC_T1_R1_135           0x96e
#define SSD1926_REG_JPEG_AC_T1_R1_136           0x970
#define SSD1926_REG_JPEG_AC_T1_R1_137           0x972
#define SSD1926_REG_JPEG_AC_T1_R1_138           0x974
#define SSD1926_REG_JPEG_AC_T1_R1_139           0x976
#define SSD1926_REG_JPEG_AC_T1_R1_140           0x978
#define SSD1926_REG_JPEG_AC_T1_R1_141           0x97a
#define SSD1926_REG_JPEG_AC_T1_R1_142           0x97c
#define SSD1926_REG_JPEG_AC_T1_R1_143           0x97e
#define SSD1926_REG_JPEG_AC_T1_R1_144           0x980
#define SSD1926_REG_JPEG_AC_T1_R1_145           0x982
#define SSD1926_REG_JPEG_AC_T1_R1_146           0x984
#define SSD1926_REG_JPEG_AC_T1_R1_147           0x986
#define SSD1926_REG_JPEG_AC_T1_R1_148           0x988
#define SSD1926_REG_JPEG_AC_T1_R1_149           0x98a
#define SSD1926_REG_JPEG_AC_T1_R1_150           0x98c
#define SSD1926_REG_JPEG_AC_T1_R1_151           0x98e
#define SSD1926_REG_JPEG_AC_T1_R1_152           0x990
#define SSD1926_REG_JPEG_AC_T1_R1_153           0x992
#define SSD1926_REG_JPEG_AC_T1_R1_154           0x994
#define SSD1926_REG_JPEG_AC_T1_R1_155           0x996
#define SSD1926_REG_JPEG_AC_T1_R1_156           0x998
#define SSD1926_REG_JPEG_AC_T1_R1_157           0x99a
#define SSD1926_REG_JPEG_AC_T1_R1_158           0x99c
#define SSD1926_REG_JPEG_AC_T1_R1_159           0x99e
#define SSD1926_REG_JPEG_AC_T1_R1_160           0x9a0
#define SSD1926_REG_JPEG_AC_T1_R1_161           0x9a2
#define SSD1926_REG_JPEG_QTABLE_CONST_0         0x9b0
#define SSD1926_REG_JPEG_QTABLE_CONST_1         0x9b2
#define SSD1926_REG_JPEG_QTABLE_CONST_2         0x9b4
#define SSD1926_REG_JPEG_QTABLE_CONST_3         0x9b6
#define SSD1926_REG_JPEG_QTABLE0_SAMPLE         0x9b8
#define SSD1926_REG_JPEG_QTABLE1_SAMPLE         0x9bc
#define SSD1926_REG_JPEG_QTABLE2_SAMPLE         0x9c0
#define SSD1926_REG_JPEG_DRI_CONST_0            0x9c4
#define SSD1926_REG_JPEG_DRI_CONST_1            0x9c6
#define SSD1926_REG_JPEG_DRI_CONST_2            0x9c8
#define SSD1926_REG_JPEG_DRI_CONST_3            0x9ca
#define SSD1926_REG_JPEG_SOS_CONST_0            0x9cc
#define SSD1926_REG_JPEG_SOS_CONST_1            0x9ce
#define SSD1926_REG_JPEG_SOS_CONST_2            0x9d0
#define SSD1926_REG_JPEG_SOS_CONST_3            0x9d2
#define SSD1926_REG_JPEG_SOS_CONST_4            0x9d4
#define SSD1926_REG_JPEG_EOI_CONST_0            0x9e4
#define SSD1926_REG_JPEG_EOI_CONST_1            0x9e6
#define SSD1926_REG_JPEG_EOI_CONST_2            0x9e8
#define SSD1926_REG_JPEG_EOI_CONST_3            0x9ea
#define SSD1926_REG_JPEG_EOI_CONST_4            0x9ec
#define SSD1926_REG_JPEG_VERT_PIX_SIZE0         0x9f0
#define SSD1926_REG_JPEG_VERT_PIX_SIZE1         0x9f2
#define SSD1926_REG_JPEG_HORI_PIX_SIZE0         0x9f4
#define SSD1926_REG_JPEG_HORI_PIX_SIZE1         0x9f6
#define SSD1926_REG_JPEG_DRI_CONFIG0            0x9f8
#define SSD1926_REG_JPEG_DRI_CONFIG1            0x9fa
//DOM-IGNORE-END

#ifdef __cplusplus
    }
#endif
    
#endif // _DRV_GFX_SSD1926_H


