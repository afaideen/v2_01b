/*******************************************************************************
  SD Host Controller Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdhc_sd.h

  Summary:
    SD Host Controller Driver SD Definitions

  Description:
    The SD Host Controller driver provides a simple interface to manage the SD 
    Host Controller peripheral.  This file defines the interface definitions 
    and prototypes for the SD Host Controller driver.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_SDHC_SD_H
#define _DRV_SDHC_SD_H

#include "system_config.h"
#include "system_definitions.h"

/*The size of the card common control registers block.*/
#define CCCR_TOTAL_SIZE 256

/* The base address for the CCCR.*/
#define CCCR_BASE_ADRS  0x0


/* The offsets for the various bytes in the CCCR REGISTER*/
#define CCCR_IO_ENABLE_OFFSET     0x2
#define CCCR_IO_READY_OFFSET      0x3
#define CCCR_INT_ENABLE_OFFSET    0x4
#define CCCR_BUS_INTF_CTL_OFFSET  0x7
#define CCCR_CARD_CAP_OFFSET      0x8
#define CCCR_BUS_SPD_SEL_OFFSET   0x13
#define CCCR_UHS_I_SUP_OFFSET     0x14
#define CCCR_DEVICE_MSG_CODE      0xF0
#define CCCR_HOST_MSG_CODE        0xF1


// Bit positions within the bus interface control byte (0x7) in the CCCR.
#define BUS_INTF_CTL_BUS_WIDTH_MASK (0x3 << 0)
#define SUPPORT_8BIT_BUS   (1 << 2)

// Bit positions within the bus speed select byte (0x13) in the CCCR.
#define SUPPORT_HIGH_SPEED (1 << 0)
#define BUS_SPEED_SELECT_MASK (0x7 << 1)

// Bit positions within the card capability byte (0x8) in the CCCR.
#define CARD_CAP_LSC_MASK  (1 << 6)
#define CARD_CAP_4BLS_MASK (1 << 7)

// Bit positions within the UHS1 Support byte (0x14) in the CCCR.
#define SSDR50_SUPPORT     (1 << 0)
#define SSDR104_SUPPORT    (1 << 1)
#define SDDR50_SUPPORT     (1 << 2)


/* CSD field definitions */
#define CSD_SPEC_VER_4      4           /* Implements system specification 4.0 - 4.1 */

/* EXT_CSD fields*/
#define EXT_CSD_PARTITION_ATTRIBUTE	156	/* R/W */
#define EXT_CSD_PARTITION_SUPPORT	160	/* RO */
#define EXT_CSD_WR_REL_PARAM		166	/* RO */
#define EXT_CSD_ERASE_GROUP_DEF		175	/* R/W */
#define EXT_CSD_PART_CONFIG		179	/* R/W */
#define EXT_CSD_ERASED_MEM_CONT		181	/* RO */
#define EXT_CSD_HS_TIMING		185	/* R/W */
#define EXT_CSD_REV			192	/* RO */
#define EXT_CSD_STRUCTURE		194	/* RO */
#define EXT_CSD_CARD_TYPE		196	/* RO */
#define EXT_CSD_PART_SWITCH_TIME        199     /* RO */
#define EXT_CSD_SEC_CNT			212	/* RO, 4 bytes */
#define EXT_CSD_S_A_TIMEOUT		217	/* RO */
#define EXT_CSD_REL_WR_SEC_C		222	/* RO */
#define EXT_CSD_HC_WP_GRP_SIZE		221	/* RO */
#define EXT_CSD_ERASE_TIMEOUT_MULT	223	/* RO */
#define EXT_CSD_HC_ERASE_GRP_SIZE	224	/* RO */
#define EXT_CSD_BOOT_MULT		226	/* RO */
#define EXT_CSD_SEC_TRIM_MULT		229	/* RO */
#define EXT_CSD_SEC_ERASE_MULT		230	/* RO */
#define EXT_CSD_SEC_FEATURE_SUPPORT	231	/* RO */
#define EXT_CSD_TRIM_MULT		232	/* RO */



/* EXT_CSD field definitions */

#define EXT_CSD_CMD_SET_NORMAL		(1<<0)

#define EXT_CSD_CARD_TYPE_26	        (1<<0)	/* Card can run at 26MHz */
#define EXT_CSD_CARD_TYPE_52	        (1<<1)	/* Card can run at 52MHz */
#define EXT_CSD_CARD_TYPE_MASK	         0xF	/* Mask out reserved bits */
#define EXT_CSD_CARD_TYPE_DDR_1_8V      (1<<2)  /* Card can run at 52MHz */  /* DDR mode @1.8V or 3V I/O */
#define EXT_CSD_CARD_TYPE_DDR_1_2V      (1<<3)   /* Card can run at 52MHz */ /* DDR mode @1.2V I/O */
#define EXT_CSD_CARD_TYPE_DDR_52       (EXT_CSD_CARD_TYPE_DDR_1_8V | EXT_CSD_CARD_TYPE_DDR_1_2V)


// The function base register attributes.

// The base address for the FBR.
#define FBR_BASE_ADRS            0x100
#define FBR_INTF_CODE_OFFSET     0x1


/* SD card states */
#define SD_CARD_STATE_IDLE        0
#define SD_CARD_STATE_STANDBY     3
#define SD_CARD_STATE_TRANSFER    4
#define SD_CARD_STATE_PROGRAMMING 7


/*SD BUS MODES*/
#define SD_SDR12_BUS_MODE    0x00
#define SD_SDR25_BUS_MODE   0x01
#define SD_SDR50_BUS_MODE   0x02
#define SD_SDR104_BUS_MODE  0x03
#define SD_DDR50_BUS_MODE   0x04

#define SD_CMD6_DATA_SIZE_IN_BYTES 64

/* ADMA Descriptor Table Attribute Mask */
#define DRV_SDHC_ADMA_DESC_TBL_ATT_ACT12_NOP  0x00
#define DRV_SDHC_ADMA_DESC_TBL_ATT_ACT12_TRANFER_ONE_LINE  0x20
#define DRV_SDHC_ADMA_DESC_TBL_ATT_ACT12_LINK_TO_NEXT_LINE 0x30

#define DRV_SDHC_ADMA_DESC_TBL_ATT_INT_EN_BIT (1<<2)
#define DRV_SDHC_ADMA_DESC_TBL_ATT_LAST_LINE_BIT (1<<1)
#define DRV_SDHC_ADMA_DESC_TBL_ATT_VALID_BIT (1<<0)

// Enums
/**
 *
 * \brief Used in CMD6 to select either "Check function" or "Switch function"
 * See Physical Layer Spec 2.00 Chapter 4.3.10
 *
 */
typedef enum sd_switch_function_mode_e
{
    sd_switch_function_mode_check, /*!< Option to select when we want to tell
                                     CMD 6 to check the current function mode */

    sd_switch_function_mode_switch /*!< Option to select when we want to tell
                                    CMD 6 to switch the current function mode */

} sd_switch_function_mode_t;

/**
 *
 * \brief Used in CMD6 to select a function group
 * See Physical Layer Spec 2.00 Chapter 4.3.10
 *
 */
typedef enum sd_switch_function_group_e
{
    sd_switch_function_group_1, /*!< Option to select when we want to tell
                                 CMD 6 to use function group 1 for its check
                                 or switch */

    sd_switch_function_group_2 /*!< Option to select when we want to tell
                                CMD 6 to use function group 2 for its check
                                or switch */

} sd_switch_function_group_t;

/* SCR Masks */
#define SD_SCR_4_BIT_BUS_MASK (1<<2)


#define MMC_CARD_BUSY	0x8000   /*Card Power up status bit(ocr register)*/

/* MMC_SWITCH access modes */
#define MMC_SWITCH_MODE_WRITE_BYTE	0x03	/* Set target to value */

/* Delay values */
#define SD_DELAY_1_SECONDS         100 

/*brief Available bus-widths in SD*/
typedef enum sd_bus_width_e
{
    sd_bus_width_1bit, /*!< bus width is 1-bit */

    sd_bus_width_4bit,  /*!< bus width is 4-bit */

	sd_bus_width_8bit  /*!< bus width is 8-bit */

} sd_bus_width_t;


/*brief Options to select or deselect a card, used in command 7*/

typedef enum sd_select_option_e
{
    sd_select_option_sel,    /*!< select a card */

    sd_select_option_desel  /*!< deselect a card */

} sd_select_option_t;

/*brief SD Speed classes, as defined in the 2.0 Physical layer spec.*/

typedef enum sd_speed_class_e
{
    sd_speed_class0, /*!< Represents the card's speed class 0 */

    sd_speed_class2, /*!< Represents the card's speed class 2 */

    sd_speed_class4, /*!< Represents the card's speed class 4 */

    sd_speed_class6  /*!< Represents the card's speed class 6 */

} sd_speed_class_t;

/* brief SD Specification Version definitions*/
 
typedef enum sd_spec_version_e
{
    sd_spec_version_1_01, /*!< Represents the card's spec version (1.01) */

    sd_spec_version_1_10, /*!< Represents the card's spec version (1.10) */

    sd_spec_version_2_00,  /*!< Represents the card's spec version (2.00) */
      
    sd_spec_version_3_00  /*!< Represents the card's spec version (3.00) */

} sd_spec_version_t;

/*brief Options for read transfer or write transfer*/
typedef enum sd_transfer_option_e
{
    sd_transfer_option_read, /*!< Option used when wanting to do a read transfer */

    sd_transfer_option_write /*!< Option used when wanting to do a write transfer */

} sd_transfer_option_t;

/*brief options for command 0 states*/
typedef enum mmc_cmd0_states_e
{
	mmc_go_pre_idle,
		
	mmc_go_idle,
	
	mmc_get_boot_code
	
}mmc_cmd0_states_t;

typedef enum sd_response_type_e
{
    sd_response_none,   /*!< no response type */
    sd_response_r1,     /*!< normal response command */
    sd_response_r1b,    /*!< normal with busy signal */
    sd_response_r2,     /*!< CID, CSD register */
    sd_response_r3,     /*!< OCR register */
    sd_response_r4,     /*!< */
    sd_response_r5,     /*!< */
    sd_response_r6,     /*!< Published RCA response  */
    sd_response_r7      /*!< Card interface condition  */
      
}sd_response_type_t;

typedef enum resp5_error_e
{
    RESP5_RANGE_ERR       = 0,
    RESP5_INVALID_FNC_NUM = 1,
    RESP5_UNKNOWN_ERR     = 3,
    RESP5_ILLEGAL_CMD     = 6,
    RESP5_CRC_ERR         = 7,
    RESP5_NO_ERR          = 8

} resp5_error_t;

typedef struct sd_cmd_s 
{
    sd_response_type_t  resp_type;       /*The command's response type*/                                          
    uint32_t         argument;            /*The argument parameter*/       
    uint16_t         xfer_mode;          /*The transfer mode register value to be used for this command*/
    uint16_t         opcode;             /*A valid SD command number.*/                                
    uint16_t         rca;                /*The card's relative card address */ 
    uint16_t         response[8];        /*The response, to be filled after the command completes. */
    bool             data_present;       /*Indicates whether data is present or not*/  
	bool			 cmd_completed;
	bool			 data_completed;
	bool			 buffer_ready;
	bool             data_crc_error_flag;
    uint32_t         cmd_error_flag;
} sd_cmd_t;

/*brief Options for ACMD42 for setting or clearing the internal card detect*/
typedef enum sd_card_detect_option_e
{
    sd_card_detect_option_set,  /*!< Tell ACMD42 to set the internal card
                                  detect */

    sd_card_detect_option_clear /*!< Tell ACMD42 to clear the internal
                                  card detect */

} sd_card_detect_option_t;

/*brief Interpreted data from the CSD register as needed */
typedef struct sd_csd_info_s
{
    uint32_t    block_size;         /*!< The card's block size as interpreted
                                     from the card's CSD register */

    uint32_t    total_blocks;       /*!< The total number of blocks as
                                     interpreted from the card's CSD register */

    bool        is_high_capacity;   /*!< true if the block is high capacity
                                     and false otherwise (as intepreted from
                                     the card's CSD register) */

} sd_csd_info_t;

typedef struct sd_mem_ocr_info_s
{
    bool        power_up;       /*!< Indicates whether (true) or not (false)
                                 this card has been fully powered up yet. */

    bool        ccs;            /*!< Indicates (after the card's OCR has
                                  been read) if this card supports (true)
                                 high capacity or not (false)*/

    bool S18A;								

    uint32_t    voltage_window; 

} sd_mem_ocr_info_t;

/*brief Enumeration of the SD card that is plugged in*/
typedef enum sd_card_type_e
{
    SD_CARD_TYPE_NONE,
    SD_CARD_TYPE_MEMORY_ONLY,
    SD_CARD_TYPE_MMC,
    SD_CARD_TYPE_IO_ONLY,
    SD_CARD_TYPE_COMBO

} sd_card_type_t;

/* CSD structure for emmc */
typedef struct mmc_csd {
	uint8_t		    structure;
	uint8_t 	    mmca_vsn;
	uint32_t        capacity;
	uint8_t		    max_dtr;
}mmc_csd_t;

/* EXT_CSD structure for emmc */
typedef struct mmc_ext_csd {
	
		uint8_t			rev;
		uint8_t			erase_group_def;
		uint8_t			sec_feature_support;
		uint8_t			rel_sectors;
		uint8_t			rel_param;
		uint8_t			part_config;
		uint32_t		part_time;		/* Units: ms */
		uint32_t		sa_timeout; 	/* Units: 100ns */
		uint32_t		hs_max_dtr;
		uint32_t		sectors;
		uint32_t		card_type;
		uint32_t		hc_erase_size;		/* In sectors */
		uint32_t		hc_erase_timeout;	/* In milliseconds */
		uint32_t		sec_trim_mult;	/* Secure trim multiplier  */
		uint32_t		sec_erase_mult; /* Secure erase multiplier */
		uint32_t		trim_timeout;		/* In milliseconds */
		uint32_t		enhanced_area_size; /* Units: KB */
		uint32_t		boot_size;		/* in bytes */
		uint8_t			raw_partition_support;	/* 160 */
		uint8_t			raw_erased_mem_count;	/* 181 */
		uint8_t			raw_ext_csd_structure;	/* 194 */
		uint8_t			raw_card_type;		/* 196 */
		uint8_t			raw_s_a_timeout;		/* 217 */
		uint8_t			raw_hc_erase_gap_size;	/* 221 */
		uint8_t			raw_erase_timeout_mult; /* 223 */
		uint8_t			raw_hc_erase_grp_size;	/* 224 */
		uint8_t			raw_sec_trim_mult;	/* 229 */
		uint8_t			raw_sec_erase_mult; /* 230 */
		uint8_t			raw_sec_feature_support;/* 231 */
		uint8_t			raw_trim_mult;		/* 232 */
		uint8_t			raw_sectors[4]; 	/* 212 - 4 bytes */
		bool			enhanced_area_en;	/* enable bit */
		unsigned long long	enhanced_area_offset;	/* Units: Byte */
		
}mmc_ext_csd_t;


typedef struct SDIO_Fuse_settings_s{		
		uint32_t sdio_bootcode_addr;
		uint32_t shared_clk;
		uint32_t shared_int;
		uint8_t ip_mode;
		uint8_t slot_type;
		uint8_t card_type;
		uint8_t bus_width;
		uint8_t bus_mode;
		bool use_dma;
		bool use_preset;
}drv_sdhc_fuse_settings_t;

typedef struct ADMA_Descriptor
{
    uint16_t    attribute;    
    uint16_t    length;
    uint32_t    address;
}drv_sdhc_adma_descriptor;

typedef struct aizyc_sd_host
{
  uint32_t base_addr;
  uint32_t config_addr;
  uint32_t int_num;
  uint32_t version;
  uint32_t flags;
  uint32_t clk;  
  uint32_t timeout_clk;
  uint32_t max_clk;
  uint32_t min_clk;
  uint32_t shared_clk;
  uint32_t shared_int;  
  uint32_t pwr;
  uint32_t caps1;
  uint32_t caps2;
  uint16_t cur_blk_size;
  uint16_t cur_blk_cnt;
  const drv_sdhc_fuse_settings_t *fuse_settings;
  sd_cmd_t* command;
  drv_sdhc_adma_descriptor *descriptor_table;
} drv_sdhc_sd_host_t;

typedef enum sd_io_interface_e
{
    SD_IO_INTERFACE_UART,
    SD_IO_INTERFACE_BLUETOOTH_TYPE_A,
    SD_IO_INTERFACE_BLUETOOTH_TYPE_B,
    SD_IO_INTERFACE_GPS,
    SD_IO_INTERFACE_CAMERA,
    SD_IO_INTERFACE_PHS,
    SD_IO_INTERFACE_WLAN,
    SD_IO_INTERFACE_ATA,
    SD_IO_INTERFACE_BLUETOOTH_TYPE_A_AMP

} sd_io_interface_t;
typedef struct fbr_reg_info_s
{
    uint16_t         func_num;
    sd_io_interface_t intf_code;

} fbr_reg_info_t;


typedef struct fbr_all_regs_s
{
    uint8_t        num_fbr;
    fbr_reg_info_t *fbr_regs;

} fbr_all_regs_t;


typedef struct aizyc_host_shared{

	uint32_t  number_of_clk_pins;
	uint32_t  number_of_intr_pins;
	uint32_t  bus_width_preset;
	uint32_t  clk_selected;
}aizyc_host_shared_t;

typedef struct sdio_op_cond_info_s
{
    uint8_t  card_rdy_after_init; /*!< Whether the card is ready to operate after
                                       initialization. */

    uint8_t  num_io_fncs; /*!< The number of I/O functions present. */

    uint8_t  mem_present; /*!< Whether there is memory present, a combo card. */

    uint32_t min_max_VDD; /*!<  */
    
    uint8_t S18_A;   /*additional feature for sdio3.0*/
    
} sdio_op_cond_info_t;
    

typedef struct sd_handle_s
{
	uint16_t cid[8];
	uint16_t raw_csd[8];
	uint16_t ocr;
	uint16_t rocr_low;
	uint16_t rocr_high;
	uint8_t sd_bus_speed;
	
    bool                        is_open; /*true:if the bus assocaited with this handle has been opened, false otherwise */
    bool                        csd_is_valid;
    bool                        supports_4_bit_bus; 
    bool                        is_transfer_ready; /*!< true:if the card has been initialised and is ready to start transferring data */
    bool                        card_init_detect_run;
	bool						mmc_supports_dual_voltage;
	bool                        cmd8_support;
    bool                        signal_switch;
    
    sd_speed_class_t            speed_class;    /*The card's speed class*/
    sd_spec_version_t           spec_version;   /*the card's spec version*/                                   
    sd_csd_info_t               csd_info;       /*CSD info */
    sd_card_type_t              cur_card_type;  /*card types*/
    drv_sdhc_sd_host_t             *host;           /*host structure*/
	aizyc_host_shared_t			*shost;

    uint16_t                rca;   /*The card's relative address*/ 
    uint8_t                 num_io_functions;
    
    mmc_csd_t	          csd;		/* card specific */
    mmc_ext_csd_t	      ext_csd;	/* mmc v4 extended card specific */
    uint8_t          	  erased_byte;	/* value of erased bytes */
    uint32_t              state;        /* card state */
    unsigned long	      caps;		/* Host capabilities */
    
   sdio_op_cond_info_t *io_resp;
   
    #define SD_SCR_SIZE_IN_BYTES 8
      unsigned char scr[SD_SCR_SIZE_IN_BYTES]; /*scr register*/
    #define SD_STATUS_SIZE_IN_BYTES 64 
      unsigned char sd_status[SD_STATUS_SIZE_IN_BYTES]; /*sd status register*/

     uint8_t switch_status[SD_CMD6_DATA_SIZE_IN_BYTES];  /*switch status register*/
	 
    /*sdio related*/
   fbr_all_regs_t              *fbr_all_regs; /*!< */
}drv_sdhc_sd_handle_t;

 /**
 * \brief Used with SDIO. This will set whether to access a register through
 * CMD52 direct access (single) or CMD53, extended access (multiple). */
typedef enum sd_io_reg_acs_e
{
       SD_IO_REG_ACS_DIRECT = 1,
       SD_IO_REG_ACS_EXTENDED

} sd_io_reg_acs_t;


typedef enum ip_mode_e{
		HOST_SD,
		HOST_SPI,
		DEV_SD,
		DEV_SPI
} ip_mode_t;

typedef enum slot_type_e{
		REMOVABLE,
		SHARED_BUS
}slot_type_t;

typedef enum card_type_e {
		TYPE_MMC,
		TYPE_SD,
		TYPE_SDIO,
		TYPE_eMMC,
		TYPE_eSD,
		TYPE_eSDIO
}card_type_t;

typedef enum bus_speed_mode_e{
	    SDR12_MODE,
		SDR25_MODE,
		SDR50_MODE,
		SDR104_MODE,
		DDR_50_MODE,
		MMC_SDR25_MODE,
		MMC_SDR50_MODE,
		MMC_1_2V_DDR_MODE,
		MMC_1_8V_DDR_MODE
}bus_speed_mode_t;

typedef enum bus_width_e{
		BIT_1,
		BIT_4,
		BIT_8
}bus_width_t;


#endif	
