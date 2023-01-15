/*******************************************************************************/

#ifndef _WIFI_FLASH_USE_H
#define _WIFI_FLASH_USE_H

#include "system_config.h"
#include "system_definitions.h"


#define IPF_WIFI_CALIBRATION_START 0x001FE000
#define IPF_WIFI_CALIBRATION_LENTH 256

#define IPF_WIFI_REGDOMAINCONFIG_START (IPF_WIFI_CALIBRATION_START+0x1000)

#define IPF_WIFI_MACADDRESS_START 0x001FE200
#define IPF_WIFI_MACADDRESS_LENGTH 12

#define IPF_WIFI_CONFIG_START               0x001F7000
#define DEFAULT_SSID                        "microchip"
#define DEFAULT_COUNTRYCODE                 "USA"

typedef enum
{
	/* Application's state machine's initial state. */
    IPF_STATE_INIT=0,
    IPF_STATE_READ,
    IPF_STATE_READ_REGDOMAINCONFIG,
    IPF_STATE_READ_CAL,
    IPF_STATE_READ_MACADDR,
    IPF_STATE_WRITE,
    IPF_STATE_WRITE_REGDOMAINCONFIG,
    IPF_STATE_ERASE,
    IPF_STATE_ERASE_REGDOMAINCONFIG,
    IPF_STATE_PROTECTCAL,
    IPF_STATE_UNPROTECTCAL,
    IPF_STATE_ERROR,
    IPF_STATE_IDLE,
} IPF_STATES;

typedef struct
{
    IPF_STATES state;     /* The IPF's current state from WiFi*/
    IPF_STATES prevState;
    DRV_HANDLE IPF_Handle;
    uint8_t reqComplete;
    /* TODO: Define any additional data used by the application. */
} IPF_INFO;

IPF_INFO ipfInfo;

typedef enum
{
	CC_GEN	=  0,	/*Enable all supported channels*/
	CC_ALL,
	CC_TWN  =  158,
	CC_SGP  =  702,
	CC_USA  =  840, 	/* NON-FCC*/
	CC_JPN  =  392,
	CC_RUS  =  643,
	CC_KOR  =  410,
	CC_ISR  =  376, 	/* EMEA*/
	CC_CHN  =  156,
	MAX_REGIONS = 10,
}TxPowerCountryCode;
typedef enum
{
	WLAN_OPER_GET_TX_PWR=4,
	WLAN_OPER_RX_PWR_ADJ,
    WLAN_OPER_COUNTRYCODE,
    WLAN_OPER_2GENB2040,
    WLAN_OPER_CHNBITMAP2G,
    WLAN_OPER_CHNBITMAP5G,
}WLAN_OPER_ID;
typedef enum
{
	/* MAC Boot Mode */
    BOOT_STA_MODE =0,    
    BOOT_AP_MODE,
    BOOT_INVALID_MODE,
} MAC_BOOT_MODES;

bool WIFI_initialization();
bool WIFI_IPF_open();
void WIFI_IPF_write();
void WIFI_IPF_read();
void WIFI_IPF_erase(void);
bool WIFI_IPF_usrconfig_memalloc();
bool WIFI_IPF_usrconfig_memfree();
void WIFI_IPF_readAndBackup();
void do_switchMode();
void WIFI_IPF_save_regdom();
void WIFI_IPF_save();
void WIFI_IPF_read_cal();
bool WIFI_IPF_cal_memalloc();
bool WIFI_IPF_cal_memfree();
bool WIFI_IPF_regdomconfig_memalloc();
bool WIFI_IPF_regdomconfig_memfree();
void WIFI_IPF_calregion_protect();
void WIFI_IPF_calregion_unprotect();

#endif
/*******************************************************************************
 End of File
 */

