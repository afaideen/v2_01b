/*******************************************************************************
Using Flash to Read and Write WiFi configuration

  Summary:
    To Access flash for Setting and Reading WiFi Configuration

  Description:
    - Saves the WiFi configuration data
    - Reads the WiFi configuration data
    - Set the saved WiFi configuration data in Driver
    
 *******************************************************************************/

/*******************************************************************************
File Name: wifi_flash_use.c

Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
#include "wifi_flash_use.h"
#include "WiFi_MW.h"
#include "drv_mchpwlan.h"
#include "WiFi_commands.h"
#ifdef DRV_WIFI_OTA_ENABLE
#include "utilities/hex_parser/hex_parser.h"
#endif
#include "tcpip/src/tcpip_manager_control.h" 
#include "driver/wifi/pic32wk/src/wifi_web_config.h"

/*******************************************************************************
    Global Variables
    Info: Flash uses only Global variables for writing/reading data
*******************************************************************************/
char IPF_ready_use_state = 0;
unsigned int g_bootMode; 
uint8_t macaddr_mem[12]; 
uint8_t macaddr_val[6];
int web_data = 0;

//Read Write testing buffers
uint8_t buf[100];
uint8_t ipf_read_buf[100] = {0, };

//Calibration Memory
unsigned int *Calib_mem = NULL; 

//Wifi Config Data to Save in Flash
extern wifiConfigData    *g_wifiConfigData;
extern wifiRegdomainConfig *g_wifiRegdomainConfig;
extern t_wfEasyConfigCtx    g_easyConfigCtx;
static wifiConfigData temp_wifiConfigData;
static wifiRegdomainConfig  temp_wifiRegdomainConfig;

void DRV_IPF_UnProtectMemoryVolatile (DRV_HANDLE clientHandle,DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle, 
                                        uintptr_t memAddress,DRV_IPF_PROT_MODE protMode);
void DRV_IPF_ProtectMemoryVolatile (DRV_HANDLE clientHandle,DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle, 
                                        uintptr_t memAddress,DRV_IPF_PROT_MODE protMode);
#ifdef DRV_WIFI_OTA_ENABLE
void InitializeFlashHandler(DRV_HANDLE handle,DRV_IPF_EVENT_HANDLER eventHandler);
#endif

extern uint8_t             g_macaddress[6];
typedef int (*FUNC_PTR)(WLAN_OPER_ID operId, unsigned char *dataPtr);
extern int hook_wlan_ctrl_handle(FUNC_PTR wlan_ctl_handle);
/*******************************************************************************/
/*****************************************************************************                                                                        
  Function Name : checkreadssid                                  
                                                                           
  Description   : This function check the SSID received from IPF. 
                                        
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : return 1        - if SSID in IPF is not set(default erase value of IPF).
                  else return 0   - if SSID in IPF is set(User set SSID)                                                  
*****************************************************************************/

bool checkreadssid ()
{
    unsigned char idx =0;
    for (idx =0;((idx<32) && (g_wifiConfigData->ssid[idx] == 0xff));idx++);
    
    if(idx == 32){
        return 1;
     }
     return 0;
}
/****************************************************************************************************                                                                        
  Function Name : checkregdomainconfig                                  
                                                                           
  Description   : This function check the regulatory domain configuration value from IPF. 
                                        
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : return 1        - if Reg domain values in IPF is not set(default erase value of IPF).
                  else return 0   - if Reg domain values in IPF is set(either set by BIST mode or by user)                                                  
*******************************************************************************************************/

bool checkregdomainconfig()
{
    
    if((g_wifiRegdomainConfig->ChannelBitmap2Ghz == 0xff) && (g_wifiRegdomainConfig->ChannelBitmap5Ghz== 0xff))
    {
        return 1;
    }
    return 0;
}
/*****************************************************************************                                                                        
  Function Name : updatedefault_wifiuserconfig                                  
                                                                           
  Description   : This function set default setting of wlan user configuration .
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/

void updatedefault_wifiuserconfig()
{
   //updating Default ssid
    strcpy((char *)g_wifiConfigData->ssid,DEFAULT_SSID);
    
    
}
/*****************************************************************************                                                                        
  Function Name : updatedefault_regdomainconfig                                  
                                                                           
  Description   : This function set default setting of regulatory domain configuration .
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/

void updatedefault_regdomainconfig()
{

    unsigned char tx2G_20_US[14]      =   {133,141,141,141,141,141,141,141,141,141,131,1,1,1};
    unsigned char tx2G_BB_US[14]      =   {111,111,111,174,174,174,174,174,174,174,173,1,1,1};
    unsigned char tx2G_40_US[11]      =   {1,1,124,131,131,131,131,131,90,1,1};
    unsigned char tx5G_20_US[28]      =   {125,157,157,158,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,132,124,124,124,127,1,1,1,1};
    unsigned char tx5G_40_US[11]      =   {118,162,1,1,1,1,1,1,1,136,137};
    unsigned char tx2G_20_EU[14]      =   {142,141,141,141,141,141,141,141,141,141,141,141,149,1};
    unsigned char tx2G_BB_EU[14]      =   {140,154,154,154,154,154,154,154,154,154,154,154,154,1};
    unsigned char tx2G_40_EU[11]      =   {1,1,125,134,134,134,134,134,134,134,129};
    unsigned char tx5G_20_EU[28]      =   {123,123,123,123,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
    unsigned char tx5G_40_EU[11]      =   {124,124,1,1,1,1,1,1,1,1,1};
    unsigned char tx2G_20_JP[14]      =   {142,141,141,141,141,141,141,141,141,141,141,141,149,1};
    unsigned char tx2G_BB_JP[14]      =   {140,154,154,154,154,154,154,154,154,154,154,154,154,1};
    unsigned char tx2G_40_JP[14]      =   {1,1,125,134,134,134,134,134,134,134,129};
    unsigned char tx5G_20_JP[28]      =   {123,123,123,123,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
    unsigned char tx5G_40_JP[11]      =   {124,124,1,1,1,1,1,1,1,1,1};
    unsigned char tx2G_20_GN[14]      =   {170,170,170,170,170,170,170,170,170,170,170,170,170,170};
    unsigned char tx2G_BB_GN[14]      =   {200,200,200,200,200,200,200,200,200,200,200,200,200,200};
    unsigned char tx2G_40_GN[11]      =   {170,170,170,170,170,170,170,170,170,170,170};
    unsigned char tx5G_20_GN[28]      =   {170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170,170};
    unsigned char tx5G_40_GN[11]      =   {170,170,170,170,170,170,170,170,170,170,170};
    unsigned char tx2G_20_CUST[14]    =   {142,141,141,141,141,141,141,141,141,141,131,1,1,1};
    unsigned char tx2G_BB_CUST[14]    =   {185,188,188,188,188,188,188,188,188,188,179,1,1,1};
    unsigned char tx2G_40_CUST[11]    =   {1,1,124,131,131,131,131,131,90,1,1};
    unsigned char tx5G_20_CUST[28]    =   {123,123,123,123,123,123,123,123,123,123,123,123,123,123,123,123,123,123,123,100,123,123,123,123,1,1,1,1};
    unsigned char tx5G_40_CUST[11]    =   {124,124,124,124,124,124,124,124,124,107,124};
    
    unsigned char   rx2G_20[14]       =   {98,98,98,98,98,98,98,98,98,98,98,98,98,98};
    unsigned char   rx2G_40[11]       =   {98,98,98,98,98,98,98,98,98,98,98};
    unsigned char   rx5G_20[28]       =   {100,100,100,100,100,100,100,100,100,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,94,94,94,94};
    unsigned char   rx5G_40[11]       =   {100,100,100,100,100,96,96,96,96,96,96};
 
    
    g_wifiRegdomainConfig->ChannelBitmap5Ghz = g_wifiRegdomainConfig->ChannelBitmap2Ghz = 0;
    
    strcpy((char *)g_wifiRegdomainConfig->CountryCode,DEFAULT_COUNTRYCODE);
    g_wifiRegdomainConfig->FreqBand               = 3;
    g_wifiRegdomainConfig->Enable2g2040           = 0;
    
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_20_US,(const char *)tx2G_20_US);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_BB_US,(const char *)tx2G_BB_US);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_40_US,(const char *)tx2G_40_US);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_20_US,(const char *)tx5G_20_US);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_40_US,(const char *)tx5G_40_US);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_20_EU,(const char *)tx2G_20_EU);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_BB_EU,(const char *)tx2G_BB_EU);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_40_EU,(const char *)tx2G_40_EU);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_20_EU,(const char *)tx5G_20_EU);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_40_EU,(const char *)tx5G_40_EU);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_20_JP,(const char *)tx2G_20_JP);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_BB_JP,(const char *)tx2G_BB_JP);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_40_JP,(const char *)tx2G_40_JP);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_20_JP,(const char *)tx5G_20_JP);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_40_JP,(const char *)tx5G_40_JP);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_20_GN,(const char *)tx2G_20_GN);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_BB_GN,(const char *)tx2G_BB_GN);    
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_40_GN,(const char *)tx2G_40_GN);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_20_GN,(const char *)tx5G_20_GN);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_40_GN,(const char *)tx5G_40_GN);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_20_CUST,(const char *)tx2G_20_CUST);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_BB_CUST,(const char *)tx2G_BB_CUST);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_2G_40_CUST,(const char *)tx2G_40_CUST);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_20_CUST,(const char *)tx5G_20_CUST);
    strcpy((char *)g_wifiRegdomainConfig->Txpow_5G_40_CUST,(const char *)tx5G_40_CUST);
    g_wifiRegdomainConfig->Txpow_ANT_GAIN_2G      = 20;
    g_wifiRegdomainConfig->Txpow_ANT_GAIN_5G      = 35 ;
 
    strcpy((char *)g_wifiRegdomainConfig->RxpowAdj_2G_20,(const char *)rx2G_20);
    strcpy((char *)g_wifiRegdomainConfig->RxpowAdj_2G_40,(const char *)rx2G_40);
    strcpy((char *)g_wifiRegdomainConfig->RxpowAdj_5G_20,(const char *)rx5G_20);
    strcpy((char *)g_wifiRegdomainConfig->RxpowAdj_5G_40,(const char *)rx5G_40);
     
}

TxPowerCountryCode convertCountryCode(const char *code)
{
	if (strncmp(code, "EMEA", 4)==0) 
	{
		return CC_ISR;
	} 
	else if (strncmp(code, "NFCC", 4)==0) 
	{
		return CC_USA;
	}
	else if (strncmp(code, "RUS", 3)==0) 
	{
		return CC_RUS;
	} 
	else if (strncmp(code, "USA", 3)==0) 
	{
		return CC_USA;
	}
	else if (strncmp(code, "TWN", 3)==0)
	{
		return CC_TWN;
	}
	else if (strncmp(code, "ISR", 3)==0)
	{
		return CC_ISR;
	}
	else if (strncmp(code, "CHN", 3)==0)
	{
		return CC_CHN;
	}
	else if (strncmp(code, "JPN", 3)==0)
	{
		return CC_JPN;
	}
	else if (strncmp(code, "SGP", 3)==0)
	{
		return CC_SGP;
	}
	else if (strncmp(code, "KOR", 3)==0)
	{
		return CC_KOR;
	}	
	else if(strncmp(code, "GEN", 3)==0)
	{
		return CC_GEN;
	}
	else if(strncmp(code, "GN", 2) == 0)
	{
		return CC_GEN;
	}
	else if(strncmp(code, "JP", 2) == 0)
	{
		return CC_JPN;
	}
	else if(strncmp(code, "EU", 2) == 0)
	{
		return CC_ISR;
	}
	else if(strncmp(code, "US", 2) == 0)
	{
		return CC_USA;
	}	
	else
	{
		return CC_GEN;
	}
}

int getConfigTxPowerLevel(unsigned char* pwr_val)
{
    TxPowerCountryCode CountryCode = CC_GEN;
    unsigned char *str_20_2G;
    unsigned char *str_BB_2G;
    unsigned char *str_40_2G;
    unsigned char *str_20_5G;
    unsigned char *str_40_5G;
    unsigned char Inx =0;
    
    CountryCode=convertCountryCode((const char *)g_wifiRegdomainConfig->CountryCode);
    if(g_wifiRegdomainConfig->ChannelBitmap2Ghz != 0 || g_wifiRegdomainConfig->ChannelBitmap5Ghz != 0)
	{
        str_20_2G = g_wifiRegdomainConfig->Txpow_2G_20_CUST;
        str_BB_2G = g_wifiRegdomainConfig->Txpow_2G_BB_CUST;
        str_40_2G = g_wifiRegdomainConfig->Txpow_2G_40_CUST;
        str_20_5G = g_wifiRegdomainConfig->Txpow_5G_20_CUST;
        str_40_5G = g_wifiRegdomainConfig->Txpow_5G_40_CUST;
  	} else {
        
        if(CC_USA == CountryCode)
		{
            str_20_2G = g_wifiRegdomainConfig->Txpow_2G_20_US;
            str_BB_2G = g_wifiRegdomainConfig->Txpow_2G_BB_US;
            str_40_2G = g_wifiRegdomainConfig->Txpow_2G_40_US;
            str_20_5G = g_wifiRegdomainConfig->Txpow_5G_20_US;
            str_40_5G = g_wifiRegdomainConfig->Txpow_5G_40_US;
           
		}
		else if(CC_ISR == CountryCode) //Channel mapping is same as EMEA
		{
            str_20_2G = g_wifiRegdomainConfig->Txpow_2G_20_EU;
            str_BB_2G = g_wifiRegdomainConfig->Txpow_2G_BB_EU;
            str_40_2G = g_wifiRegdomainConfig->Txpow_2G_40_EU;
            str_20_5G = g_wifiRegdomainConfig->Txpow_5G_20_EU;
            str_40_5G = g_wifiRegdomainConfig->Txpow_5G_40_EU;
		}
		else if(CC_JPN == CountryCode)
		{
            str_20_2G = g_wifiRegdomainConfig->Txpow_2G_20_JP;
            str_BB_2G = g_wifiRegdomainConfig->Txpow_2G_BB_JP;
            str_40_2G = g_wifiRegdomainConfig->Txpow_2G_40_JP;
            str_20_5G = g_wifiRegdomainConfig->Txpow_5G_20_JP;
            str_40_5G = g_wifiRegdomainConfig->Txpow_5G_40_JP;
		}
		else
		{
            str_20_2G = g_wifiRegdomainConfig->Txpow_2G_20_GN;
            str_BB_2G = g_wifiRegdomainConfig->Txpow_2G_BB_GN;
            str_40_2G = g_wifiRegdomainConfig->Txpow_2G_40_GN;
            str_20_5G = g_wifiRegdomainConfig->Txpow_5G_20_GN;
            str_40_5G = g_wifiRegdomainConfig->Txpow_5G_40_GN;
		}
	}
    for(Inx=0;Inx<14;Inx++){
        pwr_val[Inx]=str_20_2G[Inx];
    }
   
    for(Inx=0;Inx<14;Inx++){
        pwr_val[14+Inx]=str_BB_2G[Inx];
    }
    for(Inx=0;Inx<11;Inx++){
        pwr_val[28+Inx]=str_40_2G[Inx];
    }    
    for(Inx=0;Inx<28;Inx++){
        pwr_val[39+Inx]=str_20_5G[Inx];
    }    
    for(Inx=0;Inx<11;Inx++){
        pwr_val[67+Inx]=str_40_5G[Inx];
    }    

    pwr_val[78] = (CC_USA == CountryCode)?g_wifiRegdomainConfig->Txpow_ANT_GAIN_2G:20;
    pwr_val[79] = (CC_USA == CountryCode)?g_wifiRegdomainConfig->Txpow_ANT_GAIN_5G:35;
 
    return 0;
}
int getConfigRxPowerLevel(unsigned char* pwr_val)
{
    unsigned char Inx =0;
    
    for(Inx=0;Inx<14;Inx++){
        pwr_val[Inx]=g_wifiRegdomainConfig->RxpowAdj_2G_20[Inx];
    }
    
    for(Inx=0;Inx<11;Inx++){
        pwr_val[14+Inx]=g_wifiRegdomainConfig->RxpowAdj_2G_40[Inx];
    }
    
    for(Inx=0;Inx<11;Inx++){
        pwr_val[25+Inx]=g_wifiRegdomainConfig->RxpowAdj_5G_20[Inx];
    }
    
    for(Inx=0;Inx<28;Inx++){
        pwr_val[25+Inx]=g_wifiRegdomainConfig->RxpowAdj_5G_20[Inx];
    }
    
    for(Inx=0;Inx<11;Inx++){
        pwr_val[53+Inx]=g_wifiRegdomainConfig->RxpowAdj_5G_40[Inx];
    }
    return 0;
}
/*****************************************************************************                                                                        
  Function Name : WIFI_oper_Handle                                  
                                                                           
  Description   : This function provide access to wlan configuration.           
                                                                           
  Inputs        : operationId: operation id
                  argStruct : where request id information need to be updated                
                                                                           
  return      : integer  0: Success, -1 : failure                                                     
*****************************************************************************/
int WIFI_oper_Handle(WLAN_OPER_ID operationId, unsigned char *argStruct)
{
    
    switch (operationId)
	{
           case WLAN_OPER_GET_TX_PWR:
	    {
			if(getConfigTxPowerLevel(argStruct) != 0)
			{
				return -1;
			}
			break;
	    }
	    case WLAN_OPER_RX_PWR_ADJ:
	    {
			if(getConfigRxPowerLevel(argStruct) != 0)
			{
				return -1;
			}
			break;
	    }
    	case WLAN_OPER_COUNTRYCODE:
		{
            unsigned short *country_code= (unsigned short *)argStruct;
			*country_code = convertCountryCode((const char *)g_wifiRegdomainConfig->CountryCode);            
			break;
		}
        case WLAN_OPER_2GENB2040:
		{
            *argStruct = g_wifiRegdomainConfig->Enable2g2040;            
			break;
		}
       case WLAN_OPER_CHNBITMAP2G:
		{
            unsigned int *chnbitmap2g= (unsigned int *)argStruct;
			*chnbitmap2g = g_wifiRegdomainConfig->ChannelBitmap2Ghz;            
			break;
		}
        case WLAN_OPER_CHNBITMAP5G:
		{
            unsigned int *chnbitmap5g= (unsigned int *)argStruct;
           *chnbitmap5g = g_wifiRegdomainConfig->ChannelBitmap5Ghz;            
			break;
		}
        default:
			break;

    }
    return 0;
}
void WIFI_IPF_calregion_protect()
{
    /*Enabling Write protection for Wi-Fi calibration region*/
        DRV_IPF_ProtectMemoryVolatile(ipfInfo.IPF_Handle,NULL,IPF_WIFI_CALIBRATION_START,DRV_IPF_WRITE_PROTECT);
        ipfInfo.prevState = ipfInfo.state ;
        ipfInfo.state = IPF_STATE_PROTECTCAL;
}
void WIFI_IPF_calregion_unprotect()
{
        /*Disabling Write protection for Wi-Fi calibration region*/
        DRV_IPF_UnProtectMemoryVolatile(ipfInfo.IPF_Handle,NULL,IPF_WIFI_CALIBRATION_START,DRV_IPF_WRITE_PROTECT);
        ipfInfo.prevState = ipfInfo.state;
        ipfInfo.state = IPF_STATE_UNPROTECTCAL;
}
/*****************************************************************************                                                                        
  Function Name : WIFI_initialization                                  
                                                                           
  Description   : This function do WIFI initialization           
                                                                           
  Inputs        : None                
                                                                           
  return      : None                                                     
*****************************************************************************/
bool WIFI_initialization()
{
    bool flash_open_status = 0;
    WiFi_Commands_Init();
    hook_wlan_ctrl_handle(WIFI_oper_Handle);
    flash_open_status =  WIFI_IPF_open();
    if(flash_open_status)
    {
        WIFI_IPF_calregion_protect();
    }
    return flash_open_status;
}
/*****************************************************************************                                                                        
  Function Name : WIFI_IPF_setConfig                                  
                                                                           
  Description   : This function sets the configuration after Reading from Flash
                            using DRV_WIFI_Write call with WID and value.            
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void WIFI_IPF_setConfig()
{
    bool status = 1;
#if 0
    //Set SSID
    status = DRV_WIFI_Write(DRV_WIFI_WID_SSID, g_wifiConfigData->ssid);

    //Set PSK   
    status = DRV_WIFI_Write(DRV_WIFI_WID_11I_PSK, "12345678");
    status = DRV_WIFI_Write(DRV_WIFI_WID_11I_MODE, 73);    
    status = DRV_WIFI_Write(DRV_WIFI_WID_AUTH_TYPE, 0);
    status = DRV_WIFI_Write(DRV_WIFI_WID_11G_OPERATING_MODE, 2);
    status = DRV_WIFI_Write(DRV_WIFI_WID_ACK_POLICY, 0);
    status = DRV_WIFI_Write(DRV_WIFI_WID_11N_ENABLE, 1);
#endif    
#if 1
        if(g_wifiConfigData->ssid != NULL)
        {
            printf("\nSet SSID:%s to MAC, keyLen:%d\n",g_wifiConfigData->ssid,g_wifiConfigData->SecurityKeyLength);
            status = DRV_WIFI_Write(DRV_WIFI_WID_SSID,(unsigned int)g_wifiConfigData->ssid);
        }
        if(g_wifiConfigData->SecurityKeyLength != 0 && g_wifiConfigData->SecurityKeyLength != 0xff)
        {
            printf("\nSet Mode:%d and Key:%s to MAC", g_wifiConfigData->cipher,g_wifiConfigData->key);
            status = DRV_WIFI_Write(DRV_WIFI_WID_11I_PSK,(unsigned int)g_wifiConfigData->key);
            status = DRV_WIFI_Write(DRV_WIFI_WID_11I_MODE,(unsigned int)g_wifiConfigData->cipher);    
            status = DRV_WIFI_Write(DRV_WIFI_WID_AUTH_TYPE, 0);
            status = DRV_WIFI_Write(DRV_WIFI_WID_11G_OPERATING_MODE, 2);
            status = DRV_WIFI_Write(DRV_WIFI_WID_ACK_POLICY, 0);
            status = DRV_WIFI_Write(DRV_WIFI_WID_11N_ENABLE, 1);            
        }
        if(status == 0)
        {
            SYS_CONSOLE_MESSAGE("SUCCESSFUL \r\n");
        }
        else
        {
            SYS_CONSOLE_MESSAGE("ERROR \r\n");
        } 
        status = DRV_WIFI_Write(DRV_WIFI_WID_START_SCAN_REQ,1);
#endif        
}

/*****************************************************************************                                                                        
  Function Name : WIFI_IPF_readMacAddr                                  
                                                                           
  Description   : This function reads the saved/stored MAC address from specified location
                            using DRV_IPF_BlockRead call.
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void WIFI_IPF_readMacAddr()
{
    ipfInfo.reqComplete = 0;	 
    DRV_IPF_BlockRead(ipfInfo.IPF_Handle, NULL, &macaddr_mem[0] , IPF_WIFI_MACADDRESS_START, IPF_WIFI_MACADDRESS_LENGTH);    
    ipfInfo.state = IPF_STATE_READ_MACADDR;
    
    return;
}
/*******************************************************************************************                                                            
  Function Name : WIFI_IPF_write_regdomain                                  
                                                                           
  Description   : This function is to write the data of g_wifiRegdomainConfig into IPF        
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
******************************************************************************************/
void WIFI_IPF_write_regdomain()
{
    int cfg_data_size;
    
    ipfInfo.reqComplete = 0;
    cfg_data_size = sizeof(wifiRegdomainConfig);
   
    DRV_IPF_BlockWrite (ipfInfo.IPF_Handle, NULL,(uint8_t *)g_wifiRegdomainConfig, IPF_WIFI_REGDOMAINCONFIG_START,cfg_data_size);
    ipfInfo.state = IPF_STATE_WRITE_REGDOMAINCONFIG;
    return;
}
/******************************************************************************************************                                                                        
  Function Name : WIFI_IPF_read_regdomain                                  
                                                                           
  Description   : This function reads the saved/stored Regulatory domain configuration from IPF
                            using DRV_IPF_BlockRead call.
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*******************************************************************************************************/
void WIFI_IPF_read_regdomain()
{
    
    ipfInfo.reqComplete = 0;	 
    DRV_IPF_BlockRead(ipfInfo.IPF_Handle, NULL,(uint8_t *)g_wifiRegdomainConfig , IPF_WIFI_REGDOMAINCONFIG_START, sizeof(wifiRegdomainConfig));    
    ipfInfo.state = IPF_STATE_READ_REGDOMAINCONFIG;
    
    return;
}

/*****************************************************************************                                                                        
  Function Name : update_bootMode                                  
                                                                           
  Description   : This function updates the BOOTMODE of Device to
					 global variable g_bootMode
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void update_bootMode()
{
    if(g_wifiConfigData->bootMode == 1)    
        g_bootMode = BOOT_AP_MODE;
    
    else if(g_wifiConfigData->bootMode == 2)
        g_bootMode = BOOT_STA_MODE;
    
    else
        g_bootMode = BOOT_INVALID_MODE;

    return;
}

/*****************************************************************************                                                                        
  Function Name : switch_mac_operation_mode                                  
                                                                           
  Description   : This function switches the mode of MAC to the g_bootMode value
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void switch_mac_operation_mode()
{
    bool status = 1;
    TCPIP_NET_HANDLE netH;

        //Set SwitchMode WID
        status = DRV_WIFI_Write(DRV_WIFI_WID_SWITCH_MODE, g_bootMode);
    
        if(status == 0)
        {
            printf("\nBootMode Switch successful\n");
        }
        else
        {
            printf("\nBootMode Switch Failed\n");
        }

        //Switch the DHCP/DHCPS as per WiFi mode

        if(g_bootMode == BOOT_AP_MODE)
        {
           netH = TCPIP_STACK_NetHandleGet(TCPIP_NETWORK_DEFAULT_INTERFACE_NAME);
            if(TCPIP_DHCP_IsEnabled(netH) == true)       
                {
                    printf("\nStop DHCP Clinet");
                    TCPIP_DHCP_Disable(netH);
                    TCPIP_STACK_AddressServiceSelect((TCPIP_NET_IF*)&netH, TCPIP_NETWORK_CONFIG_DHCP_SERVER_ON);
                    printf("\nStart DHCP Server");
                    TCPIP_DHCPS_Enable(netH);        
                }
            else
                printf("\nAlready DHCP Server running...!");            
        }
        else if(g_bootMode == BOOT_STA_MODE)
        {
            netH = TCPIP_STACK_NetHandleGet(TCPIP_NETWORK_DEFAULT_INTERFACE_NAME);
            if(TCPIP_DHCPS_IsEnabled(netH) == true)
                {
                    printf("\nStop DHCP Server");
                    TCPIP_DHCPS_Disable(netH);
            
                    TCPIP_STACK_AddressServiceSelect((TCPIP_NET_IF*)&netH, TCPIP_NETWORK_CONFIG_DHCP_CLIENT_ON);
                    printf("\nStart DHCP Client");
                    TCPIP_DHCP_Enable(netH);                  
                }
            else
                printf("\nAlready DHCP Client running...!");
        }
}

/*****************************************************************************                                                                        
  Function Name : wifi_do_softReboot                                  
                                                                           
  Description   : This function does a soft reboot
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void wifi_do_softReboot()
{
  //  int tempCounter;
    SYSKEY = 0xAA996655;  //write key1 to SYSKEY
    SYSKEY = 0x556699AA;  //write key2 to SYSKEY
                
    // OSCCON is now unlocked
    /* set SWRST bit to arm reset */
    RSWRSTSET = 1;
    /* read RSWRST register to trigger reset */
    //tempCounter = RSWRST;
    /* prevent any unwanted code execution until reset occurs*/
    while(1);

    return;
}
/*****************************************************************************                                                            
  Function Name : IPF_RW_UpdateStatus                                  
                                                                           
  Description   : This function is a callback function after completing every
                          Read/Write/Erase on Flash            
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void IPF_RW_UpdateStatus
(
    DRV_IPF_BLOCK_EVENT event,
    DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle,
    uintptr_t context
)
{
    uint8_t i,j=0,temp;
    if(event == DRV_IPF_EVENT_BLOCK_COMMAND_COMPLETE)
    {
//        printf("\n Read_Write is sucess, present IPF state:%d",ipfInfo.state);     
        ipfInfo.reqComplete = 1;
    if(ipfInfo.state == IPF_STATE_PROTECTCAL){
        
        if(ipfInfo.prevState == IPF_STATE_INIT){
            WIFI_IPF_read_cal();
        }
        ipfInfo.prevState = IPF_STATE_PROTECTCAL;
    }else if(ipfInfo.state == IPF_STATE_UNPROTECTCAL){
        
        WIFI_IPF_read_regdomain();        
        ipfInfo.prevState = IPF_STATE_UNPROTECTCAL;
    }else if(ipfInfo.state == IPF_STATE_READ_CAL)
    {
        
            WIFI_IPF_readMacAddr();
            ipfInfo.prevState = IPF_STATE_READ_CAL;
	}
    else if(ipfInfo.state == IPF_STATE_READ_MACADDR)
       {
                   
            ipfInfo.state = IPF_STATE_IDLE;

            //Process the received MAC address content
            for(i=0;i<6;i++)
            {
                macaddr_val[i] = macaddr_mem[j] - '0';
                if(macaddr_val[i] > 10)
                    macaddr_val[i] -= 7;        
                if(macaddr_val[i] > 15)                
                    macaddr_val[i] -= 32;            
                macaddr_val[i] = (macaddr_val[i] << 4);
                
                temp = (macaddr_mem[j+1] - '0');            
                if(temp > 10) 
                    temp -= 7;
                if(temp > 15) 
                    temp -= 32;                
                macaddr_val[i] = (macaddr_val[i] | temp);    
                j=j+2;
            }

            //printf("\nThe Mac address Value stored in FLASH is %x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x",
            printf("\nThe Mac address Value stored in FLASH is %d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d",            
                                    macaddr_mem[0],macaddr_mem[1],
                                    macaddr_mem[2],macaddr_mem[3],
                                    macaddr_mem[4],macaddr_mem[5],
                                    macaddr_mem[6],macaddr_mem[7],
                                    macaddr_mem[8],macaddr_mem[9],
                                    macaddr_mem[10],macaddr_mem[11]);
            
            printf("\nThe Mac address Value stored in FLASH is %x:%x:%x:%x:%x:%x",
                                    macaddr_val[0],macaddr_val[1],
                                    macaddr_val[2],macaddr_val[3],
                                    macaddr_val[4],macaddr_val[5]);

            //Set the Global MAC address
            for(i=0;i<6;i++)
            g_macaddress[i] = macaddr_val[i];
            WIFI_IPF_read_regdomain();
            ipfInfo.prevState = IPF_STATE_READ_MACADDR;
        }
       else if(ipfInfo.state == IPF_STATE_ERASE)
       {
            WIFI_IPF_write();
            ipfInfo.prevState = IPF_STATE_ERASE;
       }
        else if(ipfInfo.state == IPF_STATE_ERASE_REGDOMAINCONFIG)
       {
            WIFI_IPF_write_regdomain();
            ipfInfo.prevState = IPF_STATE_ERASE_REGDOMAINCONFIG;
       }
       else if(ipfInfo.state == IPF_STATE_READ_REGDOMAINCONFIG)
       {
             if(checkregdomainconfig())
              {               
               updatedefault_regdomainconfig();
             }
            if(ipfInfo.prevState == IPF_STATE_READ_MACADDR) {
                WIFI_IPF_read();
            }else{             
                ipfInfo.state = IPF_STATE_IDLE;
            }
             ipfInfo.prevState = IPF_STATE_READ_REGDOMAINCONFIG;
       }         
       else if(ipfInfo.state == IPF_STATE_READ)
       {
            //Call to set the configuration
            if(checkreadssid())
            {
                updatedefault_wifiuserconfig();
            }
               printf("\nSSID:%s \n",g_wifiConfigData->ssid);
             //  WIFI_IPF_setConfig();
            
            printf("Boot mode Stored in Flash is:%d",g_wifiConfigData->bootMode);
            if(g_wifiConfigData->bootMode == 0xff)
                g_wifiConfigData->bootMode = 1; //1=AP , 2=STA and default is AP if not set in IPF

             if(strcmp(DRV_WIFI_MODE,"STA_MODE") == 0)
                {
                    g_wifiConfigData->bootMode = 2;
                    printf("\nWifi Mode is %s",DRV_WIFI_MODE);
                }
             
            printf("\nBoot mode:%d",g_wifiConfigData->bootMode);                
            update_bootMode();
            
            /*After Successful Read move to the IDLE state*/
            ipfInfo.state = IPF_STATE_IDLE;
			 IPF_ready_use_state = 1;

        if(web_data == 1)
            {
                /*Save the received SSID in Flash*/
                strcpy((char *)g_wifiConfigData->ssid,(const char *)g_easyConfigCtx.ssid);

                g_wifiConfigData->SecurityKeyLength = g_easyConfigCtx.SecurityKeyLength;
                printf("\nSecurity Key Length:%d",g_wifiConfigData->SecurityKeyLength);
                /*Save Security Parameters*/
                if(g_wifiConfigData->SecurityKeyLength !=0)
                {
                    strcpy((char *)g_wifiConfigData->key,(const char *)g_easyConfigCtx.key);
                    g_wifiConfigData->cipher = g_easyConfigCtx.cipher;
                }
                
                /*Boot mode set to STA-mode, and connects to above configuration AP*/
                g_wifiConfigData->bootMode = 2;

                printf("\nWEB SSID:%s",g_wifiConfigData->ssid);           
                printf("\nWEB Boot mode:%d",g_wifiConfigData->bootMode);
                WIFI_IPF_save();
            }
            ipfInfo.prevState = IPF_STATE_READ;
        }
        else if(ipfInfo.state == IPF_STATE_WRITE_REGDOMAINCONFIG)
       {
            
            ipfInfo.state = IPF_STATE_IDLE;
            ipfInfo.prevState = IPF_STATE_WRITE_REGDOMAINCONFIG;
       } 
        else if(ipfInfo.state == IPF_STATE_WRITE)
        {
                printf("\nweb_data:%d,g_bootMode:%d",web_data,g_bootMode);     
            if(web_data == 1)
            {
                if(g_bootMode == BOOT_STA_MODE)
                {
                    printf("\nUpdate the SSID of AP received from Wep-page in STA mode, to connect to that AP\n");
                    WIFI_IPF_setConfig();
                }
                else if(g_bootMode == BOOT_AP_MODE)
                {
                    //DO Soft-reboot of the DEVICE
                    printf("\nDoing Soft Reboot......!");
                    wifi_do_softReboot();
                }
                web_data = 0;
            }
            /*After Successful Write, free the memory created before write and move to the IDLE state*/
            WIFI_IPF_usrconfig_memfree();
            ipfInfo.state = IPF_STATE_IDLE;
            ipfInfo.prevState = IPF_STATE_WRITE;
        }		
    }
    else        
        ipfInfo.state = IPF_STATE_ERROR;
    //DRV_IPF_Close(IPF_Handle);  
    return;
}

/*****************************************************************************                                                            
  Function Name : WIFI_IPF_open                                  
                                                                           
  Description   : This function is to open the Flash Driver
                          After successful opening of the driver, set the state of ipfInfo to
                          IPF_STATE_INIT
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
bool WIFI_IPF_open()
{
    bool status = 0;
    ipfInfo.state = IPF_STATE_ERROR;
    ipfInfo.IPF_Handle = DRV_IPF_Open(DRV_IPF_INDEX_0, DRV_IO_INTENT_READWRITE);
    if(ipfInfo.IPF_Handle != (DRV_HANDLE) DRV_HANDLE_INVALID)
    {
    
        WIFI_IPF_regdomconfig_memalloc();
        WIFI_IPF_usrconfig_memalloc();        
    	WIFI_IPF_cal_memalloc();
    	DRV_IPF_BlockEventHandlerSet(ipfInfo.IPF_Handle, IPF_RW_UpdateStatus, (uintptr_t) NULL);
#ifdef DRV_WIFI_OTA_ENABLE         
        InitializeFlashHandler(ipfInfo.IPF_Handle, IPF_RW_UpdateStatus);        
#endif        
        ipfInfo.state= IPF_STATE_INIT;
         printf("\n IPF Driver opened");
        status = 1;
    }
    else
    {
        printf("\n error opening driver\r%s, line %u\r\n", __FUNCTION__, __LINE__);
    }

    return status;
}

/*****************************************************************************                                                            
  Function Name : WIFI_IPF_write                                  
                                                                           
  Description   : This function is to write the data g_wifiConfigData into flash        
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void WIFI_IPF_write()
{
    int cfg_data_size;
    
    ipfInfo.reqComplete = 0;
    cfg_data_size = sizeof(wifiConfigData);

    printf("\n\t\t\tWriting SSID:%s, Configdata size:%d",g_wifiConfigData->ssid, cfg_data_size);    
    DRV_IPF_BlockWrite (ipfInfo.IPF_Handle, NULL,(uint8_t *) g_wifiConfigData, IPF_WIFI_CONFIG_START,cfg_data_size);
#if 0    
    for(abc=0;abc<100;abc++){    
           buf[abc] = 0x48;//abc+1;
        }                   

    printf("\nWriting %s\n",buf);          
    DRV_IPF_BlockWrite (ipfInfo.IPF_Handle, NULL, &buf[0], 0,100);
#endif

    ipfInfo.state = IPF_STATE_WRITE;
    return;
}

/*****************************************************************************                                                            
  Function Name : WIFI_IPF_read                                  
                                                                           
  Description   : This function is to read the data g_wifiConfigData into flash
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/

void WIFI_IPF_read()
{
    int cfg_data_size;

    ipfInfo.reqComplete = 0;
    cfg_data_size = sizeof(wifiConfigData);
    
//    DRV_IPF_BlockRead(ipfInfo.IPF_Handle, NULL, ipf_read_buf , 0x90001000, 100);
    DRV_IPF_BlockRead(ipfInfo.IPF_Handle, NULL,(uint8_t *) g_wifiConfigData , IPF_WIFI_CONFIG_START, cfg_data_size);    
    ipfInfo.state = IPF_STATE_READ;

    return;
}

void WIFI_IPF_read_cal()
{
	 ipfInfo.reqComplete = 0;	 
	 DRV_IPF_BlockRead(ipfInfo.IPF_Handle, NULL,(uint8_t *)Calib_mem , IPF_WIFI_CALIBRATION_START, IPF_WIFI_CALIBRATION_LENTH);    
    ipfInfo.state = IPF_STATE_READ_CAL;
    
    

    return;

}
bool WIFI_IPF_cal_memfree()
{
    if (Calib_mem){
		free(Calib_mem);
		Calib_mem = NULL;
    }
    return 0;
}
bool WIFI_IPF_usrconfig_memfree()
{
    if (g_wifiConfigData){
        
		//free(g_wifiConfigData);
		//g_wifiConfigData = NULL;
        return 0;
    }
    return 1;
}
bool WIFI_IPF_regdomconfig_memfree()
{
    if (g_wifiRegdomainConfig){
        
		//free(g_wifiRegdomainConfig);
		//g_wifiRegdomainConfig = NULL;
        return 0;
    }
    return 1;
}

bool WIFI_IPF_cal_memalloc()
{
	
    Calib_mem = malloc(IPF_WIFI_CALIBRATION_LENTH);
	if (Calib_mem){
		return 0;
	}
	return 1;
}
bool WIFI_IPF_usrconfig_memalloc()
{
    g_wifiConfigData = &temp_wifiConfigData;
    //g_wifiConfigData = malloc(sizeof(wifiConfigData));
	if (g_wifiConfigData){
        
		return 0;
	}
	return 1;
}
bool WIFI_IPF_regdomconfig_memalloc()
{
    
    g_wifiRegdomainConfig = &temp_wifiRegdomainConfig;
    //g_wifiRegdomainConfig = malloc(sizeof(wifiRegdomainConfig));
	if (g_wifiRegdomainConfig){
        
		return 0;
	}
	return 1;
}
unsigned int  * get_PersParam()     
{
    if(IPF_ready_use_state == 1) {
	return Calib_mem;
    } 
    return NULL;
}

void get_PersParamfree()
{
    WIFI_IPF_setConfig();    
    WIFI_IPF_cal_memfree();
    WIFI_IPF_usrconfig_memfree();
    WIFI_IPF_regdomconfig_memfree();
}

void do_switchMode()
{
    if(g_bootMode < BOOT_INVALID_MODE)
	{
	    printf("\nBoot Switch started for mode:%d",g_bootMode);
           switch_mac_operation_mode();	
	}
    return;
}

/*****************************************************************************                                                            
  Function Name : WIFI_IPF_erase                                  
                                                                           
  Description   : This function is a to erase the flash, before doing write operation            
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void WIFI_IPF_erase(void)
{
    ipfInfo.reqComplete = 0;

    DRV_IPF_BlockErase(ipfInfo.IPF_Handle, NULL, IPF_WIFI_CONFIG_START, 1);
    ipfInfo.state = IPF_STATE_ERASE;
   
}
/*****************************************************************************                                                            
  Function Name : WIFI_IPF_erase_regdomain                                  
                                                                           
  Description   : This function is a to erase the flash, before doing write operation            
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void WIFI_IPF_erase_regdomain(void)
{
    ipfInfo.reqComplete = 0;

    DRV_IPF_BlockErase(ipfInfo.IPF_Handle, NULL, IPF_WIFI_REGDOMAINCONFIG_START, 1);
    ipfInfo.state = IPF_STATE_ERASE_REGDOMAINCONFIG;
}


/*****************************************************************************                                                            
  Function Name : WIFI_IPF_save                                  
                                                                           
  Description   : This function is to save the g_wifiConfigData data which was filled on
                          taking user data into the Flash
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void WIFI_IPF_save()
{
    printf("\nipfInfo.state:%d",ipfInfo.state);
    if(ipfInfo.state == IPF_STATE_IDLE)
        {
            WIFI_IPF_erase();
        }
    else
            printf("\nSave failed");
}
/*****************************************************************************                                                            
  Function Name : WIFI_IPF_save_regdom                                  
                                                                           
  Description   : This function is to save the g_wifiConfigData data which was filled on
                          taking user data into the Flash
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void WIFI_IPF_save_regdom()
{
    
    if(ipfInfo.state == IPF_STATE_IDLE)
        {
            WIFI_IPF_erase_regdomain();
        }
    else
            printf("\n RegDom Save failed");
}
/*****************************************************************************                                                            
  Function Name : WIFI_IPF_readAndBackup                                  
                                                                           
  Description   : This function is to read and backup the g_wifiConfigData data before
                          updating new user data into the Flash
                                                                           
  Inputs        : None                
                                                                           
  Outputs      : None                                                     
*****************************************************************************/
void WIFI_IPF_readAndBackup()
{
    printf("\nipfInfo.state:%d",ipfInfo.state);
    if(ipfInfo.state == IPF_STATE_IDLE)
        {
            if(WIFI_IPF_usrconfig_memalloc() == 0)
                {
                        WIFI_IPF_read();
                }
            else
                printf("\n Memory allocation for g_wifiConfigData failed");
        }
    else
        printf("\nRead to Backup Failed");
}

/*******************************************************************************
 End of File
 */

