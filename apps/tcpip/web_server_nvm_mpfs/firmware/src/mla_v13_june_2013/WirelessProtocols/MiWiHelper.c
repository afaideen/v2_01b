
#include "GenericTypeDefs.h"
#include "MiWiHelper.h"
#include "WirelessProtocols/MCHP_API.h"

#if defined(MIWI_HELPER)
extern void DemoOutput_ChannelError(BYTE channel);
extern void DemoOutput_Channel(BYTE channel, BYTE Step);
extern void LCDDisplay(char *text, BYTE value, BOOL delay);
extern BYTE DemoOutput_ActiveScanResults(BYTE num);
extern void DemoOutput_Rescan(void);
void DemoOutput_Channel_Addr(BYTE channel, BYTE high_addr, BYTE low_addr);

BOOL MiWiHelper_SendData(BYTE conn_index, BYTE *data);

BOOL MiWiHelper_SendData(BYTE conn_index, BYTE *data)
{
    BYTE i;
    MiApp_FlushTx();
    for(i = 0; i < strlen(data); i++)
    {
        MiApp_WriteData(data[i]);
    }
    if( MiApp_UnicastConnection(conn_index, TRUE) == FALSE )
    {
        return FALSE;
    }
    else{
        return TRUE;
    }
}

//BOOL CreateNewConnectionWithLeastNoise(DWORD scan_chnl, BYTE channel)
BOOL CreateNewConnectionWithLeastNoise(DWORD scan_chnl)
{  
    DWORD scan_chnl_;
    BOOL connstat = FALSE;
//    //Pre-condition compulsary to init channel setting
//    // Set default channel
//    if( MiApp_SetChannel(channel) == FALSE )
//    {
//        DemoOutput_ChannelError(channel);
//        #if defined(__18CXX)
//            return;
//        #else
//            return 0;
//        #endif
//    }
    
//    scan_chnl = 0b0000000011111000 << 11;
    scan_chnl_ = scan_chnl << 11;
//  scan_chnl = 0xFFFF << 11; //move the 16-bit 0xFFFF to the left by 11 bits. Scan from channel bit 11 to bit 26. bit0-bit10, bit27-bit31 are considered invalid. 
    
    while(!connstat){
        Printf("\r\nCreate connection...");
        //    MiApp_StartConnection(START_CONN_CS_SCN , 10, 0);//not yet available
        connstat = MiApp_StartConnection(START_CONN_ENERGY_SCN , 10, scan_chnl_);//create connection with least noise energy

    }
    return connstat;
}
BOOL CreateNewConnectionAtChannel(BYTE channel)
{
    BOOL connstat;
    // Set default channel
    if( MiApp_SetChannel(channel) == FALSE )
    {
        DemoOutput_ChannelError(channel);
        #if defined(__18CXX)
            return FALSE;
        #else
            return 0;
        #endif
    }
    DemoOutput_Channel(channel, 0);
    Printf("\r\nCreate connection...");
    MiApp_ConnectionMode(ENABLE_ALL_CONN);
    
    connstat = MiApp_StartConnection(START_CONN_DIRECT, 10, 0xFFFFFFFF);//create connection at selected channel/myChannel
//    BOOL connstat = MiApp_StartConnection(START_CONN_DIRECT, 0, 0);//also okay!
    return connstat;
}

#ifdef ENABLE_ACTIVE_SCAN
WORD AutoSearchActiveConnection(DWORD scan_chnl)
{
    WORD o = 0;
    BYTE j, OperatingChannel = 0xFF, retry = 3;
    DWORD scan_chnl_;
    Printf("Autosearching...");
//    LCDDisplay((char *)"Search Ch %d", channel, TRUE);
    LCDDisplay((char *)"Searching...", 0, FALSE);
    while(retry)
    {
        /*********************************************************************/
        // Function MiApp_SearchConnection will return the number of existing 
        // connections in all channels. It will help to decide which channel 
        // to operate on and which connection to add
        // The return value is the number of connections. The connection data
        //     are stored in global variable ActiveScanResults. Maximum active
        //     scan result is defined as ACTIVE_SCAN_RESULT_SIZE
        // The first parameter is the scan duration, which has the same 
        //     definition in Energy Scan. 10 is roughly 1 second. 9 is a half 
        //     second and 11 is 2 seconds. Maximum scan duration is 14, or 
        //     roughly 16 seconds.
        // The second parameter is the channel map. Bit 0 of the double
        //     word parameter represents channel 0. For the 2.4GHz frequency
        //     band, all possible channels are channel 11 to channel 26.
        //     As the result, the bit map is 0x07FFF800. Stack will filter
        //     out all invalid channels, so the application only needs to pay
        //     attention to the channels that are not preferred.
        /*********************************************************************/
//        scan_chnl = 0b0000000011111000 << 11;
//        scan_chnl_ = scan_chnl << 11;
//        scan_chnl_ = 1 << scan_chnl;
        j = MiApp_SearchConnection(10, scan_chnl);
//        j = MiApp_SearchConnection(10, scan_chnl_);
//        j = MiApp_SearchConnection(11, scan_chnl_);
//        j = MiApp_SearchConnection(10, 0xFFFFFFFF);
        OperatingChannel = DemoOutput_ActiveScanResults(j);

        if( OperatingChannel != 0xFF )
        {
            /*******************************************************************/
            // Function MiApp_SetChannel assign the operation channel(frequency)
            // for the transceiver. Channels 0 - 31 has been defined for the 
            // wireless protocols, but not all channels are valid for all 
            // transceivers, depending on their hardware design, operating
            // frequency band, data rate and other RF parameters. Check the
            // definition of FULL_CHANNEL_MAP for details.
            /*******************************************************************/
            MiApp_SetChannel(OperatingChannel);
            break;
        }
        DemoOutput_Rescan();
        retry--;
    }
    o = j;
    o = o << 8 ;
    o = o | OperatingChannel;
//     o = ((WORD)j << 8) | OperatingChannel;
    return o;

}

#endif

BYTE JoinAvailableChannel(DWORD channel, WORD short_addr)
{
    DWORD channel_map;
    WORD k;
    WORD_VAL addr;
    BYTE i, n, j = 0;
    
    i = 0xff;
    addr.Val = short_addr;
    Printf("Joining available channel...");
     MiApp_ConnectionMode(ENABLE_ALL_CONN); 
//     MiApp_ConnectionMode(ENABLE_PREV_CONN); 
    /*******************************************************************/
    // Function MiApp_EstablishConnection try to establish a new 
    // connection with peer device. 
    // The first parameter is the index to the active scan result, 
    //      which is acquired by discovery process (active scan). If 
    //      the value of the index is 0xFF, try to establish a 
    //      connection with any peer.
    // The second parameter is the mode to establish connection, 
    //      either direct or indirect. Direct mode means connection 
    //      within the radio range; indirect mode means connection 
    //      may or may not in the radio range. 
    /*******************************************************************/   

    if( channel >= 11 && channel <= 26)
    {
        // Set default channel
        if( MiApp_SetChannel(channel) == FALSE )
        {
            DemoOutput_ChannelError(channel);
            #if defined(__18CXX)
                return 0xff;
            #else
                return 0;
            #endif
        }
        if(short_addr != 0x0000)
        {
            //convert to channel map
            if(channel != 0xffffffff)
            {
    //            channel_map = (0x00000001&0xffffffff) << channel | (0x00000001&0xffffffff) << 12;
                channel_map = (0x00000001&0xffffffff) << channel;
            }
            else
                channel_map = 0xffffffff;
            k = AutoSearchActiveConnection(channel_map);
            n = k >> 8;
            
            if( n != 0x00 )
            {
                channel = (BYTE)k & 0x00ff;
            
                for( j = 0; j < n; j++)
                {
                    if( ActiveScanResults[j].Address[0] == addr.v[0] && ActiveScanResults[j].Address[1] == addr.v[1] )
                    {
                        i = MiApp_EstablishConnection(j, CONN_MODE_DIRECT);
                        break;
                    }
                    if( (j == n - 1) && ActiveScanResults[j].Capability.bits.Role == ROLE_COORDINATOR)
                        i = MiApp_EstablishConnection(j, CONN_MODE_DIRECT);
                }
                if(j == n)
                    j = n - 1;
            }
            if( i == 0xff )
            {
                i = MiApp_EstablishConnection(0xFF, CONN_MODE_DIRECT);
                j = 0;
            }
        }
        else{
            i = MiApp_EstablishConnection(0xFF, CONN_MODE_DIRECT);
//        i = MiApp_EstablishConnection(0xFF, CONN_MODE_INDIRECT);
//        while( (i = MiApp_EstablishConnection(0xFF, CONN_MODE_DIRECT)) == 0xFF );
            j = 0;
        }
    }
    else
        while( (i = MiApp_EstablishConnection(0xFF, CONN_MODE_DIRECT)) == 0xFF );//active scanning all channels available

    
    LCDBacklightON();
   
    /*******************************************************************/
    // Display current opertion on LCD of demo board, if applicable
    /*******************************************************************/
    if( i != 0xFF )
    {
        //Join channel successful
        #if defined(ENABLE_ACTIVE_SCAN)
            DemoOutput_Channel_Addr(ActiveScanResults[j].Channel, ActiveScanResults[j].Address[1], ActiveScanResults[j].Address[0]);
//            DemoOutput_Channel(ActiveScanResults[0].Channel, 1);//Connected peer
//            DemoOutput_Channel(ActiveScanResults[myParent].Channel, 1);//Connected peer
//            DemoOutput_Channel(channel, 1);//Connected peer
        #else
            DemoOutput_Channel(channel, 1);//Connected peer
        #endif  
    }
    else{
        DemoOutput_Channel(channel, 0);//Connecting peer
        //Join fail then creating network
        MiApp_StartConnection(START_CONN_DIRECT, 10, 0);
    }
    LCDBacklightOFF(); 
    
    return i;
}

#endif  //MIWI_HELPER
