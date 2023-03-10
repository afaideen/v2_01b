config SYS_DMA_INSTANCES_NUMBER_GT_${INSTANCE+1}
    depends on USE_SYS_DMA
    bool
<#if INSTANCE != 0>
	default n if SYS_DMA_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>
   	default n if DRV_DMA_INSTANCES_NUMBER = ${INSTANCE+1}
	default y
	
config SYS_DMA_CHANNEL_${INSTANCE}
    depends on USE_SYS_DMA 
<#if INSTANCE != 0>
	             && SYS_DMA_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "DMA System Channel Instance ${INSTANCE}"
    default y

ifblock SYS_DMA_CHANNEL_${INSTANCE}


config SYS_DMA_CHANNEL_ID_IDX${INSTANCE}
    depends on USE_SYS_DMA
    string "DMA Channel"
    range DMA_CHANNEL
    default "DMA_CHANNEL_${INSTANCE}"


config SYS_DMA_INT_PRIO_NUM_CH${INSTANCE}
	string
	depends on SYS_DMA_INTERRUPT_MODE_CH${INSTANCE}
	default "0" if SYS_DMA_INTERRUPT_PRIORITY_CH${INSTANCE} = "INT_DISABLE_INTERRUPT"
        default "1" if SYS_DMA_INTERRUPT_PRIORITY_CH${INSTANCE} = "INT_PRIORITY_LEVEL1"
        default "2" if SYS_DMA_INTERRUPT_PRIORITY_CH${INSTANCE} = "INT_PRIORITY_LEVEL2"
        default "3" if SYS_DMA_INTERRUPT_PRIORITY_CH${INSTANCE} = "INT_PRIORITY_LEVEL3"
        default "4" if SYS_DMA_INTERRUPT_PRIORITY_CH${INSTANCE} = "INT_PRIORITY_LEVEL4"
        default "5" if SYS_DMA_INTERRUPT_PRIORITY_CH${INSTANCE} = "INT_PRIORITY_LEVEL5"
        default "6" if SYS_DMA_INTERRUPT_PRIORITY_CH${INSTANCE} = "INT_PRIORITY_LEVEL6"
        default "7" if SYS_DMA_INTERRUPT_PRIORITY_CH${INSTANCE} = "INT_PRIORITY_LEVEL7"

config SYS_DMA_ISR_VECTOR_CH${INSTANCE}
	string
	depends on SYS_DMA_INTERRUPT_MODE_CH${INSTANCE}
        default "_DMA0_VECTOR" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_0"
        default "_DMA1_VECTOR" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_1"
        default "_DMA2_VECTOR" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_2"
        default "_DMA3_VECTOR" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_3"
        default "_DMA4_VECTOR" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_4"
        default "_DMA5_VECTOR" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_5"
        default "_DMA6_VECTOR" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_6"
        default "_DMA7_VECTOR" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_7"

config SYS_DMA_INT_VECTOR_CH${INSTANCE}
	string
	depends on SYS_DMA_INTERRUPT_MODE_CH${INSTANCE}
        default "INT_VECTOR_DMA0" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_0"
        default "INT_VECTOR_DMA1" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_1"
        default "INT_VECTOR_DMA2" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_2"
        default "INT_VECTOR_DMA3" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_3"
        default "INT_VECTOR_DMA4" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_4"
        default "INT_VECTOR_DMA5" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_5"
        default "INT_VECTOR_DMA6" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_6"
        default "INT_VECTOR_DMA7" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_7"

config SYS_DMA_ISR_SOURCE_CH${INSTANCE}
	string
	depends on SYS_DMA_INTERRUPT_MODE_CH${INSTANCE}
        default "INT_SOURCE_DMA_0" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_0"
        default "INT_SOURCE_DMA_1" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_1"
        default "INT_SOURCE_DMA_2" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_2"
        default "INT_SOURCE_DMA_3" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_3"
        default "INT_SOURCE_DMA_4" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_4"
        default "INT_SOURCE_DMA_5" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_5"
        default "INT_SOURCE_DMA_6" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_6"
        default "INT_SOURCE_DMA_7" if SYS_DMA_CHANNEL_ID_IDX${INSTANCE} = "DMA_CHANNEL_7"
    
config SYS_DMA_IGNORE_MATCH_CH${INSTANCE}
    string
    range SYS_DMA_CHANNEL_IGNORE_MATCH
    default "SYS_DMA_CHANNEL_IGNORE_MATCH_DISABLE"
	
config SYS_DMA_CRC_MODE_CH${INSTANCE}
    string
    range SYS_DMA_CHANNEL_CRC_MODE
    default "SYS_DMA_CHANNEL_CRC_MODE_BACKGROUND"
    
config SYS_DMA_CRC_WRITE_ORDER_CH${INSTANCE}
    string
    range SYS_DMA_CRC_WRITE_ORDER
    default "SYS_DMA_CRC_WRITE_ORDER_MAINTAIN"
    
config SYS_DMA_OP_MODE_CH${INSTANCE}
    string
    range SYS_DMA_CHANNEL_OP_MODE
    default "SYS_DMA_CHANNEL_OP_MODE_BASIC"

config SYS_DMA_INTERRUPT_MODE_CH${INSTANCE}
	bool
	depends on USE_SYS_DMA
	select USE_SYS_INT_NEEDED
	default y

config SYS_DMA_INTERRUPT_PRIORITY_CH${INSTANCE}
	string "Interrupt Priority"
	depends on SYS_DMA_INTERRUPT_MODE_CH${INSTANCE}
	range INT_PRIORITY_LEVEL
	default "INT_PRIORITY_LEVEL1"
		---help---
		IDH_HTML_INT_PRIORITY_LEVEL
		---endhelp---
	
config SYS_DMA_INTERRUPT_SUBPRIORITY_CH${INSTANCE}
	string "Interrupt Sub-priority"
	depends on SYS_DMA_INTERRUPT_MODE_CH${INSTANCE}
	range INT_SUBPRIORITY_LEVEL
	default "INT_SUBPRIORITY_LEVEL0"
		---help---
		IDH_HTML_INT_SUBPRIORITY_LEVEL
		---endhelp---

ifblock SYS_DMA_MODE = "STATIC"
choice
    prompt "Mode Options"
    depends on USE_SYS_INT_NEEDED
	default SYS_DMA_NONE_INTERRUPT_MODE${INSTANCE}
	
config SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE${INSTANCE}
    bool "Block Transfer Interrupt"
    depends on USE_SYS_INT_NEEDED
	
config SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE${INSTANCE}
    bool "Half and Full Destination Interrupt"
    depends on USE_SYS_INT_NEEDED
	
config SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE${INSTANCE}
    bool "Half and Full Source Interrupt"
    depends on USE_SYS_INT_NEEDED

config SYS_DMA_NONE_INTERRUPT_MODE${INSTANCE}
    bool "None"
    depends on USE_SYS_INT_NEEDED
		
endchoice #Mode Options 
endif    # end static 		

ifblock SYS_DMA_MODE = "STATIC"
config SYS_DMA_CHANNEL_CALL_BACK_ENABLE${INSTANCE}
    depends on USE_SYS_DMA 
    bool "Call-Back Function"
    default n

ifblock SYS_DMA_CHANNEL_CALL_BACK_ENABLE${INSTANCE}
	
config DRV_DMA_CALL_BACK_FUNCTION${INSTANCE}
    string "Call-Back Function"
    default "DRV_SYS_DMA_CH_${INSTANCE}_Callback"
	
endif     # call-back enable

config DRV_DMA_AUTO_ENABLE${INSTANCE}
    bool "Auto Enable Channel?"
    default n

config DRV_DMA_SOURCE_ADDRESS${INSTANCE}
    string "Source Address"
    default "0"

config DRV_DMA_SOURCE_SIZE${INSTANCE}
    int "Source Size"
    default "2"

config DRV_DMA_DESTINATION_ADDRESS${INSTANCE}
    string "Destination Address"
    default "0"

config DRV_DMA_DESTINATION_SIZE${INSTANCE}
    int "Destination Size"
    default "2"

config DRV_DMA_CELL_SIZE${INSTANCE}
    int "Cell Size"
    default "2"

config DRV_DMA_TRIGGER${INSTANCE}
    string "Trigger Source"
    range DMA_TRIGGER_SOURCE
    default "DMA_TRIGGER_SOURCE_NONE" 
endif
    
endif     #end static
