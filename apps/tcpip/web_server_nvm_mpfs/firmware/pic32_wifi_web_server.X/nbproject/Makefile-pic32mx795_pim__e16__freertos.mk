#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-pic32mx795_pim__e16__freertos.mk)" "nbproject/Makefile-local-pic32mx795_pim__e16__freertos.mk"
include nbproject/Makefile-local-pic32mx795_pim__e16__freertos.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=pic32mx795_pim__e16__freertos
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/pic32_wifi_web_server.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/pic32_wifi_web_server.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/system_config/pic32mx795_pim__e16__freertos/bsp/bsp.c ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_tasks.c ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_api.c ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_master_dma_tasks.c ../src/system_config/pic32mx795_pim__e16__freertos/framework/net/pres/net_pres_enc_glue.c ../src/system_config/pic32mx795_pim__e16__freertos/framework/system/clk/src/sys_clk_static.c ../src/system_config/pic32mx795_pim__e16__freertos/framework/system/ports/src/sys_ports_static.c ../src/system_config/pic32mx795_pim__e16__freertos/system_init.c ../src/system_config/pic32mx795_pim__e16__freertos/system_interrupt.c ../src/system_config/pic32mx795_pim__e16__freertos/system_exceptions.c ../src/system_config/pic32mx795_pim__e16__freertos/system_tasks.c ../src/system_config/pic32mx795_pim__e16__freertos/system_interrupt_a.S ../src/system_config/pic32mx795_pim__e16__freertos/rtos_hooks.c ../src/main.c ../src/app.c ../src/http_print.c ../src/custom_http_app.c ../src/mpfs_img2.c ../src/app_wifi.c ../../../../../framework/crypto/src/zlib-1.2.7/adler32.c ../../../../../framework/crypto/src/zlib-1.2.7/compress.c ../../../../../framework/crypto/src/zlib-1.2.7/crc32.c ../../../../../framework/crypto/src/zlib-1.2.7/deflate.c ../../../../../framework/crypto/src/zlib-1.2.7/infback.c ../../../../../framework/crypto/src/zlib-1.2.7/inffast.c ../../../../../framework/crypto/src/zlib-1.2.7/inflate.c ../../../../../framework/crypto/src/zlib-1.2.7/inftrees.c ../../../../../framework/crypto/src/zlib-1.2.7/trees.c ../../../../../framework/crypto/src/zlib-1.2.7/uncompr.c ../../../../../framework/crypto/src/zlib-1.2.7/zutil.c ../../../../../framework/crypto/src/ecc.c ../../../../../framework/crypto/src/arc4.c ../../../../../framework/crypto/src/pwdbased.c ../../../../../framework/crypto/src/tfm.c ../../../../../framework/crypto/src/asn.c ../../../../../framework/crypto/src/des3.c ../../../../../framework/crypto/src/rsa.c ../../../../../framework/crypto/src/aes.c ../../../../../framework/crypto/src/md5.c ../../../../../framework/crypto/src/sha.c ../../../../../framework/crypto/src/sha256.c ../../../../../framework/crypto/src/sha512.c ../../../../../framework/crypto/src/hmac.c ../../../../../framework/crypto/src/hash.c ../../../../../framework/crypto/src/compress.c ../../../../../framework/crypto/src/random.c ../../../../../framework/crypto/src/crypto.c ../../../../../framework/crypto/src/asm.c ../../../../../framework/crypto/src/coding.c ../../../../../framework/crypto/src/error.c ../../../../../framework/crypto/src/integer.c ../../../../../framework/crypto/src/logging.c ../../../../../framework/crypto/src/memory.c ../../../../../framework/crypto/src/misc.c ../../../../../framework/crypto/src/port.c ../../../../../framework/driver/nvm/src/dynamic/drv_nvm.c ../../../../../framework/driver/nvm/src/dynamic/drv_nvm_erasewrite.c ../../../../../framework/driver/spi/src/dynamic/drv_spi.c ../../../../../framework/driver/spi/src/drv_spi_sys_queue_fifo.c ../../../../../framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../../framework/driver/usart/src/dynamic/drv_usart.c ../../../../../framework/driver/usart/src/dynamic/drv_usart_buffer_queue.c ../../../../../framework/driver/usart/src/dynamic/drv_usart_read_write.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_com.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_commands.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_config_data.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_algorithm.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_manager.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_profile.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_context_loader.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_debug_output.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_eint.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_events.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_event_handler.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_init.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_iwpriv.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_mac.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_main.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_mgmt_msg.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_param_msg.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_pbkdf2.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_power_save.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_raw.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_scan.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_scan_helper.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_softap_client_cache.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_spi.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_tx_power.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_rtos_wrapper.c ../../../../../framework/net/pres/src/net_pres.c ../../../../../framework/osal/src/osal_freertos.c ../../../../../framework/system/command/src/sys_command.c ../../../../../framework/system/console/src/sys_console.c ../../../../../framework/system/console/src/sys_console_uart.c ../../../../../framework/system/debug/src/sys_debug.c ../../../../../framework/system/devcon/src/sys_devcon.c ../../../../../framework/system/devcon/src/sys_devcon_pic32mx.c ../../../../../framework/system/dma/src/sys_dma.c ../../../../../framework/system/fs/src/dynamic/sys_fs.c ../../../../../framework/system/fs/src/dynamic/sys_fs_media_manager.c ../../../../../framework/system/fs/mpfs/src/mpfs.c ../../../../../framework/system/int/src/sys_int_pic32.c ../../../../../framework/system/random/src/sys_random.c ../../../../../framework/system/reset/src/sys_reset.c ../../../../../framework/system/tmr/src/sys_tmr.c ../../../../../framework/tcpip/src/common/sys_fs_wrapper.c ../../../../../framework/tcpip/src/common/helpers.c ../../../../../framework/tcpip/src/tcp.c ../../../../../framework/tcpip/src/udp.c ../../../../../framework/tcpip/src/tcpip_heap_alloc.c ../../../../../framework/tcpip/src/tcpip_heap_internal.c ../../../../../framework/tcpip/src/arp.c ../../../../../framework/tcpip/src/dhcp.c ../../../../../framework/tcpip/src/http.c ../../../../../framework/tcpip/src/icmp.c ../../../../../framework/tcpip/src/nbns.c ../../../../../framework/tcpip/src/smtp.c ../../../../../framework/tcpip/src/zero_conf_helper.c ../../../../../framework/tcpip/src/zero_conf_link_local.c ../../../../../framework/tcpip/src/zero_conf_multicast_dns.c ../../../../../framework/tcpip/src/iperf.c ../../../../../framework/tcpip/src/tcpip_commands.c ../../../../../framework/tcpip/src/hash_fnv.c ../../../../../framework/tcpip/src/oahash.c ../../../../../framework/tcpip/src/tcpip_helpers.c ../../../../../framework/tcpip/src/tcpip_helper_c32.S ../../../../../framework/tcpip/src/tcpip_manager.c ../../../../../framework/tcpip/src/tcpip_notify.c ../../../../../framework/tcpip/src/tcpip_packet.c ../../../../../framework/tcpip/src/ipv4.c ../../../../../framework/tcpip/src/dns.c ../../../../../third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_2.c ../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c ../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S ../../../../../third_party/rtos/FreeRTOS/Source/croutine.c ../../../../../third_party/rtos/FreeRTOS/Source/list.c ../../../../../third_party/rtos/FreeRTOS/Source/queue.c ../../../../../third_party/rtos/FreeRTOS/Source/tasks.c ../../../../../third_party/rtos/FreeRTOS/Source/timers.c ../../../../../third_party/rtos/FreeRTOS/Source/event_groups.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/975901697/bsp.o ${OBJECTDIR}/_ext/1593769052/drv_spi_tasks.o ${OBJECTDIR}/_ext/1593769052/drv_spi_api.o ${OBJECTDIR}/_ext/1593769052/drv_spi_master_ebm_tasks.o ${OBJECTDIR}/_ext/1593769052/drv_spi_master_dma_tasks.o ${OBJECTDIR}/_ext/448233041/net_pres_enc_glue.o ${OBJECTDIR}/_ext/1256380230/sys_clk_static.o ${OBJECTDIR}/_ext/996991370/sys_ports_static.o ${OBJECTDIR}/_ext/1096222929/system_init.o ${OBJECTDIR}/_ext/1096222929/system_interrupt.o ${OBJECTDIR}/_ext/1096222929/system_exceptions.o ${OBJECTDIR}/_ext/1096222929/system_tasks.o ${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o ${OBJECTDIR}/_ext/1096222929/rtos_hooks.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/http_print.o ${OBJECTDIR}/_ext/1360937237/custom_http_app.o ${OBJECTDIR}/_ext/1360937237/mpfs_img2.o ${OBJECTDIR}/_ext/1360937237/app_wifi.o ${OBJECTDIR}/_ext/2046866571/adler32.o ${OBJECTDIR}/_ext/2046866571/compress.o ${OBJECTDIR}/_ext/2046866571/crc32.o ${OBJECTDIR}/_ext/2046866571/deflate.o ${OBJECTDIR}/_ext/2046866571/infback.o ${OBJECTDIR}/_ext/2046866571/inffast.o ${OBJECTDIR}/_ext/2046866571/inflate.o ${OBJECTDIR}/_ext/2046866571/inftrees.o ${OBJECTDIR}/_ext/2046866571/trees.o ${OBJECTDIR}/_ext/2046866571/uncompr.o ${OBJECTDIR}/_ext/2046866571/zutil.o ${OBJECTDIR}/_ext/29024758/ecc.o ${OBJECTDIR}/_ext/29024758/arc4.o ${OBJECTDIR}/_ext/29024758/pwdbased.o ${OBJECTDIR}/_ext/29024758/tfm.o ${OBJECTDIR}/_ext/29024758/asn.o ${OBJECTDIR}/_ext/29024758/des3.o ${OBJECTDIR}/_ext/29024758/rsa.o ${OBJECTDIR}/_ext/29024758/aes.o ${OBJECTDIR}/_ext/29024758/md5.o ${OBJECTDIR}/_ext/29024758/sha.o ${OBJECTDIR}/_ext/29024758/sha256.o ${OBJECTDIR}/_ext/29024758/sha512.o ${OBJECTDIR}/_ext/29024758/hmac.o ${OBJECTDIR}/_ext/29024758/hash.o ${OBJECTDIR}/_ext/29024758/compress.o ${OBJECTDIR}/_ext/29024758/random.o ${OBJECTDIR}/_ext/29024758/crypto.o ${OBJECTDIR}/_ext/29024758/asm.o ${OBJECTDIR}/_ext/29024758/coding.o ${OBJECTDIR}/_ext/29024758/error.o ${OBJECTDIR}/_ext/29024758/integer.o ${OBJECTDIR}/_ext/29024758/logging.o ${OBJECTDIR}/_ext/29024758/memory.o ${OBJECTDIR}/_ext/29024758/misc.o ${OBJECTDIR}/_ext/29024758/port.o ${OBJECTDIR}/_ext/184581597/drv_nvm.o ${OBJECTDIR}/_ext/184581597/drv_nvm_erasewrite.o ${OBJECTDIR}/_ext/1324760662/drv_spi.o ${OBJECTDIR}/_ext/1385053818/drv_spi_sys_queue_fifo.o ${OBJECTDIR}/_ext/941160041/drv_tmr.o ${OBJECTDIR}/_ext/821589181/drv_usart.o ${OBJECTDIR}/_ext/821589181/drv_usart_buffer_queue.o ${OBJECTDIR}/_ext/821589181/drv_usart_read_write.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_com.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_commands.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_config_data.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_algorithm.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_manager.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_profile.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_context_loader.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_debug_output.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_eint.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_events.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_event_handler.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_init.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_iwpriv.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_mac.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_main.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_mgmt_msg.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_param_msg.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_pbkdf2.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_power_save.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_raw.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan_helper.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_softap_client_cache.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_spi.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_tx_power.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_rtos_wrapper.o ${OBJECTDIR}/_ext/871719959/net_pres.o ${OBJECTDIR}/_ext/308321468/osal_freertos.o ${OBJECTDIR}/_ext/1891376032/sys_command.o ${OBJECTDIR}/_ext/1434663852/sys_console.o ${OBJECTDIR}/_ext/1434663852/sys_console_uart.o ${OBJECTDIR}/_ext/2137108136/sys_debug.o ${OBJECTDIR}/_ext/482662494/sys_devcon.o ${OBJECTDIR}/_ext/482662494/sys_devcon_pic32mx.o ${OBJECTDIR}/_ext/853959373/sys_dma.o ${OBJECTDIR}/_ext/1750042194/sys_fs.o ${OBJECTDIR}/_ext/1750042194/sys_fs_media_manager.o ${OBJECTDIR}/_ext/614935175/mpfs.o ${OBJECTDIR}/_ext/1042686532/sys_int_pic32.o ${OBJECTDIR}/_ext/470001640/sys_random.o ${OBJECTDIR}/_ext/64182180/sys_reset.o ${OBJECTDIR}/_ext/2110151058/sys_tmr.o ${OBJECTDIR}/_ext/1886247299/sys_fs_wrapper.o ${OBJECTDIR}/_ext/1886247299/helpers.o ${OBJECTDIR}/_ext/1164207549/tcp.o ${OBJECTDIR}/_ext/1164207549/udp.o ${OBJECTDIR}/_ext/1164207549/tcpip_heap_alloc.o ${OBJECTDIR}/_ext/1164207549/tcpip_heap_internal.o ${OBJECTDIR}/_ext/1164207549/arp.o ${OBJECTDIR}/_ext/1164207549/dhcp.o ${OBJECTDIR}/_ext/1164207549/http.o ${OBJECTDIR}/_ext/1164207549/icmp.o ${OBJECTDIR}/_ext/1164207549/nbns.o ${OBJECTDIR}/_ext/1164207549/smtp.o ${OBJECTDIR}/_ext/1164207549/zero_conf_helper.o ${OBJECTDIR}/_ext/1164207549/zero_conf_link_local.o ${OBJECTDIR}/_ext/1164207549/zero_conf_multicast_dns.o ${OBJECTDIR}/_ext/1164207549/iperf.o ${OBJECTDIR}/_ext/1164207549/tcpip_commands.o ${OBJECTDIR}/_ext/1164207549/hash_fnv.o ${OBJECTDIR}/_ext/1164207549/oahash.o ${OBJECTDIR}/_ext/1164207549/tcpip_helpers.o ${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o ${OBJECTDIR}/_ext/1164207549/tcpip_manager.o ${OBJECTDIR}/_ext/1164207549/tcpip_notify.o ${OBJECTDIR}/_ext/1164207549/tcpip_packet.o ${OBJECTDIR}/_ext/1164207549/ipv4.o ${OBJECTDIR}/_ext/1164207549/dns.o ${OBJECTDIR}/_ext/1107379230/heap_2.o ${OBJECTDIR}/_ext/1282498059/port.o ${OBJECTDIR}/_ext/1282498059/port_asm.o ${OBJECTDIR}/_ext/373060831/croutine.o ${OBJECTDIR}/_ext/373060831/list.o ${OBJECTDIR}/_ext/373060831/queue.o ${OBJECTDIR}/_ext/373060831/tasks.o ${OBJECTDIR}/_ext/373060831/timers.o ${OBJECTDIR}/_ext/373060831/event_groups.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/975901697/bsp.o.d ${OBJECTDIR}/_ext/1593769052/drv_spi_tasks.o.d ${OBJECTDIR}/_ext/1593769052/drv_spi_api.o.d ${OBJECTDIR}/_ext/1593769052/drv_spi_master_ebm_tasks.o.d ${OBJECTDIR}/_ext/1593769052/drv_spi_master_dma_tasks.o.d ${OBJECTDIR}/_ext/448233041/net_pres_enc_glue.o.d ${OBJECTDIR}/_ext/1256380230/sys_clk_static.o.d ${OBJECTDIR}/_ext/996991370/sys_ports_static.o.d ${OBJECTDIR}/_ext/1096222929/system_init.o.d ${OBJECTDIR}/_ext/1096222929/system_interrupt.o.d ${OBJECTDIR}/_ext/1096222929/system_exceptions.o.d ${OBJECTDIR}/_ext/1096222929/system_tasks.o.d ${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o.d ${OBJECTDIR}/_ext/1096222929/rtos_hooks.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1360937237/app.o.d ${OBJECTDIR}/_ext/1360937237/http_print.o.d ${OBJECTDIR}/_ext/1360937237/custom_http_app.o.d ${OBJECTDIR}/_ext/1360937237/mpfs_img2.o.d ${OBJECTDIR}/_ext/1360937237/app_wifi.o.d ${OBJECTDIR}/_ext/2046866571/adler32.o.d ${OBJECTDIR}/_ext/2046866571/compress.o.d ${OBJECTDIR}/_ext/2046866571/crc32.o.d ${OBJECTDIR}/_ext/2046866571/deflate.o.d ${OBJECTDIR}/_ext/2046866571/infback.o.d ${OBJECTDIR}/_ext/2046866571/inffast.o.d ${OBJECTDIR}/_ext/2046866571/inflate.o.d ${OBJECTDIR}/_ext/2046866571/inftrees.o.d ${OBJECTDIR}/_ext/2046866571/trees.o.d ${OBJECTDIR}/_ext/2046866571/uncompr.o.d ${OBJECTDIR}/_ext/2046866571/zutil.o.d ${OBJECTDIR}/_ext/29024758/ecc.o.d ${OBJECTDIR}/_ext/29024758/arc4.o.d ${OBJECTDIR}/_ext/29024758/pwdbased.o.d ${OBJECTDIR}/_ext/29024758/tfm.o.d ${OBJECTDIR}/_ext/29024758/asn.o.d ${OBJECTDIR}/_ext/29024758/des3.o.d ${OBJECTDIR}/_ext/29024758/rsa.o.d ${OBJECTDIR}/_ext/29024758/aes.o.d ${OBJECTDIR}/_ext/29024758/md5.o.d ${OBJECTDIR}/_ext/29024758/sha.o.d ${OBJECTDIR}/_ext/29024758/sha256.o.d ${OBJECTDIR}/_ext/29024758/sha512.o.d ${OBJECTDIR}/_ext/29024758/hmac.o.d ${OBJECTDIR}/_ext/29024758/hash.o.d ${OBJECTDIR}/_ext/29024758/compress.o.d ${OBJECTDIR}/_ext/29024758/random.o.d ${OBJECTDIR}/_ext/29024758/crypto.o.d ${OBJECTDIR}/_ext/29024758/asm.o.d ${OBJECTDIR}/_ext/29024758/coding.o.d ${OBJECTDIR}/_ext/29024758/error.o.d ${OBJECTDIR}/_ext/29024758/integer.o.d ${OBJECTDIR}/_ext/29024758/logging.o.d ${OBJECTDIR}/_ext/29024758/memory.o.d ${OBJECTDIR}/_ext/29024758/misc.o.d ${OBJECTDIR}/_ext/29024758/port.o.d ${OBJECTDIR}/_ext/184581597/drv_nvm.o.d ${OBJECTDIR}/_ext/184581597/drv_nvm_erasewrite.o.d ${OBJECTDIR}/_ext/1324760662/drv_spi.o.d ${OBJECTDIR}/_ext/1385053818/drv_spi_sys_queue_fifo.o.d ${OBJECTDIR}/_ext/941160041/drv_tmr.o.d ${OBJECTDIR}/_ext/821589181/drv_usart.o.d ${OBJECTDIR}/_ext/821589181/drv_usart_buffer_queue.o.d ${OBJECTDIR}/_ext/821589181/drv_usart_read_write.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_com.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_commands.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_config_data.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_algorithm.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_manager.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_profile.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_context_loader.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_debug_output.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_eint.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_events.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_event_handler.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_init.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_iwpriv.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_mac.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_main.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_mgmt_msg.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_param_msg.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_pbkdf2.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_power_save.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_raw.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan_helper.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_softap_client_cache.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_spi.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_tx_power.o.d ${OBJECTDIR}/_ext/1023149444/drv_wifi_rtos_wrapper.o.d ${OBJECTDIR}/_ext/871719959/net_pres.o.d ${OBJECTDIR}/_ext/308321468/osal_freertos.o.d ${OBJECTDIR}/_ext/1891376032/sys_command.o.d ${OBJECTDIR}/_ext/1434663852/sys_console.o.d ${OBJECTDIR}/_ext/1434663852/sys_console_uart.o.d ${OBJECTDIR}/_ext/2137108136/sys_debug.o.d ${OBJECTDIR}/_ext/482662494/sys_devcon.o.d ${OBJECTDIR}/_ext/482662494/sys_devcon_pic32mx.o.d ${OBJECTDIR}/_ext/853959373/sys_dma.o.d ${OBJECTDIR}/_ext/1750042194/sys_fs.o.d ${OBJECTDIR}/_ext/1750042194/sys_fs_media_manager.o.d ${OBJECTDIR}/_ext/614935175/mpfs.o.d ${OBJECTDIR}/_ext/1042686532/sys_int_pic32.o.d ${OBJECTDIR}/_ext/470001640/sys_random.o.d ${OBJECTDIR}/_ext/64182180/sys_reset.o.d ${OBJECTDIR}/_ext/2110151058/sys_tmr.o.d ${OBJECTDIR}/_ext/1886247299/sys_fs_wrapper.o.d ${OBJECTDIR}/_ext/1886247299/helpers.o.d ${OBJECTDIR}/_ext/1164207549/tcp.o.d ${OBJECTDIR}/_ext/1164207549/udp.o.d ${OBJECTDIR}/_ext/1164207549/tcpip_heap_alloc.o.d ${OBJECTDIR}/_ext/1164207549/tcpip_heap_internal.o.d ${OBJECTDIR}/_ext/1164207549/arp.o.d ${OBJECTDIR}/_ext/1164207549/dhcp.o.d ${OBJECTDIR}/_ext/1164207549/http.o.d ${OBJECTDIR}/_ext/1164207549/icmp.o.d ${OBJECTDIR}/_ext/1164207549/nbns.o.d ${OBJECTDIR}/_ext/1164207549/smtp.o.d ${OBJECTDIR}/_ext/1164207549/zero_conf_helper.o.d ${OBJECTDIR}/_ext/1164207549/zero_conf_link_local.o.d ${OBJECTDIR}/_ext/1164207549/zero_conf_multicast_dns.o.d ${OBJECTDIR}/_ext/1164207549/iperf.o.d ${OBJECTDIR}/_ext/1164207549/tcpip_commands.o.d ${OBJECTDIR}/_ext/1164207549/hash_fnv.o.d ${OBJECTDIR}/_ext/1164207549/oahash.o.d ${OBJECTDIR}/_ext/1164207549/tcpip_helpers.o.d ${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o.d ${OBJECTDIR}/_ext/1164207549/tcpip_manager.o.d ${OBJECTDIR}/_ext/1164207549/tcpip_notify.o.d ${OBJECTDIR}/_ext/1164207549/tcpip_packet.o.d ${OBJECTDIR}/_ext/1164207549/ipv4.o.d ${OBJECTDIR}/_ext/1164207549/dns.o.d ${OBJECTDIR}/_ext/1107379230/heap_2.o.d ${OBJECTDIR}/_ext/1282498059/port.o.d ${OBJECTDIR}/_ext/1282498059/port_asm.o.d ${OBJECTDIR}/_ext/373060831/croutine.o.d ${OBJECTDIR}/_ext/373060831/list.o.d ${OBJECTDIR}/_ext/373060831/queue.o.d ${OBJECTDIR}/_ext/373060831/tasks.o.d ${OBJECTDIR}/_ext/373060831/timers.o.d ${OBJECTDIR}/_ext/373060831/event_groups.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/975901697/bsp.o ${OBJECTDIR}/_ext/1593769052/drv_spi_tasks.o ${OBJECTDIR}/_ext/1593769052/drv_spi_api.o ${OBJECTDIR}/_ext/1593769052/drv_spi_master_ebm_tasks.o ${OBJECTDIR}/_ext/1593769052/drv_spi_master_dma_tasks.o ${OBJECTDIR}/_ext/448233041/net_pres_enc_glue.o ${OBJECTDIR}/_ext/1256380230/sys_clk_static.o ${OBJECTDIR}/_ext/996991370/sys_ports_static.o ${OBJECTDIR}/_ext/1096222929/system_init.o ${OBJECTDIR}/_ext/1096222929/system_interrupt.o ${OBJECTDIR}/_ext/1096222929/system_exceptions.o ${OBJECTDIR}/_ext/1096222929/system_tasks.o ${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o ${OBJECTDIR}/_ext/1096222929/rtos_hooks.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/http_print.o ${OBJECTDIR}/_ext/1360937237/custom_http_app.o ${OBJECTDIR}/_ext/1360937237/mpfs_img2.o ${OBJECTDIR}/_ext/1360937237/app_wifi.o ${OBJECTDIR}/_ext/2046866571/adler32.o ${OBJECTDIR}/_ext/2046866571/compress.o ${OBJECTDIR}/_ext/2046866571/crc32.o ${OBJECTDIR}/_ext/2046866571/deflate.o ${OBJECTDIR}/_ext/2046866571/infback.o ${OBJECTDIR}/_ext/2046866571/inffast.o ${OBJECTDIR}/_ext/2046866571/inflate.o ${OBJECTDIR}/_ext/2046866571/inftrees.o ${OBJECTDIR}/_ext/2046866571/trees.o ${OBJECTDIR}/_ext/2046866571/uncompr.o ${OBJECTDIR}/_ext/2046866571/zutil.o ${OBJECTDIR}/_ext/29024758/ecc.o ${OBJECTDIR}/_ext/29024758/arc4.o ${OBJECTDIR}/_ext/29024758/pwdbased.o ${OBJECTDIR}/_ext/29024758/tfm.o ${OBJECTDIR}/_ext/29024758/asn.o ${OBJECTDIR}/_ext/29024758/des3.o ${OBJECTDIR}/_ext/29024758/rsa.o ${OBJECTDIR}/_ext/29024758/aes.o ${OBJECTDIR}/_ext/29024758/md5.o ${OBJECTDIR}/_ext/29024758/sha.o ${OBJECTDIR}/_ext/29024758/sha256.o ${OBJECTDIR}/_ext/29024758/sha512.o ${OBJECTDIR}/_ext/29024758/hmac.o ${OBJECTDIR}/_ext/29024758/hash.o ${OBJECTDIR}/_ext/29024758/compress.o ${OBJECTDIR}/_ext/29024758/random.o ${OBJECTDIR}/_ext/29024758/crypto.o ${OBJECTDIR}/_ext/29024758/asm.o ${OBJECTDIR}/_ext/29024758/coding.o ${OBJECTDIR}/_ext/29024758/error.o ${OBJECTDIR}/_ext/29024758/integer.o ${OBJECTDIR}/_ext/29024758/logging.o ${OBJECTDIR}/_ext/29024758/memory.o ${OBJECTDIR}/_ext/29024758/misc.o ${OBJECTDIR}/_ext/29024758/port.o ${OBJECTDIR}/_ext/184581597/drv_nvm.o ${OBJECTDIR}/_ext/184581597/drv_nvm_erasewrite.o ${OBJECTDIR}/_ext/1324760662/drv_spi.o ${OBJECTDIR}/_ext/1385053818/drv_spi_sys_queue_fifo.o ${OBJECTDIR}/_ext/941160041/drv_tmr.o ${OBJECTDIR}/_ext/821589181/drv_usart.o ${OBJECTDIR}/_ext/821589181/drv_usart_buffer_queue.o ${OBJECTDIR}/_ext/821589181/drv_usart_read_write.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_com.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_commands.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_config_data.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_algorithm.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_manager.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_profile.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_context_loader.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_debug_output.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_eint.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_events.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_event_handler.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_init.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_iwpriv.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_mac.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_main.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_mgmt_msg.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_param_msg.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_pbkdf2.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_power_save.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_raw.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan_helper.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_softap_client_cache.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_spi.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_tx_power.o ${OBJECTDIR}/_ext/1023149444/drv_wifi_rtos_wrapper.o ${OBJECTDIR}/_ext/871719959/net_pres.o ${OBJECTDIR}/_ext/308321468/osal_freertos.o ${OBJECTDIR}/_ext/1891376032/sys_command.o ${OBJECTDIR}/_ext/1434663852/sys_console.o ${OBJECTDIR}/_ext/1434663852/sys_console_uart.o ${OBJECTDIR}/_ext/2137108136/sys_debug.o ${OBJECTDIR}/_ext/482662494/sys_devcon.o ${OBJECTDIR}/_ext/482662494/sys_devcon_pic32mx.o ${OBJECTDIR}/_ext/853959373/sys_dma.o ${OBJECTDIR}/_ext/1750042194/sys_fs.o ${OBJECTDIR}/_ext/1750042194/sys_fs_media_manager.o ${OBJECTDIR}/_ext/614935175/mpfs.o ${OBJECTDIR}/_ext/1042686532/sys_int_pic32.o ${OBJECTDIR}/_ext/470001640/sys_random.o ${OBJECTDIR}/_ext/64182180/sys_reset.o ${OBJECTDIR}/_ext/2110151058/sys_tmr.o ${OBJECTDIR}/_ext/1886247299/sys_fs_wrapper.o ${OBJECTDIR}/_ext/1886247299/helpers.o ${OBJECTDIR}/_ext/1164207549/tcp.o ${OBJECTDIR}/_ext/1164207549/udp.o ${OBJECTDIR}/_ext/1164207549/tcpip_heap_alloc.o ${OBJECTDIR}/_ext/1164207549/tcpip_heap_internal.o ${OBJECTDIR}/_ext/1164207549/arp.o ${OBJECTDIR}/_ext/1164207549/dhcp.o ${OBJECTDIR}/_ext/1164207549/http.o ${OBJECTDIR}/_ext/1164207549/icmp.o ${OBJECTDIR}/_ext/1164207549/nbns.o ${OBJECTDIR}/_ext/1164207549/smtp.o ${OBJECTDIR}/_ext/1164207549/zero_conf_helper.o ${OBJECTDIR}/_ext/1164207549/zero_conf_link_local.o ${OBJECTDIR}/_ext/1164207549/zero_conf_multicast_dns.o ${OBJECTDIR}/_ext/1164207549/iperf.o ${OBJECTDIR}/_ext/1164207549/tcpip_commands.o ${OBJECTDIR}/_ext/1164207549/hash_fnv.o ${OBJECTDIR}/_ext/1164207549/oahash.o ${OBJECTDIR}/_ext/1164207549/tcpip_helpers.o ${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o ${OBJECTDIR}/_ext/1164207549/tcpip_manager.o ${OBJECTDIR}/_ext/1164207549/tcpip_notify.o ${OBJECTDIR}/_ext/1164207549/tcpip_packet.o ${OBJECTDIR}/_ext/1164207549/ipv4.o ${OBJECTDIR}/_ext/1164207549/dns.o ${OBJECTDIR}/_ext/1107379230/heap_2.o ${OBJECTDIR}/_ext/1282498059/port.o ${OBJECTDIR}/_ext/1282498059/port_asm.o ${OBJECTDIR}/_ext/373060831/croutine.o ${OBJECTDIR}/_ext/373060831/list.o ${OBJECTDIR}/_ext/373060831/queue.o ${OBJECTDIR}/_ext/373060831/tasks.o ${OBJECTDIR}/_ext/373060831/timers.o ${OBJECTDIR}/_ext/373060831/event_groups.o

# Source Files
SOURCEFILES=../src/system_config/pic32mx795_pim__e16__freertos/bsp/bsp.c ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_tasks.c ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_api.c ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_master_dma_tasks.c ../src/system_config/pic32mx795_pim__e16__freertos/framework/net/pres/net_pres_enc_glue.c ../src/system_config/pic32mx795_pim__e16__freertos/framework/system/clk/src/sys_clk_static.c ../src/system_config/pic32mx795_pim__e16__freertos/framework/system/ports/src/sys_ports_static.c ../src/system_config/pic32mx795_pim__e16__freertos/system_init.c ../src/system_config/pic32mx795_pim__e16__freertos/system_interrupt.c ../src/system_config/pic32mx795_pim__e16__freertos/system_exceptions.c ../src/system_config/pic32mx795_pim__e16__freertos/system_tasks.c ../src/system_config/pic32mx795_pim__e16__freertos/system_interrupt_a.S ../src/system_config/pic32mx795_pim__e16__freertos/rtos_hooks.c ../src/main.c ../src/app.c ../src/http_print.c ../src/custom_http_app.c ../src/mpfs_img2.c ../src/app_wifi.c ../../../../../framework/crypto/src/zlib-1.2.7/adler32.c ../../../../../framework/crypto/src/zlib-1.2.7/compress.c ../../../../../framework/crypto/src/zlib-1.2.7/crc32.c ../../../../../framework/crypto/src/zlib-1.2.7/deflate.c ../../../../../framework/crypto/src/zlib-1.2.7/infback.c ../../../../../framework/crypto/src/zlib-1.2.7/inffast.c ../../../../../framework/crypto/src/zlib-1.2.7/inflate.c ../../../../../framework/crypto/src/zlib-1.2.7/inftrees.c ../../../../../framework/crypto/src/zlib-1.2.7/trees.c ../../../../../framework/crypto/src/zlib-1.2.7/uncompr.c ../../../../../framework/crypto/src/zlib-1.2.7/zutil.c ../../../../../framework/crypto/src/ecc.c ../../../../../framework/crypto/src/arc4.c ../../../../../framework/crypto/src/pwdbased.c ../../../../../framework/crypto/src/tfm.c ../../../../../framework/crypto/src/asn.c ../../../../../framework/crypto/src/des3.c ../../../../../framework/crypto/src/rsa.c ../../../../../framework/crypto/src/aes.c ../../../../../framework/crypto/src/md5.c ../../../../../framework/crypto/src/sha.c ../../../../../framework/crypto/src/sha256.c ../../../../../framework/crypto/src/sha512.c ../../../../../framework/crypto/src/hmac.c ../../../../../framework/crypto/src/hash.c ../../../../../framework/crypto/src/compress.c ../../../../../framework/crypto/src/random.c ../../../../../framework/crypto/src/crypto.c ../../../../../framework/crypto/src/asm.c ../../../../../framework/crypto/src/coding.c ../../../../../framework/crypto/src/error.c ../../../../../framework/crypto/src/integer.c ../../../../../framework/crypto/src/logging.c ../../../../../framework/crypto/src/memory.c ../../../../../framework/crypto/src/misc.c ../../../../../framework/crypto/src/port.c ../../../../../framework/driver/nvm/src/dynamic/drv_nvm.c ../../../../../framework/driver/nvm/src/dynamic/drv_nvm_erasewrite.c ../../../../../framework/driver/spi/src/dynamic/drv_spi.c ../../../../../framework/driver/spi/src/drv_spi_sys_queue_fifo.c ../../../../../framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../../framework/driver/usart/src/dynamic/drv_usart.c ../../../../../framework/driver/usart/src/dynamic/drv_usart_buffer_queue.c ../../../../../framework/driver/usart/src/dynamic/drv_usart_read_write.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_com.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_commands.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_config_data.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_algorithm.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_manager.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_profile.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_context_loader.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_debug_output.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_eint.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_events.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_event_handler.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_init.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_iwpriv.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_mac.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_main.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_mgmt_msg.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_param_msg.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_pbkdf2.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_power_save.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_raw.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_scan.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_scan_helper.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_softap_client_cache.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_spi.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_tx_power.c ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_rtos_wrapper.c ../../../../../framework/net/pres/src/net_pres.c ../../../../../framework/osal/src/osal_freertos.c ../../../../../framework/system/command/src/sys_command.c ../../../../../framework/system/console/src/sys_console.c ../../../../../framework/system/console/src/sys_console_uart.c ../../../../../framework/system/debug/src/sys_debug.c ../../../../../framework/system/devcon/src/sys_devcon.c ../../../../../framework/system/devcon/src/sys_devcon_pic32mx.c ../../../../../framework/system/dma/src/sys_dma.c ../../../../../framework/system/fs/src/dynamic/sys_fs.c ../../../../../framework/system/fs/src/dynamic/sys_fs_media_manager.c ../../../../../framework/system/fs/mpfs/src/mpfs.c ../../../../../framework/system/int/src/sys_int_pic32.c ../../../../../framework/system/random/src/sys_random.c ../../../../../framework/system/reset/src/sys_reset.c ../../../../../framework/system/tmr/src/sys_tmr.c ../../../../../framework/tcpip/src/common/sys_fs_wrapper.c ../../../../../framework/tcpip/src/common/helpers.c ../../../../../framework/tcpip/src/tcp.c ../../../../../framework/tcpip/src/udp.c ../../../../../framework/tcpip/src/tcpip_heap_alloc.c ../../../../../framework/tcpip/src/tcpip_heap_internal.c ../../../../../framework/tcpip/src/arp.c ../../../../../framework/tcpip/src/dhcp.c ../../../../../framework/tcpip/src/http.c ../../../../../framework/tcpip/src/icmp.c ../../../../../framework/tcpip/src/nbns.c ../../../../../framework/tcpip/src/smtp.c ../../../../../framework/tcpip/src/zero_conf_helper.c ../../../../../framework/tcpip/src/zero_conf_link_local.c ../../../../../framework/tcpip/src/zero_conf_multicast_dns.c ../../../../../framework/tcpip/src/iperf.c ../../../../../framework/tcpip/src/tcpip_commands.c ../../../../../framework/tcpip/src/hash_fnv.c ../../../../../framework/tcpip/src/oahash.c ../../../../../framework/tcpip/src/tcpip_helpers.c ../../../../../framework/tcpip/src/tcpip_helper_c32.S ../../../../../framework/tcpip/src/tcpip_manager.c ../../../../../framework/tcpip/src/tcpip_notify.c ../../../../../framework/tcpip/src/tcpip_packet.c ../../../../../framework/tcpip/src/ipv4.c ../../../../../framework/tcpip/src/dns.c ../../../../../third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_2.c ../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c ../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S ../../../../../third_party/rtos/FreeRTOS/Source/croutine.c ../../../../../third_party/rtos/FreeRTOS/Source/list.c ../../../../../third_party/rtos/FreeRTOS/Source/queue.c ../../../../../third_party/rtos/FreeRTOS/Source/tasks.c ../../../../../third_party/rtos/FreeRTOS/Source/timers.c ../../../../../third_party/rtos/FreeRTOS/Source/event_groups.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-pic32mx795_pim__e16__freertos.mk dist/${CND_CONF}/${IMAGE_TYPE}/pic32_wifi_web_server.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX795F512L
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o: ../src/system_config/pic32mx795_pim__e16__freertos/system_interrupt_a.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1096222929" 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o.ok ${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o.d" "${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/pic32mx795_pim__e16__freertos" -MMD -MF "${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o.d"  -o ${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o ../src/system_config/pic32mx795_pim__e16__freertos/system_interrupt_a.S  -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1
	
${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o: ../../../../../framework/tcpip/src/tcpip_helper_c32.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o.ok ${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o.d" "${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/pic32mx795_pim__e16__freertos" -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o.d"  -o ${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o ../../../../../framework/tcpip/src/tcpip_helper_c32.S  -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1
	
${OBJECTDIR}/_ext/1282498059/port_asm.o: ../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1282498059" 
	@${RM} ${OBJECTDIR}/_ext/1282498059/port_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1282498059/port_asm.o 
	@${RM} ${OBJECTDIR}/_ext/1282498059/port_asm.o.ok ${OBJECTDIR}/_ext/1282498059/port_asm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1282498059/port_asm.o.d" "${OBJECTDIR}/_ext/1282498059/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1 -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/pic32mx795_pim__e16__freertos" -MMD -MF "${OBJECTDIR}/_ext/1282498059/port_asm.o.d"  -o ${OBJECTDIR}/_ext/1282498059/port_asm.o ../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S  -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1282498059/port_asm.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1
	
else
${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o: ../src/system_config/pic32mx795_pim__e16__freertos/system_interrupt_a.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1096222929" 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o.ok ${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o.d" "${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/pic32mx795_pim__e16__freertos" -MMD -MF "${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o.d"  -o ${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o ../src/system_config/pic32mx795_pim__e16__freertos/system_interrupt_a.S  -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1096222929/system_interrupt_a.o.asm.d",--gdwarf-2
	
${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o: ../../../../../framework/tcpip/src/tcpip_helper_c32.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o.ok ${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o.d" "${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/pic32mx795_pim__e16__freertos" -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o.d"  -o ${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o ../../../../../framework/tcpip/src/tcpip_helper_c32.S  -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1164207549/tcpip_helper_c32.o.asm.d",--gdwarf-2
	
${OBJECTDIR}/_ext/1282498059/port_asm.o: ../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1282498059" 
	@${RM} ${OBJECTDIR}/_ext/1282498059/port_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1282498059/port_asm.o 
	@${RM} ${OBJECTDIR}/_ext/1282498059/port_asm.o.ok ${OBJECTDIR}/_ext/1282498059/port_asm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1282498059/port_asm.o.d" "${OBJECTDIR}/_ext/1282498059/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/pic32mx795_pim__e16__freertos" -MMD -MF "${OBJECTDIR}/_ext/1282498059/port_asm.o.d"  -o ${OBJECTDIR}/_ext/1282498059/port_asm.o ../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S  -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1282498059/port_asm.o.asm.d",--gdwarf-2
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/975901697/bsp.o: ../src/system_config/pic32mx795_pim__e16__freertos/bsp/bsp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/975901697" 
	@${RM} ${OBJECTDIR}/_ext/975901697/bsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/975901697/bsp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/975901697/bsp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/975901697/bsp.o.d" -o ${OBJECTDIR}/_ext/975901697/bsp.o ../src/system_config/pic32mx795_pim__e16__freertos/bsp/bsp.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1593769052/drv_spi_tasks.o: ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1593769052" 
	@${RM} ${OBJECTDIR}/_ext/1593769052/drv_spi_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1593769052/drv_spi_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1593769052/drv_spi_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1593769052/drv_spi_tasks.o.d" -o ${OBJECTDIR}/_ext/1593769052/drv_spi_tasks.o ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_tasks.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1593769052/drv_spi_api.o: ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_api.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1593769052" 
	@${RM} ${OBJECTDIR}/_ext/1593769052/drv_spi_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/1593769052/drv_spi_api.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1593769052/drv_spi_api.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1593769052/drv_spi_api.o.d" -o ${OBJECTDIR}/_ext/1593769052/drv_spi_api.o ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_api.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1593769052/drv_spi_master_ebm_tasks.o: ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1593769052" 
	@${RM} ${OBJECTDIR}/_ext/1593769052/drv_spi_master_ebm_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1593769052/drv_spi_master_ebm_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1593769052/drv_spi_master_ebm_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1593769052/drv_spi_master_ebm_tasks.o.d" -o ${OBJECTDIR}/_ext/1593769052/drv_spi_master_ebm_tasks.o ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1593769052/drv_spi_master_dma_tasks.o: ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_master_dma_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1593769052" 
	@${RM} ${OBJECTDIR}/_ext/1593769052/drv_spi_master_dma_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1593769052/drv_spi_master_dma_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1593769052/drv_spi_master_dma_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1593769052/drv_spi_master_dma_tasks.o.d" -o ${OBJECTDIR}/_ext/1593769052/drv_spi_master_dma_tasks.o ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_master_dma_tasks.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/448233041/net_pres_enc_glue.o: ../src/system_config/pic32mx795_pim__e16__freertos/framework/net/pres/net_pres_enc_glue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/448233041" 
	@${RM} ${OBJECTDIR}/_ext/448233041/net_pres_enc_glue.o.d 
	@${RM} ${OBJECTDIR}/_ext/448233041/net_pres_enc_glue.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/448233041/net_pres_enc_glue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/448233041/net_pres_enc_glue.o.d" -o ${OBJECTDIR}/_ext/448233041/net_pres_enc_glue.o ../src/system_config/pic32mx795_pim__e16__freertos/framework/net/pres/net_pres_enc_glue.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1256380230/sys_clk_static.o: ../src/system_config/pic32mx795_pim__e16__freertos/framework/system/clk/src/sys_clk_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1256380230" 
	@${RM} ${OBJECTDIR}/_ext/1256380230/sys_clk_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1256380230/sys_clk_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1256380230/sys_clk_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1256380230/sys_clk_static.o.d" -o ${OBJECTDIR}/_ext/1256380230/sys_clk_static.o ../src/system_config/pic32mx795_pim__e16__freertos/framework/system/clk/src/sys_clk_static.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/996991370/sys_ports_static.o: ../src/system_config/pic32mx795_pim__e16__freertos/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/996991370" 
	@${RM} ${OBJECTDIR}/_ext/996991370/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/996991370/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/996991370/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/996991370/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/996991370/sys_ports_static.o ../src/system_config/pic32mx795_pim__e16__freertos/framework/system/ports/src/sys_ports_static.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1096222929/system_init.o: ../src/system_config/pic32mx795_pim__e16__freertos/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1096222929" 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1096222929/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1096222929/system_init.o.d" -o ${OBJECTDIR}/_ext/1096222929/system_init.o ../src/system_config/pic32mx795_pim__e16__freertos/system_init.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1096222929/system_interrupt.o: ../src/system_config/pic32mx795_pim__e16__freertos/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1096222929" 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1096222929/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1096222929/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1096222929/system_interrupt.o ../src/system_config/pic32mx795_pim__e16__freertos/system_interrupt.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1096222929/system_exceptions.o: ../src/system_config/pic32mx795_pim__e16__freertos/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1096222929" 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1096222929/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1096222929/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1096222929/system_exceptions.o ../src/system_config/pic32mx795_pim__e16__freertos/system_exceptions.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1096222929/system_tasks.o: ../src/system_config/pic32mx795_pim__e16__freertos/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1096222929" 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1096222929/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1096222929/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1096222929/system_tasks.o ../src/system_config/pic32mx795_pim__e16__freertos/system_tasks.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1096222929/rtos_hooks.o: ../src/system_config/pic32mx795_pim__e16__freertos/rtos_hooks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1096222929" 
	@${RM} ${OBJECTDIR}/_ext/1096222929/rtos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1096222929/rtos_hooks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1096222929/rtos_hooks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1096222929/rtos_hooks.o.d" -o ${OBJECTDIR}/_ext/1096222929/rtos_hooks.o ../src/system_config/pic32mx795_pim__e16__freertos/rtos_hooks.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/http_print.o: ../src/http_print.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/http_print.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/http_print.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/http_print.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/http_print.o.d" -o ${OBJECTDIR}/_ext/1360937237/http_print.o ../src/http_print.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/custom_http_app.o: ../src/custom_http_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/custom_http_app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/custom_http_app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/custom_http_app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/custom_http_app.o.d" -o ${OBJECTDIR}/_ext/1360937237/custom_http_app.o ../src/custom_http_app.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/mpfs_img2.o: ../src/mpfs_img2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mpfs_img2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mpfs_img2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/mpfs_img2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/mpfs_img2.o.d" -o ${OBJECTDIR}/_ext/1360937237/mpfs_img2.o ../src/mpfs_img2.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/app_wifi.o: ../src/app_wifi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app_wifi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app_wifi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/app_wifi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/app_wifi.o.d" -o ${OBJECTDIR}/_ext/1360937237/app_wifi.o ../src/app_wifi.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/adler32.o: ../../../../../framework/crypto/src/zlib-1.2.7/adler32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/adler32.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/adler32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/adler32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/adler32.o.d" -o ${OBJECTDIR}/_ext/2046866571/adler32.o ../../../../../framework/crypto/src/zlib-1.2.7/adler32.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/compress.o: ../../../../../framework/crypto/src/zlib-1.2.7/compress.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/compress.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/compress.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/compress.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/compress.o.d" -o ${OBJECTDIR}/_ext/2046866571/compress.o ../../../../../framework/crypto/src/zlib-1.2.7/compress.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/crc32.o: ../../../../../framework/crypto/src/zlib-1.2.7/crc32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/crc32.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/crc32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/crc32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/crc32.o.d" -o ${OBJECTDIR}/_ext/2046866571/crc32.o ../../../../../framework/crypto/src/zlib-1.2.7/crc32.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/deflate.o: ../../../../../framework/crypto/src/zlib-1.2.7/deflate.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/deflate.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/deflate.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/deflate.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/deflate.o.d" -o ${OBJECTDIR}/_ext/2046866571/deflate.o ../../../../../framework/crypto/src/zlib-1.2.7/deflate.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/infback.o: ../../../../../framework/crypto/src/zlib-1.2.7/infback.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/infback.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/infback.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/infback.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/infback.o.d" -o ${OBJECTDIR}/_ext/2046866571/infback.o ../../../../../framework/crypto/src/zlib-1.2.7/infback.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/inffast.o: ../../../../../framework/crypto/src/zlib-1.2.7/inffast.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/inffast.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/inffast.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/inffast.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/inffast.o.d" -o ${OBJECTDIR}/_ext/2046866571/inffast.o ../../../../../framework/crypto/src/zlib-1.2.7/inffast.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/inflate.o: ../../../../../framework/crypto/src/zlib-1.2.7/inflate.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/inflate.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/inflate.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/inflate.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/inflate.o.d" -o ${OBJECTDIR}/_ext/2046866571/inflate.o ../../../../../framework/crypto/src/zlib-1.2.7/inflate.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/inftrees.o: ../../../../../framework/crypto/src/zlib-1.2.7/inftrees.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/inftrees.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/inftrees.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/inftrees.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/inftrees.o.d" -o ${OBJECTDIR}/_ext/2046866571/inftrees.o ../../../../../framework/crypto/src/zlib-1.2.7/inftrees.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/trees.o: ../../../../../framework/crypto/src/zlib-1.2.7/trees.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/trees.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/trees.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/trees.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/trees.o.d" -o ${OBJECTDIR}/_ext/2046866571/trees.o ../../../../../framework/crypto/src/zlib-1.2.7/trees.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/uncompr.o: ../../../../../framework/crypto/src/zlib-1.2.7/uncompr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/uncompr.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/uncompr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/uncompr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/uncompr.o.d" -o ${OBJECTDIR}/_ext/2046866571/uncompr.o ../../../../../framework/crypto/src/zlib-1.2.7/uncompr.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/zutil.o: ../../../../../framework/crypto/src/zlib-1.2.7/zutil.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/zutil.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/zutil.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/zutil.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/zutil.o.d" -o ${OBJECTDIR}/_ext/2046866571/zutil.o ../../../../../framework/crypto/src/zlib-1.2.7/zutil.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/ecc.o: ../../../../../framework/crypto/src/ecc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/ecc.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/ecc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/ecc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/ecc.o.d" -o ${OBJECTDIR}/_ext/29024758/ecc.o ../../../../../framework/crypto/src/ecc.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/arc4.o: ../../../../../framework/crypto/src/arc4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/arc4.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/arc4.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/arc4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/arc4.o.d" -o ${OBJECTDIR}/_ext/29024758/arc4.o ../../../../../framework/crypto/src/arc4.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/pwdbased.o: ../../../../../framework/crypto/src/pwdbased.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/pwdbased.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/pwdbased.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/pwdbased.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/pwdbased.o.d" -o ${OBJECTDIR}/_ext/29024758/pwdbased.o ../../../../../framework/crypto/src/pwdbased.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/tfm.o: ../../../../../framework/crypto/src/tfm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/tfm.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/tfm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/tfm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/tfm.o.d" -o ${OBJECTDIR}/_ext/29024758/tfm.o ../../../../../framework/crypto/src/tfm.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/asn.o: ../../../../../framework/crypto/src/asn.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/asn.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/asn.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/asn.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/asn.o.d" -o ${OBJECTDIR}/_ext/29024758/asn.o ../../../../../framework/crypto/src/asn.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/des3.o: ../../../../../framework/crypto/src/des3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/des3.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/des3.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/des3.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/des3.o.d" -o ${OBJECTDIR}/_ext/29024758/des3.o ../../../../../framework/crypto/src/des3.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/rsa.o: ../../../../../framework/crypto/src/rsa.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/rsa.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/rsa.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/rsa.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/rsa.o.d" -o ${OBJECTDIR}/_ext/29024758/rsa.o ../../../../../framework/crypto/src/rsa.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/aes.o: ../../../../../framework/crypto/src/aes.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/aes.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/aes.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/aes.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/aes.o.d" -o ${OBJECTDIR}/_ext/29024758/aes.o ../../../../../framework/crypto/src/aes.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/md5.o: ../../../../../framework/crypto/src/md5.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/md5.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/md5.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/md5.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/md5.o.d" -o ${OBJECTDIR}/_ext/29024758/md5.o ../../../../../framework/crypto/src/md5.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/sha.o: ../../../../../framework/crypto/src/sha.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/sha.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/sha.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/sha.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/sha.o.d" -o ${OBJECTDIR}/_ext/29024758/sha.o ../../../../../framework/crypto/src/sha.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/sha256.o: ../../../../../framework/crypto/src/sha256.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/sha256.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/sha256.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/sha256.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/sha256.o.d" -o ${OBJECTDIR}/_ext/29024758/sha256.o ../../../../../framework/crypto/src/sha256.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/sha512.o: ../../../../../framework/crypto/src/sha512.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/sha512.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/sha512.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/sha512.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/sha512.o.d" -o ${OBJECTDIR}/_ext/29024758/sha512.o ../../../../../framework/crypto/src/sha512.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/hmac.o: ../../../../../framework/crypto/src/hmac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/hmac.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/hmac.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/hmac.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/hmac.o.d" -o ${OBJECTDIR}/_ext/29024758/hmac.o ../../../../../framework/crypto/src/hmac.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/hash.o: ../../../../../framework/crypto/src/hash.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/hash.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/hash.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/hash.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/hash.o.d" -o ${OBJECTDIR}/_ext/29024758/hash.o ../../../../../framework/crypto/src/hash.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/compress.o: ../../../../../framework/crypto/src/compress.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/compress.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/compress.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/compress.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/compress.o.d" -o ${OBJECTDIR}/_ext/29024758/compress.o ../../../../../framework/crypto/src/compress.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/random.o: ../../../../../framework/crypto/src/random.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/random.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/random.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/random.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/random.o.d" -o ${OBJECTDIR}/_ext/29024758/random.o ../../../../../framework/crypto/src/random.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/crypto.o: ../../../../../framework/crypto/src/crypto.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/crypto.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/crypto.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/crypto.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/crypto.o.d" -o ${OBJECTDIR}/_ext/29024758/crypto.o ../../../../../framework/crypto/src/crypto.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/asm.o: ../../../../../framework/crypto/src/asm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/asm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/asm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/asm.o.d" -o ${OBJECTDIR}/_ext/29024758/asm.o ../../../../../framework/crypto/src/asm.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/coding.o: ../../../../../framework/crypto/src/coding.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/coding.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/coding.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/coding.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/coding.o.d" -o ${OBJECTDIR}/_ext/29024758/coding.o ../../../../../framework/crypto/src/coding.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/error.o: ../../../../../framework/crypto/src/error.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/error.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/error.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/error.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/error.o.d" -o ${OBJECTDIR}/_ext/29024758/error.o ../../../../../framework/crypto/src/error.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/integer.o: ../../../../../framework/crypto/src/integer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/integer.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/integer.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/integer.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/integer.o.d" -o ${OBJECTDIR}/_ext/29024758/integer.o ../../../../../framework/crypto/src/integer.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/logging.o: ../../../../../framework/crypto/src/logging.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/logging.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/logging.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/logging.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/logging.o.d" -o ${OBJECTDIR}/_ext/29024758/logging.o ../../../../../framework/crypto/src/logging.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/memory.o: ../../../../../framework/crypto/src/memory.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/memory.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/memory.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/memory.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/memory.o.d" -o ${OBJECTDIR}/_ext/29024758/memory.o ../../../../../framework/crypto/src/memory.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/misc.o: ../../../../../framework/crypto/src/misc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/misc.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/misc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/misc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/misc.o.d" -o ${OBJECTDIR}/_ext/29024758/misc.o ../../../../../framework/crypto/src/misc.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/port.o: ../../../../../framework/crypto/src/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/port.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/port.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/port.o.d" -o ${OBJECTDIR}/_ext/29024758/port.o ../../../../../framework/crypto/src/port.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/184581597/drv_nvm.o: ../../../../../framework/driver/nvm/src/dynamic/drv_nvm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/184581597" 
	@${RM} ${OBJECTDIR}/_ext/184581597/drv_nvm.o.d 
	@${RM} ${OBJECTDIR}/_ext/184581597/drv_nvm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/184581597/drv_nvm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/184581597/drv_nvm.o.d" -o ${OBJECTDIR}/_ext/184581597/drv_nvm.o ../../../../../framework/driver/nvm/src/dynamic/drv_nvm.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/184581597/drv_nvm_erasewrite.o: ../../../../../framework/driver/nvm/src/dynamic/drv_nvm_erasewrite.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/184581597" 
	@${RM} ${OBJECTDIR}/_ext/184581597/drv_nvm_erasewrite.o.d 
	@${RM} ${OBJECTDIR}/_ext/184581597/drv_nvm_erasewrite.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/184581597/drv_nvm_erasewrite.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/184581597/drv_nvm_erasewrite.o.d" -o ${OBJECTDIR}/_ext/184581597/drv_nvm_erasewrite.o ../../../../../framework/driver/nvm/src/dynamic/drv_nvm_erasewrite.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1324760662/drv_spi.o: ../../../../../framework/driver/spi/src/dynamic/drv_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1324760662" 
	@${RM} ${OBJECTDIR}/_ext/1324760662/drv_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1324760662/drv_spi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1324760662/drv_spi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1324760662/drv_spi.o.d" -o ${OBJECTDIR}/_ext/1324760662/drv_spi.o ../../../../../framework/driver/spi/src/dynamic/drv_spi.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1385053818/drv_spi_sys_queue_fifo.o: ../../../../../framework/driver/spi/src/drv_spi_sys_queue_fifo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1385053818" 
	@${RM} ${OBJECTDIR}/_ext/1385053818/drv_spi_sys_queue_fifo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1385053818/drv_spi_sys_queue_fifo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1385053818/drv_spi_sys_queue_fifo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1385053818/drv_spi_sys_queue_fifo.o.d" -o ${OBJECTDIR}/_ext/1385053818/drv_spi_sys_queue_fifo.o ../../../../../framework/driver/spi/src/drv_spi_sys_queue_fifo.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/941160041/drv_tmr.o: ../../../../../framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/941160041" 
	@${RM} ${OBJECTDIR}/_ext/941160041/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/941160041/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/941160041/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/941160041/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/941160041/drv_tmr.o ../../../../../framework/driver/tmr/src/dynamic/drv_tmr.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/821589181/drv_usart.o: ../../../../../framework/driver/usart/src/dynamic/drv_usart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/821589181" 
	@${RM} ${OBJECTDIR}/_ext/821589181/drv_usart.o.d 
	@${RM} ${OBJECTDIR}/_ext/821589181/drv_usart.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/821589181/drv_usart.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/821589181/drv_usart.o.d" -o ${OBJECTDIR}/_ext/821589181/drv_usart.o ../../../../../framework/driver/usart/src/dynamic/drv_usart.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/821589181/drv_usart_buffer_queue.o: ../../../../../framework/driver/usart/src/dynamic/drv_usart_buffer_queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/821589181" 
	@${RM} ${OBJECTDIR}/_ext/821589181/drv_usart_buffer_queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/821589181/drv_usart_buffer_queue.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/821589181/drv_usart_buffer_queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/821589181/drv_usart_buffer_queue.o.d" -o ${OBJECTDIR}/_ext/821589181/drv_usart_buffer_queue.o ../../../../../framework/driver/usart/src/dynamic/drv_usart_buffer_queue.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/821589181/drv_usart_read_write.o: ../../../../../framework/driver/usart/src/dynamic/drv_usart_read_write.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/821589181" 
	@${RM} ${OBJECTDIR}/_ext/821589181/drv_usart_read_write.o.d 
	@${RM} ${OBJECTDIR}/_ext/821589181/drv_usart_read_write.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/821589181/drv_usart_read_write.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/821589181/drv_usart_read_write.o.d" -o ${OBJECTDIR}/_ext/821589181/drv_usart_read_write.o ../../../../../framework/driver/usart/src/dynamic/drv_usart_read_write.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_com.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_com.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_com.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_com.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_com.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_com.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_com.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_com.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_commands.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_commands.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_commands.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_commands.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_commands.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_commands.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_commands.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_commands.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_config_data.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_config_data.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_config_data.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_config_data.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_config_data.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_config_data.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_config_data.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_config_data.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_algorithm.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_algorithm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_algorithm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_algorithm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_algorithm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_algorithm.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_algorithm.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_algorithm.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_manager.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_manager.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_manager.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_manager.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_manager.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_manager.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_profile.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_profile.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_profile.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_profile.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_profile.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_profile.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_profile.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_profile.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_context_loader.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_context_loader.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_context_loader.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_context_loader.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_context_loader.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_context_loader.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_context_loader.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_context_loader.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_debug_output.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_debug_output.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_debug_output.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_debug_output.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_debug_output.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_debug_output.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_debug_output.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_debug_output.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_eint.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_eint.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_eint.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_eint.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_eint.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_eint.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_eint.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_eint.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_events.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_events.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_events.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_events.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_events.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_events.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_events.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_events.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_event_handler.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_event_handler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_event_handler.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_event_handler.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_event_handler.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_event_handler.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_event_handler.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_event_handler.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_init.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_init.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_init.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_init.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_iwpriv.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_iwpriv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_iwpriv.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_iwpriv.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_iwpriv.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_iwpriv.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_iwpriv.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_iwpriv.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_mac.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_mac.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_mac.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_mac.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_mac.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_mac.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_mac.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_main.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_main.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_main.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_main.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_mgmt_msg.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_mgmt_msg.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_mgmt_msg.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_mgmt_msg.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_mgmt_msg.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_mgmt_msg.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_mgmt_msg.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_mgmt_msg.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_param_msg.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_param_msg.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_param_msg.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_param_msg.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_param_msg.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_param_msg.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_param_msg.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_param_msg.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_pbkdf2.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_pbkdf2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_pbkdf2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_pbkdf2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_pbkdf2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_pbkdf2.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_pbkdf2.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_pbkdf2.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_power_save.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_power_save.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_power_save.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_power_save.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_power_save.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_power_save.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_power_save.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_power_save.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_raw.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_raw.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_raw.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_raw.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_raw.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_raw.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_raw.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_raw.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_scan.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_scan.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_scan.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_scan.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_scan.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_scan_helper.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_scan_helper.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan_helper.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan_helper.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_scan_helper.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_scan_helper.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan_helper.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_scan_helper.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_softap_client_cache.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_softap_client_cache.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_softap_client_cache.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_softap_client_cache.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_softap_client_cache.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_softap_client_cache.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_softap_client_cache.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_softap_client_cache.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_spi.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_spi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_spi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_spi.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_spi.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_spi.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_tx_power.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_tx_power.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_tx_power.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_tx_power.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_tx_power.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_tx_power.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_tx_power.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_tx_power.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_rtos_wrapper.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_rtos_wrapper.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_rtos_wrapper.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_rtos_wrapper.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_rtos_wrapper.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_rtos_wrapper.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_rtos_wrapper.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_rtos_wrapper.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/871719959/net_pres.o: ../../../../../framework/net/pres/src/net_pres.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/871719959" 
	@${RM} ${OBJECTDIR}/_ext/871719959/net_pres.o.d 
	@${RM} ${OBJECTDIR}/_ext/871719959/net_pres.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/871719959/net_pres.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/871719959/net_pres.o.d" -o ${OBJECTDIR}/_ext/871719959/net_pres.o ../../../../../framework/net/pres/src/net_pres.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/308321468/osal_freertos.o: ../../../../../framework/osal/src/osal_freertos.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/308321468" 
	@${RM} ${OBJECTDIR}/_ext/308321468/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/308321468/osal_freertos.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/308321468/osal_freertos.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/308321468/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/308321468/osal_freertos.o ../../../../../framework/osal/src/osal_freertos.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1891376032/sys_command.o: ../../../../../framework/system/command/src/sys_command.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1891376032" 
	@${RM} ${OBJECTDIR}/_ext/1891376032/sys_command.o.d 
	@${RM} ${OBJECTDIR}/_ext/1891376032/sys_command.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1891376032/sys_command.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1891376032/sys_command.o.d" -o ${OBJECTDIR}/_ext/1891376032/sys_command.o ../../../../../framework/system/command/src/sys_command.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1434663852/sys_console.o: ../../../../../framework/system/console/src/sys_console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1434663852" 
	@${RM} ${OBJECTDIR}/_ext/1434663852/sys_console.o.d 
	@${RM} ${OBJECTDIR}/_ext/1434663852/sys_console.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1434663852/sys_console.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1434663852/sys_console.o.d" -o ${OBJECTDIR}/_ext/1434663852/sys_console.o ../../../../../framework/system/console/src/sys_console.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1434663852/sys_console_uart.o: ../../../../../framework/system/console/src/sys_console_uart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1434663852" 
	@${RM} ${OBJECTDIR}/_ext/1434663852/sys_console_uart.o.d 
	@${RM} ${OBJECTDIR}/_ext/1434663852/sys_console_uart.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1434663852/sys_console_uart.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1434663852/sys_console_uart.o.d" -o ${OBJECTDIR}/_ext/1434663852/sys_console_uart.o ../../../../../framework/system/console/src/sys_console_uart.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2137108136/sys_debug.o: ../../../../../framework/system/debug/src/sys_debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2137108136" 
	@${RM} ${OBJECTDIR}/_ext/2137108136/sys_debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/2137108136/sys_debug.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2137108136/sys_debug.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2137108136/sys_debug.o.d" -o ${OBJECTDIR}/_ext/2137108136/sys_debug.o ../../../../../framework/system/debug/src/sys_debug.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/482662494/sys_devcon.o: ../../../../../framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/482662494" 
	@${RM} ${OBJECTDIR}/_ext/482662494/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/482662494/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/482662494/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/482662494/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/482662494/sys_devcon.o ../../../../../framework/system/devcon/src/sys_devcon.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/482662494/sys_devcon_pic32mx.o: ../../../../../framework/system/devcon/src/sys_devcon_pic32mx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/482662494" 
	@${RM} ${OBJECTDIR}/_ext/482662494/sys_devcon_pic32mx.o.d 
	@${RM} ${OBJECTDIR}/_ext/482662494/sys_devcon_pic32mx.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/482662494/sys_devcon_pic32mx.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/482662494/sys_devcon_pic32mx.o.d" -o ${OBJECTDIR}/_ext/482662494/sys_devcon_pic32mx.o ../../../../../framework/system/devcon/src/sys_devcon_pic32mx.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/853959373/sys_dma.o: ../../../../../framework/system/dma/src/sys_dma.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/853959373" 
	@${RM} ${OBJECTDIR}/_ext/853959373/sys_dma.o.d 
	@${RM} ${OBJECTDIR}/_ext/853959373/sys_dma.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/853959373/sys_dma.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/853959373/sys_dma.o.d" -o ${OBJECTDIR}/_ext/853959373/sys_dma.o ../../../../../framework/system/dma/src/sys_dma.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1750042194/sys_fs.o: ../../../../../framework/system/fs/src/dynamic/sys_fs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1750042194" 
	@${RM} ${OBJECTDIR}/_ext/1750042194/sys_fs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1750042194/sys_fs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1750042194/sys_fs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1750042194/sys_fs.o.d" -o ${OBJECTDIR}/_ext/1750042194/sys_fs.o ../../../../../framework/system/fs/src/dynamic/sys_fs.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1750042194/sys_fs_media_manager.o: ../../../../../framework/system/fs/src/dynamic/sys_fs_media_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1750042194" 
	@${RM} ${OBJECTDIR}/_ext/1750042194/sys_fs_media_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/1750042194/sys_fs_media_manager.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1750042194/sys_fs_media_manager.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1750042194/sys_fs_media_manager.o.d" -o ${OBJECTDIR}/_ext/1750042194/sys_fs_media_manager.o ../../../../../framework/system/fs/src/dynamic/sys_fs_media_manager.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/614935175/mpfs.o: ../../../../../framework/system/fs/mpfs/src/mpfs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/614935175" 
	@${RM} ${OBJECTDIR}/_ext/614935175/mpfs.o.d 
	@${RM} ${OBJECTDIR}/_ext/614935175/mpfs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/614935175/mpfs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/614935175/mpfs.o.d" -o ${OBJECTDIR}/_ext/614935175/mpfs.o ../../../../../framework/system/fs/mpfs/src/mpfs.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1042686532/sys_int_pic32.o: ../../../../../framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1042686532" 
	@${RM} ${OBJECTDIR}/_ext/1042686532/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1042686532/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1042686532/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1042686532/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/1042686532/sys_int_pic32.o ../../../../../framework/system/int/src/sys_int_pic32.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/470001640/sys_random.o: ../../../../../framework/system/random/src/sys_random.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/470001640" 
	@${RM} ${OBJECTDIR}/_ext/470001640/sys_random.o.d 
	@${RM} ${OBJECTDIR}/_ext/470001640/sys_random.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/470001640/sys_random.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/470001640/sys_random.o.d" -o ${OBJECTDIR}/_ext/470001640/sys_random.o ../../../../../framework/system/random/src/sys_random.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/64182180/sys_reset.o: ../../../../../framework/system/reset/src/sys_reset.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/64182180" 
	@${RM} ${OBJECTDIR}/_ext/64182180/sys_reset.o.d 
	@${RM} ${OBJECTDIR}/_ext/64182180/sys_reset.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/64182180/sys_reset.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/64182180/sys_reset.o.d" -o ${OBJECTDIR}/_ext/64182180/sys_reset.o ../../../../../framework/system/reset/src/sys_reset.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2110151058/sys_tmr.o: ../../../../../framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2110151058" 
	@${RM} ${OBJECTDIR}/_ext/2110151058/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/2110151058/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2110151058/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2110151058/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/2110151058/sys_tmr.o ../../../../../framework/system/tmr/src/sys_tmr.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1886247299/sys_fs_wrapper.o: ../../../../../framework/tcpip/src/common/sys_fs_wrapper.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1886247299" 
	@${RM} ${OBJECTDIR}/_ext/1886247299/sys_fs_wrapper.o.d 
	@${RM} ${OBJECTDIR}/_ext/1886247299/sys_fs_wrapper.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1886247299/sys_fs_wrapper.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1886247299/sys_fs_wrapper.o.d" -o ${OBJECTDIR}/_ext/1886247299/sys_fs_wrapper.o ../../../../../framework/tcpip/src/common/sys_fs_wrapper.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1886247299/helpers.o: ../../../../../framework/tcpip/src/common/helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1886247299" 
	@${RM} ${OBJECTDIR}/_ext/1886247299/helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1886247299/helpers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1886247299/helpers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1886247299/helpers.o.d" -o ${OBJECTDIR}/_ext/1886247299/helpers.o ../../../../../framework/tcpip/src/common/helpers.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/tcp.o: ../../../../../framework/tcpip/src/tcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcp.o.d" -o ${OBJECTDIR}/_ext/1164207549/tcp.o ../../../../../framework/tcpip/src/tcp.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/udp.o: ../../../../../framework/tcpip/src/udp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/udp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/udp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/udp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/udp.o.d" -o ${OBJECTDIR}/_ext/1164207549/udp.o ../../../../../framework/tcpip/src/udp.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/tcpip_heap_alloc.o: ../../../../../framework/tcpip/src/tcpip_heap_alloc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_heap_alloc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_heap_alloc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcpip_heap_alloc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcpip_heap_alloc.o.d" -o ${OBJECTDIR}/_ext/1164207549/tcpip_heap_alloc.o ../../../../../framework/tcpip/src/tcpip_heap_alloc.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/tcpip_heap_internal.o: ../../../../../framework/tcpip/src/tcpip_heap_internal.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_heap_internal.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_heap_internal.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcpip_heap_internal.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcpip_heap_internal.o.d" -o ${OBJECTDIR}/_ext/1164207549/tcpip_heap_internal.o ../../../../../framework/tcpip/src/tcpip_heap_internal.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/arp.o: ../../../../../framework/tcpip/src/arp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/arp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/arp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/arp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/arp.o.d" -o ${OBJECTDIR}/_ext/1164207549/arp.o ../../../../../framework/tcpip/src/arp.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/dhcp.o: ../../../../../framework/tcpip/src/dhcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/dhcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/dhcp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/dhcp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/dhcp.o.d" -o ${OBJECTDIR}/_ext/1164207549/dhcp.o ../../../../../framework/tcpip/src/dhcp.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/http.o: ../../../../../framework/tcpip/src/http.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/http.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/http.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/http.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/http.o.d" -o ${OBJECTDIR}/_ext/1164207549/http.o ../../../../../framework/tcpip/src/http.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/icmp.o: ../../../../../framework/tcpip/src/icmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/icmp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/icmp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/icmp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/icmp.o.d" -o ${OBJECTDIR}/_ext/1164207549/icmp.o ../../../../../framework/tcpip/src/icmp.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/nbns.o: ../../../../../framework/tcpip/src/nbns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/nbns.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/nbns.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/nbns.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/nbns.o.d" -o ${OBJECTDIR}/_ext/1164207549/nbns.o ../../../../../framework/tcpip/src/nbns.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/smtp.o: ../../../../../framework/tcpip/src/smtp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/smtp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/smtp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/smtp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/smtp.o.d" -o ${OBJECTDIR}/_ext/1164207549/smtp.o ../../../../../framework/tcpip/src/smtp.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/zero_conf_helper.o: ../../../../../framework/tcpip/src/zero_conf_helper.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/zero_conf_helper.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/zero_conf_helper.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/zero_conf_helper.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/zero_conf_helper.o.d" -o ${OBJECTDIR}/_ext/1164207549/zero_conf_helper.o ../../../../../framework/tcpip/src/zero_conf_helper.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/zero_conf_link_local.o: ../../../../../framework/tcpip/src/zero_conf_link_local.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/zero_conf_link_local.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/zero_conf_link_local.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/zero_conf_link_local.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/zero_conf_link_local.o.d" -o ${OBJECTDIR}/_ext/1164207549/zero_conf_link_local.o ../../../../../framework/tcpip/src/zero_conf_link_local.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/zero_conf_multicast_dns.o: ../../../../../framework/tcpip/src/zero_conf_multicast_dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/zero_conf_multicast_dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/zero_conf_multicast_dns.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/zero_conf_multicast_dns.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/zero_conf_multicast_dns.o.d" -o ${OBJECTDIR}/_ext/1164207549/zero_conf_multicast_dns.o ../../../../../framework/tcpip/src/zero_conf_multicast_dns.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/iperf.o: ../../../../../framework/tcpip/src/iperf.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/iperf.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/iperf.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/iperf.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/iperf.o.d" -o ${OBJECTDIR}/_ext/1164207549/iperf.o ../../../../../framework/tcpip/src/iperf.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/tcpip_commands.o: ../../../../../framework/tcpip/src/tcpip_commands.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_commands.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_commands.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcpip_commands.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcpip_commands.o.d" -o ${OBJECTDIR}/_ext/1164207549/tcpip_commands.o ../../../../../framework/tcpip/src/tcpip_commands.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/hash_fnv.o: ../../../../../framework/tcpip/src/hash_fnv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/hash_fnv.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/hash_fnv.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/hash_fnv.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/hash_fnv.o.d" -o ${OBJECTDIR}/_ext/1164207549/hash_fnv.o ../../../../../framework/tcpip/src/hash_fnv.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/oahash.o: ../../../../../framework/tcpip/src/oahash.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/oahash.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/oahash.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/oahash.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/oahash.o.d" -o ${OBJECTDIR}/_ext/1164207549/oahash.o ../../../../../framework/tcpip/src/oahash.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/tcpip_helpers.o: ../../../../../framework/tcpip/src/tcpip_helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_helpers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcpip_helpers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcpip_helpers.o.d" -o ${OBJECTDIR}/_ext/1164207549/tcpip_helpers.o ../../../../../framework/tcpip/src/tcpip_helpers.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/tcpip_manager.o: ../../../../../framework/tcpip/src/tcpip_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_manager.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcpip_manager.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcpip_manager.o.d" -o ${OBJECTDIR}/_ext/1164207549/tcpip_manager.o ../../../../../framework/tcpip/src/tcpip_manager.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/tcpip_notify.o: ../../../../../framework/tcpip/src/tcpip_notify.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_notify.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_notify.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcpip_notify.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcpip_notify.o.d" -o ${OBJECTDIR}/_ext/1164207549/tcpip_notify.o ../../../../../framework/tcpip/src/tcpip_notify.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/tcpip_packet.o: ../../../../../framework/tcpip/src/tcpip_packet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_packet.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_packet.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcpip_packet.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcpip_packet.o.d" -o ${OBJECTDIR}/_ext/1164207549/tcpip_packet.o ../../../../../framework/tcpip/src/tcpip_packet.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/ipv4.o: ../../../../../framework/tcpip/src/ipv4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/ipv4.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/ipv4.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/ipv4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/ipv4.o.d" -o ${OBJECTDIR}/_ext/1164207549/ipv4.o ../../../../../framework/tcpip/src/ipv4.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/dns.o: ../../../../../framework/tcpip/src/dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/dns.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/dns.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/dns.o.d" -o ${OBJECTDIR}/_ext/1164207549/dns.o ../../../../../framework/tcpip/src/dns.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1107379230/heap_2.o: ../../../../../third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1107379230" 
	@${RM} ${OBJECTDIR}/_ext/1107379230/heap_2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1107379230/heap_2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1107379230/heap_2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1107379230/heap_2.o.d" -o ${OBJECTDIR}/_ext/1107379230/heap_2.o ../../../../../third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_2.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1282498059/port.o: ../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1282498059" 
	@${RM} ${OBJECTDIR}/_ext/1282498059/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1282498059/port.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1282498059/port.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1282498059/port.o.d" -o ${OBJECTDIR}/_ext/1282498059/port.o ../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/373060831/croutine.o: ../../../../../third_party/rtos/FreeRTOS/Source/croutine.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/373060831" 
	@${RM} ${OBJECTDIR}/_ext/373060831/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/373060831/croutine.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/373060831/croutine.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/373060831/croutine.o.d" -o ${OBJECTDIR}/_ext/373060831/croutine.o ../../../../../third_party/rtos/FreeRTOS/Source/croutine.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/373060831/list.o: ../../../../../third_party/rtos/FreeRTOS/Source/list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/373060831" 
	@${RM} ${OBJECTDIR}/_ext/373060831/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/373060831/list.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/373060831/list.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/373060831/list.o.d" -o ${OBJECTDIR}/_ext/373060831/list.o ../../../../../third_party/rtos/FreeRTOS/Source/list.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/373060831/queue.o: ../../../../../third_party/rtos/FreeRTOS/Source/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/373060831" 
	@${RM} ${OBJECTDIR}/_ext/373060831/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/373060831/queue.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/373060831/queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/373060831/queue.o.d" -o ${OBJECTDIR}/_ext/373060831/queue.o ../../../../../third_party/rtos/FreeRTOS/Source/queue.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/373060831/tasks.o: ../../../../../third_party/rtos/FreeRTOS/Source/tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/373060831" 
	@${RM} ${OBJECTDIR}/_ext/373060831/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/373060831/tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/373060831/tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/373060831/tasks.o.d" -o ${OBJECTDIR}/_ext/373060831/tasks.o ../../../../../third_party/rtos/FreeRTOS/Source/tasks.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/373060831/timers.o: ../../../../../third_party/rtos/FreeRTOS/Source/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/373060831" 
	@${RM} ${OBJECTDIR}/_ext/373060831/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/373060831/timers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/373060831/timers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/373060831/timers.o.d" -o ${OBJECTDIR}/_ext/373060831/timers.o ../../../../../third_party/rtos/FreeRTOS/Source/timers.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/373060831/event_groups.o: ../../../../../third_party/rtos/FreeRTOS/Source/event_groups.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/373060831" 
	@${RM} ${OBJECTDIR}/_ext/373060831/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/373060831/event_groups.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/373060831/event_groups.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/373060831/event_groups.o.d" -o ${OBJECTDIR}/_ext/373060831/event_groups.o ../../../../../third_party/rtos/FreeRTOS/Source/event_groups.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
else
${OBJECTDIR}/_ext/975901697/bsp.o: ../src/system_config/pic32mx795_pim__e16__freertos/bsp/bsp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/975901697" 
	@${RM} ${OBJECTDIR}/_ext/975901697/bsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/975901697/bsp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/975901697/bsp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/975901697/bsp.o.d" -o ${OBJECTDIR}/_ext/975901697/bsp.o ../src/system_config/pic32mx795_pim__e16__freertos/bsp/bsp.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1593769052/drv_spi_tasks.o: ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1593769052" 
	@${RM} ${OBJECTDIR}/_ext/1593769052/drv_spi_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1593769052/drv_spi_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1593769052/drv_spi_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1593769052/drv_spi_tasks.o.d" -o ${OBJECTDIR}/_ext/1593769052/drv_spi_tasks.o ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_tasks.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1593769052/drv_spi_api.o: ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_api.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1593769052" 
	@${RM} ${OBJECTDIR}/_ext/1593769052/drv_spi_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/1593769052/drv_spi_api.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1593769052/drv_spi_api.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1593769052/drv_spi_api.o.d" -o ${OBJECTDIR}/_ext/1593769052/drv_spi_api.o ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_api.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1593769052/drv_spi_master_ebm_tasks.o: ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1593769052" 
	@${RM} ${OBJECTDIR}/_ext/1593769052/drv_spi_master_ebm_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1593769052/drv_spi_master_ebm_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1593769052/drv_spi_master_ebm_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1593769052/drv_spi_master_ebm_tasks.o.d" -o ${OBJECTDIR}/_ext/1593769052/drv_spi_master_ebm_tasks.o ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_master_ebm_tasks.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1593769052/drv_spi_master_dma_tasks.o: ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_master_dma_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1593769052" 
	@${RM} ${OBJECTDIR}/_ext/1593769052/drv_spi_master_dma_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1593769052/drv_spi_master_dma_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1593769052/drv_spi_master_dma_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1593769052/drv_spi_master_dma_tasks.o.d" -o ${OBJECTDIR}/_ext/1593769052/drv_spi_master_dma_tasks.o ../src/system_config/pic32mx795_pim__e16__freertos/framework/driver/spi/dynamic/drv_spi_master_dma_tasks.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/448233041/net_pres_enc_glue.o: ../src/system_config/pic32mx795_pim__e16__freertos/framework/net/pres/net_pres_enc_glue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/448233041" 
	@${RM} ${OBJECTDIR}/_ext/448233041/net_pres_enc_glue.o.d 
	@${RM} ${OBJECTDIR}/_ext/448233041/net_pres_enc_glue.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/448233041/net_pres_enc_glue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/448233041/net_pres_enc_glue.o.d" -o ${OBJECTDIR}/_ext/448233041/net_pres_enc_glue.o ../src/system_config/pic32mx795_pim__e16__freertos/framework/net/pres/net_pres_enc_glue.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1256380230/sys_clk_static.o: ../src/system_config/pic32mx795_pim__e16__freertos/framework/system/clk/src/sys_clk_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1256380230" 
	@${RM} ${OBJECTDIR}/_ext/1256380230/sys_clk_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1256380230/sys_clk_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1256380230/sys_clk_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1256380230/sys_clk_static.o.d" -o ${OBJECTDIR}/_ext/1256380230/sys_clk_static.o ../src/system_config/pic32mx795_pim__e16__freertos/framework/system/clk/src/sys_clk_static.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/996991370/sys_ports_static.o: ../src/system_config/pic32mx795_pim__e16__freertos/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/996991370" 
	@${RM} ${OBJECTDIR}/_ext/996991370/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/996991370/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/996991370/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/996991370/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/996991370/sys_ports_static.o ../src/system_config/pic32mx795_pim__e16__freertos/framework/system/ports/src/sys_ports_static.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1096222929/system_init.o: ../src/system_config/pic32mx795_pim__e16__freertos/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1096222929" 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1096222929/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1096222929/system_init.o.d" -o ${OBJECTDIR}/_ext/1096222929/system_init.o ../src/system_config/pic32mx795_pim__e16__freertos/system_init.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1096222929/system_interrupt.o: ../src/system_config/pic32mx795_pim__e16__freertos/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1096222929" 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1096222929/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1096222929/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1096222929/system_interrupt.o ../src/system_config/pic32mx795_pim__e16__freertos/system_interrupt.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1096222929/system_exceptions.o: ../src/system_config/pic32mx795_pim__e16__freertos/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1096222929" 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1096222929/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1096222929/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1096222929/system_exceptions.o ../src/system_config/pic32mx795_pim__e16__freertos/system_exceptions.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1096222929/system_tasks.o: ../src/system_config/pic32mx795_pim__e16__freertos/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1096222929" 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1096222929/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1096222929/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1096222929/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1096222929/system_tasks.o ../src/system_config/pic32mx795_pim__e16__freertos/system_tasks.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1096222929/rtos_hooks.o: ../src/system_config/pic32mx795_pim__e16__freertos/rtos_hooks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1096222929" 
	@${RM} ${OBJECTDIR}/_ext/1096222929/rtos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1096222929/rtos_hooks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1096222929/rtos_hooks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1096222929/rtos_hooks.o.d" -o ${OBJECTDIR}/_ext/1096222929/rtos_hooks.o ../src/system_config/pic32mx795_pim__e16__freertos/rtos_hooks.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/http_print.o: ../src/http_print.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/http_print.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/http_print.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/http_print.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/http_print.o.d" -o ${OBJECTDIR}/_ext/1360937237/http_print.o ../src/http_print.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/custom_http_app.o: ../src/custom_http_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/custom_http_app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/custom_http_app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/custom_http_app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/custom_http_app.o.d" -o ${OBJECTDIR}/_ext/1360937237/custom_http_app.o ../src/custom_http_app.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/mpfs_img2.o: ../src/mpfs_img2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mpfs_img2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mpfs_img2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/mpfs_img2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/mpfs_img2.o.d" -o ${OBJECTDIR}/_ext/1360937237/mpfs_img2.o ../src/mpfs_img2.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/app_wifi.o: ../src/app_wifi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app_wifi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app_wifi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/app_wifi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1360937237/app_wifi.o.d" -o ${OBJECTDIR}/_ext/1360937237/app_wifi.o ../src/app_wifi.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/adler32.o: ../../../../../framework/crypto/src/zlib-1.2.7/adler32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/adler32.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/adler32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/adler32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/adler32.o.d" -o ${OBJECTDIR}/_ext/2046866571/adler32.o ../../../../../framework/crypto/src/zlib-1.2.7/adler32.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/compress.o: ../../../../../framework/crypto/src/zlib-1.2.7/compress.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/compress.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/compress.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/compress.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/compress.o.d" -o ${OBJECTDIR}/_ext/2046866571/compress.o ../../../../../framework/crypto/src/zlib-1.2.7/compress.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/crc32.o: ../../../../../framework/crypto/src/zlib-1.2.7/crc32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/crc32.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/crc32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/crc32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/crc32.o.d" -o ${OBJECTDIR}/_ext/2046866571/crc32.o ../../../../../framework/crypto/src/zlib-1.2.7/crc32.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/deflate.o: ../../../../../framework/crypto/src/zlib-1.2.7/deflate.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/deflate.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/deflate.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/deflate.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/deflate.o.d" -o ${OBJECTDIR}/_ext/2046866571/deflate.o ../../../../../framework/crypto/src/zlib-1.2.7/deflate.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/infback.o: ../../../../../framework/crypto/src/zlib-1.2.7/infback.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/infback.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/infback.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/infback.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/infback.o.d" -o ${OBJECTDIR}/_ext/2046866571/infback.o ../../../../../framework/crypto/src/zlib-1.2.7/infback.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/inffast.o: ../../../../../framework/crypto/src/zlib-1.2.7/inffast.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/inffast.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/inffast.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/inffast.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/inffast.o.d" -o ${OBJECTDIR}/_ext/2046866571/inffast.o ../../../../../framework/crypto/src/zlib-1.2.7/inffast.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/inflate.o: ../../../../../framework/crypto/src/zlib-1.2.7/inflate.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/inflate.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/inflate.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/inflate.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/inflate.o.d" -o ${OBJECTDIR}/_ext/2046866571/inflate.o ../../../../../framework/crypto/src/zlib-1.2.7/inflate.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/inftrees.o: ../../../../../framework/crypto/src/zlib-1.2.7/inftrees.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/inftrees.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/inftrees.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/inftrees.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/inftrees.o.d" -o ${OBJECTDIR}/_ext/2046866571/inftrees.o ../../../../../framework/crypto/src/zlib-1.2.7/inftrees.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/trees.o: ../../../../../framework/crypto/src/zlib-1.2.7/trees.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/trees.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/trees.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/trees.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/trees.o.d" -o ${OBJECTDIR}/_ext/2046866571/trees.o ../../../../../framework/crypto/src/zlib-1.2.7/trees.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/uncompr.o: ../../../../../framework/crypto/src/zlib-1.2.7/uncompr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/uncompr.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/uncompr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/uncompr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/uncompr.o.d" -o ${OBJECTDIR}/_ext/2046866571/uncompr.o ../../../../../framework/crypto/src/zlib-1.2.7/uncompr.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2046866571/zutil.o: ../../../../../framework/crypto/src/zlib-1.2.7/zutil.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2046866571" 
	@${RM} ${OBJECTDIR}/_ext/2046866571/zutil.o.d 
	@${RM} ${OBJECTDIR}/_ext/2046866571/zutil.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2046866571/zutil.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2046866571/zutil.o.d" -o ${OBJECTDIR}/_ext/2046866571/zutil.o ../../../../../framework/crypto/src/zlib-1.2.7/zutil.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/ecc.o: ../../../../../framework/crypto/src/ecc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/ecc.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/ecc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/ecc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/ecc.o.d" -o ${OBJECTDIR}/_ext/29024758/ecc.o ../../../../../framework/crypto/src/ecc.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/arc4.o: ../../../../../framework/crypto/src/arc4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/arc4.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/arc4.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/arc4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/arc4.o.d" -o ${OBJECTDIR}/_ext/29024758/arc4.o ../../../../../framework/crypto/src/arc4.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/pwdbased.o: ../../../../../framework/crypto/src/pwdbased.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/pwdbased.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/pwdbased.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/pwdbased.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/pwdbased.o.d" -o ${OBJECTDIR}/_ext/29024758/pwdbased.o ../../../../../framework/crypto/src/pwdbased.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/tfm.o: ../../../../../framework/crypto/src/tfm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/tfm.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/tfm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/tfm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/tfm.o.d" -o ${OBJECTDIR}/_ext/29024758/tfm.o ../../../../../framework/crypto/src/tfm.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/asn.o: ../../../../../framework/crypto/src/asn.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/asn.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/asn.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/asn.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/asn.o.d" -o ${OBJECTDIR}/_ext/29024758/asn.o ../../../../../framework/crypto/src/asn.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/des3.o: ../../../../../framework/crypto/src/des3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/des3.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/des3.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/des3.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/des3.o.d" -o ${OBJECTDIR}/_ext/29024758/des3.o ../../../../../framework/crypto/src/des3.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/rsa.o: ../../../../../framework/crypto/src/rsa.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/rsa.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/rsa.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/rsa.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/rsa.o.d" -o ${OBJECTDIR}/_ext/29024758/rsa.o ../../../../../framework/crypto/src/rsa.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/aes.o: ../../../../../framework/crypto/src/aes.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/aes.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/aes.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/aes.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/aes.o.d" -o ${OBJECTDIR}/_ext/29024758/aes.o ../../../../../framework/crypto/src/aes.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/md5.o: ../../../../../framework/crypto/src/md5.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/md5.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/md5.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/md5.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/md5.o.d" -o ${OBJECTDIR}/_ext/29024758/md5.o ../../../../../framework/crypto/src/md5.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/sha.o: ../../../../../framework/crypto/src/sha.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/sha.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/sha.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/sha.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/sha.o.d" -o ${OBJECTDIR}/_ext/29024758/sha.o ../../../../../framework/crypto/src/sha.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/sha256.o: ../../../../../framework/crypto/src/sha256.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/sha256.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/sha256.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/sha256.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/sha256.o.d" -o ${OBJECTDIR}/_ext/29024758/sha256.o ../../../../../framework/crypto/src/sha256.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/sha512.o: ../../../../../framework/crypto/src/sha512.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/sha512.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/sha512.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/sha512.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/sha512.o.d" -o ${OBJECTDIR}/_ext/29024758/sha512.o ../../../../../framework/crypto/src/sha512.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/hmac.o: ../../../../../framework/crypto/src/hmac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/hmac.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/hmac.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/hmac.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/hmac.o.d" -o ${OBJECTDIR}/_ext/29024758/hmac.o ../../../../../framework/crypto/src/hmac.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/hash.o: ../../../../../framework/crypto/src/hash.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/hash.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/hash.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/hash.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/hash.o.d" -o ${OBJECTDIR}/_ext/29024758/hash.o ../../../../../framework/crypto/src/hash.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/compress.o: ../../../../../framework/crypto/src/compress.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/compress.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/compress.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/compress.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/compress.o.d" -o ${OBJECTDIR}/_ext/29024758/compress.o ../../../../../framework/crypto/src/compress.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/random.o: ../../../../../framework/crypto/src/random.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/random.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/random.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/random.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/random.o.d" -o ${OBJECTDIR}/_ext/29024758/random.o ../../../../../framework/crypto/src/random.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/crypto.o: ../../../../../framework/crypto/src/crypto.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/crypto.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/crypto.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/crypto.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/crypto.o.d" -o ${OBJECTDIR}/_ext/29024758/crypto.o ../../../../../framework/crypto/src/crypto.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/asm.o: ../../../../../framework/crypto/src/asm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/asm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/asm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/asm.o.d" -o ${OBJECTDIR}/_ext/29024758/asm.o ../../../../../framework/crypto/src/asm.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/coding.o: ../../../../../framework/crypto/src/coding.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/coding.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/coding.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/coding.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/coding.o.d" -o ${OBJECTDIR}/_ext/29024758/coding.o ../../../../../framework/crypto/src/coding.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/error.o: ../../../../../framework/crypto/src/error.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/error.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/error.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/error.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/error.o.d" -o ${OBJECTDIR}/_ext/29024758/error.o ../../../../../framework/crypto/src/error.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/integer.o: ../../../../../framework/crypto/src/integer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/integer.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/integer.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/integer.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/integer.o.d" -o ${OBJECTDIR}/_ext/29024758/integer.o ../../../../../framework/crypto/src/integer.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/logging.o: ../../../../../framework/crypto/src/logging.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/logging.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/logging.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/logging.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/logging.o.d" -o ${OBJECTDIR}/_ext/29024758/logging.o ../../../../../framework/crypto/src/logging.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/memory.o: ../../../../../framework/crypto/src/memory.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/memory.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/memory.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/memory.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/memory.o.d" -o ${OBJECTDIR}/_ext/29024758/memory.o ../../../../../framework/crypto/src/memory.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/misc.o: ../../../../../framework/crypto/src/misc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/misc.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/misc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/misc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/misc.o.d" -o ${OBJECTDIR}/_ext/29024758/misc.o ../../../../../framework/crypto/src/misc.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/29024758/port.o: ../../../../../framework/crypto/src/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/29024758" 
	@${RM} ${OBJECTDIR}/_ext/29024758/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/29024758/port.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/29024758/port.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/29024758/port.o.d" -o ${OBJECTDIR}/_ext/29024758/port.o ../../../../../framework/crypto/src/port.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/184581597/drv_nvm.o: ../../../../../framework/driver/nvm/src/dynamic/drv_nvm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/184581597" 
	@${RM} ${OBJECTDIR}/_ext/184581597/drv_nvm.o.d 
	@${RM} ${OBJECTDIR}/_ext/184581597/drv_nvm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/184581597/drv_nvm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/184581597/drv_nvm.o.d" -o ${OBJECTDIR}/_ext/184581597/drv_nvm.o ../../../../../framework/driver/nvm/src/dynamic/drv_nvm.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/184581597/drv_nvm_erasewrite.o: ../../../../../framework/driver/nvm/src/dynamic/drv_nvm_erasewrite.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/184581597" 
	@${RM} ${OBJECTDIR}/_ext/184581597/drv_nvm_erasewrite.o.d 
	@${RM} ${OBJECTDIR}/_ext/184581597/drv_nvm_erasewrite.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/184581597/drv_nvm_erasewrite.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/184581597/drv_nvm_erasewrite.o.d" -o ${OBJECTDIR}/_ext/184581597/drv_nvm_erasewrite.o ../../../../../framework/driver/nvm/src/dynamic/drv_nvm_erasewrite.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1324760662/drv_spi.o: ../../../../../framework/driver/spi/src/dynamic/drv_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1324760662" 
	@${RM} ${OBJECTDIR}/_ext/1324760662/drv_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1324760662/drv_spi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1324760662/drv_spi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1324760662/drv_spi.o.d" -o ${OBJECTDIR}/_ext/1324760662/drv_spi.o ../../../../../framework/driver/spi/src/dynamic/drv_spi.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1385053818/drv_spi_sys_queue_fifo.o: ../../../../../framework/driver/spi/src/drv_spi_sys_queue_fifo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1385053818" 
	@${RM} ${OBJECTDIR}/_ext/1385053818/drv_spi_sys_queue_fifo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1385053818/drv_spi_sys_queue_fifo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1385053818/drv_spi_sys_queue_fifo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1385053818/drv_spi_sys_queue_fifo.o.d" -o ${OBJECTDIR}/_ext/1385053818/drv_spi_sys_queue_fifo.o ../../../../../framework/driver/spi/src/drv_spi_sys_queue_fifo.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/941160041/drv_tmr.o: ../../../../../framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/941160041" 
	@${RM} ${OBJECTDIR}/_ext/941160041/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/941160041/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/941160041/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/941160041/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/941160041/drv_tmr.o ../../../../../framework/driver/tmr/src/dynamic/drv_tmr.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/821589181/drv_usart.o: ../../../../../framework/driver/usart/src/dynamic/drv_usart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/821589181" 
	@${RM} ${OBJECTDIR}/_ext/821589181/drv_usart.o.d 
	@${RM} ${OBJECTDIR}/_ext/821589181/drv_usart.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/821589181/drv_usart.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/821589181/drv_usart.o.d" -o ${OBJECTDIR}/_ext/821589181/drv_usart.o ../../../../../framework/driver/usart/src/dynamic/drv_usart.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/821589181/drv_usart_buffer_queue.o: ../../../../../framework/driver/usart/src/dynamic/drv_usart_buffer_queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/821589181" 
	@${RM} ${OBJECTDIR}/_ext/821589181/drv_usart_buffer_queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/821589181/drv_usart_buffer_queue.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/821589181/drv_usart_buffer_queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/821589181/drv_usart_buffer_queue.o.d" -o ${OBJECTDIR}/_ext/821589181/drv_usart_buffer_queue.o ../../../../../framework/driver/usart/src/dynamic/drv_usart_buffer_queue.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/821589181/drv_usart_read_write.o: ../../../../../framework/driver/usart/src/dynamic/drv_usart_read_write.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/821589181" 
	@${RM} ${OBJECTDIR}/_ext/821589181/drv_usart_read_write.o.d 
	@${RM} ${OBJECTDIR}/_ext/821589181/drv_usart_read_write.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/821589181/drv_usart_read_write.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/821589181/drv_usart_read_write.o.d" -o ${OBJECTDIR}/_ext/821589181/drv_usart_read_write.o ../../../../../framework/driver/usart/src/dynamic/drv_usart_read_write.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_com.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_com.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_com.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_com.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_com.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_com.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_com.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_com.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_commands.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_commands.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_commands.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_commands.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_commands.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_commands.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_commands.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_commands.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_config_data.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_config_data.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_config_data.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_config_data.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_config_data.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_config_data.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_config_data.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_config_data.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_algorithm.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_algorithm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_algorithm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_algorithm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_algorithm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_algorithm.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_algorithm.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_algorithm.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_manager.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_manager.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_manager.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_manager.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_manager.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_manager.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_profile.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_profile.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_profile.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_profile.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_profile.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_profile.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_connection_profile.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_connection_profile.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_context_loader.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_context_loader.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_context_loader.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_context_loader.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_context_loader.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_context_loader.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_context_loader.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_context_loader.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_debug_output.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_debug_output.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_debug_output.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_debug_output.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_debug_output.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_debug_output.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_debug_output.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_debug_output.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_eint.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_eint.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_eint.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_eint.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_eint.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_eint.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_eint.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_eint.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_events.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_events.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_events.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_events.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_events.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_events.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_events.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_events.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_event_handler.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_event_handler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_event_handler.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_event_handler.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_event_handler.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_event_handler.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_event_handler.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_event_handler.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_init.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_init.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_init.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_init.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_iwpriv.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_iwpriv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_iwpriv.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_iwpriv.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_iwpriv.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_iwpriv.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_iwpriv.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_iwpriv.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_mac.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_mac.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_mac.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_mac.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_mac.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_mac.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_mac.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_main.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_main.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_main.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_main.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_mgmt_msg.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_mgmt_msg.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_mgmt_msg.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_mgmt_msg.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_mgmt_msg.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_mgmt_msg.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_mgmt_msg.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_mgmt_msg.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_param_msg.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_param_msg.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_param_msg.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_param_msg.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_param_msg.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_param_msg.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_param_msg.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_param_msg.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_pbkdf2.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_pbkdf2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_pbkdf2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_pbkdf2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_pbkdf2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_pbkdf2.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_pbkdf2.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_pbkdf2.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_power_save.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_power_save.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_power_save.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_power_save.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_power_save.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_power_save.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_power_save.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_power_save.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_raw.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_raw.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_raw.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_raw.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_raw.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_raw.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_raw.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_raw.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_scan.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_scan.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_scan.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_scan.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_scan.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_scan_helper.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_scan_helper.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan_helper.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan_helper.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_scan_helper.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_scan_helper.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_scan_helper.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_scan_helper.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_softap_client_cache.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_softap_client_cache.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_softap_client_cache.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_softap_client_cache.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_softap_client_cache.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_softap_client_cache.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_softap_client_cache.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_softap_client_cache.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_spi.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_spi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_spi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_spi.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_spi.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_spi.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_tx_power.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_tx_power.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_tx_power.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_tx_power.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_tx_power.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_tx_power.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_tx_power.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_tx_power.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1023149444/drv_wifi_rtos_wrapper.o: ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_rtos_wrapper.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1023149444" 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_rtos_wrapper.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023149444/drv_wifi_rtos_wrapper.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1023149444/drv_wifi_rtos_wrapper.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1023149444/drv_wifi_rtos_wrapper.o.d" -o ${OBJECTDIR}/_ext/1023149444/drv_wifi_rtos_wrapper.o ../../../../../framework/driver/wifi/mrf24w/src/drv_wifi_rtos_wrapper.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/871719959/net_pres.o: ../../../../../framework/net/pres/src/net_pres.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/871719959" 
	@${RM} ${OBJECTDIR}/_ext/871719959/net_pres.o.d 
	@${RM} ${OBJECTDIR}/_ext/871719959/net_pres.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/871719959/net_pres.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/871719959/net_pres.o.d" -o ${OBJECTDIR}/_ext/871719959/net_pres.o ../../../../../framework/net/pres/src/net_pres.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/308321468/osal_freertos.o: ../../../../../framework/osal/src/osal_freertos.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/308321468" 
	@${RM} ${OBJECTDIR}/_ext/308321468/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/308321468/osal_freertos.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/308321468/osal_freertos.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/308321468/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/308321468/osal_freertos.o ../../../../../framework/osal/src/osal_freertos.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1891376032/sys_command.o: ../../../../../framework/system/command/src/sys_command.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1891376032" 
	@${RM} ${OBJECTDIR}/_ext/1891376032/sys_command.o.d 
	@${RM} ${OBJECTDIR}/_ext/1891376032/sys_command.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1891376032/sys_command.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1891376032/sys_command.o.d" -o ${OBJECTDIR}/_ext/1891376032/sys_command.o ../../../../../framework/system/command/src/sys_command.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1434663852/sys_console.o: ../../../../../framework/system/console/src/sys_console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1434663852" 
	@${RM} ${OBJECTDIR}/_ext/1434663852/sys_console.o.d 
	@${RM} ${OBJECTDIR}/_ext/1434663852/sys_console.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1434663852/sys_console.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1434663852/sys_console.o.d" -o ${OBJECTDIR}/_ext/1434663852/sys_console.o ../../../../../framework/system/console/src/sys_console.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1434663852/sys_console_uart.o: ../../../../../framework/system/console/src/sys_console_uart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1434663852" 
	@${RM} ${OBJECTDIR}/_ext/1434663852/sys_console_uart.o.d 
	@${RM} ${OBJECTDIR}/_ext/1434663852/sys_console_uart.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1434663852/sys_console_uart.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1434663852/sys_console_uart.o.d" -o ${OBJECTDIR}/_ext/1434663852/sys_console_uart.o ../../../../../framework/system/console/src/sys_console_uart.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2137108136/sys_debug.o: ../../../../../framework/system/debug/src/sys_debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2137108136" 
	@${RM} ${OBJECTDIR}/_ext/2137108136/sys_debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/2137108136/sys_debug.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2137108136/sys_debug.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2137108136/sys_debug.o.d" -o ${OBJECTDIR}/_ext/2137108136/sys_debug.o ../../../../../framework/system/debug/src/sys_debug.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/482662494/sys_devcon.o: ../../../../../framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/482662494" 
	@${RM} ${OBJECTDIR}/_ext/482662494/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/482662494/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/482662494/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/482662494/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/482662494/sys_devcon.o ../../../../../framework/system/devcon/src/sys_devcon.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/482662494/sys_devcon_pic32mx.o: ../../../../../framework/system/devcon/src/sys_devcon_pic32mx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/482662494" 
	@${RM} ${OBJECTDIR}/_ext/482662494/sys_devcon_pic32mx.o.d 
	@${RM} ${OBJECTDIR}/_ext/482662494/sys_devcon_pic32mx.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/482662494/sys_devcon_pic32mx.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/482662494/sys_devcon_pic32mx.o.d" -o ${OBJECTDIR}/_ext/482662494/sys_devcon_pic32mx.o ../../../../../framework/system/devcon/src/sys_devcon_pic32mx.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/853959373/sys_dma.o: ../../../../../framework/system/dma/src/sys_dma.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/853959373" 
	@${RM} ${OBJECTDIR}/_ext/853959373/sys_dma.o.d 
	@${RM} ${OBJECTDIR}/_ext/853959373/sys_dma.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/853959373/sys_dma.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/853959373/sys_dma.o.d" -o ${OBJECTDIR}/_ext/853959373/sys_dma.o ../../../../../framework/system/dma/src/sys_dma.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1750042194/sys_fs.o: ../../../../../framework/system/fs/src/dynamic/sys_fs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1750042194" 
	@${RM} ${OBJECTDIR}/_ext/1750042194/sys_fs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1750042194/sys_fs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1750042194/sys_fs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1750042194/sys_fs.o.d" -o ${OBJECTDIR}/_ext/1750042194/sys_fs.o ../../../../../framework/system/fs/src/dynamic/sys_fs.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1750042194/sys_fs_media_manager.o: ../../../../../framework/system/fs/src/dynamic/sys_fs_media_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1750042194" 
	@${RM} ${OBJECTDIR}/_ext/1750042194/sys_fs_media_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/1750042194/sys_fs_media_manager.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1750042194/sys_fs_media_manager.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1750042194/sys_fs_media_manager.o.d" -o ${OBJECTDIR}/_ext/1750042194/sys_fs_media_manager.o ../../../../../framework/system/fs/src/dynamic/sys_fs_media_manager.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/614935175/mpfs.o: ../../../../../framework/system/fs/mpfs/src/mpfs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/614935175" 
	@${RM} ${OBJECTDIR}/_ext/614935175/mpfs.o.d 
	@${RM} ${OBJECTDIR}/_ext/614935175/mpfs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/614935175/mpfs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/614935175/mpfs.o.d" -o ${OBJECTDIR}/_ext/614935175/mpfs.o ../../../../../framework/system/fs/mpfs/src/mpfs.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1042686532/sys_int_pic32.o: ../../../../../framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1042686532" 
	@${RM} ${OBJECTDIR}/_ext/1042686532/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1042686532/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1042686532/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1042686532/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/1042686532/sys_int_pic32.o ../../../../../framework/system/int/src/sys_int_pic32.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/470001640/sys_random.o: ../../../../../framework/system/random/src/sys_random.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/470001640" 
	@${RM} ${OBJECTDIR}/_ext/470001640/sys_random.o.d 
	@${RM} ${OBJECTDIR}/_ext/470001640/sys_random.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/470001640/sys_random.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/470001640/sys_random.o.d" -o ${OBJECTDIR}/_ext/470001640/sys_random.o ../../../../../framework/system/random/src/sys_random.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/64182180/sys_reset.o: ../../../../../framework/system/reset/src/sys_reset.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/64182180" 
	@${RM} ${OBJECTDIR}/_ext/64182180/sys_reset.o.d 
	@${RM} ${OBJECTDIR}/_ext/64182180/sys_reset.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/64182180/sys_reset.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/64182180/sys_reset.o.d" -o ${OBJECTDIR}/_ext/64182180/sys_reset.o ../../../../../framework/system/reset/src/sys_reset.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2110151058/sys_tmr.o: ../../../../../framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2110151058" 
	@${RM} ${OBJECTDIR}/_ext/2110151058/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/2110151058/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2110151058/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/2110151058/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/2110151058/sys_tmr.o ../../../../../framework/system/tmr/src/sys_tmr.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1886247299/sys_fs_wrapper.o: ../../../../../framework/tcpip/src/common/sys_fs_wrapper.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1886247299" 
	@${RM} ${OBJECTDIR}/_ext/1886247299/sys_fs_wrapper.o.d 
	@${RM} ${OBJECTDIR}/_ext/1886247299/sys_fs_wrapper.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1886247299/sys_fs_wrapper.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1886247299/sys_fs_wrapper.o.d" -o ${OBJECTDIR}/_ext/1886247299/sys_fs_wrapper.o ../../../../../framework/tcpip/src/common/sys_fs_wrapper.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1886247299/helpers.o: ../../../../../framework/tcpip/src/common/helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1886247299" 
	@${RM} ${OBJECTDIR}/_ext/1886247299/helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1886247299/helpers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1886247299/helpers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1886247299/helpers.o.d" -o ${OBJECTDIR}/_ext/1886247299/helpers.o ../../../../../framework/tcpip/src/common/helpers.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/tcp.o: ../../../../../framework/tcpip/src/tcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcp.o.d" -o ${OBJECTDIR}/_ext/1164207549/tcp.o ../../../../../framework/tcpip/src/tcp.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/udp.o: ../../../../../framework/tcpip/src/udp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/udp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/udp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/udp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/udp.o.d" -o ${OBJECTDIR}/_ext/1164207549/udp.o ../../../../../framework/tcpip/src/udp.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/tcpip_heap_alloc.o: ../../../../../framework/tcpip/src/tcpip_heap_alloc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_heap_alloc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_heap_alloc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcpip_heap_alloc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcpip_heap_alloc.o.d" -o ${OBJECTDIR}/_ext/1164207549/tcpip_heap_alloc.o ../../../../../framework/tcpip/src/tcpip_heap_alloc.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/tcpip_heap_internal.o: ../../../../../framework/tcpip/src/tcpip_heap_internal.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_heap_internal.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_heap_internal.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcpip_heap_internal.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcpip_heap_internal.o.d" -o ${OBJECTDIR}/_ext/1164207549/tcpip_heap_internal.o ../../../../../framework/tcpip/src/tcpip_heap_internal.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/arp.o: ../../../../../framework/tcpip/src/arp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/arp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/arp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/arp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/arp.o.d" -o ${OBJECTDIR}/_ext/1164207549/arp.o ../../../../../framework/tcpip/src/arp.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/dhcp.o: ../../../../../framework/tcpip/src/dhcp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/dhcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/dhcp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/dhcp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/dhcp.o.d" -o ${OBJECTDIR}/_ext/1164207549/dhcp.o ../../../../../framework/tcpip/src/dhcp.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/http.o: ../../../../../framework/tcpip/src/http.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/http.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/http.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/http.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/http.o.d" -o ${OBJECTDIR}/_ext/1164207549/http.o ../../../../../framework/tcpip/src/http.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/icmp.o: ../../../../../framework/tcpip/src/icmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/icmp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/icmp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/icmp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/icmp.o.d" -o ${OBJECTDIR}/_ext/1164207549/icmp.o ../../../../../framework/tcpip/src/icmp.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/nbns.o: ../../../../../framework/tcpip/src/nbns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/nbns.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/nbns.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/nbns.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/nbns.o.d" -o ${OBJECTDIR}/_ext/1164207549/nbns.o ../../../../../framework/tcpip/src/nbns.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/smtp.o: ../../../../../framework/tcpip/src/smtp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/smtp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/smtp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/smtp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/smtp.o.d" -o ${OBJECTDIR}/_ext/1164207549/smtp.o ../../../../../framework/tcpip/src/smtp.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/zero_conf_helper.o: ../../../../../framework/tcpip/src/zero_conf_helper.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/zero_conf_helper.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/zero_conf_helper.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/zero_conf_helper.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/zero_conf_helper.o.d" -o ${OBJECTDIR}/_ext/1164207549/zero_conf_helper.o ../../../../../framework/tcpip/src/zero_conf_helper.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/zero_conf_link_local.o: ../../../../../framework/tcpip/src/zero_conf_link_local.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/zero_conf_link_local.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/zero_conf_link_local.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/zero_conf_link_local.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/zero_conf_link_local.o.d" -o ${OBJECTDIR}/_ext/1164207549/zero_conf_link_local.o ../../../../../framework/tcpip/src/zero_conf_link_local.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/zero_conf_multicast_dns.o: ../../../../../framework/tcpip/src/zero_conf_multicast_dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/zero_conf_multicast_dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/zero_conf_multicast_dns.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/zero_conf_multicast_dns.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/zero_conf_multicast_dns.o.d" -o ${OBJECTDIR}/_ext/1164207549/zero_conf_multicast_dns.o ../../../../../framework/tcpip/src/zero_conf_multicast_dns.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/iperf.o: ../../../../../framework/tcpip/src/iperf.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/iperf.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/iperf.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/iperf.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/iperf.o.d" -o ${OBJECTDIR}/_ext/1164207549/iperf.o ../../../../../framework/tcpip/src/iperf.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/tcpip_commands.o: ../../../../../framework/tcpip/src/tcpip_commands.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_commands.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_commands.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcpip_commands.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcpip_commands.o.d" -o ${OBJECTDIR}/_ext/1164207549/tcpip_commands.o ../../../../../framework/tcpip/src/tcpip_commands.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/hash_fnv.o: ../../../../../framework/tcpip/src/hash_fnv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/hash_fnv.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/hash_fnv.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/hash_fnv.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/hash_fnv.o.d" -o ${OBJECTDIR}/_ext/1164207549/hash_fnv.o ../../../../../framework/tcpip/src/hash_fnv.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/oahash.o: ../../../../../framework/tcpip/src/oahash.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/oahash.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/oahash.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/oahash.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/oahash.o.d" -o ${OBJECTDIR}/_ext/1164207549/oahash.o ../../../../../framework/tcpip/src/oahash.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/tcpip_helpers.o: ../../../../../framework/tcpip/src/tcpip_helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_helpers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcpip_helpers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcpip_helpers.o.d" -o ${OBJECTDIR}/_ext/1164207549/tcpip_helpers.o ../../../../../framework/tcpip/src/tcpip_helpers.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/tcpip_manager.o: ../../../../../framework/tcpip/src/tcpip_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_manager.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcpip_manager.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcpip_manager.o.d" -o ${OBJECTDIR}/_ext/1164207549/tcpip_manager.o ../../../../../framework/tcpip/src/tcpip_manager.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/tcpip_notify.o: ../../../../../framework/tcpip/src/tcpip_notify.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_notify.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_notify.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcpip_notify.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcpip_notify.o.d" -o ${OBJECTDIR}/_ext/1164207549/tcpip_notify.o ../../../../../framework/tcpip/src/tcpip_notify.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/tcpip_packet.o: ../../../../../framework/tcpip/src/tcpip_packet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_packet.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/tcpip_packet.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/tcpip_packet.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/tcpip_packet.o.d" -o ${OBJECTDIR}/_ext/1164207549/tcpip_packet.o ../../../../../framework/tcpip/src/tcpip_packet.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/ipv4.o: ../../../../../framework/tcpip/src/ipv4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/ipv4.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/ipv4.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/ipv4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/ipv4.o.d" -o ${OBJECTDIR}/_ext/1164207549/ipv4.o ../../../../../framework/tcpip/src/ipv4.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1164207549/dns.o: ../../../../../framework/tcpip/src/dns.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1164207549" 
	@${RM} ${OBJECTDIR}/_ext/1164207549/dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/1164207549/dns.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1164207549/dns.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1164207549/dns.o.d" -o ${OBJECTDIR}/_ext/1164207549/dns.o ../../../../../framework/tcpip/src/dns.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1107379230/heap_2.o: ../../../../../third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1107379230" 
	@${RM} ${OBJECTDIR}/_ext/1107379230/heap_2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1107379230/heap_2.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1107379230/heap_2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1107379230/heap_2.o.d" -o ${OBJECTDIR}/_ext/1107379230/heap_2.o ../../../../../third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_2.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1282498059/port.o: ../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1282498059" 
	@${RM} ${OBJECTDIR}/_ext/1282498059/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1282498059/port.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1282498059/port.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/1282498059/port.o.d" -o ${OBJECTDIR}/_ext/1282498059/port.o ../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/373060831/croutine.o: ../../../../../third_party/rtos/FreeRTOS/Source/croutine.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/373060831" 
	@${RM} ${OBJECTDIR}/_ext/373060831/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/373060831/croutine.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/373060831/croutine.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/373060831/croutine.o.d" -o ${OBJECTDIR}/_ext/373060831/croutine.o ../../../../../third_party/rtos/FreeRTOS/Source/croutine.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/373060831/list.o: ../../../../../third_party/rtos/FreeRTOS/Source/list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/373060831" 
	@${RM} ${OBJECTDIR}/_ext/373060831/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/373060831/list.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/373060831/list.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/373060831/list.o.d" -o ${OBJECTDIR}/_ext/373060831/list.o ../../../../../third_party/rtos/FreeRTOS/Source/list.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/373060831/queue.o: ../../../../../third_party/rtos/FreeRTOS/Source/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/373060831" 
	@${RM} ${OBJECTDIR}/_ext/373060831/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/373060831/queue.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/373060831/queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/373060831/queue.o.d" -o ${OBJECTDIR}/_ext/373060831/queue.o ../../../../../third_party/rtos/FreeRTOS/Source/queue.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/373060831/tasks.o: ../../../../../third_party/rtos/FreeRTOS/Source/tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/373060831" 
	@${RM} ${OBJECTDIR}/_ext/373060831/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/373060831/tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/373060831/tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/373060831/tasks.o.d" -o ${OBJECTDIR}/_ext/373060831/tasks.o ../../../../../third_party/rtos/FreeRTOS/Source/tasks.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/373060831/timers.o: ../../../../../third_party/rtos/FreeRTOS/Source/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/373060831" 
	@${RM} ${OBJECTDIR}/_ext/373060831/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/373060831/timers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/373060831/timers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/373060831/timers.o.d" -o ${OBJECTDIR}/_ext/373060831/timers.o ../../../../../third_party/rtos/FreeRTOS/Source/timers.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/373060831/event_groups.o: ../../../../../third_party/rtos/FreeRTOS/Source/event_groups.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/373060831" 
	@${RM} ${OBJECTDIR}/_ext/373060831/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/373060831/event_groups.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/373060831/event_groups.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../../../../../framework" -I"../src/system_config/pic32mx795_pim__e16__freertos/framework" -I"../src/system_config/pic32mx795_pim__e16__freertos" -I"../src/system_config/pic32mx795_pim__e16__freertos/bsp" -I"../../../../../third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../third_party/rtos/FreeRTOS/Source/include" -Werror -Wall -MMD -MF "${OBJECTDIR}/_ext/373060831/event_groups.o.d" -o ${OBJECTDIR}/_ext/373060831/event_groups.o ../../../../../third_party/rtos/FreeRTOS/Source/event_groups.c    -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD) 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/pic32_wifi_web_server.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../bin/framework/peripheral/PIC32MX795F512L_peripherals.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_REAL_ICE=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/pic32_wifi_web_server.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ..\..\..\..\..\bin\framework\peripheral\PIC32MX795F512L_peripherals.a      -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD)   -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,--defsym=_min_heap_size=53248,--defsym=_min_stack_size=2048,--gc-sections,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/pic32_wifi_web_server.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../bin/framework/peripheral/PIC32MX795F512L_peripherals.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/pic32_wifi_web_server.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ..\..\..\..\..\bin\framework\peripheral\PIC32MX795F512L_peripherals.a      -DXPRJ_pic32mx795_pim__e16__freertos=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=53248,--defsym=_min_stack_size=2048,--gc-sections,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/pic32_wifi_web_server.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/pic32mx795_pim__e16__freertos
	${RM} -r dist/pic32mx795_pim__e16__freertos

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
