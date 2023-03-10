#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a pre- and a post- target defined where you can add customization code.
#
# This makefile implements macros and targets common to all configurations.
#
# NOCDDL


# Building and Cleaning subprojects are done by default, but can be controlled with the SUB
# macro. If SUB=no, subprojects will not be built or cleaned. The following macro
# statements set BUILD_SUB-CONF and CLEAN_SUB-CONF to .build-reqprojects-conf
# and .clean-reqprojects-conf unless SUB has the value 'no'
SUB_no=NO
SUBPROJECTS=${SUB_${SUB}}
BUILD_SUBPROJECTS_=.build-subprojects
BUILD_SUBPROJECTS_NO=
BUILD_SUBPROJECTS=${BUILD_SUBPROJECTS_${SUBPROJECTS}}
CLEAN_SUBPROJECTS_=.clean-subprojects
CLEAN_SUBPROJECTS_NO=
CLEAN_SUBPROJECTS=${CLEAN_SUBPROJECTS_${SUBPROJECTS}}


# Project Name
PROJECTNAME=pic32_wifi_web_server.X

# Active Configuration
DEFAULTCONF=pic32mx695f_wireless_eval_board_miwi
CONF=${DEFAULTCONF}

# All Configurations
ALLCONFS=chipkit_wf32 pic32mx795_pim__e16 pic32mx795_pim__e16__11n__freertos pic32mx795_pim__e16__freertos pic32mx695f_wireless_eval_board_miwi 32mx_wireless_eval_board_easyconf 


# build
.build-impl: .build-pre
	${MAKE} -f nbproject/Makefile-${CONF}.mk SUBPROJECTS=${SUBPROJECTS} .build-conf


# clean
.clean-impl: .clean-pre
	${MAKE} -f nbproject/Makefile-${CONF}.mk SUBPROJECTS=${SUBPROJECTS} .clean-conf

# clobber
.clobber-impl: .clobber-pre .depcheck-impl
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=chipkit_wf32 clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx795_pim__e16 clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx795_pim__e16__11n__freertos clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx795_pim__e16__freertos clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx695f_wireless_eval_board_miwi clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=32mx_wireless_eval_board_easyconf clean



# all
.all-impl: .all-pre .depcheck-impl
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=chipkit_wf32 build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx795_pim__e16 build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx795_pim__e16__11n__freertos build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx795_pim__e16__freertos build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx695f_wireless_eval_board_miwi build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=32mx_wireless_eval_board_easyconf build



# dependency checking support
.depcheck-impl:
#	@echo "# This code depends on make tool being used" >.dep.inc
#	@if [ -n "${MAKE_VERSION}" ]; then \
#	    echo "DEPFILES=\$$(wildcard \$$(addsuffix .d, \$${OBJECTFILES}))" >>.dep.inc; \
#	    echo "ifneq (\$${DEPFILES},)" >>.dep.inc; \
#	    echo "include \$${DEPFILES}" >>.dep.inc; \
#	    echo "endif" >>.dep.inc; \
#	else \
#	    echo ".KEEP_STATE:" >>.dep.inc; \
#	    echo ".KEEP_STATE_FILE:.make.state.\$${CONF}" >>.dep.inc; \
#	fi
