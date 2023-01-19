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
PROJECTNAME=wifi_easy_configuration.X

# Active Configuration
DEFAULTCONF=pic32mx_wireless_eval_board
CONF=${DEFAULTCONF}

# All Configurations
ALLCONFS=pic32mx795_pim__e16 pic32mx795_pim__e16__11n__freertos pic32mx795_pim__e16__freertos pic32mx_eth_sk__ioexp pic32mx_eth_sk__ioexp__11n__freertos pic32mx_eth_sk__ioexp__freertos pic32mz_ef_curiosity pic32mz_ef_sk__ioexp pic32mz_ef_sk__ioexp__11n__freertos pic32mz_ef_sk__ioexp__freertos pic32mz_ef_sk__meb2 pic32mz_ef_sk__meb2__11n__freertos pic32mz_ef_sk__meb2__freertos pic32mx_wireless_eval_board 


# build
.build-impl: .build-pre
	${MAKE} -f nbproject/Makefile-${CONF}.mk SUBPROJECTS=${SUBPROJECTS} .build-conf


# clean
.clean-impl: .clean-pre
	${MAKE} -f nbproject/Makefile-${CONF}.mk SUBPROJECTS=${SUBPROJECTS} .clean-conf

# clobber
.clobber-impl: .clobber-pre .depcheck-impl
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx795_pim__e16 clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx795_pim__e16__11n__freertos clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx795_pim__e16__freertos clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx_eth_sk__ioexp clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx_eth_sk__ioexp__11n__freertos clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx_eth_sk__ioexp__freertos clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mz_ef_curiosity clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mz_ef_sk__ioexp clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mz_ef_sk__ioexp__11n__freertos clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mz_ef_sk__ioexp__freertos clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mz_ef_sk__meb2 clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mz_ef_sk__meb2__11n__freertos clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mz_ef_sk__meb2__freertos clean
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx_wireless_eval_board clean



# all
.all-impl: .all-pre .depcheck-impl
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx795_pim__e16 build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx795_pim__e16__11n__freertos build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx795_pim__e16__freertos build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx_eth_sk__ioexp build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx_eth_sk__ioexp__11n__freertos build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx_eth_sk__ioexp__freertos build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mz_ef_curiosity build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mz_ef_sk__ioexp build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mz_ef_sk__ioexp__11n__freertos build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mz_ef_sk__ioexp__freertos build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mz_ef_sk__meb2 build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mz_ef_sk__meb2__11n__freertos build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mz_ef_sk__meb2__freertos build
	    ${MAKE} SUBPROJECTS=${SUBPROJECTS} CONF=pic32mx_wireless_eval_board build



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
