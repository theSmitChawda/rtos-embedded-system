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
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=cof
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=${DISTDIR}/Project4.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=${DISTDIR}/Project4.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS
SUB_IMAGE_ADDRESS_COMMAND=--image-address $(SUB_IMAGE_ADDRESS)
else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=RtosMain.c salvomem.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/RtosMain.o ${OBJECTDIR}/salvomem.o
POSSIBLE_DEPFILES=${OBJECTDIR}/RtosMain.o.d ${OBJECTDIR}/salvomem.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/RtosMain.o ${OBJECTDIR}/salvomem.o

# Source Files
SOURCEFILES=RtosMain.c salvomem.c



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
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/Project4.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=24FJ128GA010
MP_LINKER_FILE_OPTION=,--script=p24FJ128GA010.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/RtosMain.o: RtosMain.c  .generated_files/flags/default/99c7ef8c921cda067d776552778eb5c871fe6d10 .generated_files/flags/default/935296292c9a6f91ccf98053fa2612fe307a8f58
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/RtosMain.o.d 
	@${RM} ${OBJECTDIR}/RtosMain.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  RtosMain.c  -o ${OBJECTDIR}/RtosMain.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/RtosMain.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=coff -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"D:/Sheridan College/Semester 6/COURSES/Embedded Systems/Labs/Project4.X" -I"C:/Pumpkin/Salvo/Src/MCC30" -I"C:/Pumpkin/Salvo/Inc" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/salvomem.o: salvomem.c  .generated_files/flags/default/8001d34c168de82727801350ff1028d66b34e182 .generated_files/flags/default/935296292c9a6f91ccf98053fa2612fe307a8f58
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/salvomem.o.d 
	@${RM} ${OBJECTDIR}/salvomem.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  salvomem.c  -o ${OBJECTDIR}/salvomem.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/salvomem.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=coff -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"D:/Sheridan College/Semester 6/COURSES/Embedded Systems/Labs/Project4.X" -I"C:/Pumpkin/Salvo/Src/MCC30" -I"C:/Pumpkin/Salvo/Inc" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
else
${OBJECTDIR}/RtosMain.o: RtosMain.c  .generated_files/flags/default/67aac4aeb0b97cbb060bb8207994c41799e5d662 .generated_files/flags/default/935296292c9a6f91ccf98053fa2612fe307a8f58
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/RtosMain.o.d 
	@${RM} ${OBJECTDIR}/RtosMain.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  RtosMain.c  -o ${OBJECTDIR}/RtosMain.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/RtosMain.o.d"        -g -omf=coff -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"D:/Sheridan College/Semester 6/COURSES/Embedded Systems/Labs/Project4.X" -I"C:/Pumpkin/Salvo/Src/MCC30" -I"C:/Pumpkin/Salvo/Inc" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
${OBJECTDIR}/salvomem.o: salvomem.c  .generated_files/flags/default/75f8cbf7835c50b0af0c947f9b6e8075e4fc8c1a .generated_files/flags/default/935296292c9a6f91ccf98053fa2612fe307a8f58
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/salvomem.o.d 
	@${RM} ${OBJECTDIR}/salvomem.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  salvomem.c  -o ${OBJECTDIR}/salvomem.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/salvomem.o.d"        -g -omf=coff -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"D:/Sheridan College/Semester 6/COURSES/Embedded Systems/Labs/Project4.X" -I"C:/Pumpkin/Salvo/Src/MCC30" -I"C:/Pumpkin/Salvo/Inc" -O0 -msmart-io=1 -Wall -msfr-warn=off    -mdfp="${DFP_DIR}/xc16"
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/Project4.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  C:/Pumpkin/Salvo/Lib/MCC30-v3/libsalvolmcc30s-t.a  
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/Project4.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    C:\Pumpkin\Salvo\Lib\MCC30-v3\libsalvolmcc30s-t.a  -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=coff -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"D:/Sheridan College/Semester 6/COURSES/Embedded Systems/Labs/Project4.X" -I"C:/Pumpkin/Salvo/Src/MCC30" -I"C:/Pumpkin/Salvo/Inc"  -mreserve=data@0x800:0x81F -mreserve=data@0x820:0x821 -mreserve=data@0x822:0x823 -mreserve=data@0x824:0x825 -mreserve=data@0x826:0x84F   -Wl,,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_ICD3=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	
else
${DISTDIR}/Project4.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  C:/Pumpkin/Salvo/Lib/MCC30-v3/libsalvolmcc30s-t.a 
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o ${DISTDIR}/Project4.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    C:\Pumpkin\Salvo\Lib\MCC30-v3\libsalvolmcc30s-t.a  -mcpu=$(MP_PROCESSOR_OPTION)        -omf=coff -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"D:/Sheridan College/Semester 6/COURSES/Embedded Systems/Labs/Project4.X" -I"C:/Pumpkin/Salvo/Src/MCC30" -I"C:/Pumpkin/Salvo/Inc" -Wl,,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,${DISTDIR}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp="${DFP_DIR}/xc16" 
	${MP_CC_DIR}\\xc16-bin2hex ${DISTDIR}/Project4.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=coff   -mdfp="${DFP_DIR}/xc16" 
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
