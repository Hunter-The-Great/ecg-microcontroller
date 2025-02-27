################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /Applications/ti/ccs2002/ccs/tools/compiler/ti-cgt-armllvm_4.0.1.LTS

GEN_OPTS__FLAG := @"syscfg/ti_utils_build_compiler.opt" 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./syscfg/ti_ble_config.o" \
"./syscfg/ti_radio_config.o" \
"./syscfg/ti_drivers_config.o" \
"./syscfg/ti_sysbios_config.o" \
"./Application/project_zero.o" \
"./Application/rcosc_calibration.o" \
"./Drivers/NV/crc.o" \
"./Drivers/NV/nvocmp.o" \
"./OAD/crc32.o" \
"./OAD/flash_interface_ext_rtos_NVS.o" \
"./OAD/oad.o" \
"./OAD/oad_image_header_app.o" \
"./Profiles/button_service.o" \
"./Profiles/data_service.o" \
"./Profiles/devinfoservice.o" \
"./Profiles/led_service.o" \
"./Startup/UartLog.o" \
"./Startup/main.o" \
"./Startup/osal_icall_ble.o" \
"./Startup/rom_init.o" \
"./common/Util/util.o" \
"./iCall/icall.o" \
"./iCall/icall_cc2650.o" \
"./iCall/icall_user_config.o" \
"./iCallBLE/ble_user_config.o" \
"./iCallBLE/ble_user_config_stack.o" \
"./iCallBLE/icall_api_lite.o" \
"../cc13x1_cc26x1_app_tirtos7.cmd" \
$(GEN_CMDS__FLAG) \
-Wl,-lti_utils_build_linker.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include Application/subdir_vars.mk
-include Drivers/NV/subdir_vars.mk
-include OAD/subdir_vars.mk
-include Profiles/subdir_vars.mk
-include Startup/subdir_vars.mk
-include common/Util/subdir_vars.mk
-include iCall/subdir_vars.mk
-include iCallBLE/subdir_vars.mk
-include subdir_rules.mk
-include Application/subdir_rules.mk
-include Drivers/NV/subdir_rules.mk
-include OAD/subdir_rules.mk
-include Profiles/subdir_rules.mk
-include Startup/subdir_rules.mk
-include common/Util/subdir_rules.mk
-include iCall/subdir_rules.mk
-include iCallBLE/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
project_zero_LP_CC2651R3SIPA_tirtos7_ticlang.out 

EXE_OUTPUTS__QUOTED += \
"project_zero_LP_CC2651R3SIPA_tirtos7_ticlang.out" 


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "project_zero_LP_CC2651R3SIPA_tirtos7_ticlang.out"

# Tool invocations
project_zero_LP_CC2651R3SIPA_tirtos7_ticlang.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"/Applications/ti/ccs2002/ccs/tools/compiler/ti-cgt-armllvm_4.0.1.LTS/bin/tiarmclang" @"/Users/Benjamin/ti/simplelink_cc13xx_cc26xx_sdk_7_41_00_17/source/ti/ble5stack_flash/config/build_components.opt" @"/Users/Benjamin/ti/simplelink_cc13xx_cc26xx_sdk_7_41_00_17/source/ti/ble5stack_flash/config/factory_config.opt"  -mcpu=cortex-m4 -mfloat-abi=soft -mfpu=none -mlittle-endian -mthumb -Oz -Duartlog_FILE="\"\"" -DSECURITY -DOAD_IMG_E=1 -DFLASH_ONLY_BUILD -DNVOCMP_NWSAMEITEM=1 -DHEAPMGR_CONFIG=0x80 -DHEAPMGR_SIZE=0x0 -gdwarf-3 -march=armv7e-m -Wl,-m"project_zero_LP_CC2651R3SIPA_tirtos7_ticlang.map" -Wl,-i"/Users/Benjamin/ti/simplelink_cc13xx_cc26xx_sdk_7_41_00_17/source" -Wl,-i"/Users/Benjamin/ti/simplelink_cc13xx_cc26xx_sdk_7_41_00_17/kernel/tirtos7/packages" -Wl,-i"/Users/Benjamin/workspace_ccstheia/project_zero_LP_CC2651R3SIPA_tirtos7_ticlang/Release/syscfg" -Wl,-i"/Applications/ti/ccs2002/ccs/tools/compiler/ti-cgt-armllvm_4.0.1.LTS/lib" -Wl,--reread_libs -Wl,--define=OAD_IMG_E=1 -Wl,--define=SECURITY -Wl,--define=FLASH_ONLY_BUILD=2 -Wl,--diag_suppress=16002-D -Wl,--diag_suppress=10247-D -Wl,--diag_suppress=10325-D -Wl,--diag_suppress=10229-D -Wl,--diag_suppress=16032-D -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="project_zero_LP_CC2651R3SIPA_tirtos7_ticlang_linkInfo.xml" -Wl,--rom_model -o "project_zero_LP_CC2651R3SIPA_tirtos7_ticlang.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "syscfg/ti_ble_config.o" "syscfg/ti_radio_config.o" "syscfg/ti_drivers_config.o" "syscfg/ti_sysbios_config.o" "Application/project_zero.o" "Application/rcosc_calibration.o" "Drivers/NV/crc.o" "Drivers/NV/nvocmp.o" "OAD/crc32.o" "OAD/flash_interface_ext_rtos_NVS.o" "OAD/oad.o" "OAD/oad_image_header_app.o" "Profiles/button_service.o" "Profiles/data_service.o" "Profiles/devinfoservice.o" "Profiles/led_service.o" "Startup/UartLog.o" "Startup/main.o" "Startup/osal_icall_ble.o" "Startup/rom_init.o" "common/Util/util.o" "iCall/icall.o" "iCall/icall_cc2650.o" "iCall/icall_user_config.o" "iCallBLE/ble_user_config.o" "iCallBLE/ble_user_config_stack.o" "iCallBLE/icall_api_lite.o" 
	-$(RM) "syscfg/ti_ble_config.d" "syscfg/ti_radio_config.d" "syscfg/ti_drivers_config.d" "syscfg/ti_sysbios_config.d" "Application/project_zero.d" "Application/rcosc_calibration.d" "Drivers/NV/crc.d" "Drivers/NV/nvocmp.d" "OAD/crc32.d" "OAD/flash_interface_ext_rtos_NVS.d" "OAD/oad.d" "OAD/oad_image_header_app.d" "Profiles/button_service.d" "Profiles/data_service.d" "Profiles/devinfoservice.d" "Profiles/led_service.d" "Startup/UartLog.d" "Startup/main.d" "Startup/osal_icall_ble.d" "Startup/rom_init.d" "common/Util/util.d" "iCall/icall.d" "iCall/icall_cc2650.d" "iCall/icall_user_config.d" "iCallBLE/ble_user_config.d" "iCallBLE/ble_user_config_stack.d" "iCallBLE/icall_api_lite.d" 
	-$(RMDIR) $(GEN_MISC_DIRS__QUOTED)
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-/Applications/ti/ccs2002/ccs/tools/compiler/ti-cgt-armllvm_4.0.1.LTS/bin/tiarmobjcopy -O ihex project_zero_LP_CC2651R3SIPA_tirtos7_ticlang.out project_zero_LP_CC2651R3SIPA_tirtos7_ticlang.hex
	-/Users/Benjamin/ti/simplelink_cc13xx_cc26xx_sdk_7_41_00_17/tools/common/oad/oad_image_tool --verbose ccs /Users/Benjamin/workspace_ccstheia/project_zero_LP_CC2651R3SIPA_tirtos7_ticlang/Release 7 -hex1 /Users/Benjamin/workspace_ccstheia/project_zero_LP_CC2651R3SIPA_tirtos7_ticlang/Release/project_zero_LP_CC2651R3SIPA_tirtos7_ticlang.hex -k /Users/Benjamin/ti/simplelink_cc13xx_cc26xx_sdk_7_41_00_17/tools/common/oad/private.pem -o /Users/Benjamin/workspace_ccstheia/project_zero_LP_CC2651R3SIPA_tirtos7_ticlang/Release/project_zero_LP_CC2651R3SIPA_tirtos7_ticlang_oad
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

