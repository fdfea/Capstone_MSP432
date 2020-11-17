################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/ti/ccs1011/ccs/tools/compiler/ti-cgt-arm_20.2.1.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./EVE3.obj" \
"./MSP_EXP432P401R.obj" \
"./SMO.obj" \
"./get_time.obj" \
"./i2cmaster.obj" \
"./main_tirtos.obj" \
"./network_if.obj" \
"./rtc.obj" \
"./sl_wifi_callbacks.obj" \
"./uart_term.obj" \
"./utils/ustdlib.obj" \
"../MSP_EXP432P401R_TIRTOS.cmd" \
$(GEN_CMDS__FLAG) \
-l"/source/ti/display/lib/display.aem4f" \
-l"/source/ti/drivers/lib/drivers_msp432p401x.aem4f" \
-l"D:/ti/simplelink_sdk_wifi_plugin_4_20_00_10/source/ti/net/lib/ccs/m4f/slnetsock_release.a" \
-l"D:/ti/simplelink_sdk_wifi_plugin_4_20_00_10/source/ti/drivers/net/wifi/slnetif/ccs/Release/slnetifwifi.a" \
-l"/source/third_party/fatfs/lib/fatfs.aem4f" \
-l"D:/ti/simplelink_sdk_wifi_plugin_4_20_00_10/source/ti/drivers/net/wifi/ccs/rtos/msp432p4/simplelink.a" \
-l"/kernel/tirtos/packages/ti/dpl/lib/dpl_msp432p401x.aem4f" \
-l"/source/ti/devices/msp432p4xx/driverlib/ccs/msp432p4xx_driverlib.lib" \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include utils/subdir_vars.mk
-include subdir_rules.mk
-include utils/subdir_rules.mk
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
MSP432_Wifi_Test.out \

EXE_OUTPUTS__QUOTED += \
"MSP432_Wifi_Test.out" \

BIN_OUTPUTS += \
MSP432_Wifi_Test.hex \

BIN_OUTPUTS__QUOTED += \
"MSP432_Wifi_Test.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "MSP432_Wifi_Test.out"

# Tool invocations
MSP432_Wifi_Test.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: ARM Linker'
	"D:/ti/ccs1011/ccs/tools/compiler/ti-cgt-arm_20.2.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --advice:power=none --define=__MSP432P401R__ --define=DeviceFamily_MSP432P401x -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on -z -m"MSP432_Wifi_Test.map" -i"D:/ti/ccs1011/ccs/tools/compiler/ti-cgt-arm_20.2.1.LTS/lib" -i"D:/ti/ccs1011/ccs/tools/compiler/ti-cgt-arm_20.2.1.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="MSP432_Wifi_Test_linkInfo.xml" --rom_model -o "MSP432_Wifi_Test.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

MSP432_Wifi_Test.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: ARM Hex Utility'
	"D:/ti/ccs1011/ccs/tools/compiler/ti-cgt-arm_20.2.1.LTS/bin/armhex"  -o "MSP432_Wifi_Test.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "EVE3.obj" "MSP_EXP432P401R.obj" "SMO.obj" "get_time.obj" "i2cmaster.obj" "main_tirtos.obj" "network_if.obj" "rtc.obj" "sl_wifi_callbacks.obj" "uart_term.obj" "utils\ustdlib.obj" 
	-$(RM) "EVE3.d" "MSP_EXP432P401R.d" "SMO.d" "get_time.d" "i2cmaster.d" "main_tirtos.d" "network_if.d" "rtc.d" "sl_wifi_callbacks.d" "uart_term.d" "utils\ustdlib.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

