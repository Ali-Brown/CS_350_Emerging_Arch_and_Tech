################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /Applications/ti/ccs1260/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./gpiointerrupt.obj" \
"./syscfg/ti_drivers_config.obj" \
"./main_nortos.obj" \
"../cc32xxs_nortos.cmd" \
$(GEN_CMDS__FLAG) \
-lti_utils_build_linker.cmd.genlibs \
-l"ti/devices/cc32xx/driverlib/ccs/Release/driverlib.a" \
-llibc.a \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
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
gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.out 

EXE_OUTPUTS__QUOTED += \
"gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.out" 

CUSTOM_TOOL_OUTPUTS_654291168 += \
gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.bin 

CUSTOM_TOOL_OUTPUTS_654291168__QUOTED += \
"gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.bin" 

CUSTOM_TOOL_OUTPUTS_1024746469 += \
syscfg/gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.sli 

CUSTOM_TOOL_OUTPUTS_1024746469__QUOTED += \
"syscfg/gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.sli" 


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.out" secondary-outputs

# Tool invocations
gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"/Applications/ti/ccs1260/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=vfplib -me -O3 --define=DeviceFamily_CC3220 --define=NORTOS_SUPPORT -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on -z -m"gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.map" -i"/Applications/ti/simplelink_cc32xx_sdk_6_10_00_05/source" -i"/Applications/ti/simplelink_cc32xx_sdk_6_10_00_05/kernel/nortos" -i"/Users/alibrown-mac/workspace_v12/gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs/MCU+Image/syscfg" -i"/Applications/ti/ccs1260/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/lib" --diag_suppress=10063 --diag_wrap=off --display_error_number --warn_sections --xml_link_info="gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs_linkInfo.xml" --entry_point=resetISR --rom_model -o "gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.bin: $(EXE_OUTPUTS) gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.out
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: Arm ObjCopy'
	"/Applications/ti/ccs1260/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armobjcopy" -O binary --only-section .text --only-section .const --only-section .cinit --only-section .resetVecs "gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.out" "gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.bin"
	@echo 'Finished building secondary target: "$@"'
	@echo ' '

syscfg/gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.sli: $(EXE_OUTPUTS) gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.bin syscfg/ti_drivers_net_wifi_config.json
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: Image Creator'
	"/Applications/ti/simplelink_cc32xx_sdk_6_10_00_05/source/ti/drivers/net/imagecreator/bin/SLImageCreator" syscfg create_image --sdk_path "/Applications/ti/simplelink_cc32xx_sdk_6_10_00_05" --json "/Users/alibrown-mac/workspace_v12/gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs/MCU+Image/syscfg/ti_drivers_net_wifi_config.json" --file "/Users/alibrown-mac/workspace_v12/gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs/MCU+Image/syscfg/gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.sli" --mcu "/Users/alibrown-mac/workspace_v12/gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs/MCU+Image/gpiointerrupt_CC3220S_LAUNCHXL_nortos_ccs.bin"
	@echo 'Finished building secondary target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CUSTOM_TOOL_OUTPUTS_1024746469__QUOTED)$(CUSTOM_TOOL_OUTPUTS_654291168__QUOTED)$(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "gpiointerrupt.obj" "syscfg/ti_drivers_config.obj" "main_nortos.obj" 
	-$(RM) "gpiointerrupt.d" "syscfg/ti_drivers_config.d" "main_nortos.d" 
	-$(RMDIR) $(GEN_MISC_DIRS__QUOTED)
	-@echo 'Finished clean'
	-@echo ' '

secondary-outputs: $(CUSTOM_TOOL_OUTPUTS_654291168) $(CUSTOM_TOOL_OUTPUTS_1024746469)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

