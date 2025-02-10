################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -Ooff --fp_mode=relaxed --include_path="C:/Users/Ever_/workspace_v10/3phInverter" --include_path="C:/ti/controlSUITE/libs/app_libs/solar/v1.2" --include_path="C:/ti/controlSUITE/libs/app_libs/solar/v1.2/float/include" --include_path="C:/ti/controlSUITE/libs/app_libs/solar/v1.2/float/source" --include_path="C:/ti/c2000/C2000Ware_5_00_00_00/device_support/f2837xd/headers/source" --include_path="C:/ti/c2000/C2000Ware_5_00_00_00/device_support/f2837xd/headers/include" --include_path="C:/ti/c2000/C2000Ware_5_00_00_00/device_support/f2837xd/common/source" --include_path="C:/ti/c2000/C2000Ware_5_00_00_00/device_support/f2837xd/common/include" --include_path="C:/Users/Ever_/workspace_v10/3phInverter/application" --include_path="C:/Users/Ever_/workspace_v10/3phInverter/application/fis" --include_path="C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/include" --advice:performance=all --define=CPU1 -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -Ooff --fp_mode=relaxed --include_path="C:/Users/Ever_/workspace_v10/3phInverter" --include_path="C:/ti/controlSUITE/libs/app_libs/solar/v1.2" --include_path="C:/ti/controlSUITE/libs/app_libs/solar/v1.2/float/include" --include_path="C:/ti/controlSUITE/libs/app_libs/solar/v1.2/float/source" --include_path="C:/ti/c2000/C2000Ware_5_00_00_00/device_support/f2837xd/headers/source" --include_path="C:/ti/c2000/C2000Ware_5_00_00_00/device_support/f2837xd/headers/include" --include_path="C:/ti/c2000/C2000Ware_5_00_00_00/device_support/f2837xd/common/source" --include_path="C:/ti/c2000/C2000Ware_5_00_00_00/device_support/f2837xd/common/include" --include_path="C:/Users/Ever_/workspace_v10/3phInverter/application" --include_path="C:/Users/Ever_/workspace_v10/3phInverter/application/fis" --include_path="C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-c2000_20.2.5.LTS/include" --advice:performance=all --define=CPU1 -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


