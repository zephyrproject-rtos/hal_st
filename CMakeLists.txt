# Makefile - STLIB SDK
#
# Copyright (c) 2016 Linaro Limited
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_HAS_STLIB)

zephyr_library()

if(CONFIG_VL53L0X)
zephyr_include_directories(sensor/vl53l0x/api/core/inc
			   ${PROJECT_SOURCE_DIR}/drivers/sensor/vl53l0x)
zephyr_library_sources(sensor/vl53l0x/api/core/src/vl53l0x_api.c)
zephyr_library_sources(sensor/vl53l0x/api/core/src/vl53l0x_api_ranging.c)
zephyr_library_sources(sensor/vl53l0x/api/core/src/vl53l0x_api_calibration.c)
zephyr_library_sources(sensor/vl53l0x/api/core/src/vl53l0x_api_core.c)
zephyr_library_sources(sensor/vl53l0x/api/core/src/vl53l0x_api_strings.c)
endif()

if(CONFIG_VL53L1X)
zephyr_include_directories(sensor/vl53l1x/api/core/inc
			   ${PROJECT_SOURCE_DIR}/drivers/sensor/vl53l1x)
zephyr_library_sources(sensor/vl53l1x/api/core/src/vl53l1_api.c)
zephyr_library_sources(sensor/vl53l1x/api/core/src/vl53l1_api_calibration.c)
zephyr_library_sources(sensor/vl53l1x/api/core/src/vl53l1_wait.c)
zephyr_library_sources(sensor/vl53l1x/api/core/src/vl53l1_core.c)
zephyr_library_sources(sensor/vl53l1x/api/core/src/vl53l1_api_core.c)
zephyr_library_sources(sensor/vl53l1x/api/core/src/vl53l1_api_preset_modes.c)
zephyr_library_sources(sensor/vl53l1x/api/core/src/vl53l1_core_support.c)
zephyr_library_sources(sensor/vl53l1x/api/core/src/vl53l1_register_funcs.c)
endif()

if(CONFIG_VL53LX)
zephyr_include_directories(sensor/vl53lx/api/core/inc 
				${PROJECT_SOURCE_DIR}/drivers/sensor/vl53lx)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_api_calibration.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_api_core.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_api_debug.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_api_preset_modes.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_api.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_core_support.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_core.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_dmax.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_hist_algos_gen3.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_hist_algos_gen4.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_hist_char.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_hist_core.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_hist_funcs.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_nvm_debug.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_nvm.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_register_funcs.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_sigma_estimate.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_silicon_core.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_wait.c)
zephyr_library_sources(sensor/vl53lx/api/core/src/vl53lx_xtalk.c)
endif()

if(CONFIG_AUDIO_MPXXDTYY)
zephyr_include_directories(audio/microphone)
zephyr_library_sources(audio/microphone/OpenPDMFilter.c)
endif()

endif()

# STMEMSC - Hardware Abstraction Layer for ST sensor
add_subdirectory_ifdef(CONFIG_HAS_STMEMSC sensor/stmemsc)
