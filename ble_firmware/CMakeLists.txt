# Copyright (c) 2024 STMicroelectronics
#
# SPDX-License-Identifier: Apache-2.0

set(hal_dir            ${ZEPHYR_HAL_ST_MODULE_DIR})
set(hal_blobs_dir      ${hal_dir}/zephyr/blobs/img/bluetooth/steval-mkboxpro)
set(blob_gen_inc_file  ${ZEPHYR_BINARY_DIR}/include/generated/dtm.bin.inc)

# bluenrg-lp
set(blob_hcd_file      ${hal_blobs_dir}/dtm.bin)

# generate Bluetooth include blob from .bin
if(EXISTS ${blob_hcd_file})
  message(INFO " generate include of blob Bluetooth file: ${blob_hcd_file}")

  generate_inc_file_for_target(app ${blob_hcd_file} ${blob_gen_inc_file})
  zephyr_library_sources(${CMAKE_CURRENT_SOURCE_DIR}/ble_fw.c)
endif()
