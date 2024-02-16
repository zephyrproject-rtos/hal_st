/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

const uint8_t ble_fw_img[] = {
	#include <dtm.bin.inc>
};

const uint32_t ble_fw_img_len = sizeof(ble_fw_img);
