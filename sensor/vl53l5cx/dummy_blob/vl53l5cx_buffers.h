/**
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/*
 * This is a dummy firmware, in order to allow building the driver even when the
 * blobs data are not available
 */


#ifndef VL53L5CX_BUFFERS_H_
#define VL53L5CX_BUFFERS_H_

#include "platform.h"

/**
 * @brief Inner internal number of targets.
 */

#if VL53L5CX_NB_TARGET_PER_ZONE == 1
#define VL53L5CX_FW_NBTAR_RANGING	2
#else
#define VL53L5CX_FW_NBTAR_RANGING	VL53L5CX_NB_TARGET_PER_ZONE
#endif

const uint8_t VL53L5CX_FIRMWARE[0x15000];

const uint8_t VL53L5CX_DEFAULT_CONFIGURATION[0x3000];

const uint8_t VL53L5CX_DEFAULT_XTALK[VL53L5CX_XTALK_BUFFER_SIZE];

const uint8_t VL53L5CX_GET_NVM_CMD[0];

#endif /* VL53L5CX_BUFFERS_H_ */
