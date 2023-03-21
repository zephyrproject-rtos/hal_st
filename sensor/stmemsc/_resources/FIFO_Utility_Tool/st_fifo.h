/*
 ******************************************************************************
 * @file    st_fifo.h
 * @author  Sensor Solutions Software Team
 * @brief   Utility for managing FIFO data decoding and decompression.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ST_FIFO_H
#define ST_FIFO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup FIFO utility
  * @{
  *
  */

/** @defgroup FIFO public definitions
  * @{
  *
  */

typedef enum
{
  ST_FIFO_OK,
  ST_FIFO_ERR
} st_fifo_status;

typedef enum
{
  ST_FIFO_GYROSCOPE,
  ST_FIFO_ACCELEROMETER,
  ST_FIFO_TEMPERATURE,
  ST_FIFO_EXT_SENSOR0,
  ST_FIFO_EXT_SENSOR1,
  ST_FIFO_EXT_SENSOR2,
  ST_FIFO_EXT_SENSOR3,
  ST_FIFO_STEP_COUNTER,
  ST_FIFO_6X_GAME_RV,
  ST_FIFO_6X_GEOM_RV,
  ST_FIFO_9X_RV,
  ST_FIFO_GYRO_BIAS,
  ST_FIFO_GRAVITY,
  ST_FIFO_MAGNETOMETER_CALIB,
  ST_FIFO_EXT_SENSOR_NACK,
  ST_FIFO_MLC_RESULT,
  ST_FIFO_MLC_FILTER,
  ST_FIFO_MLC_FEATURE,
  ST_FIFO_DUALC_ACCELEROMETER,
  ST_FIFO_EIS_GYROSCOPE,
  ST_FIFO_NONE
} st_fifo_sensor_type;

typedef struct
{
  uint8_t fifo_data_out[7]; /* registers from mems (78h -> 7Dh) */
} st_fifo_raw_slot;

typedef struct
{
  uint32_t timestamp;
  st_fifo_sensor_type sensor_tag;
  union
  {
    uint8_t raw_data[6]; /* bytes */
    int16_t data[3]; /* 3-axis mems */
    struct
    {
      int16_t x;
      int16_t y;
      int16_t z;
    }; /* 3-axis mems */
    struct
    {
      int16_t temp;
    }; /* temperature sensor */
    struct
    {
      uint16_t steps;
      uint8_t steps_t[4];
    }; /* step counter */
    struct
    {
      uint16_t qx;
      uint16_t qy;
      uint16_t qz;
    }; /* quaternion */
    struct
    {
      uint8_t nack;
    }; /* ext sensor nack index */
    struct
    {
      uint8_t mlc_res;
      uint8_t mlc_idx;
      uint8_t mlc_t[4];
    }; /* mlc result */
    struct
    {
      uint16_t mlc_value;
      uint16_t mlc_id;
      uint16_t reserved;
    }; /* mlc filter / feature */
  } sensor_data;
} st_fifo_out_slot;

typedef enum
{
  ST_FIFO_LSM6DSR,
  ST_FIFO_LSM6DSRX,
  ST_FIFO_ASM330LHH,
  ST_FIFO_ASM330LHHX,
  ST_FIFO_ISM330DHCX,
  ST_FIFO_LSM6DSO,
  ST_FIFO_LSM6DSOX,
  ST_FIFO_LSM6DSO32,
  ST_FIFO_LSM6DSO32X,
  ST_FIFO_LSM6DSV,
  ST_FIFO_LSM6DSV16X,
  ST_FIFO_LSM6DSV32X,
} st_fifo_device;

typedef struct
{
  st_fifo_device device; /* device to select */
  float bdr_xl; /* accelerometer batch data rate in Hz */
  float bdr_gy; /* gyroscope batch data rate in Hz */
  float bdr_vsens; /* virtual sensor batch data rate in Hz */
} st_fifo_conf;

/**
  * @}
  *
  */

st_fifo_status st_fifo_init(st_fifo_conf *conf);
st_fifo_status st_fifo_decode(st_fifo_out_slot *fifo_out_slot,
                              st_fifo_raw_slot *fifo_raw_slot, uint16_t *out_slot_size, uint16_t stream_size);
void st_fifo_sort(st_fifo_out_slot *fifo_out_slot, uint16_t out_slot_size);
uint16_t st_fifo_get_sensor_occurrence(st_fifo_out_slot *fifo_out_slot,
                                       uint16_t out_slot_size, st_fifo_sensor_type sensor_type);
void st_fifo_extract_sensor(st_fifo_out_slot *sensor_out_slot,
                            st_fifo_out_slot *fifo_out_slot, uint16_t out_slot_size,
                            st_fifo_sensor_type sensor_type);

#ifdef __cplusplus
}
#endif

#endif /* ST_FIFO_H */

/**
  * @}
  *
  */
