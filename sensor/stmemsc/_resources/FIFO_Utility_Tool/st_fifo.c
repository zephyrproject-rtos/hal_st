/*
 ******************************************************************************
 * @file    fifo_utility.c
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

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <float.h>
#include <math.h>
#include "st_fifo.h"

/**
  * @defgroup  FIFO utility
  * @brief     This file provides a set of functions needed to manage FIFO data
  *            decoding and decompression.
  * @{
  *
  */

/* Private constants  --------------------------------------------------------*/
#define BDR_XL_MASK                (0x0Fu)
#define BDR_XL_SHIFT               (0x00u)

#define BDR_GY_MASK                (0xF0u)
#define BDR_GY_SHIFT               (0x04u)

#define BDR_VSENS_MASK             (0x0Fu)
#define BDR_VSENS_SHIFT            (0x00u)

#define TAG_COUNTER_MASK           (0x06u)
#define TAG_SENSOR_MASK            (0xF8u)
#define TAG_COUNTER_SHIFT          (0x01u)
#define TAG_SENSOR_SHIFT           (0x03u)

#define TAG_EMPTY                  (0x00u)
#define TAG_GY                     (0x01u)
#define TAG_XL                     (0x02u)
#define TAG_TEMP                   (0x03u)
#define TAG_TS                     (0x04u)
#define TAG_ODRCHG                 (0x05u)
#define TAG_XL_UNCOMPRESSED_T_2    (0x06u)
#define TAG_XL_UNCOMPRESSED_T_1    (0x07u)
#define TAG_XL_COMPRESSED_2X       (0x08u)
#define TAG_XL_COMPRESSED_3X       (0x09u)
#define TAG_GY_UNCOMPRESSED_T_2    (0x0Au)
#define TAG_GY_UNCOMPRESSED_T_1    (0x0Bu)
#define TAG_GY_COMPRESSED_2X       (0x0Cu)
#define TAG_GY_COMPRESSED_3X       (0x0Du)
#define TAG_EXT_SENS_0             (0x0Eu)
#define TAG_EXT_SENS_1             (0x0Fu)
#define TAG_EXT_SENS_2             (0x10u)
#define TAG_EXT_SENS_3             (0x11u)
#define TAG_STEP_COUNTER           (0x12u)
#define TAG_GAME_RV                (0x13u)
#define TAG_GEOM_RV                (0x14u)
#define TAG_NORM_RV                (0x15u)
#define TAG_GYRO_BIAS              (0x16u)
#define TAG_GRAVITIY               (0x17u)
#define TAG_MAG_CAL                (0x18u)
#define TAG_EXT_SENS_NACK          (0x19u)
#define TAG_MLC_RESULT             (0x1Au)
#define TAG_MLC_FILTER             (0x1Bu)
#define TAG_MLC_FEATURE            (0x1Cu)
#define TAG_DUALC_XL               (0x1Du)
#define TAG_EIS_GY                 (0x1Eu)

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  ST_FIFO_COMPRESSION_NC,
  ST_FIFO_COMPRESSION_NC_T_1,
  ST_FIFO_COMPRESSION_NC_T_2,
  ST_FIFO_COMPRESSION_2X,
  ST_FIFO_COMPRESSION_3X
} st_fifo_compression_type;

/* Private functions ---------------------------------------------------------*/
/* Functions declare in this section are defined at the end of this file.     */
static uint8_t bdr_get_index(const float *bdr, float n);
static uint8_t has_even_parity(uint8_t x);
static st_fifo_sensor_type get_sensor_type(uint8_t tag);
static st_fifo_compression_type get_compression_type(uint8_t tag);
static uint8_t is_tag_valid(uint8_t tag);
static void get_diff_2x(int16_t diff[6], uint8_t input[6]);
static void get_diff_3x(int16_t diff[9], uint8_t input[6]);

/* Private variables ---------------------------------------------------------*/
static const struct
{
  float bdr_acc[16];
  float bdr_gyr[16];
  float bdr_vsens[16];
  uint32_t dtime[16];
  uint8_t tag_valid_limit;
} device[] =
{
  {
    .bdr_acc = { 0, 13, 26, 52, 104, 208, 416, 833, 1666, 3333, 6666, 1.625, 0, 0, 0, 0 },
    .bdr_gyr = { 0, 13, 26, 52, 104, 208, 416, 833, 1666, 3333, 6666, 0, 0, 0, 0, 0 },
    .bdr_vsens = { 0, 13, 26, 52, 104, 208, 416, 0, 0, 0, 0, 1.625, 0, 0, 0, 0 },
    .dtime = { 0, 3072, 1536, 768, 384, 192, 96, 48, 24, 12, 6, 24576, 0, 0, 0, 0 },
    .tag_valid_limit = 0x19,
  },
  {
    .bdr_acc = { 0, 1.875, 7.5, 15, 30, 60, 120, 240, 480, 960, 1920, 3840, 7680, 0, 0, 0 },
    .bdr_gyr = { 0, 1.875, 7.5, 15, 30, 60, 120, 240, 480, 960, 1920, 3840, 7680, 0, 0, 0 },
    .bdr_vsens = { 0, 1.875, 7.5, 15, 30, 60, 120, 240, 480, 960, 0, 0, 0, 0, 0, 0 },
    .dtime = { 0, 24576, 6144, 3072, 1536, 768, 384, 192, 96, 48, 24, 12, 6, 0, 0, 0 },
    .tag_valid_limit = 0x1E,
  },
};

static uint8_t fifo_ver;
static uint8_t tag_counter_old;
static uint32_t dtime_xl;
static uint32_t dtime_gy;
static uint32_t dtime_min;
static uint32_t dtime_xl_old;
static uint32_t dtime_gy_old;
static uint32_t timestamp;
static uint32_t last_timestamp_xl;
static uint32_t last_timestamp_gy;
static uint8_t bdr_chg_xl_flag;
static uint8_t bdr_chg_gy_flag;
static int16_t last_data_xl[3];
static int16_t last_data_gy[3];

/**
  * @defgroup  FIFO public functions
  * @brief     This section provide a set of APIs for managing FIFO data
  *            decoding and decompression.
  * @{
  *
  */

/**
  * @brief  Initialize the FIFO utility library.
  *
  * @param  conf              library configuration to set (BDR parameters
  *                           can be set to 0 Hz if ODR_CHG_EN is enabled
  *                           or timestamp sensor is batched in FIFO).
  *
  * @retval st_fifo_status    ST_FIFO_OK / ST_FIFO_ERR
  *
  */
st_fifo_status st_fifo_init(st_fifo_conf *conf)
{
  float bdr_xl, bdr_gy, bdr_vsens, bdr_max;

  if (conf->bdr_xl < 0.0f || conf->bdr_gy < 0.0f || conf->bdr_vsens < 0.0f)
  {
    return ST_FIFO_ERR;
  }

  if (conf->device < ST_FIFO_LSM6DSV)
  {
    fifo_ver = 0;
  }
  else
  {
    fifo_ver = 1;
  }

  tag_counter_old = 0;
  bdr_xl = conf->bdr_xl;
  bdr_gy = conf->bdr_gy;
  bdr_vsens = conf->bdr_vsens;
  bdr_max = MAX(bdr_xl, bdr_gy);
  bdr_max = MAX(bdr_max, bdr_vsens);
  dtime_min = device[fifo_ver].dtime[bdr_get_index(device[fifo_ver].bdr_acc, bdr_max)];
  dtime_xl = device[fifo_ver].dtime[bdr_get_index(device[fifo_ver].bdr_acc, bdr_xl)];
  dtime_gy = device[fifo_ver].dtime[bdr_get_index(device[fifo_ver].bdr_gyr, bdr_gy)];
  dtime_xl_old = dtime_xl;
  dtime_gy_old = dtime_gy;
  timestamp = 0;
  bdr_chg_xl_flag = 0;
  bdr_chg_gy_flag = 0;
  last_timestamp_xl = 0;
  last_timestamp_gy = 0;

  for (uint8_t i = 0; i < 3u; i++)
  {
    last_data_xl[i] = 0;
    last_data_gy[i] = 0;
  }

  return ST_FIFO_OK;
}

/**
  * @brief  Decode and decompress a raw FIFO stream.
  *
  * @param  fifo_out_slot     decoded output stream.(ptr)
  * @param  fifo_raw_slot     compressed raw input data stream.(ptr)
  * @param  out_slot_size     decoded stream size.(ptr)
  * @param  stream_size       raw input stream size.
  *
  * @retval st_fifo_status    ST_FIFO_OK / ST_FIFO_ERR
  *
  */
st_fifo_status st_fifo_decode(st_fifo_out_slot *fifo_out_slot,
                              st_fifo_raw_slot *fifo_raw_slot, uint16_t *out_slot_size, uint16_t stream_size)
{
  uint16_t j = 0;

  for (uint16_t i = 0; i < stream_size; i++)
  {

    uint8_t tag = (fifo_raw_slot[i].fifo_data_out[0] & TAG_SENSOR_MASK) >> TAG_SENSOR_SHIFT;
    uint8_t tag_counter = (fifo_raw_slot[i].fifo_data_out[0] & TAG_COUNTER_MASK) >> TAG_COUNTER_SHIFT;

    if (fifo_ver == 0u && has_even_parity(fifo_raw_slot[i].fifo_data_out[0]) == 0u)
    {
      return ST_FIFO_ERR;
    }

    if (is_tag_valid(tag) == 0u)
    {
      return ST_FIFO_ERR;
    }

    if ((tag_counter != (tag_counter_old)) && dtime_min != 0u)
    {

      uint8_t diff_tag_counter;

      if (tag_counter < tag_counter_old)
      {
        diff_tag_counter = tag_counter + 4u - tag_counter_old;
      }
      else
      {
        diff_tag_counter = tag_counter - tag_counter_old;
      }

      timestamp += dtime_min * diff_tag_counter;
    }

    if (tag == TAG_ODRCHG)
    {

      uint8_t bdr_acc_cfg = (fifo_raw_slot[i].fifo_data_out[6] & BDR_XL_MASK) >> BDR_XL_SHIFT;
      uint8_t bdr_gyr_cfg = (fifo_raw_slot[i].fifo_data_out[6] & BDR_GY_MASK) >> BDR_GY_SHIFT;
      uint8_t bdr_vsens_cfg = (fifo_raw_slot[i].fifo_data_out[4] & BDR_VSENS_MASK) >> BDR_VSENS_SHIFT;

      float bdr_xl = device[fifo_ver].bdr_acc[bdr_acc_cfg];
      float bdr_gy = device[fifo_ver].bdr_gyr[bdr_gyr_cfg];
      float bdr_vsens = device[fifo_ver].bdr_vsens[bdr_vsens_cfg];
      float bdr_max = MAX(bdr_xl, bdr_gy);
      bdr_max = MAX(bdr_max, bdr_vsens);

      dtime_xl_old = dtime_xl;
      dtime_gy_old = dtime_gy;
      dtime_min = device[fifo_ver].dtime[bdr_get_index(device[fifo_ver].bdr_acc, bdr_max)];
      dtime_xl = device[fifo_ver].dtime[bdr_get_index(device[fifo_ver].bdr_acc, bdr_xl)];
      dtime_gy = device[fifo_ver].dtime[bdr_get_index(device[fifo_ver].bdr_gyr, bdr_gy)];

      bdr_chg_xl_flag = 1;
      bdr_chg_gy_flag = 1;

    }
    else if (tag == TAG_TS)
    {

      (void)memcpy(&timestamp, &fifo_raw_slot[i].fifo_data_out[1], 4);

    }
    else
    {

      st_fifo_compression_type compression_type = get_compression_type(tag);
      st_fifo_sensor_type sensor_type = get_sensor_type(tag);

      if (compression_type == ST_FIFO_COMPRESSION_NC)
      {

        if (tag == TAG_EMPTY)
        {
          continue;
        }

        if (tag == TAG_STEP_COUNTER || tag == TAG_MLC_RESULT)
        {
          (void)memcpy(&fifo_out_slot[j].timestamp, &fifo_raw_slot[i].fifo_data_out[3],
                       4);
        }
        else
        {
          fifo_out_slot[j].timestamp = timestamp;
        }

        fifo_out_slot[j].sensor_tag = sensor_type;
        (void)memcpy(fifo_out_slot[j].sensor_data.raw_data,
                     &fifo_raw_slot[i].fifo_data_out[1], 6);

        if (sensor_type == ST_FIFO_ACCELEROMETER)
        {
          (void)memcpy(last_data_xl, fifo_out_slot[j].sensor_data.raw_data, 6);
          last_timestamp_xl = timestamp;
          bdr_chg_xl_flag = 0;
        }

        if (sensor_type == ST_FIFO_GYROSCOPE)
        {
          (void)memcpy(last_data_gy, fifo_out_slot[j].sensor_data.raw_data, 6);
          last_timestamp_gy = timestamp;
          bdr_chg_gy_flag = 0;
        }

        j++;

      }
      else if (compression_type == ST_FIFO_COMPRESSION_NC_T_1)
      {

        fifo_out_slot[j].sensor_tag = get_sensor_type(tag);
        (void)memcpy(fifo_out_slot[j].sensor_data.raw_data,
                     &fifo_raw_slot[i].fifo_data_out[1], 6);

        if (sensor_type == ST_FIFO_ACCELEROMETER)
        {
          uint32_t last_timestamp;

          if (bdr_chg_xl_flag == 1u)
          {
            last_timestamp = last_timestamp_xl + dtime_xl_old;
          }
          else
          {
            last_timestamp = timestamp - dtime_xl;
          }

          fifo_out_slot[j].timestamp = last_timestamp;
          (void)memcpy(last_data_xl, fifo_out_slot[j].sensor_data.raw_data, 6);
          last_timestamp_xl = last_timestamp;
        }

        if (sensor_type == ST_FIFO_GYROSCOPE)
        {
          uint32_t last_timestamp;

          if (bdr_chg_gy_flag == 1u)
          {
            last_timestamp = last_timestamp_gy + dtime_gy_old;
          }
          else
          {
            last_timestamp = timestamp - dtime_gy;
          }

          fifo_out_slot[j].timestamp = last_timestamp;
          (void)memcpy(last_data_gy, fifo_out_slot[j].sensor_data.raw_data, 6);
          last_timestamp_gy = last_timestamp;
        }

        j++;

      }
      else if (compression_type == ST_FIFO_COMPRESSION_NC_T_2)
      {

        fifo_out_slot[j].sensor_tag = get_sensor_type(tag);
        (void)memcpy(fifo_out_slot[j].sensor_data.raw_data,
                     &fifo_raw_slot[i].fifo_data_out[1], 6);

        if (sensor_type == ST_FIFO_ACCELEROMETER)
        {
          uint32_t last_timestamp;

          if (bdr_chg_xl_flag == 1u)
          {
            last_timestamp = last_timestamp_xl + dtime_xl_old;
          }
          else
          {
            last_timestamp = timestamp - 2u * dtime_xl;
          }

          fifo_out_slot[j].timestamp = last_timestamp;
          (void)memcpy(last_data_xl, fifo_out_slot[j].sensor_data.raw_data, 6);
          last_timestamp_xl = last_timestamp;
        }
        if (sensor_type == ST_FIFO_GYROSCOPE)
        {
          uint32_t last_timestamp;

          if (bdr_chg_gy_flag == 1u)
          {
            last_timestamp = last_timestamp_gy + dtime_gy_old;
          }
          else
          {
            last_timestamp = timestamp - 2u * dtime_gy;
          }

          fifo_out_slot[j].timestamp = last_timestamp;
          (void)memcpy(last_data_gy, fifo_out_slot[j].sensor_data.raw_data, 6);
          last_timestamp_gy = last_timestamp;
        }

        j++;

      }
      else if (compression_type == ST_FIFO_COMPRESSION_2X)
      {

        int16_t diff[6];
        get_diff_2x(diff, &fifo_raw_slot[i].fifo_data_out[1]);

        fifo_out_slot[j].sensor_tag = sensor_type;

        if (sensor_type == ST_FIFO_ACCELEROMETER)
        {
          fifo_out_slot[j].sensor_data.data[0] = last_data_xl[0] + diff[0];
          fifo_out_slot[j].sensor_data.data[1] = last_data_xl[1] + diff[1];
          fifo_out_slot[j].sensor_data.data[2] = last_data_xl[2] + diff[2];
          fifo_out_slot[j].timestamp = timestamp - 2u * dtime_xl;
          (void)memcpy(last_data_xl, fifo_out_slot[j].sensor_data.raw_data, 6);
        }

        if (sensor_type == ST_FIFO_GYROSCOPE)
        {
          fifo_out_slot[j].sensor_data.data[0] = last_data_gy[0] + diff[0];
          fifo_out_slot[j].sensor_data.data[1] = last_data_gy[1] + diff[1];
          fifo_out_slot[j].sensor_data.data[2] = last_data_gy[2] + diff[2];
          fifo_out_slot[j].timestamp = timestamp - 2u * dtime_gy;
          (void)memcpy(last_data_gy, fifo_out_slot[j].sensor_data.raw_data, 6);
        }

        j++;

        fifo_out_slot[j].sensor_tag = sensor_type;

        if (sensor_type == ST_FIFO_ACCELEROMETER)
        {
          uint32_t last_timestamp = timestamp - dtime_xl;
          fifo_out_slot[j].sensor_data.data[0] = last_data_xl[0] + diff[3];
          fifo_out_slot[j].sensor_data.data[1] = last_data_xl[1] + diff[4];
          fifo_out_slot[j].sensor_data.data[2] = last_data_xl[2] + diff[5];
          fifo_out_slot[j].timestamp = last_timestamp;
          (void)memcpy(last_data_xl, fifo_out_slot[j].sensor_data.raw_data, 6);
          last_timestamp_xl = last_timestamp;
        }

        if (sensor_type == ST_FIFO_GYROSCOPE)
        {
          uint32_t last_timestamp = timestamp - dtime_gy;
          fifo_out_slot[j].sensor_data.data[0] = last_data_gy[0] + diff[3];
          fifo_out_slot[j].sensor_data.data[1] = last_data_gy[1] + diff[4];
          fifo_out_slot[j].sensor_data.data[2] = last_data_gy[2] + diff[5];
          fifo_out_slot[j].timestamp = last_timestamp;
          (void)memcpy(last_data_gy, fifo_out_slot[j].sensor_data.raw_data, 6);
          last_timestamp_gy = last_timestamp;
        }

        j++;

      }
      else /* compression_type == ST_FIFO_COMPRESSION_3X */
      {
        int16_t diff[9];
        get_diff_3x(diff, &fifo_raw_slot[i].fifo_data_out[1]);

        fifo_out_slot[j].sensor_tag = sensor_type;

        if (sensor_type == ST_FIFO_ACCELEROMETER)
        {
          fifo_out_slot[j].sensor_data.data[0] = last_data_xl[0] + diff[0];
          fifo_out_slot[j].sensor_data.data[1] = last_data_xl[1] + diff[1];
          fifo_out_slot[j].sensor_data.data[2] = last_data_xl[2] + diff[2];
          fifo_out_slot[j].timestamp = timestamp - 2u * dtime_xl;
          (void)memcpy(last_data_xl, fifo_out_slot[j].sensor_data.raw_data, 6);
        }

        if (sensor_type == ST_FIFO_GYROSCOPE)
        {
          fifo_out_slot[j].sensor_data.data[0] = last_data_gy[0] + diff[0];
          fifo_out_slot[j].sensor_data.data[1] = last_data_gy[1] + diff[1];
          fifo_out_slot[j].sensor_data.data[2] = last_data_gy[2] + diff[2];
          fifo_out_slot[j].timestamp = timestamp - 2u * dtime_gy;
          (void)memcpy(last_data_gy, fifo_out_slot[j].sensor_data.raw_data, 6);
        }

        j++;

        fifo_out_slot[j].sensor_tag = sensor_type;

        if (sensor_type == ST_FIFO_ACCELEROMETER)
        {
          fifo_out_slot[j].sensor_data.data[0] = last_data_xl[0] + diff[3];
          fifo_out_slot[j].sensor_data.data[1] = last_data_xl[1] + diff[4];
          fifo_out_slot[j].sensor_data.data[2] = last_data_xl[2] + diff[5];
          fifo_out_slot[j].timestamp = timestamp - dtime_xl;
          (void)memcpy(last_data_xl, fifo_out_slot[j].sensor_data.raw_data, 6);
        }

        if (sensor_type == ST_FIFO_GYROSCOPE)
        {
          fifo_out_slot[j].sensor_data.data[0] = last_data_gy[0] + diff[3];
          fifo_out_slot[j].sensor_data.data[1] = last_data_gy[1] + diff[4];
          fifo_out_slot[j].sensor_data.data[2] = last_data_gy[2] + diff[5];
          fifo_out_slot[j].timestamp = timestamp - dtime_gy;
          (void)memcpy(last_data_gy, fifo_out_slot[j].sensor_data.raw_data, 6);
        }

        j++;

        fifo_out_slot[j].timestamp = timestamp;
        fifo_out_slot[j].sensor_tag = sensor_type;

        if (sensor_type == ST_FIFO_ACCELEROMETER)
        {
          fifo_out_slot[j].sensor_data.data[0] = last_data_xl[0] + diff[6];
          fifo_out_slot[j].sensor_data.data[1] = last_data_xl[1] + diff[7];
          fifo_out_slot[j].sensor_data.data[2] = last_data_xl[2] + diff[8];
          (void)memcpy(last_data_xl, fifo_out_slot[j].sensor_data.raw_data, 6);
          last_timestamp_xl = timestamp;
        }

        if (sensor_type == ST_FIFO_GYROSCOPE)
        {
          fifo_out_slot[j].sensor_data.data[0] = last_data_gy[0] + diff[6];
          fifo_out_slot[j].sensor_data.data[1] = last_data_gy[1] + diff[7];
          fifo_out_slot[j].sensor_data.data[2] = last_data_gy[2] + diff[8];
          (void)memcpy(last_data_gy, fifo_out_slot[j].sensor_data.raw_data, 6);
          last_timestamp_gy = timestamp;
        }

        j++;
      }

      *out_slot_size = j;
    }

    tag_counter_old = tag_counter;
  }

  return ST_FIFO_OK;
}

/**
  * @brief  Sort FIFO stream from older to newer timestamp.
  *
  * @param  fifo_out_slot     decoded output stream to sort.(ptr)
  * @param  out_slot_size     decoded stream size.
  *
  */
void st_fifo_sort(st_fifo_out_slot *fifo_out_slot, uint16_t out_slot_size)
{
  int32_t i;
  int32_t j;
  st_fifo_out_slot temp;

  for (i = 1; i < (int32_t)out_slot_size; i++)
  {
    (void)memcpy(&temp, &fifo_out_slot[i], sizeof(st_fifo_out_slot));

    j = i - 1;

    while (j >= 0 && fifo_out_slot[j].timestamp > temp.timestamp)
    {
      (void)memcpy(&fifo_out_slot[j + 1], &fifo_out_slot[j],
                   sizeof(st_fifo_out_slot));
      j--;
    }

    (void)memcpy(&fifo_out_slot[j + 1], &temp, sizeof(st_fifo_out_slot));
  }

  return;
}

/**
  * @brief  Return the number of a sensor tag occurrencies in a
  *         decoded FIFO stream.
  *
  * @param  fifo_out_slot     decoded output stream.(ptr)
  * @param  out_slot_size     decoded stream size.
  * @param  sensor_type       sensor type for the number of occurrencies count.
  *
  * @retval uint16_t          the number of a sensor tag occurrencies in a
  *                           decoded FIFO stream.
  *
  */
uint16_t st_fifo_get_sensor_occurrence(st_fifo_out_slot *fifo_out_slot,
                                       uint16_t out_slot_size, st_fifo_sensor_type sensor_type)
{
  uint16_t occurrence = 0;

  for (uint16_t i = 0; i < out_slot_size; i++)
  {
    if (fifo_out_slot[i].sensor_tag == sensor_type)
    {
      occurrence++;
    }
  }

  return occurrence;
}

/**
  * @brief  This function extracts all the data of a specific sensor
  *         from a decoded FIFO stream.
  *
  * @param  sensor_out_slot   data of a specific sensor.(ptr)
  * @param  fifo_out_slot     decoded output stream.(ptr)
  * @param  out_slot_size     decoded stream size.
  * @param  sensor_type       sensor type for the number of occurrencies count.
  *
  */
void st_fifo_extract_sensor(st_fifo_out_slot *sensor_out_slot,
                            st_fifo_out_slot *fifo_out_slot, uint16_t  out_slot_size,
                            st_fifo_sensor_type sensor_type)
{
  uint16_t temp_i = 0;

  for (uint16_t i = 0; i < out_slot_size; i++)
  {
    if (fifo_out_slot[i].sensor_tag == sensor_type)
    {
      (void)memcpy(&sensor_out_slot[temp_i], &fifo_out_slot[i],
                   sizeof(st_fifo_out_slot));
      temp_i++;
    }
  }
}

/**
  * @}
  *
  */

/**
  * @defgroup  FIFO private functions
  * @brief     This section provide a set of private functions
  *            used by the public APIs.
  * @{
  *
  */

/**
  * @brief  This function indicate if a raw tag is valid or not.
  *
  * @param  tag               tag to be analyzed.
  *
  * @retval uint8_t           valid (1) or invalid (0) tag.
  *
  */
static uint8_t is_tag_valid(uint8_t tag)
{
  if (tag > device[fifo_ver].tag_valid_limit)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

/**
  * @brief  This function convert a raw tag in a sensor type.
  *
  * @param  tag                    tag to be analyzed.
  *
  * @retval st_fifo_sensor_type    sensor type.
  *
  */
static st_fifo_sensor_type get_sensor_type(uint8_t tag)
{
  st_fifo_sensor_type type;

  switch (tag)
  {
    case TAG_GY:
      type = ST_FIFO_GYROSCOPE;
      break;
    case TAG_XL:
      type = ST_FIFO_ACCELEROMETER;
      break;
    case TAG_TEMP:
      type = ST_FIFO_TEMPERATURE;
      break;
    case TAG_EXT_SENS_0:
      type = ST_FIFO_EXT_SENSOR0;
      break;
    case TAG_EXT_SENS_1:
      type = ST_FIFO_EXT_SENSOR1;
      break;
    case TAG_EXT_SENS_2:
      type = ST_FIFO_EXT_SENSOR2;
      break;
    case TAG_EXT_SENS_3:
      type = ST_FIFO_EXT_SENSOR3;
      break;
    case TAG_STEP_COUNTER:
      type = ST_FIFO_STEP_COUNTER;
      break;
    case TAG_XL_UNCOMPRESSED_T_2:
      type = ST_FIFO_ACCELEROMETER;
      break;
    case TAG_XL_UNCOMPRESSED_T_1:
      type = ST_FIFO_ACCELEROMETER;
      break;
    case TAG_XL_COMPRESSED_2X:
      type = ST_FIFO_ACCELEROMETER;
      break;
    case TAG_XL_COMPRESSED_3X:
      type = ST_FIFO_ACCELEROMETER;
      break;
    case TAG_GY_UNCOMPRESSED_T_2:
      type = ST_FIFO_GYROSCOPE;
      break;
    case TAG_GY_UNCOMPRESSED_T_1:
      type = ST_FIFO_GYROSCOPE;
      break;
    case TAG_GY_COMPRESSED_2X:
      type = ST_FIFO_GYROSCOPE;
      break;
    case TAG_GY_COMPRESSED_3X:
      type = ST_FIFO_GYROSCOPE;
      break;
    case TAG_GAME_RV:
      type = ST_FIFO_6X_GAME_RV;
      break;
    case TAG_GEOM_RV:
      type = ST_FIFO_6X_GEOM_RV;
      break;
    case TAG_NORM_RV:
      type = ST_FIFO_9X_RV;
      break;
    case TAG_GYRO_BIAS:
      type = ST_FIFO_GYRO_BIAS;
      break;
    case TAG_GRAVITIY:
      type = ST_FIFO_GRAVITY;
      break;
    case TAG_MAG_CAL:
      type = ST_FIFO_MAGNETOMETER_CALIB;
      break;
    case TAG_EXT_SENS_NACK:
      type = ST_FIFO_EXT_SENSOR_NACK;
      break;
    case TAG_MLC_RESULT:
      type = ST_FIFO_MLC_RESULT;
      break;
    case TAG_MLC_FILTER:
      type = ST_FIFO_MLC_FILTER;
      break;
    case TAG_MLC_FEATURE:
      type = ST_FIFO_MLC_FEATURE;
      break;
    case TAG_DUALC_XL:
      type = ST_FIFO_DUALC_ACCELEROMETER;
      break;
    case TAG_EIS_GY:
      type = ST_FIFO_EIS_GYROSCOPE;
      break;
    default:
      type = ST_FIFO_NONE;
      break;
  }

  return type;
}

/**
  * @brief  This function convert a raw tag in a type of compression.
  *
  * @param  tag                         tag to be analyzed.
  *
  * @retval st_fifo_compression_type    compression type.
  *
  */
static st_fifo_compression_type get_compression_type(uint8_t tag)
{
  st_fifo_compression_type type;

  switch (tag)
  {
    case TAG_XL_UNCOMPRESSED_T_2:
      type = ST_FIFO_COMPRESSION_NC_T_2;
      break;
    case TAG_XL_UNCOMPRESSED_T_1:
      type = ST_FIFO_COMPRESSION_NC_T_1;
      break;
    case TAG_XL_COMPRESSED_2X:
      type = ST_FIFO_COMPRESSION_2X;
      break;
    case TAG_XL_COMPRESSED_3X:
      type = ST_FIFO_COMPRESSION_3X;
      break;
    case TAG_GY_UNCOMPRESSED_T_2:
      type = ST_FIFO_COMPRESSION_NC_T_2;
      break;
    case TAG_GY_UNCOMPRESSED_T_1:
      type = ST_FIFO_COMPRESSION_NC_T_1;
      break;
    case TAG_GY_COMPRESSED_2X:
      type = ST_FIFO_COMPRESSION_2X;
      break;
    case TAG_GY_COMPRESSED_3X:
      type = ST_FIFO_COMPRESSION_3X;
      break;
    default:
      type = ST_FIFO_COMPRESSION_NC;
      break;
  }

  return type;
}

/**
  * @brief  This function get the index of the nearest BDR.
  *
  * @param  bdr               array with the BDR values.(ptr)
  * @param  n                 input value to be considered
  *
  * @retval uint8_t           index of the nearest BDR.
  *
  */
static uint8_t bdr_get_index(const float *bdr, float n)
{
  float diff[16], min = FLT_MAX;
  uint8_t idx = 0;

  for (uint8_t i = 0; i < 16u; i++)
  {
    diff[i] = fabsf(bdr[i] - n);
  }

  for (uint8_t i = 0; i < 16u; i++)
  {
    if (diff[i] < min)
    {
      min = diff[i];
      idx = i;
    }
  }

  return idx;
}

/**
  * @brief  This function check the parity of a byte.
  *
  * @param  x                 byte to be analyzed.
  *
  * @retval uint8_t           the byte has even parity (1) or not (0).
  *
  */
static uint8_t has_even_parity(uint8_t x)
{
  uint8_t count = 0x00, b = 0x01;

  for (uint8_t i = 0; i < 8u; i++)
  {
    if ((x & (b << i)) != 0x00u)
    {
      count++;
    }
  }

  if ((count & 0x01u) == 0x01u)
  {
    return 0;
  }

  return 1;
}

/**
  * @brief  Convert raw data FIFO into compressed data (2x).
  *
  * @param  diff[6]           compressed data (2x).
  * @param  input[6]          FIFO raw word without tag.
  *
  */
static void get_diff_2x(int16_t diff[6], uint8_t input[6])
{
  for (uint8_t i = 0; i < 6u; i++)
  {
    diff[i] = input[i] < 128u ? (int16_t)input[i] : (int16_t)input[i] - 256;
  }
}

/**
  * @brief  Convert raw data FIFO into compressed data (3x).
  *
  * @param  diff[6]           compressed data (3x).
  * @param  input[6]          FIFO raw word without tag.
  *
  */
static void get_diff_3x(int16_t diff[9], uint8_t input[6])
{
  uint16_t decode_tmp;

  for (uint8_t i = 0; i < 3u; i++)
  {

    (void)memcpy(&decode_tmp, &input[2u * i], 2);

    for (uint8_t j = 0; j < 3u; j++)
    {
      uint16_t utmp = (decode_tmp & ((uint16_t)0x1Fu << (5u * j))) >> (5u * j);
      int16_t tmp = (int16_t)utmp;
      diff[j + 3u * i] = tmp < 16 ? tmp : (tmp - 32);
    }
  }
}

/**
  * @}
  *
  */

/**
  * @}
  *
  */
