/**
  ******************************************************************************
  * @file    lis3dhh_reg.h
  * @author  Sensors Software Solution Team
  * @brief   This file contains all the functions prototypes for the
  *          lis3dhh_reg.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#ifndef LIS3DHH_REGS_H
#define LIS3DHH_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/** @addtogroup LIS3DHH
  * @{
  *
  */

/** @defgroup  Endianness definitions
  * @{
  *
  */

#ifndef DRV_BYTE_ORDER
#ifndef __BYTE_ORDER__

#define DRV_LITTLE_ENDIAN 1234
#define DRV_BIG_ENDIAN    4321

/** if _BYTE_ORDER is not defined, choose the endianness of your architecture
  * by uncommenting the define which fits your platform endianness
  */
//#define DRV_BYTE_ORDER    DRV_BIG_ENDIAN
#define DRV_BYTE_ORDER    DRV_LITTLE_ENDIAN

#else /* defined __BYTE_ORDER__ */

#define DRV_LITTLE_ENDIAN  __ORDER_LITTLE_ENDIAN__
#define DRV_BIG_ENDIAN     __ORDER_BIG_ENDIAN__
#define DRV_BYTE_ORDER     __BYTE_ORDER__

#endif /* __BYTE_ORDER__*/
#endif /* DRV_BYTE_ORDER */

/**
  * @}
  *
  */

/** @defgroup STMicroelectronics sensors common types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7       : 1;
  uint8_t bit6       : 1;
  uint8_t bit5       : 1;
  uint8_t bit4       : 1;
  uint8_t bit3       : 1;
  uint8_t bit2       : 1;
  uint8_t bit1       : 1;
  uint8_t bit0       : 1;
#endif /* DRV_BYTE_ORDER */
} bitwise_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);

typedef struct
{
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

/**
  * @}
  *
  */

#endif /* MEMS_SHARED_TYPES */

#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES

/** @defgroup    Generic address-data structure definition
  * @brief       This structure is useful to load a predefined configuration
  *              of a sensor.
  *              You can create a sensor configuration by your own or using
  *              Unico / Unicleo tools available on STMicroelectronics
  *              web site.
  *
  * @{
  *
  */

typedef struct
{
  uint8_t address;
  uint8_t data;
} ucf_line_t;

/**
  * @}
  *
  */

#endif /* MEMS_UCF_SHARED_TYPES */

/**
  * @}
  *
  */

/** @defgroup LIS3DHH_Infos
  * @{
  *
  */

/** Device Identification (Who am I) **/
#define LIS3DHH_ID            0x11U

/**
  * @}
  *
  */

#define LIS3DHH_WHO_AM_I      0x0FU
#define LIS3DHH_CTRL_REG1     0x20U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bdu              : 1;
  uint8_t drdy_pulse       : 1;
  uint8_t sw_reset         : 1;
  uint8_t boot             : 1;
  uint8_t not_used_01      : 2;
  uint8_t if_add_inc       : 1;
  uint8_t norm_mod_en      : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t norm_mod_en      : 1;
  uint8_t if_add_inc       : 1;
  uint8_t not_used_01      : 2;
  uint8_t boot             : 1;
  uint8_t sw_reset         : 1;
  uint8_t drdy_pulse       : 1;
  uint8_t bdu              : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dhh_ctrl_reg1_t;

#define LIS3DHH_INT1_CTRL     0x21U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01      : 2;
  uint8_t int1_ext         : 1;
  uint8_t int1_fth         : 1;
  uint8_t int1_fss5        : 1;
  uint8_t int1_ovr         : 1;
  uint8_t int1_boot        : 1;
  uint8_t int1_drdy        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_drdy        : 1;
  uint8_t int1_boot        : 1;
  uint8_t int1_ovr         : 1;
  uint8_t int1_fss5        : 1;
  uint8_t int1_fth         : 1;
  uint8_t int1_ext         : 1;
  uint8_t not_used_01      : 2;
#endif /* DRV_BYTE_ORDER */
} lis3dhh_int1_ctrl_t;

#define LIS3DHH_INT2_CTRL     0x22U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01      : 3;
  uint8_t int2_fth         : 1;
  uint8_t int2_fss5        : 1;
  uint8_t int2_ovr         : 1;
  uint8_t int2_boot        : 1;
  uint8_t int2_drdy        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_drdy        : 1;
  uint8_t int2_boot        : 1;
  uint8_t int2_ovr         : 1;
  uint8_t int2_fss5        : 1;
  uint8_t int2_fth         : 1;
  uint8_t not_used_01      : 3;
#endif /* DRV_BYTE_ORDER */
} lis3dhh_int2_ctrl_t;

#define LIS3DHH_CTRL_REG4     0x23U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01      : 1;
  uint8_t fifo_en          : 1;
  uint8_t pp_od            : 2;
  uint8_t st               : 2;
  uint8_t dsp              : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t dsp              : 2;
  uint8_t st               : 2;
  uint8_t pp_od            : 2;
  uint8_t fifo_en          : 1;
  uint8_t not_used_01      : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dhh_ctrl_reg4_t;

#define LIS3DHH_CTRL_REG5     0x24U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fifo_spi_hs_on   : 1;
  uint8_t not_used_01      : 7;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01      : 7;
  uint8_t fifo_spi_hs_on   : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dhh_ctrl_reg5_t;

#define LIS3DHH_OUT_TEMP_L    0x25U
#define LIS3DHH_OUT_TEMP_H    0x26U
#define LIS3DHH_STATUS        0x27U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xda              : 1;
  uint8_t yda              : 1;
  uint8_t zda              : 1;
  uint8_t zyxda            : 1;
  uint8_t _xor             : 1;
  uint8_t yor              : 1;
  uint8_t zor              : 1;
  uint8_t zyxor            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t zyxor            : 1;
  uint8_t zor              : 1;
  uint8_t yor              : 1;
  uint8_t _xor             : 1;
  uint8_t zyxda            : 1;
  uint8_t zda              : 1;
  uint8_t yda              : 1;
  uint8_t xda              : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dhh_status_t;

#define LIS3DHH_OUT_X_L_XL    0x28U
#define LIS3DHH_OUT_X_H_XL    0x29U
#define LIS3DHH_OUT_Y_L_XL    0x2AU
#define LIS3DHH_OUT_Y_H_XL    0x2BU
#define LIS3DHH_OUT_Z_L_XL    0x2CU
#define LIS3DHH_OUT_Z_H_XL    0x2DU
#define LIS3DHH_FIFO_CTRL     0x2EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fth              : 5;
  uint8_t fmode            : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fmode            : 3;
  uint8_t fth              : 5;
#endif /* DRV_BYTE_ORDER */
} lis3dhh_fifo_ctrl_t;

#define LIS3DHH_FIFO_SRC      0x2FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fss              : 6;
  uint8_t ovrn             : 1;
  uint8_t fth              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fth              : 1;
  uint8_t ovrn             : 1;
  uint8_t fss              : 6;
#endif /* DRV_BYTE_ORDER */
} lis3dhh_fifo_src_t;

/**
  * @defgroup LIS3DHH_Register_Union
  * @brief    This union group all the registers having a bit-field
  *           description.
  *           This union is useful but it's not needed by the driver.
  *
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
  */
typedef union
{
  lis3dhh_ctrl_reg1_t    ctrl_reg1;
  lis3dhh_int1_ctrl_t    int1_ctrl;
  lis3dhh_int2_ctrl_t    int2_ctrl;
  lis3dhh_ctrl_reg4_t    ctrl_reg4;
  lis3dhh_ctrl_reg5_t    ctrl_reg5;
  lis3dhh_status_t       status;
  lis3dhh_fifo_ctrl_t    fifo_ctrl;
  lis3dhh_fifo_src_t     fifo_src;
  bitwise_t              bitwise;
  uint8_t                byte;
} lis3dhh_reg_t;

/**
  * @}
  *
  */

int32_t lis3dhh_read_reg(stmdev_ctx_t *ctx, uint8_t reg,
                         uint8_t *data,
                         uint16_t len);
int32_t lis3dhh_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
                          uint8_t *data,
                          uint16_t len);

float_t lis3dhh_from_lsb_to_mg(int16_t lsb);
float_t lis3dhh_from_lsb_to_celsius(int16_t lsb);

int32_t lis3dhh_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dhh_block_data_update_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef enum
{
  LIS3DHH_POWER_DOWN  = 0,
  LIS3DHH_1kHz1       = 1,
} lis3dhh_norm_mod_en_t;
int32_t lis3dhh_data_rate_set(stmdev_ctx_t *ctx,
                              lis3dhh_norm_mod_en_t val);
int32_t lis3dhh_data_rate_get(stmdev_ctx_t *ctx,
                              lis3dhh_norm_mod_en_t *val);

int32_t lis3dhh_temperature_raw_get(stmdev_ctx_t *ctx, int16_t *val);
int32_t lis3dhh_acceleration_raw_get(stmdev_ctx_t *ctx, int16_t *val);

int32_t lis3dhh_xl_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dhh_xl_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dhh_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lis3dhh_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dhh_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dhh_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dhh_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LIS3DHH_ST_DISABLE   = 0,
  LIS3DHH_ST_POSITIVE  = 1,
  LIS3DHH_ST_NEGATIVE  = 2,
} lis3dhh_st_t;
int32_t lis3dhh_self_test_set(stmdev_ctx_t *ctx, lis3dhh_st_t val);
int32_t lis3dhh_self_test_get(stmdev_ctx_t *ctx, lis3dhh_st_t *val);

typedef enum
{
  LIS3DHH_LINEAR_PHASE_440Hz      = 0,
  LIS3DHH_LINEAR_PHASE_235Hz      = 1,
  LIS3DHH_NO_LINEAR_PHASE_440Hz   = 2,
  LIS3DHH_NO_LINEAR_PHASE_235Hz   = 3,
} lis3dhh_dsp_t;
int32_t lis3dhh_filter_config_set(stmdev_ctx_t *ctx,
                                  lis3dhh_dsp_t val);
int32_t lis3dhh_filter_config_get(stmdev_ctx_t *ctx,
                                  lis3dhh_dsp_t *val);

int32_t lis3dhh_status_get(stmdev_ctx_t *ctx, lis3dhh_status_t *val);

typedef enum
{
  LIS3DHH_LATCHED  = 0,
  LIS3DHH_PULSED   = 1,
} lis3dhh_drdy_pulse_t;
int32_t lis3dhh_drdy_notification_mode_set(stmdev_ctx_t *ctx,
                                           lis3dhh_drdy_pulse_t val);
int32_t lis3dhh_drdy_notification_mode_get(stmdev_ctx_t *ctx,
                                           lis3dhh_drdy_pulse_t *val);


typedef enum
{
  LIS3DHH_PIN_AS_INTERRUPT   = 0,
  LIS3DHH_PIN_AS_TRIGGER     = 1,
} lis3dhh_int1_ext_t;
int32_t lis3dhh_int1_mode_set(stmdev_ctx_t *ctx,
                              lis3dhh_int1_ext_t val);
int32_t lis3dhh_int1_mode_get(stmdev_ctx_t *ctx,
                              lis3dhh_int1_ext_t *val);


int32_t lis3dhh_fifo_threshold_on_int1_set(stmdev_ctx_t *ctx,
                                           uint8_t val);
int32_t lis3dhh_fifo_threshold_on_int1_get(stmdev_ctx_t *ctx,
                                           uint8_t *val);

int32_t lis3dhh_fifo_full_on_int1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dhh_fifo_full_on_int1_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lis3dhh_fifo_ovr_on_int1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dhh_fifo_ovr_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dhh_boot_on_int1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dhh_boot_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dhh_drdy_on_int1_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dhh_drdy_on_int1_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dhh_fifo_threshold_on_int2_set(stmdev_ctx_t *ctx,
                                           uint8_t val);
int32_t lis3dhh_fifo_threshold_on_int2_get(stmdev_ctx_t *ctx,
                                           uint8_t *val);

int32_t lis3dhh_fifo_full_on_int2_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dhh_fifo_full_on_int2_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lis3dhh_fifo_ovr_on_int2_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dhh_fifo_ovr_on_int2_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dhh_boot_on_int2_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dhh_boot_on_int2_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dhh_drdy_on_int2_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dhh_drdy_on_int2_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LIS3DHH_ALL_PUSH_PULL    = 0,
  LIS3DHH_INT1_OD_INT2_PP  = 1,
  LIS3DHH_INT1_PP_INT2_OD  = 2,
  LIS3DHH_ALL_OPEN_DRAIN   = 3,
} lis3dhh_pp_od_t;
int32_t lis3dhh_pin_mode_set(stmdev_ctx_t *ctx, lis3dhh_pp_od_t val);
int32_t lis3dhh_pin_mode_get(stmdev_ctx_t *ctx, lis3dhh_pp_od_t *val);

int32_t lis3dhh_fifo_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dhh_fifo_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dhh_fifo_block_spi_hs_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dhh_fifo_block_spi_hs_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lis3dhh_fifo_watermark_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dhh_fifo_watermark_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LIS3DHH_BYPASS_MODE             = 0,
  LIS3DHH_FIFO_MODE               = 1,
  LIS3DHH_STREAM_TO_FIFO_MODE     = 3,
  LIS3DHH_BYPASS_TO_STREAM_MODE   = 4,
  LIS3DHH_DYNAMIC_STREAM_MODE     = 6,
} lis3dhh_fmode_t;
int32_t lis3dhh_fifo_mode_set(stmdev_ctx_t *ctx, lis3dhh_fmode_t val);
int32_t lis3dhh_fifo_mode_get(stmdev_ctx_t *ctx,
                              lis3dhh_fmode_t *val);

int32_t lis3dhh_fifo_status_get(stmdev_ctx_t *ctx,
                                lis3dhh_fifo_src_t *val);

int32_t lis3dhh_fifo_full_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dhh_fifo_ovr_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dhh_fifo_fth_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dhh_auto_add_inc_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dhh_auto_add_inc_get(stmdev_ctx_t *ctx, uint8_t *val);

/**
  *@}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* LIS3DHH_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
