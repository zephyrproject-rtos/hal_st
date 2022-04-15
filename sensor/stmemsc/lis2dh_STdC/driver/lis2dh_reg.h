/**
  ******************************************************************************
  * @file    lis2dh_reg.h
  * @author  Sensors Software Solution Team
  * @brief   This file contains all the functions prototypes for the
  *          lis2dh_reg.c driver.
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
#ifndef LIS2DH_REGS_H
#define LIS2DH_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/** @addtogroup LIS2DH
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

/** @defgroup LIS2DH_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format if SA0=0 -> 31 if SA0=1 -> 33 **/
#define LIS2DH_I2C_ADD_L   0x31U
#define LIS2DH_I2C_ADD_H   0x33U

/** Device Identification (Who am I) **/
#define LIS2DH_ID          0x33U

/**
  * @}
  *
  */

#define LIS2DH_STATUS_REG_AUX        0x07U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01       : 2;
  uint8_t tda               : 1;
  uint8_t not_used_02       : 3;
  uint8_t tor               : 1;
  uint8_t not_used_03       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_03       : 1;
  uint8_t tor               : 1;
  uint8_t not_used_02       : 3;
  uint8_t tda               : 1;
  uint8_t not_used_01       : 2;
#endif /* DRV_BYTE_ORDER */
} lis2dh_status_reg_aux_t;

#define LIS2DH_OUT_TEMP_L            0x0CU
#define LIS2DH_OUT_TEMP_H            0x0DU
#define LIS2DH_INT_COUNTER           0x0EU
#define LIS2DH_WHO_AM_I              0x0FU
#define LIS2DH_TEMP_CFG_REG          0x1FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01       : 6;
  uint8_t temp_en           : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t temp_en           : 2;
  uint8_t not_used_01       : 6;
#endif /* DRV_BYTE_ORDER */
} lis2dh_temp_cfg_reg_t;

#define LIS2DH_CTRL_REG1             0x20U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xen               : 1;
  uint8_t yen               : 1;
  uint8_t zen               : 1;
  uint8_t lpen              : 1;
  uint8_t odr               : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr               : 4;
  uint8_t lpen              : 1;
  uint8_t zen               : 1;
  uint8_t yen               : 1;
  uint8_t xen               : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dh_ctrl_reg1_t;

#define LIS2DH_CTRL_REG2             0x21U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t hp                : 3; /* HPCLICK + HPIS[1:2] -> HP */
  uint8_t fds               : 1;
  uint8_t hpcf              : 2;
  uint8_t hpm               : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t hpm               : 2;
  uint8_t hpcf              : 2;
  uint8_t fds               : 1;
  uint8_t hp                : 3; /* HPCLICK + HPIS[1:2] -> HP */
#endif /* DRV_BYTE_ORDER */
} lis2dh_ctrl_reg2_t;

#define LIS2DH_CTRL_REG3             0x22U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t i1_overrun        : 1;
  uint8_t i1_wtm            : 1;
  uint8_t i1_drdy2          : 1;
  uint8_t i1_drdy1          : 1;
  uint8_t i1_aoi2           : 1;
  uint8_t i1_aoi1           : 1;
  uint8_t i1_click          : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t i1_click          : 1;
  uint8_t i1_aoi1           : 1;
  uint8_t i1_aoi2           : 1;
  uint8_t i1_drdy1          : 1;
  uint8_t i1_drdy2          : 1;
  uint8_t i1_wtm            : 1;
  uint8_t i1_overrun        : 1;
  uint8_t not_used_01       : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dh_ctrl_reg3_t;

#define LIS2DH_CTRL_REG4             0x23U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sim               : 1;
  uint8_t st                : 2;
  uint8_t hr                : 1;
  uint8_t fs                : 2;
  uint8_t ble               : 1;
  uint8_t bdu               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bdu               : 1;
  uint8_t ble               : 1;
  uint8_t fs                : 2;
  uint8_t hr                : 1;
  uint8_t st                : 2;
  uint8_t sim               : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dh_ctrl_reg4_t;

#define LIS2DH_CTRL_REG5             0x24U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t d4d_int2          : 1;
  uint8_t lir_int2          : 1;
  uint8_t d4d_int1          : 1;
  uint8_t lir_int1          : 1;
  uint8_t not_used_01       : 2;
  uint8_t fifo_en           : 1;
  uint8_t boot              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot              : 1;
  uint8_t fifo_en           : 1;
  uint8_t not_used_01       : 2;
  uint8_t lir_int1          : 1;
  uint8_t d4d_int1          : 1;
  uint8_t lir_int2          : 1;
  uint8_t d4d_int2          : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dh_ctrl_reg5_t;

#define LIS2DH_CTRL_REG6            0x25U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t h_lactive         : 1;
  uint8_t not_used_02       : 1;
  uint8_t p2_act            : 1;
  uint8_t boot_i2           : 1;
  uint8_t i2_int2           : 1;
  uint8_t i2_int1           : 1;
  uint8_t i2_clicken        : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t i2_clicken        : 1;
  uint8_t i2_int1           : 1;
  uint8_t i2_int2           : 1;
  uint8_t boot_i2           : 1;
  uint8_t p2_act            : 1;
  uint8_t not_used_02       : 1;
  uint8_t h_lactive         : 1;
  uint8_t not_used_01       : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dh_ctrl_reg6_t;

#define LIS2DH_REFERENCE            0x26U
#define LIS2DH_STATUS_REG           0x27U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xda               : 1;
  uint8_t yda               : 1;
  uint8_t zda               : 1;
  uint8_t zyxda             : 1;
  uint8_t _xor              : 1;
  uint8_t yor               : 1;
  uint8_t zor               : 1;
  uint8_t zyxor             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t zyxor             : 1;
  uint8_t zor               : 1;
  uint8_t yor               : 1;
  uint8_t _xor              : 1;
  uint8_t zyxda             : 1;
  uint8_t zda               : 1;
  uint8_t yda               : 1;
  uint8_t xda               : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dh_status_reg_t;

#define LIS2DH_OUT_X_L              0x28U
#define LIS2DH_OUT_X_H              0x29U
#define LIS2DH_OUT_Y_L              0x2AU
#define LIS2DH_OUT_Y_H              0x2BU
#define LIS2DH_OUT_Z_L              0x2CU
#define LIS2DH_OUT_Z_H              0x2DU
#define LIS2DH_FIFO_CTRL_REG        0x2EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fth               : 5;
  uint8_t tr                : 1;
  uint8_t fm                : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fm                : 2;
  uint8_t tr                : 1;
  uint8_t fth               : 5;
#endif /* DRV_BYTE_ORDER */
} lis2dh_fifo_ctrl_reg_t;

#define LIS2DH_FIFO_SRC_REG         0x2FU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fss               : 5;
  uint8_t empty             : 1;
  uint8_t ovrn_fifo         : 1;
  uint8_t wtm               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t wtm               : 1;
  uint8_t ovrn_fifo         : 1;
  uint8_t empty             : 1;
  uint8_t fss               : 5;
#endif /* DRV_BYTE_ORDER */
} lis2dh_fifo_src_reg_t;

#define LIS2DH_INT1_CFG             0x30U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlie              : 1;
  uint8_t xhie              : 1;
  uint8_t ylie              : 1;
  uint8_t yhie              : 1;
  uint8_t zlie              : 1;
  uint8_t zhie              : 1;
  uint8_t _6d               : 1;
  uint8_t aoi               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t aoi               : 1;
  uint8_t _6d               : 1;
  uint8_t zhie              : 1;
  uint8_t zlie              : 1;
  uint8_t yhie              : 1;
  uint8_t ylie              : 1;
  uint8_t xhie              : 1;
  uint8_t xlie              : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dh_int1_cfg_t;

#define LIS2DH_INT1_SRC             0x31U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl                : 1;
  uint8_t xh                : 1;
  uint8_t yl                : 1;
  uint8_t yh                : 1;
  uint8_t zl                : 1;
  uint8_t zh                : 1;
  uint8_t ia                : 1;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t ia                : 1;
  uint8_t zh                : 1;
  uint8_t zl                : 1;
  uint8_t yh                : 1;
  uint8_t yl                : 1;
  uint8_t xh                : 1;
  uint8_t xl                : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dh_int1_src_t;

#define LIS2DH_INT1_THS             0x32U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ths               : 7;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t ths               : 7;
#endif /* DRV_BYTE_ORDER */
} lis2dh_int1_ths_t;

#define LIS2DH_INT1_DURATION        0x33U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t d                 : 7;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t d                 : 7;
#endif /* DRV_BYTE_ORDER */
} lis2dh_int1_duration_t;

#define LIS2DH_INT2_CFG             0x34U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlie              : 1;
  uint8_t xhie              : 1;
  uint8_t ylie              : 1;
  uint8_t yhie              : 1;
  uint8_t zlie              : 1;
  uint8_t zhie              : 1;
  uint8_t _6d               : 1;
  uint8_t aoi               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t aoi               : 1;
  uint8_t _6d               : 1;
  uint8_t zhie              : 1;
  uint8_t zlie              : 1;
  uint8_t yhie              : 1;
  uint8_t ylie              : 1;
  uint8_t xhie              : 1;
  uint8_t xlie              : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dh_int2_cfg_t;

#define LIS2DH_INT2_SRC             0x35U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl                : 1;
  uint8_t xh                : 1;
  uint8_t yl                : 1;
  uint8_t yh                : 1;
  uint8_t zl                : 1;
  uint8_t zh                : 1;
  uint8_t ia                : 1;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t ia                : 1;
  uint8_t zh                : 1;
  uint8_t zl                : 1;
  uint8_t yh                : 1;
  uint8_t yl                : 1;
  uint8_t xh                : 1;
  uint8_t xl                : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dh_int2_src_t;

#define LIS2DH_INT2_THS             0x36U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ths               : 7;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t ths               : 7;
#endif /* DRV_BYTE_ORDER */
} lis2dh_int2_ths_t;

#define LIS2DH_INT2_DURATION        0x37U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t d                 : 7;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t d                 : 7;
#endif /* DRV_BYTE_ORDER */
} lis2dh_int2_duration_t;

#define LIS2DH_CLICK_CFG            0x38U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xs                : 1;
  uint8_t xd                : 1;
  uint8_t ys                : 1;
  uint8_t yd                : 1;
  uint8_t zs                : 1;
  uint8_t zd                : 1;
  uint8_t not_used_01       : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 2;
  uint8_t zd                : 1;
  uint8_t zs                : 1;
  uint8_t yd                : 1;
  uint8_t ys                : 1;
  uint8_t xd                : 1;
  uint8_t xs                : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dh_click_cfg_t;

#define LIS2DH_CLICK_SRC            0x39U
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t x                 : 1;
  uint8_t y                 : 1;
  uint8_t z                 : 1;
  uint8_t sign              : 1;
  uint8_t sclick            : 1;
  uint8_t dclick            : 1;
  uint8_t ia                : 1;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t ia                : 1;
  uint8_t dclick            : 1;
  uint8_t sclick            : 1;
  uint8_t sign              : 1;
  uint8_t z                 : 1;
  uint8_t y                 : 1;
  uint8_t x                 : 1;
#endif /* DRV_BYTE_ORDER */
} lis2dh_click_src_t;

#define LIS2DH_CLICK_THS            0x3AU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ths               : 7;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t ths               : 7;
#endif /* DRV_BYTE_ORDER */
} lis2dh_click_ths_t;

#define LIS2DH_TIME_LIMIT           0x3BU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tli               : 7;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t tli               : 7;
#endif /* DRV_BYTE_ORDER */
} lis2dh_time_limit_t;

#define LIS2DH_TIME_LATENCY         0x3CU
typedef struct
{
  uint8_t tla               : 8;
} lis2dh_time_latency_t;

#define LIS2DH_TIME_WINDOW          0x3DU
typedef struct
{
  uint8_t tw                : 8;
} lis2dh_time_window_t;

#define LIS2DH_ACT_THS              0x3EU
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t acth              : 7;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t acth              : 7;
#endif /* DRV_BYTE_ORDER */
} lis2dh_act_ths_t;

#define LIS2DH_ACT_DUR              0x3FU
typedef struct
{
  uint8_t actd              : 8;
} lis2dh_act_dur_t;

/**
  * @defgroup LIS2DH_Register_Union
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
  lis2dh_status_reg_aux_t status_reg_aux;
  lis2dh_temp_cfg_reg_t   temp_cfg_reg;
  lis2dh_ctrl_reg1_t      ctrl_reg1;
  lis2dh_ctrl_reg2_t      ctrl_reg2;
  lis2dh_ctrl_reg3_t      ctrl_reg3;
  lis2dh_ctrl_reg4_t      ctrl_reg4;
  lis2dh_ctrl_reg5_t      ctrl_reg5;
  lis2dh_ctrl_reg6_t      ctrl_reg6;
  lis2dh_status_reg_t     status_reg;
  lis2dh_fifo_ctrl_reg_t  fifo_ctrl_reg;
  lis2dh_fifo_src_reg_t   fifo_src_reg;
  lis2dh_int1_cfg_t       int1_cfg;
  lis2dh_int1_src_t       int1_src;
  lis2dh_int1_ths_t       int1_ths;
  lis2dh_int1_duration_t  int1_duration;
  lis2dh_int2_cfg_t       int2_cfg;
  lis2dh_int2_src_t       int2_src;
  lis2dh_int2_ths_t       int2_ths;
  lis2dh_int2_duration_t  int2_duration;
  lis2dh_click_cfg_t      click_cfg;
  lis2dh_click_src_t      click_src;
  lis2dh_click_ths_t      click_ths;
  lis2dh_time_limit_t     time_limit;
  lis2dh_time_latency_t   time_latency;
  lis2dh_time_window_t    time_window;
  lis2dh_act_ths_t        act_ths;
  lis2dh_act_dur_t        act_dur;
  bitwise_t               bitwise;
  uint8_t                 byte;
} lis2dh_reg_t;

/**
  * @}
  *
  */

int32_t lis2dh_read_reg(stmdev_ctx_t *ctx, uint8_t reg,
                        uint8_t *data,
                        uint16_t len);
int32_t lis2dh_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
                         uint8_t *data,
                         uint16_t len);

float_t lis2dh_from_fs2_hr_to_mg(int16_t lsb);
float_t lis2dh_from_fs4_hr_to_mg(int16_t lsb);
float_t lis2dh_from_fs8_hr_to_mg(int16_t lsb);
float_t lis2dh_from_fs16_hr_to_mg(int16_t lsb);
float_t lis2dh_from_lsb_hr_to_celsius(int16_t lsb);

float_t lis2dh_from_fs2_nm_to_mg(int16_t lsb);
float_t lis2dh_from_fs4_nm_to_mg(int16_t lsb);
float_t lis2dh_from_fs8_nm_to_mg(int16_t lsb);
float_t lis2dh_from_fs16_nm_to_mg(int16_t lsb);
float_t lis2dh_from_lsb_nm_to_celsius(int16_t lsb);

float_t lis2dh_from_fs2_lp_to_mg(int16_t lsb);
float_t lis2dh_from_fs4_lp_to_mg(int16_t lsb);
float_t lis2dh_from_fs8_lp_to_mg(int16_t lsb);
float_t lis2dh_from_fs16_lp_to_mg(int16_t lsb);
float_t lis2dh_from_lsb_lp_to_celsius(int16_t lsb);

int32_t lis2dh_int_count_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dh_temp_status_reg_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lis2dh_temp_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dh_temp_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dh_temperature_raw_get(stmdev_ctx_t *ctx, int16_t *val);

typedef enum
{
  LIS2DH_TEMP_DISABLE  = 0,
  LIS2DH_TEMP_ENABLE   = 3,
} lis2dh_temp_en_t;
int32_t lis2dh_temperature_meas_set(stmdev_ctx_t *ctx,
                                    lis2dh_temp_en_t val);
int32_t lis2dh_temperature_meas_get(stmdev_ctx_t *ctx,
                                    lis2dh_temp_en_t *val);

typedef enum
{
  LIS2DH_HR_12bit   = 0,
  LIS2DH_NM_10bit   = 1,
  LIS2DH_LP_8bit    = 2,
} lis2dh_op_md_t;
int32_t lis2dh_operating_mode_set(stmdev_ctx_t *ctx,
                                  lis2dh_op_md_t val);
int32_t lis2dh_operating_mode_get(stmdev_ctx_t *ctx,
                                  lis2dh_op_md_t *val);

typedef enum
{
  LIS2DH_POWER_DOWN                      = 0x00,
  LIS2DH_ODR_1Hz                         = 0x01,
  LIS2DH_ODR_10Hz                        = 0x02,
  LIS2DH_ODR_25Hz                        = 0x03,
  LIS2DH_ODR_50Hz                        = 0x04,
  LIS2DH_ODR_100Hz                       = 0x05,
  LIS2DH_ODR_200Hz                       = 0x06,
  LIS2DH_ODR_400Hz                       = 0x07,
  LIS2DH_ODR_1kHz620_LP                  = 0x08,
  LIS2DH_ODR_5kHz376_LP_1kHz344_NM_HP    = 0x09,
} lis2dh_odr_t;
int32_t lis2dh_data_rate_set(stmdev_ctx_t *ctx, lis2dh_odr_t val);
int32_t lis2dh_data_rate_get(stmdev_ctx_t *ctx,
                             lis2dh_odr_t *val);

int32_t lis2dh_high_pass_on_outputs_set(stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t lis2dh_high_pass_on_outputs_get(stmdev_ctx_t *ctx,
                                        uint8_t *val);

typedef enum
{
  LIS2DH_AGGRESSIVE  = 0,
  LIS2DH_STRONG      = 1,
  LIS2DH_MEDIUM      = 2,
  LIS2DH_LIGHT       = 3,
} lis2dh_hpcf_t;
int32_t lis2dh_high_pass_bandwidth_set(stmdev_ctx_t *ctx,
                                       lis2dh_hpcf_t val);
int32_t lis2dh_high_pass_bandwidth_get(stmdev_ctx_t *ctx,
                                       lis2dh_hpcf_t *val);

typedef enum
{
  LIS2DH_NORMAL_WITH_RST  = 0,
  LIS2DH_REFERENCE_MODE   = 1,
  LIS2DH_NORMAL           = 2,
  LIS2DH_AUTORST_ON_INT   = 3,
} lis2dh_hpm_t;
int32_t lis2dh_high_pass_mode_set(stmdev_ctx_t *ctx,
                                  lis2dh_hpm_t val);
int32_t lis2dh_high_pass_mode_get(stmdev_ctx_t *ctx,
                                  lis2dh_hpm_t *val);

typedef enum
{
  LIS2DH_2g   = 0,
  LIS2DH_4g   = 1,
  LIS2DH_8g   = 2,
  LIS2DH_16g  = 3,
} lis2dh_fs_t;
int32_t lis2dh_full_scale_set(stmdev_ctx_t *ctx, lis2dh_fs_t val);
int32_t lis2dh_full_scale_get(stmdev_ctx_t *ctx,
                              lis2dh_fs_t *val);

int32_t lis2dh_block_data_update_set(stmdev_ctx_t *ctx,
                                     uint8_t val);
int32_t lis2dh_block_data_update_get(stmdev_ctx_t *ctx,
                                     uint8_t *val);

int32_t lis2dh_filter_reference_set(stmdev_ctx_t *ctx,
                                    uint8_t *buff);
int32_t lis2dh_filter_reference_get(stmdev_ctx_t *ctx,
                                    uint8_t *buff);

int32_t lis2dh_xl_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dh_xl_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dh_acceleration_raw_get(stmdev_ctx_t *ctx,
                                    int16_t *val);

int32_t lis2dh_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff);

typedef enum
{
  LIS2DH_ST_DISABLE   = 0,
  LIS2DH_ST_POSITIVE  = 1,
  LIS2DH_ST_NEGATIVE  = 2,
} lis2dh_st_t;
int32_t lis2dh_self_test_set(stmdev_ctx_t *ctx, lis2dh_st_t val);
int32_t lis2dh_self_test_get(stmdev_ctx_t *ctx, lis2dh_st_t *val);

typedef enum
{
  LIS2DH_LSB_AT_LOW_ADD = 0,
  LIS2DH_MSB_AT_LOW_ADD = 1,
} lis2dh_ble_t;
int32_t lis2dh_data_format_set(stmdev_ctx_t *ctx,
                               lis2dh_ble_t val);
int32_t lis2dh_data_format_get(stmdev_ctx_t *ctx,
                               lis2dh_ble_t *val);

int32_t lis2dh_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dh_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dh_status_get(stmdev_ctx_t *ctx,
                          lis2dh_status_reg_t *val);

int32_t lis2dh_int1_gen_conf_set(stmdev_ctx_t *ctx,
                                 lis2dh_int1_cfg_t *val);
int32_t lis2dh_int1_gen_conf_get(stmdev_ctx_t *ctx,
                                 lis2dh_int1_cfg_t *val);

int32_t lis2dh_int1_gen_source_get(stmdev_ctx_t *ctx,
                                   lis2dh_int1_src_t *val);

int32_t lis2dh_int1_gen_threshold_set(stmdev_ctx_t *ctx,
                                      uint8_t val);
int32_t lis2dh_int1_gen_threshold_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lis2dh_int1_gen_duration_set(stmdev_ctx_t *ctx,
                                     uint8_t val);
int32_t lis2dh_int1_gen_duration_get(stmdev_ctx_t *ctx,
                                     uint8_t *val);

int32_t lis2dh_int2_gen_conf_set(stmdev_ctx_t *ctx,
                                 lis2dh_int2_cfg_t *val);
int32_t lis2dh_int2_gen_conf_get(stmdev_ctx_t *ctx,
                                 lis2dh_int2_cfg_t *val);

int32_t lis2dh_int2_gen_source_get(stmdev_ctx_t *ctx,
                                   lis2dh_int2_src_t *val);

int32_t lis2dh_int2_gen_threshold_set(stmdev_ctx_t *ctx,
                                      uint8_t val);
int32_t lis2dh_int2_gen_threshold_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lis2dh_int2_gen_duration_set(stmdev_ctx_t *ctx,
                                     uint8_t val);
int32_t lis2dh_int2_gen_duration_get(stmdev_ctx_t *ctx,
                                     uint8_t *val);

typedef enum
{
  LIS2DH_DISC_FROM_INT_GENERATOR  = 0,
  LIS2DH_ON_INT1_GEN              = 1,
  LIS2DH_ON_INT2_GEN              = 2,
  LIS2DH_ON_TAP_GEN               = 4,
  LIS2DH_ON_INT1_INT2_GEN         = 3,
  LIS2DH_ON_INT1_TAP_GEN          = 5,
  LIS2DH_ON_INT2_TAP_GEN          = 6,
  LIS2DH_ON_INT1_INT2_TAP_GEN     = 7,
} lis2dh_hp_t;
int32_t lis2dh_high_pass_int_conf_set(stmdev_ctx_t *ctx,
                                      lis2dh_hp_t val);
int32_t lis2dh_high_pass_int_conf_get(stmdev_ctx_t *ctx,
                                      lis2dh_hp_t *val);

int32_t lis2dh_pin_int1_config_set(stmdev_ctx_t *ctx,
                                   lis2dh_ctrl_reg3_t *val);
int32_t lis2dh_pin_int1_config_get(stmdev_ctx_t *ctx,
                                   lis2dh_ctrl_reg3_t *val);

int32_t lis2dh_int2_pin_detect_4d_set(stmdev_ctx_t *ctx,
                                      uint8_t val);
int32_t lis2dh_int2_pin_detect_4d_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef enum
{
  LIS2DH_INT2_PULSED   = 0,
  LIS2DH_INT2_LATCHED  = 1,
} lis2dh_lir_int2_t;
int32_t lis2dh_int2_pin_notification_mode_set(stmdev_ctx_t *ctx,
                                              lis2dh_lir_int2_t val);
int32_t lis2dh_int2_pin_notification_mode_get(stmdev_ctx_t *ctx,
                                              lis2dh_lir_int2_t *val);

int32_t lis2dh_int1_pin_detect_4d_set(stmdev_ctx_t *ctx,
                                      uint8_t val);
int32_t lis2dh_int1_pin_detect_4d_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef enum
{
  LIS2DH_INT1_PULSED   = 0,
  LIS2DH_INT1_LATCHED  = 1,
} lis2dh_lir_int1_t;
int32_t lis2dh_int1_pin_notification_mode_set(stmdev_ctx_t *ctx,
                                              lis2dh_lir_int1_t val);
int32_t lis2dh_int1_pin_notification_mode_get(stmdev_ctx_t *ctx,
                                              lis2dh_lir_int1_t *val);

int32_t lis2dh_pin_int2_config_set(stmdev_ctx_t *ctx,
                                   lis2dh_ctrl_reg6_t *val);
int32_t lis2dh_pin_int2_config_get(stmdev_ctx_t *ctx,
                                   lis2dh_ctrl_reg6_t *val);

int32_t lis2dh_fifo_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dh_fifo_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dh_fifo_watermark_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dh_fifo_watermark_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LIS2DH_INT1_GEN = 0,
  LIS2DH_INT2_GEN = 1,
} lis2dh_tr_t;
int32_t lis2dh_fifo_trigger_event_set(stmdev_ctx_t *ctx,
                                      lis2dh_tr_t val);
int32_t lis2dh_fifo_trigger_event_get(stmdev_ctx_t *ctx,
                                      lis2dh_tr_t *val);

typedef enum
{
  LIS2DH_BYPASS_MODE           = 0,
  LIS2DH_FIFO_MODE             = 1,
  LIS2DH_DYNAMIC_STREAM_MODE   = 2,
  LIS2DH_STREAM_TO_FIFO_MODE   = 3,
} lis2dh_fm_t;
int32_t lis2dh_fifo_mode_set(stmdev_ctx_t *ctx, lis2dh_fm_t val);
int32_t lis2dh_fifo_mode_get(stmdev_ctx_t *ctx, lis2dh_fm_t *val);

int32_t lis2dh_fifo_status_get(stmdev_ctx_t *ctx,
                               lis2dh_fifo_src_reg_t *val);

int32_t lis2dh_fifo_data_level_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dh_fifo_empty_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dh_fifo_ovr_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dh_fifo_fth_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dh_tap_conf_set(stmdev_ctx_t *ctx,
                            lis2dh_click_cfg_t *val);
int32_t lis2dh_tap_conf_get(stmdev_ctx_t *ctx,
                            lis2dh_click_cfg_t *val);

int32_t lis2dh_tap_source_get(stmdev_ctx_t *ctx,
                              lis2dh_click_src_t *val);

int32_t lis2dh_tap_threshold_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dh_tap_threshold_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dh_shock_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dh_shock_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dh_quiet_dur_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dh_quiet_dur_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dh_double_tap_timeout_set(stmdev_ctx_t *ctx,
                                      uint8_t val);
int32_t lis2dh_double_tap_timeout_get(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lis2dh_act_threshold_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dh_act_threshold_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis2dh_act_timeout_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis2dh_act_timeout_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  LIS2DH_SPI_4_WIRE = 0,
  LIS2DH_SPI_3_WIRE = 1,
} lis2dh_sim_t;
int32_t lis2dh_spi_mode_set(stmdev_ctx_t *ctx, lis2dh_sim_t val);
int32_t lis2dh_spi_mode_get(stmdev_ctx_t *ctx, lis2dh_sim_t *val);

/**
  * @}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* LIS2DH_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
