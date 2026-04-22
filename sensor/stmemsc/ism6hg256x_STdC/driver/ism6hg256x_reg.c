/**
  ******************************************************************************
  * @file    ism6hg256x_reg.c
  * @author  Sensors Software Solution Team
  * @brief   ISM6HG256X driver file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "ism6hg256x_reg.h"

int32_t __weak ism6hg256x_read_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                                   uint8_t *data,
                                   uint16_t len)
{
  int32_t ret = 0;

  if (ctx == NULL)
  {
    return -1;
  }

  ret = ctx->read_reg(ctx->handle, reg, data, len);

  return ret;
}

int32_t __weak ism6hg256x_write_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                                    uint8_t *data,
                                    uint16_t len)
{
  int32_t ret = 0;

  if (ctx == NULL)
  {
    return -1;
  }

  ret = ctx->write_reg(ctx->handle, reg, data, len);

  return ret;
}

static void bytecpy(uint8_t *target, const uint8_t *source)
{
  if ((target != NULL) && (source != NULL))
  {
    *target = *source;
  }
}

float_t ism6hg256x_from_sflp_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.061f;
}

float_t ism6hg256x_from_fs2_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.061f;
}

float_t ism6hg256x_from_fs4_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.122f;
}

float_t ism6hg256x_from_fs8_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.244f;
}

float_t ism6hg256x_from_fs16_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.488f;
}

float_t ism6hg256x_from_fs32_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.976f;
}

float_t ism6hg256x_from_fs64_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 1.952f;
}

float_t ism6hg256x_from_fs128_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 3.904f;
}

float_t ism6hg256x_from_fs256_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 10.417f;
}

float_t ism6hg256x_from_fs125_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 4.375f;
}
float_t ism6hg256x_from_fs250_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 8.750f;
}

float_t ism6hg256x_from_fs500_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 17.50f;
}

float_t ism6hg256x_from_fs1000_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 35.0f;
}

float_t ism6hg256x_from_fs2000_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 70.0f;
}

float_t ism6hg256x_from_fs4000_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 140.0f;
}

float_t ism6hg256x_from_lsb_to_celsius(int16_t lsb)
{
  return (((float_t)lsb / 256.0f) + 25.0f);
}

uint64_t ism6hg256x_from_lsb_to_nsec(uint32_t lsb)
{
  return ((uint64_t)lsb * 21700);
}

float_t ism6hg256x_from_gbias_lsb_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) * 4.375f;
}

float_t ism6hg256x_from_gravity_lsb_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.061f;
}

static float_t npy_half_to_float(uint16_t h);
float_t ism6hg256x_from_quaternion_lsb_to_float(uint16_t lsb)
{
  return npy_half_to_float(lsb);
}

static uint32_t npy_halfbits_to_floatbits(uint16_t h);
uint32_t ism6hg256x_from_f16_to_f32(uint16_t val)
{
  return npy_halfbits_to_floatbits(val);
}


int32_t ism6hg256x_xl_offset_on_out_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_ctrl9_t ctrl9 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);
  if (ret == 0)
  {
    ctrl9.usr_off_on_out = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);
  }

  return ret;
}

int32_t ism6hg256x_xl_offset_on_out_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_ctrl9_t ctrl9 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);
  if (ret != 0)
  {
    return ret;
  }
  *val = ctrl9.usr_off_on_out;

  return ret;
}


int32_t ism6hg256x_xl_offset_mg_set(const stmdev_ctx_t *ctx,
                                    ism6hg256x_xl_offset_mg_t val)
{
  ism6hg256x_z_ofs_usr_t z_ofs_usr = {0};
  ism6hg256x_y_ofs_usr_t y_ofs_usr = {0};
  ism6hg256x_x_ofs_usr_t x_ofs_usr = {0};
  ism6hg256x_ctrl9_t ctrl9 = {0};
  int32_t ret = 0;
  float_t tmp = 0.0f;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_Z_OFS_USR, (uint8_t *)&z_ofs_usr, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_Y_OFS_USR, (uint8_t *)&y_ofs_usr, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_X_OFS_USR, (uint8_t *)&x_ofs_usr, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);
  if (ret != 0)
  {
    return ret;
  }


  if ((val.x_mg < (0.0078125f * 127.0f)) && (val.x_mg > (0.0078125f * -127.0f)) &&
      (val.y_mg < (0.0078125f * 127.0f)) && (val.y_mg > (0.0078125f * -127.0f)) &&
      (val.z_mg < (0.0078125f * 127.0f)) && (val.z_mg > (0.0078125f * -127.0f)))
  {
    ctrl9.usr_off_w = 0u;

    tmp = val.z_mg / 0.0078125f;
    z_ofs_usr.z_ofs_usr = (int8_t)(int32_t)tmp;

    tmp = val.y_mg / 0.0078125f;
    y_ofs_usr.y_ofs_usr = (int8_t)(int32_t)tmp;

    tmp = val.x_mg / 0.0078125f;
    x_ofs_usr.x_ofs_usr = (int8_t)(int32_t)tmp;
  }
  else if ((val.x_mg < (0.125f * 127.0f)) && (val.x_mg > (0.125f * -127.0f)) &&
           (val.y_mg < (0.125f * 127.0f)) && (val.y_mg > (0.125f * -127.0f)) &&
           (val.z_mg < (0.125f * 127.0f)) && (val.z_mg > (0.125f * -127.0f)))
  {
    ctrl9.usr_off_w = 1u;

    tmp = val.z_mg / 0.125f;
    z_ofs_usr.z_ofs_usr = (int8_t)(int32_t)tmp;

    tmp = val.y_mg / 0.125f;
    y_ofs_usr.y_ofs_usr = (int8_t)(int32_t)tmp;

    tmp = val.x_mg / 0.125f;
    x_ofs_usr.x_ofs_usr = (int8_t)(int32_t)tmp;
  }
  else // out of limit
  {
    ctrl9.usr_off_w = 1u;
    z_ofs_usr.z_ofs_usr = 0;
    y_ofs_usr.y_ofs_usr = 0;
    x_ofs_usr.x_ofs_usr = 0;
  }

  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_Z_OFS_USR, (uint8_t *)&z_ofs_usr, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_Y_OFS_USR, (uint8_t *)&y_ofs_usr, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_X_OFS_USR, (uint8_t *)&x_ofs_usr, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);

  return ret;
}


int32_t ism6hg256x_xl_offset_mg_get(const stmdev_ctx_t *ctx,
                                    ism6hg256x_xl_offset_mg_t *val)
{
  ism6hg256x_z_ofs_usr_t z_ofs_usr = {0};
  ism6hg256x_y_ofs_usr_t y_ofs_usr = {0};
  ism6hg256x_x_ofs_usr_t x_ofs_usr = {0};
  ism6hg256x_ctrl9_t ctrl9 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_Z_OFS_USR, (uint8_t *)&z_ofs_usr, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_Y_OFS_USR, (uint8_t *)&y_ofs_usr, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_X_OFS_USR, (uint8_t *)&x_ofs_usr, 1);
  if (ret != 0)
  {
    return ret;
  }

  if (ctrl9.usr_off_w == PROPERTY_DISABLE)
  {
    val->z_mg = ((float_t)z_ofs_usr.z_ofs_usr * 0.0078125f);
    val->y_mg = ((float_t)y_ofs_usr.y_ofs_usr * 0.0078125f);
    val->x_mg = ((float_t)x_ofs_usr.x_ofs_usr * 0.0078125f);
  }
  else
  {
    val->z_mg = ((float_t)z_ofs_usr.z_ofs_usr * 0.125f);
    val->y_mg = ((float_t)y_ofs_usr.y_ofs_usr * 0.125f);
    val->x_mg = ((float_t)x_ofs_usr.x_ofs_usr * 0.125f);
  }

  return ret;
}


int32_t ism6hg256x_hg_xl_offset_mg_set(const stmdev_ctx_t *ctx,
                                       ism6hg256x_xl_offset_mg_t val)
{
  ism6hg256x_hg_z_ofs_usr_t z_ofs_usr = {0};
  ism6hg256x_hg_y_ofs_usr_t y_ofs_usr = {0};
  ism6hg256x_hg_x_ofs_usr_t x_ofs_usr = {0};
  ism6hg256x_ctrl1_xl_hg_t  ctrl1_xl_hg = {0};
  int32_t ret = 0;
  float_t tmp = 0.0f;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL1_XL_HG, (uint8_t *)&ctrl1_xl_hg, 1);
  if (ret != 0)
  {
    return ret;
  }

  if ((val.x_mg < (0.25f * 127.0f)) && (val.x_mg > (0.25f * -127.0f)) &&
      (val.y_mg < (0.25f * 127.0f)) && (val.y_mg > (0.25f * -127.0f)) &&
      (val.z_mg < (0.25f * 127.0f)) && (val.z_mg > (0.25f * -127.0f)))
  {
    ctrl1_xl_hg.hg_usr_off_on_out = 1U;

    tmp = val.z_mg / 0.25f;
    z_ofs_usr.xl_hg_z_ofs_usr = (int8_t)tmp;

    tmp = val.y_mg / 0.25f;
    y_ofs_usr.xl_hg_y_ofs_usr = (int8_t)tmp;

    tmp = val.x_mg / 0.25f;
    x_ofs_usr.xl_hg_x_ofs_usr = (int8_t)tmp;
  }
  else // out of limit
  {
    ctrl1_xl_hg.hg_usr_off_on_out = 0u;
    z_ofs_usr.xl_hg_z_ofs_usr = 0;
    y_ofs_usr.xl_hg_y_ofs_usr = 0;
    x_ofs_usr.xl_hg_x_ofs_usr = 0;
  }

  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_XL_HG_Z_OFS_USR, (uint8_t *)&z_ofs_usr, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_XL_HG_Y_OFS_USR, (uint8_t *)&y_ofs_usr, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_XL_HG_X_OFS_USR, (uint8_t *)&x_ofs_usr, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL1_XL_HG, (uint8_t *)&ctrl1_xl_hg, 1);

  return ret;
}


int32_t ism6hg256x_hg_xl_offset_mg_get(const stmdev_ctx_t *ctx,
                                       ism6hg256x_xl_offset_mg_t *val)
{
  ism6hg256x_hg_z_ofs_usr_t z_ofs_usr = {0};
  ism6hg256x_hg_y_ofs_usr_t y_ofs_usr = {0};
  ism6hg256x_hg_x_ofs_usr_t x_ofs_usr = {0};
  ism6hg256x_ctrl1_xl_hg_t  ctrl1_xl_hg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL1_XL_HG, (uint8_t *)&ctrl1_xl_hg, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_XL_HG_Z_OFS_USR, (uint8_t *)&z_ofs_usr, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_XL_HG_Y_OFS_USR, (uint8_t *)&y_ofs_usr, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_XL_HG_X_OFS_USR, (uint8_t *)&x_ofs_usr, 1);
  if (ret != 0)
  {
    return ret;
  }

  if (ctrl1_xl_hg.hg_usr_off_on_out == PROPERTY_DISABLE)
  {
    val->z_mg = 0.0f;
    val->y_mg = 0.0f;
    val->x_mg = 0.0f;
  }
  else
  {
    val->z_mg = ((float_t)z_ofs_usr.xl_hg_z_ofs_usr * 0.25f);
    val->y_mg = ((float_t)y_ofs_usr.xl_hg_y_ofs_usr * 0.25f);
    val->x_mg = ((float_t)x_ofs_usr.xl_hg_x_ofs_usr * 0.25f);
  }

  return ret;
}


int32_t ism6hg256x_reboot(const stmdev_ctx_t *ctx)
{
  ism6hg256x_ctrl3_t ctrl3 = {0};
  int32_t ret = 0;
  /* configuration to restore after reboot */
  ism6hg256x_data_rate_t xl = ISM6HG256X_ODR_OFF;
  ism6hg256x_data_rate_t gy = ISM6HG256X_ODR_OFF;
  ism6hg256x_hg_xl_data_rate_t hg_xl = ISM6HG256X_HG_XL_ODR_OFF;
  ism6hg256x_xl_mode_t xlm = ISM6HG256X_XL_UNCHANGED_MD;
  ism6hg256x_gy_mode_t gym = ISM6HG256X_GY_UNCHANGED_MD;
  uint8_t reg_out_en = 0u;

  if (ctx->mdelay == NULL)
  {
    ret = -1;
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL3, (uint8_t *)&ctrl3, 1);
  if (ret != 0)
  {
    goto exit;
  }

  /* Save current data rates */
  ret = ism6hg256x_xl_data_rate_get(ctx, &xl);
  ret += ism6hg256x_gy_data_rate_get(ctx, &gy);
  ret += ism6hg256x_hg_xl_data_rate_get(ctx, &hg_xl, &reg_out_en);
  if (ret != 0)
  {
    goto exit;
  }

  /* Save XL/GY current modes */
  ret = ism6hg256x_xl_mode_get(ctx, &xlm);
  ret += ism6hg256x_gy_mode_get(ctx, &gym);
  if (ret != 0)
  {
    goto exit;
  }

  /* 1. Set the low-g accelerometer, high-g accelerometer, and gyroscope in power-down mode */
  ret = ism6hg256x_xl_setup(ctx, ISM6HG256X_ODR_OFF, ISM6HG256X_XL_HIGH_PERFORMANCE_MD);
  ret += ism6hg256x_gy_setup(ctx, ISM6HG256X_ODR_OFF, ISM6HG256X_GY_HIGH_PERFORMANCE_MD);
  ret += ism6hg256x_hg_xl_data_rate_set(ctx, ISM6HG256X_HG_XL_ODR_OFF, 0);
  if (ret != 0)
  {
    goto exit;
  }

  /* 2. Set the BOOT bit of the CTRL3 register to 1. */
  ctrl3.boot = 1;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL3, (uint8_t *)&ctrl3, 1);
  if (ret != 0)
  {
    goto exit;
  }

  /* 3. Wait 30 ms. */
  ctx->mdelay(30);

  /* Restore data rates */
  ret = ism6hg256x_xl_setup(ctx, xl, xlm);
  ret += ism6hg256x_gy_setup(ctx, gy, gym);
  ret += ism6hg256x_hg_xl_data_rate_set(ctx, hg_xl, reg_out_en);

exit:
  return ret;
}


int32_t ism6hg256x_sw_por(const stmdev_ctx_t *ctx)
{
  ism6hg256x_func_cfg_access_t func_cfg_access = {0};
  int32_t ret = 0;

  if (ctx->mdelay == NULL)
  {
    ret = -1;
    goto exit;
  }

  /* 1. Set the SW_POR bit of the FUNC_CFG_ACCESS register to 1. */
  func_cfg_access.sw_por = 1;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  if (ret != 0)
  {
    goto exit;
  }

  /* 2. Wait 30 ms. */
  ctx->mdelay(30);

exit:
  return ret;
}


int32_t ism6hg256x_sw_reset(const stmdev_ctx_t *ctx)
{
  ism6hg256x_ctrl3_t ctrl3 = {0};
  uint8_t retry = 0u;
  int32_t ret = 0;

  if (ctx->mdelay == NULL)
  {
    ret = -1;
    goto exit;
  }

  /* 1. Set the low-g accelerometer, high-g accelerometer, and gyroscope in power-down mode */
  ret = ism6hg256x_xl_setup(ctx, ISM6HG256X_ODR_OFF, ISM6HG256X_XL_HIGH_PERFORMANCE_MD);
  ret += ism6hg256x_gy_setup(ctx, ISM6HG256X_ODR_OFF, ISM6HG256X_GY_HIGH_PERFORMANCE_MD);
  ret += ism6hg256x_hg_xl_data_rate_set(ctx, ISM6HG256X_HG_XL_ODR_OFF, 0);
  if (ret != 0)
  {
    goto exit;
  }

  /* 2. Set the SW_RESET bit of the CTRL3 register to 1. */
  ctrl3.sw_reset = 1;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL3, (uint8_t *)&ctrl3, 1);

  /* 3. Poll the SW_RESET bit of the CTRL3 register until it returns to 0. */
  do
  {
    ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL3, (uint8_t *)&ctrl3, 1);
    if (ret != 0)
    {
      goto exit;
    }

    ctx->mdelay(1);
    retry++;
  } while ((ctrl3.sw_reset == 1u) && (retry < 3u));

  return (ctrl3.sw_reset == 0u) ? 0 : -1;

exit:
  return ret;
}


int32_t ism6hg256x_mem_bank_set(const stmdev_ctx_t *ctx, ism6hg256x_mem_bank_t val)
{
  ism6hg256x_func_cfg_access_t func_cfg_access = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  if (ret != 0)
  {
    return ret;
  }

  func_cfg_access.shub_reg_access = ((uint8_t)val & 0x02U) >> 1;
  func_cfg_access.emb_func_reg_access = (uint8_t)val & 0x01U;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);

  return ret;
}


int32_t ism6hg256x_mem_bank_get(const stmdev_ctx_t *ctx, ism6hg256x_mem_bank_t *val)
{
  ism6hg256x_func_cfg_access_t func_cfg_access = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch ((func_cfg_access.shub_reg_access << 1) + func_cfg_access.emb_func_reg_access)
  {
    case 0x00:
      *val = ISM6HG256X_MAIN_MEM_BANK;
      break;

    case 0x01:
      *val = ISM6HG256X_EMBED_FUNC_MEM_BANK;
      break;

    case 0x02:
      *val = ISM6HG256X_SENSOR_HUB_MEM_BANK;
      break;

    default:
      *val = ISM6HG256X_MAIN_MEM_BANK;
      break;
  }

  return ret;
}


int32_t ism6hg256x_device_id_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_WHO_AM_I, val, 1);

  return ret;
}

int32_t ism6hg256x_xl_setup(
  const stmdev_ctx_t *ctx,
  ism6hg256x_data_rate_t xl_odr,
  ism6hg256x_xl_mode_t xl_mode)
{
  int32_t ret = 0;
  ism6hg256x_ctrl1_t ctrl1 = {0};
  ism6hg256x_ctrl2_t ctrl2 = {0};
  ism6hg256x_haodr_cfg_t haodr = {0};
  uint8_t xl_ha = ((uint8_t) xl_odr >> 4) & 0xFU;
  uint8_t both_on = 0;
  uint8_t buff[2] = {0};

  if (xl_odr == ISM6HG256X_ODR_UNCHANGED)
  {
    ret = ism6hg256x_xl_data_rate_get(ctx, &xl_odr);

    if (ret != 0)
    {
      return ret;
    }
  }

  if (xl_mode == ISM6HG256X_XL_UNCHANGED_MD)
  {
    ret = ism6hg256x_xl_mode_get(ctx, &xl_mode);

    if (ret != 0)
    {
      return ret;
    }
  }

  // Table 10 of AN6353
  // 1.875 Hz allowed only in Low-power modes
  if (xl_odr == ISM6HG256X_ODR_AT_1Hz875 &&
      xl_mode != ISM6HG256X_XL_LOW_POWER_2_AVG_MD &&
      xl_mode != ISM6HG256X_XL_LOW_POWER_4_AVG_MD &&
      xl_mode != ISM6HG256X_XL_LOW_POWER_8_AVG_MD)
  {
    ret = -1;
    goto exit;
  }
  // 7.5 Hz allowed only in normal or high-performance modes
  else if (xl_odr == ISM6HG256X_ODR_AT_7Hz5 &&
           xl_mode != ISM6HG256X_XL_NORMAL_MD && xl_mode != ISM6HG256X_XL_HIGH_PERFORMANCE_MD)
  {
    ret = -1;
    goto exit;
  }
  // if odr_xl bits has 4th bit enabled, low-power modes are not allowed
  else if (
    // odr >= 480 and low-power and normal mode
    ((uint8_t)xl_odr & 0x8) != 0u && ((uint8_t)xl_mode & 0x4) != 0u &&
    (xl_mode != ISM6HG256X_XL_NORMAL_MD || // normal mode is not allowed for some data rates
     xl_odr == ISM6HG256X_ODR_AT_3840Hz ||
     xl_odr == ISM6HG256X_ODR_AT_7680Hz))
  {
    ret = -1;
    goto exit;
  }

  // Section 3.5 of AN6353
  if (xl_mode == ISM6HG256X_XL_ODR_TRIGGERED_MD &&
      (xl_odr == ISM6HG256X_ODR_AT_1Hz875 ||
       xl_odr == ISM6HG256X_ODR_AT_7Hz5 ||
       xl_odr == ISM6HG256X_ODR_AT_7680Hz))
  {
    ret = -1;
    goto exit;
  }

  // if odr is choosed as high-accuracy value, mode should also be set in HAODR mode
  if ((xl_ha != 0u && xl_mode != ISM6HG256X_XL_HIGH_ACCURACY_ODR_MD) ||
      (xl_ha == 0u && xl_mode == ISM6HG256X_XL_HIGH_ACCURACY_ODR_MD))
  {
    ret = -1;
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL1, buff, 2);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_HAODR_CFG, (uint8_t *)&haodr, 1);

  bytecpy((uint8_t *)&ctrl1, &buff[0]);
  bytecpy((uint8_t *)&ctrl2, &buff[1]);

  if (ret != 0)
  {
    goto exit;
  }

  // cross-checking haodr mode
  both_on = ctrl1.odr_xl != (uint8_t)ISM6HG256X_ODR_OFF &&
            ctrl2.odr_g != (uint8_t)ISM6HG256X_ODR_OFF ? 1u : 0u;

  // if both on, then haodr_sel is a shared bit. Could be changed through haodr_set API
  if (both_on == 1u && (xl_ha != haodr.haodr_sel))
  {
    ret = -1;
    goto exit;
  }

  // Switching (enable/disable) HAODR mode require that all sensors must be in power-down mode.
  // Note: if both sensors are ON, ism6hg256x_haodr_set function must be used.
  if (haodr.haodr_sel != xl_ha &&
      ctrl1.op_mode_xl != (uint8_t)xl_mode && // check if mode switch is required
      (xl_mode == ISM6HG256X_XL_HIGH_ACCURACY_ODR_MD || // check if mode to set is HAODR
       ctrl1.op_mode_xl == (uint8_t)
       ISM6HG256X_XL_HIGH_ACCURACY_ODR_MD)) // check if previous mode was HAODR
  {
    ret += ism6hg256x_haodr_set(ctx, xl_odr, xl_mode, (ism6hg256x_data_rate_t)ctrl2.odr_g,
                                (ism6hg256x_gy_mode_t)ctrl2.op_mode_g);
  }
  else
  {
    // if HAODR switch is not required, just set ctrl1 settings
    ctrl1.op_mode_xl = (uint8_t)xl_mode & 0x07U;
    ctrl1.odr_xl = (uint8_t)xl_odr & 0x0FU;
    haodr.haodr_sel = xl_ha & 0x03U;
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL1, (uint8_t *)&ctrl1, 1);
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_HAODR_CFG, (uint8_t *)&haodr, 1);
  }

exit:
  return ret;
}


int32_t ism6hg256x_gy_setup(
  const stmdev_ctx_t *ctx,
  ism6hg256x_data_rate_t gy_odr,
  ism6hg256x_gy_mode_t gy_mode)
{
  int32_t ret = 0;
  ism6hg256x_ctrl1_t ctrl1 = {0};
  ism6hg256x_ctrl2_t ctrl2 = {0};
  ism6hg256x_haodr_cfg_t haodr = {0};
  uint8_t gy_ha = ((uint8_t) gy_odr >> 4) & 0xFU;
  uint8_t both_on = 0;
  uint8_t buff[2] = {0};

  if (gy_odr == ISM6HG256X_ODR_UNCHANGED)
  {
    ret = ism6hg256x_gy_data_rate_get(ctx, &gy_odr);

    if (ret != 0)
    {
      return ret;
    }
  }

  if (gy_mode == ISM6HG256X_GY_UNCHANGED_MD)
  {
    ret = ism6hg256x_gy_mode_get(ctx, &gy_mode);

    if (ret != 0)
    {
      return ret;
    }
  }

  // Table 13 of AN6353
  // 7.5Hz with HAODR mode enable, is already handled by the enum selection
  if (((uint8_t)gy_odr & 0x8) != 0u && gy_mode == ISM6HG256X_GY_LOW_POWER_MD)
  {
    ret = -1;
    goto exit;
  }

  // Section 3.5 of AN6353
  if (gy_mode == ISM6HG256X_GY_ODR_TRIGGERED_MD &&
      (gy_odr == ISM6HG256X_ODR_AT_7Hz5 ||
       gy_odr == ISM6HG256X_ODR_AT_7680Hz))
  {
    ret = -1;
    goto exit;
  }

  // if odr is choosed as high-accuracy value, mode should also be set in HAODR mode
  if ((gy_ha != 0u && gy_mode != ISM6HG256X_GY_HIGH_ACCURACY_ODR_MD) ||
      (gy_ha == 0u && gy_mode == ISM6HG256X_GY_HIGH_ACCURACY_ODR_MD))
  {
    ret = -1;
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL1, buff, 2);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_HAODR_CFG, (uint8_t *)&haodr, 1);

  bytecpy((uint8_t *)&ctrl1, &buff[0]);
  bytecpy((uint8_t *)&ctrl2, &buff[1]);

  if (ret != 0)
  {
    goto exit;
  }

  // cross-checking haodr mode
  both_on = ctrl1.odr_xl != (uint8_t)ISM6HG256X_ODR_OFF &&
            ctrl2.odr_g != (uint8_t)ISM6HG256X_ODR_OFF ? 1u : 0u;

  // if both on, then haodr_sel is a shared bit
  if (both_on == 1u && (gy_ha != haodr.haodr_sel))
  {
    ret = -1;
    goto exit;
  }

  // Switching (enable/disable) HAODR mode require that all sensors must be in power-down mode.
  // Note: ism6hg256x_haodr_set function should be called first.
  if (haodr.haodr_sel != gy_ha &&
      ctrl2.op_mode_g != (uint8_t)gy_mode && // check if mode switch is required (prev. != new)
      (gy_mode == ISM6HG256X_GY_HIGH_ACCURACY_ODR_MD || // check if mode to set is HAODR
       ctrl2.op_mode_g == (uint8_t)ISM6HG256X_GY_HIGH_ACCURACY_ODR_MD)) // check if previous mode was HAODR
  {
    ret += ism6hg256x_haodr_set(ctx, (ism6hg256x_data_rate_t)ctrl1.odr_xl,
                                (ism6hg256x_xl_mode_t)ctrl1.op_mode_xl, gy_odr, gy_mode);
  }
  else
  {
    // if HAODR switch is not required, just set ctrl2 settings
    ctrl2.op_mode_g = (uint8_t)gy_mode & 0x07U;
    ctrl2.odr_g = (uint8_t)gy_odr & 0x0FU;
    haodr.haodr_sel = gy_ha & 0x03U;
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&ctrl2, 1);
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_HAODR_CFG, (uint8_t *)&haodr, 1);
  }

exit:
  return ret;
}


int32_t ism6hg256x_haodr_set(
  const stmdev_ctx_t *ctx,
  ism6hg256x_data_rate_t xl_odr,
  ism6hg256x_xl_mode_t xl_mode,
  ism6hg256x_data_rate_t gy_odr,
  ism6hg256x_gy_mode_t gy_mode)
{
  ism6hg256x_ctrl1_t ctrl1 = {0};
  ism6hg256x_ctrl2_t ctrl2 = {0};
  ism6hg256x_haodr_cfg_t haodr = {0};
  ism6hg256x_ctrl1_xl_hg_t ctrl1_xl_hg = {0};
  ism6hg256x_xl_mode_t prev_mode = ISM6HG256X_XL_UNCHANGED_MD;
  ism6hg256x_ctrl1_xl_hg_t ctrl1_xl_hg_prev = {0};
  ism6hg256x_ctrl_eis_t ctrl_eis_prev = {0};
  ism6hg256x_ctrl_eis_t ctrl_eis = {0};
  int32_t ret = 0;

  uint8_t xl_ha = (((uint8_t)xl_odr) >> 4) & 0xFU;
  uint8_t gy_ha = (((uint8_t)gy_odr) >> 4) & 0xFU;
  uint8_t both_on = xl_odr != ISM6HG256X_ODR_OFF && gy_odr != ISM6HG256X_ODR_OFF ? 1 : 0;

  if (ctx->mdelay == NULL)
  {
    ret = -1;
    goto exit;
  }

  if (both_on == 1u && (xl_ha != gy_ha))
  {
    ret = -1;
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_HAODR_CFG, (uint8_t *)&haodr, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL1, (uint8_t *)&ctrl1, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&ctrl2, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL1_XL_HG, (uint8_t *)&ctrl1_xl_hg, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL_EIS, (uint8_t *)&ctrl_eis, 1);

  prev_mode = (ism6hg256x_xl_mode_t) ctrl1.op_mode_xl;
  ctrl1_xl_hg_prev = ctrl1_xl_hg;
  ctrl_eis_prev = ctrl_eis;

  if (ret != 0)
  {
    goto exit;
  }

  // Enabling/disabling HAODR mode require to have all sensors in power-down mode
  ctrl1.odr_xl = (uint8_t)ISM6HG256X_ODR_OFF;
  ctrl2.odr_g = (uint8_t)ISM6HG256X_ODR_OFF;
  ctrl1_xl_hg.odr_xl_hg = (uint8_t)ISM6HG256X_HG_XL_ODR_OFF;
  ctrl1_xl_hg.xl_hg_regout_en = 0u;
  ctrl_eis.odr_g_eis = (uint8_t)ISM6HG256X_EIS_ODR_OFF;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL1, (uint8_t *)&ctrl1, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&ctrl2, 1);
  // avoid turning off if already off
  if (ctrl1_xl_hg_prev.odr_xl_hg != (uint8_t)ISM6HG256X_HG_XL_ODR_OFF)
  {
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL1_XL_HG, (uint8_t *)&ctrl1_xl_hg, 1);
  }
  if (ctrl_eis_prev.odr_g_eis != (uint8_t)ISM6HG256X_EIS_ODR_OFF)
  {
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL_EIS, (uint8_t *)&ctrl_eis, 1);
  }

  // set HAODR
  haodr.haodr_sel = (xl_ha | gy_ha) & 0x03U;
  ctrl1.op_mode_xl = (uint8_t)xl_mode & 0x07U;
  ctrl2.op_mode_g = (uint8_t)gy_mode & 0x07U;

  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_HAODR_CFG, (uint8_t *)&haodr, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL1, (uint8_t *)&ctrl1, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&ctrl2, 1);

  if (prev_mode == ISM6HG256X_XL_HIGH_ACCURACY_ODR_MD)
  {
    ctx->mdelay(1); // should be at least 500 us; AN6353, section 3.4
  }

  // set xl and gy data rates and restore high-g xl and eis to their previous data rates
  ctrl1.odr_xl = (uint8_t)xl_odr & 0x0FU;
  ctrl2.odr_g = (uint8_t)gy_odr & 0x0FU;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL1, (uint8_t *)&ctrl1, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&ctrl2, 1);
  // if off, there is no need to turn them on
  if (ctrl1_xl_hg_prev.odr_xl_hg != (uint8_t)ISM6HG256X_HG_XL_ODR_OFF)
  {
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL1_XL_HG, (uint8_t *)&ctrl1_xl_hg_prev, 1);
  }
  if (ctrl_eis_prev.odr_g_eis != (uint8_t)ISM6HG256X_EIS_ODR_OFF)
  {
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL_EIS, (uint8_t *)&ctrl_eis_prev, 1);
  }

exit:
  return ret;

}

int32_t ism6hg256x_xl_data_rate_set(const stmdev_ctx_t *ctx,
                                    ism6hg256x_data_rate_t val)
{
  ism6hg256x_ctrl1_t ctrl1 = {0};
  ism6hg256x_haodr_cfg_t haodr = {0};
  uint8_t sel = 0;
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL1, (uint8_t *)&ctrl1, 1);
  if (ret != 0)
  {
    return ret;
  }

  ctrl1.odr_xl = (uint8_t)val & 0x0Fu;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL1, (uint8_t *)&ctrl1, 1);
  if (ret != 0)
  {
    return ret;
  }

  sel = ((uint8_t)val >> 4) & 0xFU;
  if (sel != 0U)
  {
    ret += ism6hg256x_read_reg(ctx, ISM6HG256X_HAODR_CFG, (uint8_t *)&haodr, 1);
    if (ret != 0)
    {
      return ret;
    }
    haodr.haodr_sel = sel & 0x03U;
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_HAODR_CFG, (uint8_t *)&haodr, 1);
  }

  return ret;
}


int32_t ism6hg256x_xl_data_rate_get(const stmdev_ctx_t *ctx,
                                    ism6hg256x_data_rate_t *val)
{
  ism6hg256x_ctrl1_t ctrl1 = {0};
  ism6hg256x_haodr_cfg_t haodr = {0};
  uint8_t sel = 0;
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL1, (uint8_t *)&ctrl1, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_HAODR_CFG, (uint8_t *)&haodr, 1);
  if (ret != 0)
  {
    return ret;
  }

  sel = haodr.haodr_sel;

  switch (ctrl1.odr_xl)
  {
    case 0x00:
      *val = ISM6HG256X_ODR_OFF;
      break;

    case 0x01:
      *val = ISM6HG256X_ODR_AT_1Hz875;
      break;

    case 0x02:
      *val = ISM6HG256X_ODR_AT_7Hz5;
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_15Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_15Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_15Hz625;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_12Hz5;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_13Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_15Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_30Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_30Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_31Hz25;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_25Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_26Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_30Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_60Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_60Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_62Hz5;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_50Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_52Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_60Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_120Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_120Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_125Hz;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_100Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_104Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_120Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_240Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_240Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_250Hz;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_200Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_208Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_240Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_480Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_480Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_500Hz;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_400Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_417Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_480Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_960Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_960Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_1000Hz;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_800Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_833Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_960Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_1920Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_1920Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_2000Hz;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_1600Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_1667Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_1920Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_3840Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_3840Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_4000Hz;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_3200Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_3333Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_3840Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_7680Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_7680Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_8000Hz;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_6400Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_6667Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_7680Hz;
          break;
      }
      break;

    default:
      *val = ISM6HG256X_ODR_OFF;
      break;
  }

  return ret;
}


int32_t ism6hg256x_hg_xl_data_rate_set(const stmdev_ctx_t *ctx,
                                       ism6hg256x_hg_xl_data_rate_t val,
                                       uint8_t reg_out_en)
{
  ism6hg256x_ctrl1_t ctrl1 = {0};
  ism6hg256x_ctrl2_t ctrl2 = {0};
  ism6hg256x_ctrl1_xl_hg_t ctrl1_xl_hg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL1, (uint8_t *)&ctrl1, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&ctrl2, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL1_XL_HG, (uint8_t *)&ctrl1_xl_hg, 1);
  if (ret != 0)
  {
    goto exit;
  }

  if (val != ISM6HG256X_HG_XL_ODR_OFF && ctrl1.odr_xl != (uint8_t)ISM6HG256X_ODR_OFF &&
      ctrl1.op_mode_xl != (uint8_t)ISM6HG256X_XL_HIGH_PERFORMANCE_MD &&
      ctrl1.op_mode_xl != (uint8_t)ISM6HG256X_XL_HIGH_ACCURACY_ODR_MD)
  {
    ret = -1;
    goto exit;
  }

  // if xl or gy are ON in odr triggered mode, high-g xl cannot be turned on
  if ((ctrl1.odr_xl != (uint8_t)ISM6HG256X_ODR_OFF &&
       ctrl1.op_mode_xl == (uint8_t)ISM6HG256X_XL_ODR_TRIGGERED_MD) ||
      (ctrl2.odr_g != (uint8_t)ISM6HG256X_ODR_OFF &&
       ctrl2.op_mode_g == (uint8_t)ISM6HG256X_GY_ODR_TRIGGERED_MD))
  {
    ret = -1;
    goto exit;
  }

  ctrl1_xl_hg.odr_xl_hg = (uint8_t)val & 0x07U;
  ctrl1_xl_hg.xl_hg_regout_en = reg_out_en & 0x1U;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL1_XL_HG, (uint8_t *)&ctrl1_xl_hg, 1);

exit:
  return ret;
}


int32_t ism6hg256x_hg_xl_data_rate_get(const stmdev_ctx_t *ctx,
                                       ism6hg256x_hg_xl_data_rate_t *val,
                                       uint8_t *reg_out_en)
{
  ism6hg256x_ctrl1_xl_hg_t ctrl1_xl_hg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL1_XL_HG, (uint8_t *)&ctrl1_xl_hg, 1);
  if (ret != 0)
  {
    return ret;
  }

  *reg_out_en = ctrl1_xl_hg.xl_hg_regout_en;

  switch (ctrl1_xl_hg.odr_xl_hg)
  {
    case 0x00:
      *val = ISM6HG256X_HG_XL_ODR_OFF;
      break;

    case 0x03:
      *val = ISM6HG256X_HG_XL_ODR_AT_480Hz;
      break;

    case 0x04:
      *val = ISM6HG256X_HG_XL_ODR_AT_960Hz;
      break;

    case 0x05:
      *val = ISM6HG256X_HG_XL_ODR_AT_1920Hz;
      break;

    case 0x06:
      *val = ISM6HG256X_HG_XL_ODR_AT_3840Hz;
      break;

    case 0x07:
      *val = ISM6HG256X_HG_XL_ODR_AT_7680Hz;
      break;

    default:
      *val = ISM6HG256X_HG_XL_ODR_OFF;
      break;
  }

  return ret;
}

int32_t ism6hg256x_xl_mode_set(const stmdev_ctx_t *ctx, ism6hg256x_xl_mode_t val)
{
  ism6hg256x_ctrl1_t ctrl1 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL1, (uint8_t *)&ctrl1, 1);

  if (ret == 0)
  {
    ctrl1.op_mode_xl = (uint8_t)val & 0x07U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL1, (uint8_t *)&ctrl1, 1);
  }

  return ret;
}


int32_t ism6hg256x_xl_mode_get(const stmdev_ctx_t *ctx, ism6hg256x_xl_mode_t *val)
{
  ism6hg256x_ctrl1_t ctrl1 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL1, (uint8_t *)&ctrl1, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl1.op_mode_xl)
  {
    case 0x00:
      *val = ISM6HG256X_XL_HIGH_PERFORMANCE_MD;
      break;

    case 0x01:
      *val = ISM6HG256X_XL_HIGH_ACCURACY_ODR_MD;
      break;

    case 0x03:
      *val = ISM6HG256X_XL_ODR_TRIGGERED_MD;
      break;

    case 0x04:
      *val = ISM6HG256X_XL_LOW_POWER_2_AVG_MD;
      break;

    case 0x05:
      *val = ISM6HG256X_XL_LOW_POWER_4_AVG_MD;
      break;

    case 0x06:
      *val = ISM6HG256X_XL_LOW_POWER_8_AVG_MD;
      break;

    case 0x07:
      *val = ISM6HG256X_XL_NORMAL_MD;
      break;

    default:
      *val = ISM6HG256X_XL_HIGH_PERFORMANCE_MD;
      break;
  }

  return ret;
}

/**
  * @brief  Gyroscope output data rate (ODR) selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      ism6hg256x_data_rate_t enum
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t ism6hg256x_gy_data_rate_set(const stmdev_ctx_t *ctx,
                                    ism6hg256x_data_rate_t val)
{
  ism6hg256x_ctrl2_t ctrl2 = {0};
  ism6hg256x_haodr_cfg_t haodr = {0};
  uint8_t sel = 0;
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&ctrl2, 1);
  if (ret != 0)
  {
    return ret;
  }
  ctrl2.odr_g = (uint8_t)val & 0x0Fu;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&ctrl2, 1);
  if (ret != 0)
  {
    return ret;
  }

  sel = ((uint8_t)val >> 4) & 0xFU;
  if (sel != 0U)
  {
    ret += ism6hg256x_read_reg(ctx, ISM6HG256X_HAODR_CFG, (uint8_t *)&haodr, 1);
    if (ret != 0)
    {
      return ret;
    }
    haodr.haodr_sel = sel & 0x03U;
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_HAODR_CFG, (uint8_t *)&haodr, 1);
  }

  return ret;
}


int32_t ism6hg256x_gy_data_rate_get(const stmdev_ctx_t *ctx,
                                    ism6hg256x_data_rate_t *val)
{
  ism6hg256x_ctrl2_t ctrl2 = {0};
  ism6hg256x_haodr_cfg_t haodr = {0};
  uint8_t sel = 0;
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&ctrl2, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_HAODR_CFG, (uint8_t *)&haodr, 1);
  if (ret != 0)
  {
    return ret;
  }

  sel = haodr.haodr_sel;

  switch (ctrl2.odr_g)
  {
    case 0x00:
      *val = ISM6HG256X_ODR_OFF;
      break;

    case 0x01:
      *val = ISM6HG256X_ODR_AT_1Hz875;
      break;

    case 0x02:
      *val = ISM6HG256X_ODR_AT_7Hz5;
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_15Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_15Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_15Hz625;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_12Hz5;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_13Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_15Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_30Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_30Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_31Hz25;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_25Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_26Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_30Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_60Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_60Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_62Hz5;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_50Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_52Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_60Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_120Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_120Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_125Hz;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_100Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_104Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_120Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_240Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_240Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_250Hz;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_200Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_208Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_240Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_480Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_480Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_500Hz;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_400Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_417Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_480Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_960Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_960Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_1000Hz;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_800Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_833Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_960Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_1920Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_1920Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_2000Hz;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_1600Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_1667Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_1920Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_3840Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_3840Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_4000Hz;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_3200Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_3333Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_3840Hz;
          break;
      }
      break;

    case (uint8_t)ISM6HG256X_ODR_AT_7680Hz:
      switch (sel)
      {
        case 0:
          *val = ISM6HG256X_ODR_AT_7680Hz;
          break;
        case 1:
          *val = ISM6HG256X_ODR_HA01_AT_8000Hz;
          break;
        case 2:
          *val = ISM6HG256X_ODR_HA02_AT_6400Hz;
          break;
        case 3:
          *val = ISM6HG256X_ODR_HA03_AT_6667Hz;
          break;
        default:
          *val = ISM6HG256X_ODR_AT_7680Hz;
          break;
      }
      break;

    default:
      *val = ISM6HG256X_ODR_OFF;
      break;
  }


  return ret;
}

int32_t ism6hg256x_gy_mode_set(const stmdev_ctx_t *ctx, ism6hg256x_gy_mode_t val)
{
  ism6hg256x_ctrl2_t ctrl2 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&ctrl2, 1);
  if (ret == 0)
  {
    ctrl2.op_mode_g = (uint8_t)val & 0x07U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&ctrl2, 1);
  }

  return ret;
}


int32_t ism6hg256x_gy_mode_get(const stmdev_ctx_t *ctx, ism6hg256x_gy_mode_t *val)
{
  ism6hg256x_ctrl2_t ctrl2 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&ctrl2, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl2.op_mode_g)
  {
    case 0x00:
      *val = ISM6HG256X_GY_HIGH_PERFORMANCE_MD;
      break;

    case 0x01:
      *val = ISM6HG256X_GY_HIGH_ACCURACY_ODR_MD;
      break;

    case 0x03:
      *val = ISM6HG256X_GY_ODR_TRIGGERED_MD;
      break;

    case 0x04:
      *val = ISM6HG256X_GY_SLEEP_MD;
      break;

    case 0x05:
      *val = ISM6HG256X_GY_LOW_POWER_MD;
      break;

    default:
      *val = ISM6HG256X_GY_HIGH_PERFORMANCE_MD;
      break;
  }

  return ret;
}


int32_t ism6hg256x_auto_increment_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_ctrl3_t ctrl3 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL3, (uint8_t *)&ctrl3, 1);
  if (ret == 0)
  {
    ctrl3.if_inc = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL3, (uint8_t *)&ctrl3, 1);
  }

  return ret;
}

int32_t ism6hg256x_auto_increment_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_ctrl3_t ctrl3 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL3, (uint8_t *)&ctrl3, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = ctrl3.if_inc;

  return ret;
}


int32_t ism6hg256x_block_data_update_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_ctrl3_t ctrl3 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL3, (uint8_t *)&ctrl3, 1);

  if (ret == 0)
  {
    ctrl3.bdu = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL3, (uint8_t *)&ctrl3, 1);
  }

  return ret;
}

int32_t ism6hg256x_block_data_update_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_ctrl3_t ctrl3 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL3, (uint8_t *)&ctrl3, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = ctrl3.bdu;

  return ret;
}


int32_t ism6hg256x_odr_trig_cfg_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_odr_trig_cfg_t odr_trig = {0};
  int32_t ret = 0;

  if (val >= 1U && val <= 3U)
  {
    return -1;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_ODR_TRIG_CFG, (uint8_t *)&odr_trig, 1);

  if (ret == 0)
  {
    odr_trig.odr_trig_nodr = val;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_ODR_TRIG_CFG, (uint8_t *)&odr_trig, 1);
  }

  return ret;
}


int32_t ism6hg256x_odr_trig_cfg_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_odr_trig_cfg_t odr_trig = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_ODR_TRIG_CFG, (uint8_t *)&odr_trig, 1);
  if (ret != 0)
  {
    return ret;
  }
  *val = odr_trig.odr_trig_nodr;

  return ret;
}


int32_t ism6hg256x_data_ready_mode_set(const stmdev_ctx_t *ctx,
                                       ism6hg256x_data_ready_mode_t val)
{
  ism6hg256x_ctrl4_t ctrl4 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL4, (uint8_t *)&ctrl4, 1);

  if (ret == 0)
  {
    ctrl4.drdy_pulsed = (uint8_t)val & 0x1U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL4, (uint8_t *)&ctrl4, 1);
  }

  return ret;
}


int32_t ism6hg256x_data_ready_mode_get(const stmdev_ctx_t *ctx,
                                       ism6hg256x_data_ready_mode_t *val)
{
  ism6hg256x_ctrl4_t ctrl4 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL4, (uint8_t *)&ctrl4, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl4.drdy_pulsed)
  {
    case 0x00:
      *val = ISM6HG256X_DRDY_LATCHED;
      break;

    case 0x01:
      *val = ISM6HG256X_DRDY_PULSED;
      break;

    default:
      *val = ISM6HG256X_DRDY_LATCHED;
      break;
  }

  return ret;
}


int32_t ism6hg256x_interrupt_enable_set(const stmdev_ctx_t *ctx,
                                        ism6hg256x_interrupt_mode_t val)
{
  ism6hg256x_tap_cfg0_t cfg = {0};
  ism6hg256x_functions_enable_t func = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&func, 1);
  if (ret != 0)
  {
    return ret;
  }

  func.interrupts_enable = val.enable;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&func, 1);

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  cfg.lir = val.lir;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&cfg, 1);

  return ret;
}


int32_t ism6hg256x_interrupt_enable_get(const stmdev_ctx_t *ctx,
                                        ism6hg256x_interrupt_mode_t *val)
{
  ism6hg256x_tap_cfg0_t cfg = {0};
  ism6hg256x_functions_enable_t func = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&func, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->enable = func.interrupts_enable;
  val->lir = cfg.lir;

  return ret;
}


int32_t ism6hg256x_gy_full_scale_set(const stmdev_ctx_t *ctx,
                                     ism6hg256x_gy_full_scale_t val)
{
  ism6hg256x_ctrl2_t ctrl2 = {0};
  ism6hg256x_ctrl2_t prev_ctrl2 = {0};
  ism6hg256x_ctrl6_t ctrl6 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL6, (uint8_t *)&ctrl6, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&ctrl2, 1);
  prev_ctrl2 = ctrl2;

  if (ret != 0)
  {
    goto exit;
  }

  // For the correct operation of the device, the user must set a
  // configuration from 001 to 101 when the gyroscope is in power-down mode.
  if (ctrl2.odr_g != (uint8_t)ISM6HG256X_ODR_OFF)
  {
    ctrl2.odr_g = (uint8_t)ISM6HG256X_ODR_OFF;
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&ctrl2, 1);
  }

  ctrl6.fs_g = (uint8_t)val & 0x07u;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL6, (uint8_t *)&ctrl6, 1);

  // restore previous odr set
  if (prev_ctrl2.odr_g != (uint8_t)ISM6HG256X_ODR_OFF)
  {
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&prev_ctrl2, 1);
  }

exit:
  return ret;
}


int32_t ism6hg256x_gy_full_scale_get(const stmdev_ctx_t *ctx,
                                     ism6hg256x_gy_full_scale_t *val)
{
  ism6hg256x_ctrl6_t ctrl6 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL6, (uint8_t *)&ctrl6, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl6.fs_g)
  {
    case 0x01:
      *val = ISM6HG256X_250dps;
      break;

    case 0x02:
      *val = ISM6HG256X_500dps;
      break;

    case 0x03:
      *val = ISM6HG256X_1000dps;
      break;

    case 0x04:
      *val = ISM6HG256X_2000dps;
      break;

    case 0x05:
      *val = ISM6HG256X_4000dps;
      break;

    default:
      *val = ISM6HG256X_250dps;
      break;
  }

  return ret;
}


int32_t ism6hg256x_xl_full_scale_set(const stmdev_ctx_t *ctx,
                                     ism6hg256x_xl_full_scale_t val)
{
  ism6hg256x_ctrl8_t ctrl8 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL8, (uint8_t *)&ctrl8, 1);

  if (ret == 0)
  {
    ctrl8.fs_xl = (uint8_t)val & 0x3U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL8, (uint8_t *)&ctrl8, 1);
  }

  return ret;
}


int32_t ism6hg256x_xl_full_scale_get(const stmdev_ctx_t *ctx,
                                     ism6hg256x_xl_full_scale_t *val)
{
  ism6hg256x_ctrl8_t ctrl8 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL8, (uint8_t *)&ctrl8, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl8.fs_xl)
  {
    case 0x00:
      *val = ISM6HG256X_2g;
      break;

    case 0x01:
      *val = ISM6HG256X_4g;
      break;

    case 0x02:
      *val = ISM6HG256X_8g;
      break;

    case 0x03:
      *val = ISM6HG256X_16g;
      break;

    default:
      *val = ISM6HG256X_2g;
      break;
  }

  return ret;
}


int32_t ism6hg256x_hg_xl_full_scale_set(const stmdev_ctx_t *ctx,
                                        ism6hg256x_hg_xl_full_scale_t val)
{
  ism6hg256x_ctrl1_xl_hg_t ctrl1 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL1_XL_HG, (uint8_t *)&ctrl1, 1);

  if (ret == 0)
  {
    ctrl1.fs_xl_hg = (uint8_t)val & 0x7U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL1_XL_HG, (uint8_t *)&ctrl1, 1);
  }

  return ret;
}


int32_t ism6hg256x_hg_xl_full_scale_get(const stmdev_ctx_t *ctx,
                                        ism6hg256x_hg_xl_full_scale_t *val)
{
  ism6hg256x_ctrl1_xl_hg_t ctrl1 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL1_XL_HG, (uint8_t *)&ctrl1, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl1.fs_xl_hg)
  {
    case 0x00:
      *val = ISM6HG256X_32g;
      break;

    case 0x01:
      *val = ISM6HG256X_64g;
      break;

    case 0x02:
      *val = ISM6HG256X_128g;
      break;

    case 0x03:
      *val = ISM6HG256X_256g;
      break;

    default:
      *val = ISM6HG256X_32g;
      break;
  }

  return ret;
}


int32_t ism6hg256x_xl_self_test_set(const stmdev_ctx_t *ctx, ism6hg256x_self_test_t val)
{
  ism6hg256x_ctrl10_t ctrl10 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL10, (uint8_t *)&ctrl10, 1);

  if (ret == 0)
  {
    ctrl10.st_xl = (uint8_t)val & 0x3U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL10, (uint8_t *)&ctrl10, 1);
  }

  return ret;
}


int32_t ism6hg256x_xl_self_test_get(const stmdev_ctx_t *ctx, ism6hg256x_self_test_t *val)
{
  ism6hg256x_ctrl10_t ctrl10 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL10, (uint8_t *)&ctrl10, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl10.st_xl)
  {
    case 0x00:
      *val = ISM6HG256X_ST_DISABLE;
      break;

    case 0x01:
      *val = ISM6HG256X_ST_POSITIVE;
      break;

    case 0x02:
      *val = ISM6HG256X_ST_NEGATIVE;
      break;

    default:
      *val = ISM6HG256X_ST_DISABLE;
      break;
  }

  return ret;
}


int32_t ism6hg256x_gy_self_test_set(const stmdev_ctx_t *ctx, ism6hg256x_self_test_t val)
{
  ism6hg256x_ctrl10_t ctrl10 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL10, (uint8_t *)&ctrl10, 1);

  if (ret == 0)
  {
    ctrl10.st_g = (uint8_t)val & 0x3U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL10, (uint8_t *)&ctrl10, 1);
  }

  return ret;
}


int32_t ism6hg256x_gy_self_test_get(const stmdev_ctx_t *ctx, ism6hg256x_self_test_t *val)
{
  ism6hg256x_ctrl10_t ctrl10 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL10, (uint8_t *)&ctrl10, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl10.st_g)
  {
    case 0x00:
      *val = ISM6HG256X_ST_DISABLE;
      break;

    case 0x01:
      *val = ISM6HG256X_ST_POSITIVE;
      break;

    case 0x02:
      *val = ISM6HG256X_ST_NEGATIVE;
      break;

    default:
      *val = ISM6HG256X_ST_DISABLE;
      break;
  }

  return ret;
}


int32_t ism6hg256x_hg_xl_self_test_set(const stmdev_ctx_t *ctx, ism6hg256x_self_test_t val)
{
  ism6hg256x_ctrl2_xl_hg_t ctrl2_xl_hg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL2_XL_HG, (uint8_t *)&ctrl2_xl_hg, 1);

  if (ret == 0)
  {
    ctrl2_xl_hg.xl_hg_st = (uint8_t)val & 0x3U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL2_XL_HG, (uint8_t *)&ctrl2_xl_hg, 1);
  }

  return ret;
}


int32_t ism6hg256x_hg_xl_self_test_get(const stmdev_ctx_t *ctx, ism6hg256x_self_test_t *val)
{
  ism6hg256x_ctrl2_xl_hg_t ctrl2_xl_hg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL2_XL_HG, (uint8_t *)&ctrl2_xl_hg, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl2_xl_hg.xl_hg_st)
  {
    case 0x00:
      *val = ISM6HG256X_ST_DISABLE;
      break;

    case 0x01:
      *val = ISM6HG256X_ST_POSITIVE;
      break;

    case 0x02:
      *val = ISM6HG256X_ST_NEGATIVE;
      break;

    default:
      *val = ISM6HG256X_ST_DISABLE;
      break;
  }

  return ret;
}

int32_t ism6hg256x_ois_xl_self_test_set(const stmdev_ctx_t *ctx, ism6hg256x_self_test_t val)
{
  ism6hg256x_if2_int_ois_t if2_int_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF2_INT_OIS, (uint8_t *)&if2_int_ois, 1);

  if (ret == 0)
  {
    if2_int_ois.st_xl_ois = ((uint8_t)val & 0x3U);
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_IF2_INT_OIS, (uint8_t *)&if2_int_ois, 1);
  }

  return ret;
}


int32_t ism6hg256x_ois_xl_self_test_get(const stmdev_ctx_t *ctx, ism6hg256x_self_test_t *val)
{
  ism6hg256x_if2_int_ois_t if2_int_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF2_INT_OIS, (uint8_t *)&if2_int_ois, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (if2_int_ois.st_xl_ois)
  {
    case 0x00:
      *val = ISM6HG256X_ST_DISABLE;
      break;

    case 0x01:
      *val = ISM6HG256X_ST_POSITIVE;
      break;

    case 0x02:
      *val = ISM6HG256X_ST_NEGATIVE;
      break;

    default:
      *val = ISM6HG256X_ST_DISABLE;
      break;
  }

  return ret;
}


int32_t ism6hg256x_ois_gy_self_test_set(const stmdev_ctx_t *ctx,
                                        ism6hg256x_ois_gy_self_test_t val)
{
  ism6hg256x_if2_int_ois_t if2_int_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF2_INT_OIS, (uint8_t *)&if2_int_ois, 1);

  if (ret == 0)
  {
    if2_int_ois.st_g_ois = ((uint8_t)val & 0x3U);
    if2_int_ois.st_ois_clampdis = ((uint8_t)val & 0x04U) >> 2;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_IF2_INT_OIS, (uint8_t *)&if2_int_ois, 1);
  }

  return ret;
}


int32_t ism6hg256x_ois_gy_self_test_get(const stmdev_ctx_t *ctx,
                                        ism6hg256x_ois_gy_self_test_t *val)
{
  ism6hg256x_if2_int_ois_t if2_int_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF2_INT_OIS, (uint8_t *)&if2_int_ois, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (if2_int_ois.st_g_ois)
  {
    case 0x00:
      *val = ISM6HG256X_OIS_GY_ST_DISABLE;
      break;

    case 0x01:
      *val = (if2_int_ois.st_ois_clampdis == 1U) ? ISM6HG256X_OIS_GY_ST_CLAMP_POS :
             ISM6HG256X_OIS_GY_ST_POSITIVE;
      break;

    case 0x02:
      *val = (if2_int_ois.st_ois_clampdis == 1U) ? ISM6HG256X_OIS_GY_ST_CLAMP_NEG :
             ISM6HG256X_OIS_GY_ST_NEGATIVE;
      break;

    default:
      *val = ISM6HG256X_OIS_GY_ST_DISABLE;
      break;
  }

  return ret;
}


int32_t ism6hg256x_hg_wake_up_cfg_set(const stmdev_ctx_t *ctx,
                                      ism6hg256x_hg_wake_up_cfg_t val)
{
  ism6hg256x_hg_functions_enable_t hg_func = {0};
  ism6hg256x_hg_wake_up_ths_t hg_wake_up_ths = {0};
  uint8_t reg[2] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_HG_FUNCTIONS_ENABLE, (uint8_t *)&hg_func, 1);
  if (ret != 0)
  {
    return ret;
  }

  hg_func.hg_shock_dur = val.hg_shock_dur;
  hg_wake_up_ths.hg_wk_ths = val.hg_wakeup_ths;

  bytecpy(&reg[0], (uint8_t *)&hg_func);
  bytecpy(&reg[1], (uint8_t *)&hg_wake_up_ths);

  return ism6hg256x_write_reg(ctx, ISM6HG256X_HG_FUNCTIONS_ENABLE, reg, 2);
}


int32_t ism6hg256x_hg_wake_up_cfg_get(const stmdev_ctx_t *ctx,
                                      ism6hg256x_hg_wake_up_cfg_t *val)
{
  ism6hg256x_hg_functions_enable_t hg_func = {0};
  ism6hg256x_hg_wake_up_ths_t hg_wake_up_ths = {0};
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_HG_FUNCTIONS_ENABLE, (uint8_t *)buff, 2);
  if (ret != 0)
  {
    return ret;
  }

  bytecpy((uint8_t *)&hg_func, &buff[0]);
  bytecpy((uint8_t *)&hg_wake_up_ths, &buff[1]);

  val->hg_shock_dur = hg_func.hg_shock_dur;
  val->hg_wakeup_ths = hg_wake_up_ths.hg_wk_ths;

  return ret;
}


int32_t ism6hg256x_hg_wu_interrupt_cfg_set(const stmdev_ctx_t *ctx,
                                           ism6hg256x_hg_wu_interrupt_cfg_t val)
{
  ism6hg256x_hg_functions_enable_t hg_func = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_HG_FUNCTIONS_ENABLE, (uint8_t *)&hg_func, 1);
  if (ret != 0)
  {
    return ret;
  }

  hg_func.hg_interrupts_enable        = val.hg_interrupts_enable;
  hg_func.hg_wu_change_int_sel        = val.hg_wakeup_int_sel;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_HG_FUNCTIONS_ENABLE, (uint8_t *)&hg_func, 1);

  return ret;
}


int32_t ism6hg256x_hg_wu_interrupt_cfg_get(const stmdev_ctx_t *ctx,
                                           ism6hg256x_hg_wu_interrupt_cfg_t *val)
{
  ism6hg256x_hg_functions_enable_t hg_func = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_HG_FUNCTIONS_ENABLE, (uint8_t *)&hg_func, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->hg_interrupts_enable = hg_func.hg_interrupts_enable;
  val->hg_wakeup_int_sel    = hg_func.hg_wu_change_int_sel;

  return ret;
}


int32_t ism6hg256x_hg_emb_usr_off_correction_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_emb_func_cfg_t emb_func_cfg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_CFG, (uint8_t *)&emb_func_cfg, 1);

  if (ret == 0)
  {
    emb_func_cfg.hg_usr_off_on_emb_func = (uint8_t)val & 0x1U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_CFG, (uint8_t *)&emb_func_cfg, 1);
  }

  return ret;
}


int32_t ism6hg256x_hg_emb_usr_off_correction_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_emb_func_cfg_t emb_func_cfg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_CFG, (uint8_t *)&emb_func_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = emb_func_cfg.hg_usr_off_on_emb_func;

  return ret;
}


int32_t ism6hg256x_hg_wu_usr_off_correction_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_ctrl2_xl_hg_t ctrl2_xl_hg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL2_XL_HG, (uint8_t *)&ctrl2_xl_hg, 1);

  if (ret == 0)
  {
    ctrl2_xl_hg.hg_usr_off_on_wu = (uint8_t)val & 0x1U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL2_XL_HG, (uint8_t *)&ctrl2_xl_hg, 1);
  }

  return ret;
}


int32_t ism6hg256x_hg_wu_usr_off_correction_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_ctrl2_xl_hg_t ctrl2_xl_hg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL2_XL_HG, (uint8_t *)&ctrl2_xl_hg, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = ctrl2_xl_hg.hg_usr_off_on_wu;

  return ret;
}


int32_t ism6hg256x_hg_event_get(const stmdev_ctx_t *ctx, ism6hg256x_hg_event_t *val)
{
  ism6hg256x_all_int_src_t          int_src = {0};
  ism6hg256x_hg_wake_up_src_t       wup_src = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_ALL_INT_SRC, (uint8_t *)&int_src, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->hg_event = int_src.hg_ia;

  /* no High-g event */
  if (int_src.hg_ia != 0)
  {
    ret = ism6hg256x_read_reg(ctx, ISM6HG256X_HG_WAKE_UP_SRC, (uint8_t *)&wup_src, 1);
    if (ret != 0)
    {
      return ret;
    }

    val->hg_wakeup_z     = wup_src.hg_z_wu;
    val->hg_wakeup_y     = wup_src.hg_y_wu;
    val->hg_wakeup_x     = wup_src.hg_x_wu;
    val->hg_wakeup       = wup_src.hg_wu_ia;
    val->hg_wakeup_chg   = wup_src.hg_wu_change_ia;
    val->hg_shock        = wup_src.hg_shock_state;
    val->hg_shock_change = wup_src.hg_shock_change_ia;
  }

  return ret;
}


int32_t ism6hg256x_pin_int1_route_set(const stmdev_ctx_t *ctx,
                                      const ism6hg256x_pin_int_route_t *val)
{
  ism6hg256x_int1_ctrl_t           int1_ctrl = {0};
  ism6hg256x_md1_cfg_t             md1_cfg = {0};
  int32_t                       ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_INT1_CTRL, (uint8_t *)&int1_ctrl, 1);
  if (ret != 0)
  {
    return ret;
  }

  int1_ctrl.int1_drdy_xl       = val->drdy_xl;
  int1_ctrl.int1_drdy_g        = val->drdy_g;
  int1_ctrl.int1_fifo_th       = val->fifo_th;
  int1_ctrl.int1_fifo_ovr      = val->fifo_ovr;
  int1_ctrl.int1_fifo_full     = val->fifo_full;
  int1_ctrl.int1_cnt_bdr       = val->cnt_bdr;

  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_INT1_CTRL, (uint8_t *)&int1_ctrl, 1);
  if (ret != 0)
  {
    return ret;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&md1_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  md1_cfg.int1_shub            = val->shub;
  md1_cfg.int1_6d              = val->sixd;
  md1_cfg.int1_single_tap      = val->single_tap;
  md1_cfg.int1_double_tap      = val->double_tap;
  md1_cfg.int1_wu              = val->wakeup;
  md1_cfg.int1_ff              = val->freefall;
  md1_cfg.int1_sleep_change    = val->sleep_change;

  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&md1_cfg, 1);

  return ret;
}


int32_t ism6hg256x_pin_int1_route_get(const stmdev_ctx_t *ctx,
                                      ism6hg256x_pin_int_route_t *val)
{
  ism6hg256x_int1_ctrl_t           int1_ctrl = {0};
  ism6hg256x_md1_cfg_t             md1_cfg = {0};
  int32_t                       ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_INT1_CTRL, (uint8_t *)&int1_ctrl, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->drdy_xl   = int1_ctrl.int1_drdy_xl;
  val->drdy_g    = int1_ctrl.int1_drdy_g;
  val->fifo_th   = int1_ctrl.int1_fifo_th;
  val->fifo_ovr  = int1_ctrl.int1_fifo_ovr;
  val->fifo_full = int1_ctrl.int1_fifo_full;
  val->cnt_bdr   = int1_ctrl.int1_cnt_bdr;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&md1_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->shub         = md1_cfg.int1_shub;
  val->sixd         = md1_cfg.int1_6d;
  val->single_tap   = md1_cfg.int1_single_tap;
  val->double_tap   = md1_cfg.int1_double_tap;
  val->wakeup       = md1_cfg.int1_wu;
  val->freefall     = md1_cfg.int1_ff;
  val->sleep_change = md1_cfg.int1_sleep_change;

  return ret;
}


int32_t ism6hg256x_pin_int2_route_set(const stmdev_ctx_t *ctx,
                                      const ism6hg256x_pin_int_route_t *val)
{
  ism6hg256x_int2_ctrl_t           int2_ctrl = {0};
  ism6hg256x_ctrl4_t               ctrl4 = {0};
  ism6hg256x_md2_cfg_t             md2_cfg = {0};
  int32_t                       ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_INT2_CTRL, (uint8_t *)&int2_ctrl, 1);
  if (ret != 0)
  {
    return ret;
  }

  int2_ctrl.int2_drdy_xl          = val->drdy_xl;
  int2_ctrl.int2_drdy_g           = val->drdy_g;
  int2_ctrl.int2_fifo_th          = val->fifo_th;
  int2_ctrl.int2_fifo_ovr         = val->fifo_ovr;
  int2_ctrl.int2_fifo_full        = val->fifo_full;
  int2_ctrl.int2_cnt_bdr          = val->cnt_bdr;
  int2_ctrl.int2_drdy_g_eis       = val->drdy_g_eis;
  int2_ctrl.int2_emb_func_endop   = val->emb_func_endop;

  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_INT2_CTRL, (uint8_t *)&int2_ctrl, 1);
  if (ret != 0)
  {
    return ret;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL4, (uint8_t *)&ctrl4, 1);
  if (ret != 0)
  {
    return ret;
  }
  ctrl4.int2_drdy_temp         = val->drdy_temp;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL4, (uint8_t *)&ctrl4, 1);
  if (ret != 0)
  {
    return ret;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&md2_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  md2_cfg.int2_timestamp       = val->timestamp;
  md2_cfg.int2_6d              = val->sixd;
  md2_cfg.int2_single_tap      = val->single_tap;
  md2_cfg.int2_double_tap      = val->double_tap;
  md2_cfg.int2_wu              = val->wakeup;
  md2_cfg.int2_ff              = val->freefall;
  md2_cfg.int2_sleep_change    = val->sleep_change;

  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&md2_cfg, 1);

  return ret;
}


int32_t ism6hg256x_pin_int2_route_get(const stmdev_ctx_t *ctx,
                                      ism6hg256x_pin_int_route_t *val)
{
  ism6hg256x_int2_ctrl_t           int2_ctrl = {0};
  ism6hg256x_ctrl4_t               ctrl4 = {0};
  ism6hg256x_md2_cfg_t             md2_cfg = {0};
  int32_t                       ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_INT2_CTRL, (uint8_t *)&int2_ctrl, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->drdy_xl        = int2_ctrl.int2_drdy_xl;
  val->drdy_g         = int2_ctrl.int2_drdy_g;
  val->fifo_th        = int2_ctrl.int2_fifo_th;
  val->fifo_ovr       = int2_ctrl.int2_fifo_ovr;
  val->fifo_full      = int2_ctrl.int2_fifo_full;
  val->cnt_bdr        = int2_ctrl.int2_cnt_bdr;
  val->drdy_g_eis     = int2_ctrl.int2_drdy_g_eis;
  val->emb_func_endop = int2_ctrl.int2_emb_func_endop;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL4, (uint8_t *)&ctrl4, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->drdy_temp      = ctrl4.int2_drdy_temp;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&md2_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->timestamp      = md2_cfg.int2_timestamp;
  val->sixd           = md2_cfg.int2_6d;
  val->single_tap     = md2_cfg.int2_single_tap;
  val->double_tap     = md2_cfg.int2_double_tap;
  val->wakeup         = md2_cfg.int2_wu;
  val->freefall       = md2_cfg.int2_ff;
  val->sleep_change   = md2_cfg.int2_sleep_change;

  return ret;
}


int32_t ism6hg256x_pin_int1_route_hg_set(const stmdev_ctx_t *ctx,
                                         const ism6hg256x_pin_int_route_t *val)
{
  ism6hg256x_ctrl7_t               ctrl7 = {0};
  ism6hg256x_hg_functions_enable_t hg_func = {0};
  ism6hg256x_inactivity_ths_t      reg_shock = {0};
  int32_t                       ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL7, (uint8_t *)&ctrl7, 1);
  if (ret != 0)
  {
    return ret;
  }

  ctrl7.int1_drdy_xl_hg        = val->drdy_hg_xl;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL7, (uint8_t *)&ctrl7, 1);
  if (ret != 0)
  {
    return ret;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_HG_FUNCTIONS_ENABLE, (uint8_t *)&hg_func, 1);
  if (ret != 0)
  {
    return ret;
  }

  hg_func.int1_hg_wu        = val->hg_wakeup;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_HG_FUNCTIONS_ENABLE, (uint8_t *)&hg_func, 1);

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_INACTIVITY_THS, (uint8_t *)&reg_shock, 1);
  if (ret != 0)
  {
    return ret;
  }

  reg_shock.int1_hg_shock_change        = val->hg_shock_change;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_INACTIVITY_THS, (uint8_t *)&reg_shock, 1);

  return ret;
}


int32_t ism6hg256x_pin_int1_route_hg_get(const stmdev_ctx_t *ctx,
                                         ism6hg256x_pin_int_route_t *val)
{
  ism6hg256x_ctrl7_t               ctrl7 = {0};
  ism6hg256x_hg_functions_enable_t hg_func = {0};
  ism6hg256x_inactivity_ths_t      reg_shock = {0};
  int32_t                       ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL7, (uint8_t *)&ctrl7, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->drdy_hg_xl = ctrl7.int1_drdy_xl_hg;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_HG_FUNCTIONS_ENABLE, (uint8_t *)&hg_func, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->hg_wakeup = hg_func.int1_hg_wu;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_INACTIVITY_THS, (uint8_t *)&reg_shock, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->hg_shock_change = reg_shock.int1_hg_shock_change;

  return ret;
}


int32_t ism6hg256x_pin_int2_route_hg_set(const stmdev_ctx_t *ctx,
                                         const ism6hg256x_pin_int_route_t *val)
{
  ism6hg256x_ctrl7_t               ctrl7 = {0};
  ism6hg256x_hg_functions_enable_t hg_func = {0};
  ism6hg256x_inactivity_ths_t      reg_shock = {0};
  int32_t                       ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL7, (uint8_t *)&ctrl7, 1);
  if (ret != 0)
  {
    return ret;
  }

  ctrl7.int2_drdy_xl_hg        = val->drdy_hg_xl;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL7, (uint8_t *)&ctrl7, 1);

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_HG_FUNCTIONS_ENABLE, (uint8_t *)&hg_func, 1);
  if (ret != 0)
  {
    return ret;
  }

  hg_func.int2_hg_wu        = val->hg_wakeup;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_HG_FUNCTIONS_ENABLE, (uint8_t *)&hg_func, 1);

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_INACTIVITY_THS, (uint8_t *)&reg_shock, 1);
  if (ret != 0)
  {
    return ret;
  }

  reg_shock.int2_hg_shock_change        = val->hg_shock_change;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_INACTIVITY_THS, (uint8_t *)&reg_shock, 1);

  return ret;
}


int32_t ism6hg256x_pin_int2_route_hg_get(const stmdev_ctx_t *ctx,
                                         ism6hg256x_pin_int_route_t *val)
{
  ism6hg256x_ctrl7_t               ctrl7 = {0};
  ism6hg256x_hg_functions_enable_t hg_func = {0};
  ism6hg256x_inactivity_ths_t      reg_shock = {0};
  int32_t                       ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL7, (uint8_t *)&ctrl7, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->drdy_hg_xl = ctrl7.int2_drdy_xl_hg;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_HG_FUNCTIONS_ENABLE, (uint8_t *)&hg_func, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->hg_wakeup = hg_func.int2_hg_wu;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_INACTIVITY_THS, (uint8_t *)&reg_shock, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->hg_shock_change = reg_shock.int2_hg_shock_change;

  return ret;
}


int32_t ism6hg256x_pin_int1_route_embedded_set(const stmdev_ctx_t *ctx,
                                               const ism6hg256x_pin_int_route_t *val)
{
  ism6hg256x_md1_cfg_t             md1_cfg = {0};
  ism6hg256x_emb_func_int1_t       emb_func_int1 = {0};
  ism6hg256x_fsm_int1_t            fsm_int1 = {0};
  ism6hg256x_mlc_int1_t            mlc_int1 = {0};
  int32_t                       ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&md1_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  md1_cfg.int1_emb_func = 1;

  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_MD1_CFG, (uint8_t *)&md1_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  /* Embedded Functions */
  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1);
  if (ret != 0)
  {
    goto exit;
  }

  emb_func_int1.int1_step_detector = val->step_detector;
  emb_func_int1.int1_tilt = val->tilt;
  emb_func_int1.int1_sig_mot = val->sig_mot;

  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1);
  if (ret != 0)
  {
    goto exit;
  }

  /* FSM */
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_FSM_INT1, (uint8_t *)&fsm_int1, 1);
  if (ret != 0)
  {
    goto exit;
  }

  fsm_int1.int1_fsm1 = val->fsm1;
  fsm_int1.int1_fsm2 = val->fsm2;
  fsm_int1.int1_fsm3 = val->fsm3;
  fsm_int1.int1_fsm4 = val->fsm4;
  fsm_int1.int1_fsm5 = val->fsm5;
  fsm_int1.int1_fsm6 = val->fsm6;
  fsm_int1.int1_fsm7 = val->fsm7;
  fsm_int1.int1_fsm8 = val->fsm8;

  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_FSM_INT1, (uint8_t *)&fsm_int1, 1);
  if (ret != 0)
  {
    goto exit;
  }

  /* MLC */
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_MLC_INT1, (uint8_t *)&mlc_int1, 1);
  if (ret != 0)
  {
    goto exit;
  }

  mlc_int1.int1_mlc1 = val->mlc1;
  mlc_int1.int1_mlc2 = val->mlc2;
  mlc_int1.int1_mlc3 = val->mlc3;
  mlc_int1.int1_mlc4 = val->mlc4;
  mlc_int1.int1_mlc5 = val->mlc5;
  mlc_int1.int1_mlc6 = val->mlc6;
  mlc_int1.int1_mlc7 = val->mlc7;
  mlc_int1.int1_mlc8 = val->mlc8;

  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_MLC_INT1, (uint8_t *)&mlc_int1, 1);
  if (ret != 0)
  {
    goto exit;
  }

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_pin_int1_route_embedded_get(const stmdev_ctx_t *ctx,
                                               ism6hg256x_pin_int_route_t *val)
{
  ism6hg256x_emb_func_int1_t       emb_func_int1 = {0};
  ism6hg256x_fsm_int1_t            fsm_int1 = {0};
  ism6hg256x_mlc_int1_t            mlc_int1 = {0};
  int32_t                       ret = 0;

  /* Embedded Functions */
  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_INT1, (uint8_t *)&emb_func_int1, 1);
  if (ret != 0)
  {
    goto exit;
  }

  val->step_detector = emb_func_int1.int1_step_detector;
  val->sig_mot       = emb_func_int1.int1_sig_mot;
  val->tilt          = emb_func_int1.int1_tilt;

  /* FSM */
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_FSM_INT1, (uint8_t *)&fsm_int1, 1);
  if (ret != 0)
  {
    goto exit;
  }

  val->fsm1 = fsm_int1.int1_fsm1;
  val->fsm2 = fsm_int1.int1_fsm2;
  val->fsm3 = fsm_int1.int1_fsm3;
  val->fsm4 = fsm_int1.int1_fsm4;
  val->fsm5 = fsm_int1.int1_fsm5;
  val->fsm6 = fsm_int1.int1_fsm6;
  val->fsm7 = fsm_int1.int1_fsm7;
  val->fsm8 = fsm_int1.int1_fsm8;

  /* MLC */
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_MLC_INT1, (uint8_t *)&mlc_int1, 1);
  if (ret != 0)
  {
    goto exit;
  }

  val->mlc1 = mlc_int1.int1_mlc1;
  val->mlc2 = mlc_int1.int1_mlc2;
  val->mlc3 = mlc_int1.int1_mlc3;
  val->mlc4 = mlc_int1.int1_mlc4;
  val->mlc5 = mlc_int1.int1_mlc5;
  val->mlc6 = mlc_int1.int1_mlc6;
  val->mlc7 = mlc_int1.int1_mlc7;
  val->mlc8 = mlc_int1.int1_mlc8;

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_pin_int2_route_embedded_set(const stmdev_ctx_t *ctx,
                                               const ism6hg256x_pin_int_route_t *val)
{
  ism6hg256x_md2_cfg_t             md2_cfg = {0};
  ism6hg256x_emb_func_int2_t       emb_func_int2 = {0};
  ism6hg256x_fsm_int2_t            fsm_int2 = {0};
  ism6hg256x_mlc_int2_t            mlc_int2 = {0};
  int32_t                       ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&md2_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  md2_cfg.int2_emb_func = 1;

  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_MD2_CFG, (uint8_t *)&md2_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  /* Embedded Functions */
  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1);
  if (ret != 0)
  {
    goto exit;
  }

  emb_func_int2.int2_step_detector = val->step_detector;
  emb_func_int2.int2_tilt = val->tilt;
  emb_func_int2.int2_sig_mot = val->sig_mot;

  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1);
  if (ret != 0)
  {
    goto exit;
  }

  /* FSM */
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_FSM_INT2, (uint8_t *)&fsm_int2, 1);
  if (ret != 0)
  {
    goto exit;
  }

  fsm_int2.int2_fsm1 = val->fsm1;
  fsm_int2.int2_fsm2 = val->fsm2;
  fsm_int2.int2_fsm3 = val->fsm3;
  fsm_int2.int2_fsm4 = val->fsm4;
  fsm_int2.int2_fsm5 = val->fsm5;
  fsm_int2.int2_fsm6 = val->fsm6;
  fsm_int2.int2_fsm7 = val->fsm7;
  fsm_int2.int2_fsm8 = val->fsm8;

  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_FSM_INT2, (uint8_t *)&fsm_int2, 1);
  if (ret != 0)
  {
    goto exit;
  }

  /* MLC */
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_MLC_INT2, (uint8_t *)&mlc_int2, 1);
  if (ret != 0)
  {
    goto exit;
  }

  mlc_int2.int2_mlc1 = val->mlc1;
  mlc_int2.int2_mlc2 = val->mlc2;
  mlc_int2.int2_mlc3 = val->mlc3;
  mlc_int2.int2_mlc4 = val->mlc4;
  mlc_int2.int2_mlc5 = val->mlc5;
  mlc_int2.int2_mlc6 = val->mlc6;
  mlc_int2.int2_mlc7 = val->mlc7;
  mlc_int2.int2_mlc8 = val->mlc8;

  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_MLC_INT2, (uint8_t *)&mlc_int2, 1);
  if (ret != 0)
  {
    goto exit;
  }

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_pin_int2_route_embedded_get(const stmdev_ctx_t *ctx,
                                               ism6hg256x_pin_int_route_t *val)
{
  ism6hg256x_emb_func_int2_t       emb_func_int2 = {0};
  ism6hg256x_fsm_int2_t            fsm_int2 = {0};
  ism6hg256x_mlc_int2_t            mlc_int2 = {0};
  int32_t                       ret = 0;

  /* Embedded Functions */
  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_INT2, (uint8_t *)&emb_func_int2, 1);
  if (ret != 0)
  {
    goto exit;
  }

  val->step_detector = emb_func_int2.int2_step_detector;
  val->sig_mot       = emb_func_int2.int2_sig_mot;
  val->tilt          = emb_func_int2.int2_tilt;

  /* FSM */
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_FSM_INT2, (uint8_t *)&fsm_int2, 1);
  if (ret != 0)
  {
    goto exit;
  }

  val->fsm1 = fsm_int2.int2_fsm1;
  val->fsm2 = fsm_int2.int2_fsm2;
  val->fsm3 = fsm_int2.int2_fsm3;
  val->fsm4 = fsm_int2.int2_fsm4;
  val->fsm5 = fsm_int2.int2_fsm5;
  val->fsm6 = fsm_int2.int2_fsm6;
  val->fsm7 = fsm_int2.int2_fsm7;
  val->fsm8 = fsm_int2.int2_fsm8;

  /* MLC */
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_MLC_INT2, (uint8_t *)&mlc_int2, 1);
  if (ret != 0)
  {
    goto exit;
  }

  val->mlc1 = mlc_int2.int2_mlc1;
  val->mlc2 = mlc_int2.int2_mlc2;
  val->mlc3 = mlc_int2.int2_mlc3;
  val->mlc4 = mlc_int2.int2_mlc4;
  val->mlc5 = mlc_int2.int2_mlc5;
  val->mlc6 = mlc_int2.int2_mlc6;
  val->mlc7 = mlc_int2.int2_mlc7;
  val->mlc8 = mlc_int2.int2_mlc8;

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_all_sources_get(const stmdev_ctx_t *ctx,
                                   ism6hg256x_all_sources_t *val)
{
  ism6hg256x_emb_func_status_mainpage_t emb_func_status_mainpage = {0};
  ism6hg256x_emb_func_exec_status_t emb_func_exec_status = {0};
  ism6hg256x_fsm_status_mainpage_t fsm_status_mainpage = {0};
  ism6hg256x_mlc_status_mainpage_t mlc_status_mainpage = {0};
  ism6hg256x_functions_enable_t functions_enable = {0};
  ism6hg256x_emb_func_src_t emb_func_src = {0};
  ism6hg256x_fifo_status2_t fifo_status2 = {0};
  ism6hg256x_all_int_src_t all_int_src = {0};
  ism6hg256x_wake_up_src_t wake_up_src = {0};
  ism6hg256x_status_reg_t status_reg = {0};
  ism6hg256x_d6d_src_t d6d_src = {0};
  ism6hg256x_tap_src_t tap_src = {0};
  ism6hg256x_ui_status_reg_ois_t status_reg_ois = {0};
  ism6hg256x_status_controller_t status_shub = {0};
  uint8_t buff[8] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1);
  if (ret != 0)
  {
    return ret;
  }

  functions_enable.dis_rst_lir_all_int = PROPERTY_ENABLE;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1);
  if (ret != 0)
  {
    return ret;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_STATUS1, (uint8_t *)&buff, 4);
  if (ret != 0)
  {
    return ret;
  }

  bytecpy((uint8_t *)&fifo_status2, &buff[1]);
  bytecpy((uint8_t *)&all_int_src, &buff[2]);
  bytecpy((uint8_t *)&status_reg, &buff[3]);

  val->fifo_ovr = fifo_status2.fifo_ovr_ia;
  val->fifo_bdr = fifo_status2.counter_bdr_ia;
  val->fifo_full = fifo_status2.fifo_full_ia;
  val->fifo_th = fifo_status2.fifo_wtm_ia;

  val->hg = all_int_src.hg_ia;
  val->free_fall = all_int_src.ff_ia;
  val->wake_up = all_int_src.wu_ia;
  val->six_d = all_int_src.d6d_ia;

  val->drdy_xl = status_reg.xlda;
  val->drdy_gy = status_reg.gda;
  val->drdy_temp = status_reg.tda;
  val->drdy_xlhgda = status_reg.xlhgda;
  val->drdy_eis = status_reg.gda_eis;
  val->drdy_ois = status_reg.ois_drdy;
  val->timestamp = status_reg.timestamp_endcount;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1);
  if (ret != 0)
  {
    return ret;
  }

  functions_enable.dis_rst_lir_all_int = PROPERTY_DISABLE;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1);
  if (ret != 0)
  {
    return ret;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_STATUS_REG_OIS, (uint8_t *)&buff, 8);
  if (ret != 0)
  {
    return ret;
  }

  bytecpy((uint8_t *)&status_reg_ois, &buff[0]);
  bytecpy((uint8_t *)&wake_up_src, &buff[1]);
  bytecpy((uint8_t *)&tap_src, &buff[2]);
  bytecpy((uint8_t *)&d6d_src, &buff[3]);
  bytecpy((uint8_t *)&emb_func_status_mainpage, &buff[5]);
  bytecpy((uint8_t *)&fsm_status_mainpage, &buff[6]);
  bytecpy((uint8_t *)&mlc_status_mainpage, &buff[7]);

  val->gy_settling = status_reg_ois.gyro_settling;
  val->sleep_change = wake_up_src.sleep_change_ia;
  val->wake_up_x = wake_up_src.x_wu;
  val->wake_up_y = wake_up_src.y_wu;
  val->wake_up_z = wake_up_src.z_wu;
  val->sleep_state = wake_up_src.sleep_state;

  val->tap_x = tap_src.x_tap;
  val->tap_y = tap_src.y_tap;
  val->tap_z = tap_src.z_tap;
  val->tap_sign = tap_src.tap_sign;
  val->double_tap = tap_src.double_tap;
  val->single_tap = tap_src.single_tap;

  val->six_d_zl = d6d_src.zl;
  val->six_d_zh = d6d_src.zh;
  val->six_d_yl = d6d_src.yl;
  val->six_d_yh = d6d_src.yh;
  val->six_d_xl = d6d_src.xl;
  val->six_d_xh = d6d_src.xh;

  val->step_detector = emb_func_status_mainpage.is_step_det;
  val->tilt = emb_func_status_mainpage.is_tilt;
  val->sig_mot = emb_func_status_mainpage.is_sigmot;
  val->fsm_lc = emb_func_status_mainpage.is_fsm_lc;

  val->fsm1 = fsm_status_mainpage.is_fsm1;
  val->fsm2 = fsm_status_mainpage.is_fsm2;
  val->fsm3 = fsm_status_mainpage.is_fsm3;
  val->fsm4 = fsm_status_mainpage.is_fsm4;
  val->fsm5 = fsm_status_mainpage.is_fsm5;
  val->fsm6 = fsm_status_mainpage.is_fsm6;
  val->fsm7 = fsm_status_mainpage.is_fsm7;
  val->fsm8 = fsm_status_mainpage.is_fsm8;

  val->mlc1 = mlc_status_mainpage.is_mlc1;
  val->mlc2 = mlc_status_mainpage.is_mlc2;
  val->mlc3 = mlc_status_mainpage.is_mlc3;
  val->mlc4 = mlc_status_mainpage.is_mlc4;
  val->mlc5 = mlc_status_mainpage.is_mlc5;
  val->mlc6 = mlc_status_mainpage.is_mlc6;
  val->mlc7 = mlc_status_mainpage.is_mlc7;
  val->mlc8 = mlc_status_mainpage.is_mlc8;


  /* embedded func */
  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_EXEC_STATUS, (uint8_t *)&emb_func_exec_status,
                             1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_SRC, (uint8_t *)&emb_func_src, 1);
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  val->emb_func_stand_by = emb_func_exec_status.emb_func_endop;
  val->emb_func_time_exceed = emb_func_exec_status.emb_func_exec_ovr;
  val->step_count_inc = emb_func_src.stepcounter_bit_set;
  val->step_count_overflow = emb_func_src.step_overflow;
  val->step_on_delta_time = emb_func_src.step_count_delta_ia;

  val->step_detector = emb_func_src.step_detected;

  /* sensor hub */
  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_STATUS_CONTROLLER_MAINPAGE, (uint8_t *)&status_shub, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->sh_endop = status_shub.sens_hub_endop;
  val->sh_wr_once = status_shub.wr_once_done;
  val->sh_target3_nack = status_shub.target3_nack;
  val->sh_target2_nack = status_shub.target2_nack;
  val->sh_target1_nack = status_shub.target1_nack;
  val->sh_target0_nack = status_shub.target0_nack;

  return ret;
}

int32_t ism6hg256x_flag_data_ready_get(const stmdev_ctx_t *ctx,
                                       ism6hg256x_data_ready_t *val)
{
  ism6hg256x_status_reg_t status = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_STATUS_REG, (uint8_t *)&status, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->drdy_hgxl = status.xlhgda;
  val->drdy_xl = status.xlda;
  val->drdy_gy = status.gda;
  val->drdy_temp = status.tda;

  return ret;
}


int32_t ism6hg256x_int_ack_mask_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_INT_ACK_MASK, &val, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_int_ack_mask_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_INT_ACK_MASK, val, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_temperature_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_OUT_TEMP_L, &buff[0], 2);
  if (ret != 0)
  {
    return ret;
  }

  *val = (int16_t)((uint16_t)buff[0] | ((uint16_t)buff[1] << 8));

  return ret;
}


int32_t ism6hg256x_angular_rate_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_OUTX_L_G, &buff[0], 6);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (int16_t)((uint16_t)buff[0] | ((uint16_t)buff[1] << 8));
  val[1] = (int16_t)((uint16_t)buff[2] | ((uint16_t)buff[3] << 8));
  val[2] = (int16_t)((uint16_t)buff[4] | ((uint16_t)buff[5] << 8));

  return ret;
}

int32_t ism6hg256x_ois_angular_rate_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF2_OUTX_L_G_OIS, &buff[0], 6);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (int16_t)((uint16_t)buff[0] | ((uint16_t)buff[1] << 8));
  val[1] = (int16_t)((uint16_t)buff[2] | ((uint16_t)buff[3] << 8));
  val[2] = (int16_t)((uint16_t)buff[4] | ((uint16_t)buff[5] << 8));

  return ret;
}


int32_t ism6hg256x_ois_eis_angular_rate_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_OUTX_L_G_OIS_EIS, &buff[0], 6);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (int16_t)buff[0] | (int16_t)((uint16_t)buff[1] << 8);
  val[1] = (int16_t)buff[2] | (int16_t)((uint16_t)buff[3] << 8);
  val[2] = (int16_t)buff[4] | (int16_t)((uint16_t)buff[5] << 8);

  return ret;
}


int32_t ism6hg256x_acceleration_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_OUTX_L_A, &buff[0], 6);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (int16_t)((uint16_t)buff[0] | ((uint16_t)buff[1] << 8));
  val[1] = (int16_t)((uint16_t)buff[2] | ((uint16_t)buff[3] << 8));
  val[2] = (int16_t)((uint16_t)buff[4] | ((uint16_t)buff[5] << 8));

  return ret;
}


int32_t ism6hg256x_hg_acceleration_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_OUTX_L_A_OIS_HG, &buff[0], 6);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (int16_t)((uint16_t)buff[0] | ((uint16_t)buff[1] << 8));
  val[1] = (int16_t)((uint16_t)buff[2] | ((uint16_t)buff[3] << 8));
  val[2] = (int16_t)((uint16_t)buff[4] | ((uint16_t)buff[5] << 8));

  return ret;
}

int32_t ism6hg256x_ois_acceleration_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_OUTX_L_A_OIS_HG, &buff[0], 6);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (int16_t)((uint16_t)buff[0] | ((uint16_t)buff[1] << 8));
  val[1] = (int16_t)((uint16_t)buff[2] | ((uint16_t)buff[3] << 8));
  val[2] = (int16_t)((uint16_t)buff[4] | ((uint16_t)buff[5] << 8));

  return ret;
}


int32_t ism6hg256x_sflp_gbias_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_SFLP_GBIASX_L, &buff[0], 6);
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (int16_t)((uint16_t)buff[0] | ((uint16_t)buff[1] << 8));
  val[1] = (int16_t)((uint16_t)buff[2] | ((uint16_t)buff[3] << 8));
  val[2] = (int16_t)((uint16_t)buff[4] | ((uint16_t)buff[5] << 8));

  return ret;
}


int32_t ism6hg256x_sflp_gravity_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_SFLP_GRAVX_L, &buff[0], 6);
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (int16_t)((uint16_t)buff[0] | ((uint16_t)buff[1] << 8));
  val[1] = (int16_t)((uint16_t)buff[2] | ((uint16_t)buff[3] << 8));
  val[2] = (int16_t)((uint16_t)buff[4] | ((uint16_t)buff[5] << 8));

  return ret;
}


int32_t ism6hg256x_sflp_quaternion_raw_get(const stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t buff[8] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_SFLP_QUATW_L, &buff[0], 8);
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  val[0] = (uint16_t)(((uint16_t)buff[1] << 8) | (uint16_t)buff[0]);
  val[1] = (uint16_t)(((uint16_t)buff[3] << 8) | (uint16_t)buff[2]);
  val[2] = (uint16_t)(((uint16_t)buff[5] << 8) | (uint16_t)buff[4]);
  val[3] = (uint16_t)(((uint16_t)buff[7] << 8) | (uint16_t)buff[6]);

  return ret;
}


int32_t ism6hg256x_sflp_quaternion_get(const stmdev_ctx_t *ctx, ism6hg256x_quaternion_t *quat)
{
  uint16_t val[4] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_sflp_quaternion_raw_get(ctx, val);
  if (ret != 0)
  {
    return ret;
  }

  quat->quat_w = ism6hg256x_from_quaternion_lsb_to_float(val[0]);
  quat->quat_x = ism6hg256x_from_quaternion_lsb_to_float(val[1]);
  quat->quat_y = ism6hg256x_from_quaternion_lsb_to_float(val[2]);
  quat->quat_z = ism6hg256x_from_quaternion_lsb_to_float(val[3]);

  return ret;
}


int32_t ism6hg256x_odr_cal_reg_get(const stmdev_ctx_t *ctx, int8_t *val)
{
  ism6hg256x_internal_freq_t internal_freq = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_INTERNAL_FREQ, (uint8_t *)&internal_freq, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = (int8_t)internal_freq.freq_fine;

  return ret;
}

int32_t ism6hg256x_disable_embedded_function_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_emb_func_cfg_t emb_func_cfg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_CFG, (uint8_t *)&emb_func_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  emb_func_cfg.emb_func_disable = val & 0x1U;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_CFG, (uint8_t *)&emb_func_cfg, 1);

  return ret;
}


int32_t ism6hg256x_disable_embedded_function_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_emb_func_cfg_t emb_func_cfg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_CFG, (uint8_t *)&emb_func_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = emb_func_cfg.emb_func_disable;

  return ret;
}


int32_t ism6hg256x_emb_func_conv_set(const stmdev_ctx_t *ctx, ism6hg256x_emb_func_conv_t val)
{
  ism6hg256x_emb_func_sensor_conv_en_t conv_reg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_SENSOR_CONV_EN, (uint8_t *)&conv_reg, 1);
  if (ret != 0)
  {
    goto exit;
  }
  conv_reg.xl_hg_conv_en = val.xl_hg_conv_en;
  conv_reg.gyro_conv_en = val.gyro_conv_en;
  conv_reg.temp_conv_en = val.temp_conv_en;
  conv_reg.ext_sensor_conv_en = val.ext_sensor_conv_en;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_SENSOR_CONV_EN, (uint8_t *)&conv_reg, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_emb_func_conv_get(const stmdev_ctx_t *ctx, ism6hg256x_emb_func_conv_t *val)
{
  ism6hg256x_emb_func_sensor_conv_en_t conv_reg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_SENSOR_CONV_EN, (uint8_t *)&conv_reg, 1);
  if (ret != 0)
  {
    goto exit;
  }
  val->xl_hg_conv_en  = conv_reg.xl_hg_conv_en;
  val->gyro_conv_en = conv_reg.gyro_conv_en;
  val->temp_conv_en = conv_reg.temp_conv_en;
  val->ext_sensor_conv_en = conv_reg.ext_sensor_conv_en;

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_ln_pg_write(const stmdev_ctx_t *ctx, uint16_t address,
                               uint8_t *buf, uint8_t len)
{
  ism6hg256x_page_address_t page_address = {0};
  ism6hg256x_page_sel_t page_sel = {0};
  ism6hg256x_page_rw_t page_rw = {0};
  uint8_t msb = {0};
  uint8_t lsb = {0};
  int32_t ret = {0};
  uint8_t i = {0};

  msb = ((uint8_t)(address >> 8) & 0x0FU);
  lsb = (uint8_t)address & 0xFFU;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  /* set page write */
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_PAGE_RW, (uint8_t *)&page_rw, 1);
  if (ret != 0)
  {
    goto exit;
  }
  page_rw.page_read = PROPERTY_DISABLE;
  page_rw.page_write = PROPERTY_ENABLE;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_PAGE_RW, (uint8_t *)&page_rw, 1);

  /* select page */
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_PAGE_SEL, (uint8_t *)&page_sel, 1);
  if (ret != 0)
  {
    goto exit;
  }
  page_sel.page_sel = msb & 0x0FU;
  page_sel.not_used0 = 1; // Default value
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_PAGE_SEL, (uint8_t *)&page_sel, 1);
  if (ret != 0)
  {
    goto exit;
  }

  /* set page addr */
  page_address.page_addr = lsb;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_PAGE_ADDRESS,
                              (uint8_t *)&page_address, 1);
  if (ret != 0)
  {
    goto exit;
  }

  for (i = 0; i < len; i++)
  {
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_PAGE_VALUE, &buf[i], 1);
    if (ret != 0)
    {
      goto exit;
    }

    lsb++;

    /* Check if page wrap */
    if ((lsb & 0xFFU) == 0x00U)
    {
      msb++;
      ret += ism6hg256x_read_reg(ctx, ISM6HG256X_PAGE_SEL, (uint8_t *)&page_sel, 1);
      if (ret != 0)
      {
        goto exit;
      }

      page_sel.page_sel = msb & 0x0FU;
      page_sel.not_used0 = 1; // Default value
      ret += ism6hg256x_write_reg(ctx, ISM6HG256X_PAGE_SEL, (uint8_t *)&page_sel, 1);
      if (ret != 0)
      {
        goto exit;
      }
    }
  }

  page_sel.page_sel = 0U;
  page_sel.not_used0 = 1U;// Default value
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_PAGE_SEL, (uint8_t *)&page_sel, 1);
  if (ret != 0)
  {
    goto exit;
  }

  /* unset page write */
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_PAGE_RW, (uint8_t *)&page_rw, 1);
  if (ret != 0)
  {
    goto exit;
  }
  page_rw.page_read = PROPERTY_DISABLE;
  page_rw.page_write = PROPERTY_DISABLE;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_PAGE_RW, (uint8_t *)&page_rw, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_ln_pg_read(const stmdev_ctx_t *ctx, uint16_t address, uint8_t *buf,
                              uint8_t len)
{
  ism6hg256x_page_address_t  page_address = {0};
  ism6hg256x_page_sel_t page_sel = {0};
  ism6hg256x_page_rw_t page_rw = {0};
  uint8_t msb = {0};
  uint8_t lsb = {0};
  int32_t ret = {0};
  uint8_t i = {0};

  msb = ((uint8_t)(address >> 8) & 0x0FU);
  lsb = (uint8_t)address & 0xFFU;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  /* set page write */
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_PAGE_RW, (uint8_t *)&page_rw, 1);
  if (ret != 0)
  {
    goto exit;
  }
  page_rw.page_read = PROPERTY_ENABLE;
  page_rw.page_write = PROPERTY_DISABLE;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_PAGE_RW, (uint8_t *)&page_rw, 1);
  if (ret != 0)
  {
    goto exit;
  }

  /* select page */
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_PAGE_SEL, (uint8_t *)&page_sel, 1);
  if (ret != 0)
  {
    goto exit;
  }
  page_sel.page_sel = msb & 0x0FU;
  page_sel.not_used0 = 1U; // Default value
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_PAGE_SEL, (uint8_t *)&page_sel, 1);
  if (ret != 0)
  {
    goto exit;
  }

  /* set page addr */
  page_address.page_addr = lsb;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_PAGE_ADDRESS,
                              (uint8_t *)&page_address, 1);
  if (ret != 0)
  {
    goto exit;
  }

  for (i = 0; i < len; i++)
  {
    ret += ism6hg256x_read_reg(ctx, ISM6HG256X_PAGE_VALUE, &buf[i], 1);
    if (ret != 0)
    {
      goto exit;
    }

    lsb++;

    /* Check if page wrap */
    if ((lsb & 0xFFU) == 0x00U)
    {
      msb++;
      ret += ism6hg256x_read_reg(ctx, ISM6HG256X_PAGE_SEL, (uint8_t *)&page_sel, 1);
      if (ret != 0)
      {
        goto exit;
      }

      page_sel.page_sel = msb & 0x0FU;
      page_sel.not_used0 = 1U; // Default value
      ret += ism6hg256x_write_reg(ctx, ISM6HG256X_PAGE_SEL, (uint8_t *)&page_sel, 1);
      if (ret != 0)
      {
        goto exit;
      }
    }
  }

  page_sel.page_sel = 0U;
  page_sel.not_used0 = 1U;// Default value
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_PAGE_SEL, (uint8_t *)&page_sel, 1);
  if (ret != 0)
  {
    goto exit;
  }

  /* unset page write */
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_PAGE_RW, (uint8_t *)&page_rw, 1);
  if (ret != 0)
  {
    goto exit;
  }
  page_rw.page_read = PROPERTY_DISABLE;
  page_rw.page_write = PROPERTY_DISABLE;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_PAGE_RW, (uint8_t *)&page_rw, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_emb_function_dbg_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_ctrl10_t ctrl10 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL10, (uint8_t *)&ctrl10, 1);

  if (ret == 0)
  {
    ctrl10.emb_func_debug = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL10, (uint8_t *)&ctrl10, 1);
  }

  return ret;
}


int32_t ism6hg256x_emb_function_dbg_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_ctrl10_t ctrl10 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL10, (uint8_t *)&ctrl10, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = ctrl10.emb_func_debug;

  return ret;
}


int32_t ism6hg256x_den_polarity_set(const stmdev_ctx_t *ctx,
                                    ism6hg256x_den_polarity_t val)
{
  ism6hg256x_ctrl4_t ctrl4 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL4, (uint8_t *)&ctrl4, 1);

  if (ret == 0)
  {
    ctrl4.int2_in_lh = (uint8_t)val & 0x1U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL4, (uint8_t *)&ctrl4, 1);
  }

  return ret;
}


int32_t ism6hg256x_den_polarity_get(const stmdev_ctx_t *ctx,
                                    ism6hg256x_den_polarity_t *val)
{
  ism6hg256x_ctrl4_t ctrl4 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL4, (uint8_t *)&ctrl4, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl4.int2_in_lh)
  {
    case 0x00:
      *val = ISM6HG256X_DEN_ACT_LOW;
      break;

    case 0x01:
      *val = ISM6HG256X_DEN_ACT_HIGH;
      break;

    default:
      *val = ISM6HG256X_DEN_ACT_LOW;
      break;
  }

  return ret;
}

int32_t ism6hg256x_eis_gy_full_scale_set(const stmdev_ctx_t *ctx,
                                         ism6hg256x_eis_gy_full_scale_t val)
{
  ism6hg256x_ctrl2_t ctrl2 = {0};
  ism6hg256x_ctrl2_t prev_ctrl2 = {0};
  ism6hg256x_ctrl_eis_t ctrl_eis = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL_EIS, (uint8_t *)&ctrl_eis, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&ctrl2, 1);
  prev_ctrl2 = ctrl2;

  if (ret != 0)
  {
    goto exit;
  }

  // For the correct operation of the device, the user must set a
  // configuration from 001 to 101 when the gyroscope is in power-down mode.
  if (ctrl2.odr_g != (uint8_t)ISM6HG256X_ODR_OFF)
  {
    ctrl2.odr_g = (uint8_t)ISM6HG256X_ODR_OFF;
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&ctrl2, 1);
  }

  ctrl_eis.fs_g_eis = (uint8_t)val & 0x7U;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL_EIS, (uint8_t *)&ctrl_eis, 1);

  // restore previous odr set
  if (prev_ctrl2.odr_g != (uint8_t)ISM6HG256X_ODR_OFF)
  {
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL2, (uint8_t *)&prev_ctrl2, 1);
  }

exit:
  return ret;
}


int32_t ism6hg256x_eis_gy_full_scale_get(const stmdev_ctx_t *ctx,
                                         ism6hg256x_eis_gy_full_scale_t *val)
{
  ism6hg256x_ctrl_eis_t ctrl_eis = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL_EIS, (uint8_t *)&ctrl_eis, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl_eis.fs_g_eis)
  {
    case 0x01:
      *val = ISM6HG256X_EIS_250dps;
      break;

    case 0x02:
      *val = ISM6HG256X_EIS_500dps;
      break;

    case 0x03:
      *val = ISM6HG256X_EIS_1000dps;
      break;

    case 0x04:
      *val = ISM6HG256X_EIS_2000dps;
      break;

    case 0x05:
      *val = ISM6HG256X_EIS_4000dps;
      break;

    default:
      *val = ISM6HG256X_EIS_250dps;
      break;
  }
  return ret;
}


int32_t ism6hg256x_eis_gy_on_if2_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_ctrl_eis_t ctrl_eis = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL_EIS, (uint8_t *)&ctrl_eis, 1);

  if (ret == 0)
  {
    ctrl_eis.g_eis_on_g_ois_out_reg = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL_EIS, (uint8_t *)&ctrl_eis, 1);
  }

  return ret;
}

int32_t ism6hg256x_eis_gy_on_if2_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_ctrl_eis_t ctrl_eis = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL_EIS, (uint8_t *)&ctrl_eis, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = ctrl_eis.g_eis_on_g_ois_out_reg;

  return ret;
}


int32_t ism6hg256x_gy_eis_data_rate_set(const stmdev_ctx_t *ctx,
                                        ism6hg256x_gy_eis_data_rate_t val)
{
  ism6hg256x_ctrl_eis_t ctrl_eis = {0};
  ism6hg256x_ctrl1_t ctrl1 = {0};
  ism6hg256x_ctrl2_t ctrl2 = {0};
  int32_t ret = 0;
  uint8_t buff[2] = {0};

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL_EIS, (uint8_t *)&ctrl_eis, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL1, buff, 2);

  if (ret != 0)
  {
    goto exit;
  }

  bytecpy((uint8_t *)&ctrl1, &buff[0]);
  bytecpy((uint8_t *)&ctrl2, &buff[1]);

  // if xl or gy are ON in odr triggered mode, eis cannot be turned on
  if (val != ISM6HG256X_EIS_ODR_OFF &&
      ((ctrl1.odr_xl != (uint8_t)ISM6HG256X_ODR_OFF &&
        ctrl1.op_mode_xl == (uint8_t)ISM6HG256X_XL_ODR_TRIGGERED_MD) ||
       (ctrl2.odr_g != (uint8_t)ISM6HG256X_ODR_OFF &&
        ctrl2.op_mode_g == (uint8_t)ISM6HG256X_GY_ODR_TRIGGERED_MD))
     )
  {
    ret = -1;
    goto exit;
  }

  ctrl_eis.odr_g_eis = (uint8_t)val & 0x03U;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL_EIS, (uint8_t *)&ctrl_eis, 1);

exit:
  return ret;
}


int32_t ism6hg256x_gy_eis_data_rate_get(const stmdev_ctx_t *ctx,
                                        ism6hg256x_gy_eis_data_rate_t *val)
{
  ism6hg256x_ctrl_eis_t ctrl_eis = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL_EIS, (uint8_t *)&ctrl_eis, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl_eis.odr_g_eis)
  {
    case 0x00:
      *val = ISM6HG256X_EIS_ODR_OFF;
      break;

    case 0x01:
      *val = ISM6HG256X_EIS_1920Hz;
      break;

    case 0x02:
      *val = ISM6HG256X_EIS_960Hz;
      break;

    default:
      *val = ISM6HG256X_EIS_1920Hz;
      break;
  }

  return ret;
}


int32_t ism6hg256x_fifo_watermark_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_fifo_ctrl1_t fifo_ctrl1 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL1, (uint8_t *)&fifo_ctrl1, 1);

  if (ret == 0)
  {
    fifo_ctrl1.wtm = val;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FIFO_CTRL1, (uint8_t *)&fifo_ctrl1, 1);
  }

  return ret;
}


int32_t ism6hg256x_fifo_watermark_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_fifo_ctrl1_t fifo_ctrl1 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL1, (uint8_t *)&fifo_ctrl1, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = fifo_ctrl1.wtm;

  return ret;
}

int32_t ism6hg256x_fifo_compress_algo_set(const stmdev_ctx_t *ctx,
                                          ism6hg256x_fifo_compress_algo_t val)
{
  ism6hg256x_fifo_ctrl2_t fifo_ctrl2 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  if (ret == 0)
  {
    fifo_ctrl2.uncompr_rate = (uint8_t)val & 0x03U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  }

  return ret;
}


int32_t ism6hg256x_fifo_compress_algo_get(const stmdev_ctx_t *ctx,
                                          ism6hg256x_fifo_compress_algo_t *val)
{
  ism6hg256x_fifo_ctrl2_t fifo_ctrl2 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (fifo_ctrl2.uncompr_rate)
  {
    case 0x00:
      *val = ISM6HG256X_CMP_DISABLE;
      break;

    case 0x01:
      *val = ISM6HG256X_CMP_8_TO_1;
      break;

    case 0x02:
      *val = ISM6HG256X_CMP_16_TO_1;
      break;

    case 0x03:
      *val = ISM6HG256X_CMP_32_TO_1;
      break;

    default:
      *val = ISM6HG256X_CMP_DISABLE;
      break;
  }
  return ret;
}


int32_t ism6hg256x_fifo_virtual_sens_odr_chg_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_fifo_ctrl2_t fifo_ctrl2 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  if (ret == 0)
  {
    fifo_ctrl2.odr_chg_en = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  }

  return ret;
}


int32_t ism6hg256x_fifo_virtual_sens_odr_chg_get(const stmdev_ctx_t *ctx,
                                                 uint8_t *val)
{
  ism6hg256x_fifo_ctrl2_t fifo_ctrl2 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = fifo_ctrl2.odr_chg_en;

  return ret;
}


int32_t ism6hg256x_fifo_compress_algo_real_time_set(const stmdev_ctx_t *ctx,
                                                    uint8_t val)
{
  ism6hg256x_emb_func_en_b_t emb_func_en_b = {0};
  ism6hg256x_fifo_ctrl2_t fifo_ctrl2 = {0};

  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  if (ret != 0)
  {
    return ret;
  }
  fifo_ctrl2.fifo_compr_rt_en = val & 0x01U;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  if (ret != 0)
  {
    return ret;
  }

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_EN_B, (uint8_t *)&emb_func_en_b, 1);
  if (ret != 0)
  {
    goto exit;
  }
  emb_func_en_b.fifo_compr_en = val & 0x01U;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_EN_B, (uint8_t *)&emb_func_en_b, 1);
exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fifo_compress_algo_real_time_get(const stmdev_ctx_t *ctx,
                                                    uint8_t *val)
{
  ism6hg256x_fifo_ctrl2_t fifo_ctrl2 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = fifo_ctrl2.fifo_compr_rt_en;

  return ret;
}


int32_t ism6hg256x_fifo_stop_on_wtm_set(const stmdev_ctx_t *ctx, ism6hg256x_fifo_event_t val)
{
  ism6hg256x_fifo_ctrl2_t fifo_ctrl2 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  if (ret == 0)
  {
    fifo_ctrl2.stop_on_wtm = (val == ISM6HG256X_FIFO_EV_WTM) ? 1U : 0U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  }

  return ret;
}


int32_t ism6hg256x_fifo_stop_on_wtm_get(const stmdev_ctx_t *ctx, ism6hg256x_fifo_event_t *val)
{
  ism6hg256x_fifo_ctrl2_t fifo_ctrl2 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL2, (uint8_t *)&fifo_ctrl2, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = (fifo_ctrl2.stop_on_wtm == 1U) ? ISM6HG256X_FIFO_EV_WTM : ISM6HG256X_FIFO_EV_FULL;

  return ret;
}


int32_t ism6hg256x_fifo_xl_batch_set(const stmdev_ctx_t *ctx,
                                     ism6hg256x_fifo_xl_batch_t val)
{
  ism6hg256x_fifo_ctrl3_t fifo_ctrl3 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL3, (uint8_t *)&fifo_ctrl3, 1);
  if (ret == 0)
  {
    fifo_ctrl3.bdr_xl = (uint8_t)val & 0xFu;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FIFO_CTRL3, (uint8_t *)&fifo_ctrl3, 1);
  }

  return ret;
}

int32_t ism6hg256x_fifo_xl_batch_get(const stmdev_ctx_t *ctx,
                                     ism6hg256x_fifo_xl_batch_t *val)
{
  ism6hg256x_fifo_ctrl3_t fifo_ctrl3 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL3, (uint8_t *)&fifo_ctrl3, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (fifo_ctrl3.bdr_xl)
  {
    case 0x00:
      *val = ISM6HG256X_XL_NOT_BATCHED;
      break;

    case 0x01:
      *val = ISM6HG256X_XL_BATCHED_AT_1Hz875;
      break;

    case 0x02:
      *val = ISM6HG256X_XL_BATCHED_AT_7Hz5;
      break;

    case 0x03:
      *val = ISM6HG256X_XL_BATCHED_AT_15Hz;
      break;

    case 0x04:
      *val = ISM6HG256X_XL_BATCHED_AT_30Hz;
      break;

    case 0x05:
      *val = ISM6HG256X_XL_BATCHED_AT_60Hz;
      break;

    case 0x06:
      *val = ISM6HG256X_XL_BATCHED_AT_120Hz;
      break;

    case 0x07:
      *val = ISM6HG256X_XL_BATCHED_AT_240Hz;
      break;

    case 0x08:
      *val = ISM6HG256X_XL_BATCHED_AT_480Hz;
      break;

    case 0x09:
      *val = ISM6HG256X_XL_BATCHED_AT_960Hz;
      break;

    case 0x0A:
      *val = ISM6HG256X_XL_BATCHED_AT_1920Hz;
      break;

    case 0x0B:
      *val = ISM6HG256X_XL_BATCHED_AT_3840Hz;
      break;

    case 0x0C:
      *val = ISM6HG256X_XL_BATCHED_AT_7680Hz;
      break;

    default:
      *val = ISM6HG256X_XL_NOT_BATCHED;
      break;
  }

  return ret;
}


int32_t ism6hg256x_fifo_gy_batch_set(const stmdev_ctx_t *ctx,
                                     ism6hg256x_fifo_gy_batch_t val)
{
  ism6hg256x_fifo_ctrl3_t fifo_ctrl3 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL3, (uint8_t *)&fifo_ctrl3, 1);
  if (ret == 0)
  {
    fifo_ctrl3.bdr_gy = (uint8_t)val & 0x0Fu;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FIFO_CTRL3, (uint8_t *)&fifo_ctrl3, 1);
  }

  return ret;
}

int32_t ism6hg256x_fifo_gy_batch_get(const stmdev_ctx_t *ctx,
                                     ism6hg256x_fifo_gy_batch_t *val)
{
  ism6hg256x_fifo_ctrl3_t fifo_ctrl3 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL3, (uint8_t *)&fifo_ctrl3, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (fifo_ctrl3.bdr_gy)
  {
    case 0x00:
      *val = ISM6HG256X_GY_NOT_BATCHED;
      break;

    case 0x01:
      *val = ISM6HG256X_GY_BATCHED_AT_1Hz875;
      break;

    case 0x02:
      *val = ISM6HG256X_GY_BATCHED_AT_7Hz5;
      break;

    case 0x03:
      *val = ISM6HG256X_GY_BATCHED_AT_15Hz;
      break;

    case 0x04:
      *val = ISM6HG256X_GY_BATCHED_AT_30Hz;
      break;

    case 0x05:
      *val = ISM6HG256X_GY_BATCHED_AT_60Hz;
      break;

    case 0x06:
      *val = ISM6HG256X_GY_BATCHED_AT_120Hz;
      break;

    case 0x07:
      *val = ISM6HG256X_GY_BATCHED_AT_240Hz;
      break;

    case 0x08:
      *val = ISM6HG256X_GY_BATCHED_AT_480Hz;
      break;

    case 0x09:
      *val = ISM6HG256X_GY_BATCHED_AT_960Hz;
      break;

    case 0x0A:
      *val = ISM6HG256X_GY_BATCHED_AT_1920Hz;
      break;

    case 0x0B:
      *val = ISM6HG256X_GY_BATCHED_AT_3840Hz;
      break;

    case 0x0C:
      *val = ISM6HG256X_GY_BATCHED_AT_7680Hz;
      break;

    default:
      *val = ISM6HG256X_GY_NOT_BATCHED;
      break;
  }
  return ret;
}


int32_t ism6hg256x_fifo_hg_xl_batch_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_counter_bdr_reg1_t cbdr_reg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_COUNTER_BDR_REG1, (uint8_t *)&cbdr_reg, 1);
  if (ret == 0)
  {
    cbdr_reg.xl_hg_batch_en = (uint8_t)val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_COUNTER_BDR_REG1, (uint8_t *)&cbdr_reg, 1);
  }

  return ret;
}


int32_t ism6hg256x_fifo_hg_xl_batch_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_counter_bdr_reg1_t cbdr_reg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_COUNTER_BDR_REG1, (uint8_t *)&cbdr_reg, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = cbdr_reg.xl_hg_batch_en;

  return ret;
}


int32_t ism6hg256x_fifo_mode_set(const stmdev_ctx_t *ctx, ism6hg256x_fifo_mode_t val)
{
  ism6hg256x_fifo_ctrl4_t fifo_ctrl4 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
  if (ret == 0)
  {
    fifo_ctrl4.fifo_mode = (uint8_t)val & 0x07U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
  }

  return ret;
}


int32_t ism6hg256x_fifo_mode_get(const stmdev_ctx_t *ctx, ism6hg256x_fifo_mode_t *val)
{
  ism6hg256x_fifo_ctrl4_t fifo_ctrl4 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (fifo_ctrl4.fifo_mode)
  {
    case 0x00:
      *val = ISM6HG256X_BYPASS_MODE;
      break;

    case 0x01:
      *val = ISM6HG256X_FIFO_MODE;
      break;

    case 0x02:
      *val = ISM6HG256X_STREAM_WTM_TO_FULL_MODE;
      break;

    case 0x03:
      *val = ISM6HG256X_STREAM_TO_FIFO_MODE;
      break;

    case 0x04:
      *val = ISM6HG256X_BYPASS_TO_STREAM_MODE;
      break;

    case 0x06:
      *val = ISM6HG256X_STREAM_MODE;
      break;

    case 0x07:
      *val = ISM6HG256X_BYPASS_TO_FIFO_MODE;
      break;

    default:
      *val = ISM6HG256X_BYPASS_MODE;
      break;
  }
  return ret;
}

int32_t ism6hg256x_fifo_gy_eis_batch_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_fifo_ctrl4_t fifo_ctrl4 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
  if (ret == 0)
  {
    fifo_ctrl4.g_eis_fifo_en = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
  }

  return ret;
}


int32_t ism6hg256x_fifo_gy_eis_batch_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_fifo_ctrl4_t fifo_ctrl4 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = fifo_ctrl4.g_eis_fifo_en;

  return ret;
}


int32_t ism6hg256x_fifo_temp_batch_set(const stmdev_ctx_t *ctx,
                                       ism6hg256x_fifo_temp_batch_t val)
{
  ism6hg256x_fifo_ctrl4_t fifo_ctrl4 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
  if (ret == 0)
  {
    fifo_ctrl4.odr_t_batch = (uint8_t)val & 0x03U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
  }

  return ret;
}


int32_t ism6hg256x_fifo_temp_batch_get(const stmdev_ctx_t *ctx,
                                       ism6hg256x_fifo_temp_batch_t *val)
{
  ism6hg256x_fifo_ctrl4_t fifo_ctrl4 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (fifo_ctrl4.odr_t_batch)
  {
    case 0x00:
      *val = ISM6HG256X_TEMP_NOT_BATCHED;
      break;

    case 0x01:
      *val = ISM6HG256X_TEMP_BATCHED_AT_1Hz875;
      break;

    case 0x02:
      *val = ISM6HG256X_TEMP_BATCHED_AT_15Hz;
      break;

    case 0x03:
      *val = ISM6HG256X_TEMP_BATCHED_AT_60Hz;
      break;

    default:
      *val = ISM6HG256X_TEMP_NOT_BATCHED;
      break;
  }
  return ret;
}


int32_t ism6hg256x_fifo_timestamp_batch_set(const stmdev_ctx_t *ctx,
                                            ism6hg256x_fifo_timestamp_batch_t val)
{
  ism6hg256x_fifo_ctrl4_t fifo_ctrl4 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
  if (ret == 0)
  {
    fifo_ctrl4.dec_ts_batch = (uint8_t)val & 0x03U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
  }

  return ret;
}


int32_t ism6hg256x_fifo_timestamp_batch_get(const stmdev_ctx_t *ctx,
                                            ism6hg256x_fifo_timestamp_batch_t *val)
{
  ism6hg256x_fifo_ctrl4_t fifo_ctrl4 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (fifo_ctrl4.dec_ts_batch)
  {
    case 0x00:
      *val = ISM6HG256X_TMSTMP_NOT_BATCHED;
      break;

    case 0x01:
      *val = ISM6HG256X_TMSTMP_DEC_1;
      break;

    case 0x02:
      *val = ISM6HG256X_TMSTMP_DEC_8;
      break;

    case 0x03:
      *val = ISM6HG256X_TMSTMP_DEC_32;
      break;

    default:
      *val = ISM6HG256X_TMSTMP_NOT_BATCHED;
      break;
  }

  return ret;
}


int32_t ism6hg256x_fifo_batch_counter_threshold_set(const stmdev_ctx_t *ctx,
                                                    uint16_t val)
{
  ism6hg256x_counter_bdr_reg1_t counter_bdr_reg1 = {0};
  ism6hg256x_counter_bdr_reg2_t counter_bdr_reg2 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_COUNTER_BDR_REG1, (uint8_t *)&counter_bdr_reg1, 1);

  if (ret == 0)
  {
    counter_bdr_reg2.cnt_bdr_th = (uint8_t)val & 0xFFU;
    counter_bdr_reg1.cnt_bdr_th = (uint8_t)(val >> 8) & 0x3U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_COUNTER_BDR_REG1, (uint8_t *)&counter_bdr_reg1, 1);
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_COUNTER_BDR_REG2, (uint8_t *)&counter_bdr_reg2, 1);
  }


  return ret;
}


int32_t ism6hg256x_fifo_batch_counter_threshold_get(const stmdev_ctx_t *ctx,
                                                    uint16_t *val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_COUNTER_BDR_REG1, &buff[0], 2);
  if (ret != 0)
  {
    return ret;
  }

  *val = (uint16_t)(buff[1] | ((uint16_t)(buff[0] & 0x3U) << 8));

  return ret;
}

int32_t ism6hg256x_fifo_batch_cnt_event_set(const stmdev_ctx_t *ctx,
                                            ism6hg256x_fifo_batch_cnt_event_t val)
{
  ism6hg256x_counter_bdr_reg1_t counter_bdr_reg1 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_COUNTER_BDR_REG1, (uint8_t *)&counter_bdr_reg1, 1);
  if (ret == 0)
  {
    counter_bdr_reg1.trig_counter_bdr = (uint8_t)val & 0x03U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_COUNTER_BDR_REG1, (uint8_t *)&counter_bdr_reg1, 1);
  }

  return ret;
}


int32_t ism6hg256x_fifo_batch_cnt_event_get(const stmdev_ctx_t *ctx,
                                            ism6hg256x_fifo_batch_cnt_event_t *val)
{
  ism6hg256x_counter_bdr_reg1_t counter_bdr_reg1 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_COUNTER_BDR_REG1, (uint8_t *)&counter_bdr_reg1, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (counter_bdr_reg1.trig_counter_bdr)
  {
    case 0:
      *val = ISM6HG256X_XL_LG_BATCH_EVENT;
      break;

    case 1:
      *val = ISM6HG256X_GY_BATCH_EVENT;
      break;

    case 2:
      *val = ISM6HG256X_GY_EIS_BATCH_EVENT;
      break;

    case 3:
      *val = ISM6HG256X_XL_HG_BATCH_EVENT;
      break;

    default:
      *val = ISM6HG256X_XL_LG_BATCH_EVENT;
      break;
  }

  return ret;
}

int32_t ism6hg256x_fifo_status_get(const stmdev_ctx_t *ctx,
                                   ism6hg256x_fifo_status_t *val)
{
  uint8_t buff[2] = {0};
  ism6hg256x_fifo_status2_t status = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_STATUS1, (uint8_t *)&buff[0], 2);
  if (ret != 0)
  {
    return ret;
  }

  bytecpy((uint8_t *)&status, &buff[1]);

  val->fifo_bdr = status.counter_bdr_ia;
  val->fifo_ovr = status.fifo_ovr_ia;
  val->fifo_full = status.fifo_full_ia;
  val->fifo_th = status.fifo_wtm_ia;

  val->fifo_level = (uint16_t)(buff[0] | ((uint16_t)(buff[1] & 0x01U) << 8));

  return ret;
}



int32_t ism6hg256x_fifo_out_raw_get(const stmdev_ctx_t *ctx,
                                    ism6hg256x_fifo_out_raw_t *val)
{
  ism6hg256x_fifo_data_out_tag_t fifo_data_out_tag = {0};
  uint8_t buff[7] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FIFO_DATA_OUT_TAG, buff, 7);
  if (ret != 0)
  {
    return ret;
  }

  bytecpy((uint8_t *)&fifo_data_out_tag, &buff[0]);

  switch (fifo_data_out_tag.tag_sensor)
  {
    case 0x00:
      val->tag = ISM6HG256X_FIFO_EMPTY;
      break;

    case 0x01:
      val->tag = ISM6HG256X_GY_NC_TAG;
      break;

    case 0x02:
      val->tag = ISM6HG256X_XL_NC_TAG;
      break;

    case 0x03:
      val->tag = ISM6HG256X_TEMPERATURE_TAG;
      break;

    case 0x04:
      val->tag = ISM6HG256X_TIMESTAMP_TAG;
      break;

    case 0x05:
      val->tag = ISM6HG256X_CFG_CHANGE_TAG;
      break;

    case 0x06:
      val->tag = ISM6HG256X_XL_NC_T_2_TAG;
      break;

    case 0x07:
      val->tag = ISM6HG256X_XL_NC_T_1_TAG;
      break;

    case 0x08:
      val->tag = ISM6HG256X_XL_2XC_TAG;
      break;

    case 0x09:
      val->tag = ISM6HG256X_XL_3XC_TAG;
      break;

    case 0x0A:
      val->tag = ISM6HG256X_GY_NC_T_2_TAG;
      break;

    case 0x0B:
      val->tag = ISM6HG256X_GY_NC_T_1_TAG;
      break;

    case 0x0C:
      val->tag = ISM6HG256X_GY_2XC_TAG;
      break;

    case 0x0D:
      val->tag = ISM6HG256X_GY_3XC_TAG;
      break;

    case 0x0E:
      val->tag = ISM6HG256X_SENSORHUB_TARGET0_TAG;
      break;

    case 0x0F:
      val->tag = ISM6HG256X_SENSORHUB_TARGET1_TAG;
      break;

    case 0x10:
      val->tag = ISM6HG256X_SENSORHUB_TARGET2_TAG;
      break;

    case 0x11:
      val->tag = ISM6HG256X_SENSORHUB_TARGET3_TAG;
      break;

    case 0x12:
      val->tag = ISM6HG256X_STEP_COUNTER_TAG;
      break;

    case 0x13:
      val->tag = ISM6HG256X_SFLP_GAME_ROTATION_VECTOR_TAG;
      break;

    case 0x16:
      val->tag = ISM6HG256X_SFLP_GYROSCOPE_BIAS_TAG;
      break;

    case 0x17:
      val->tag = ISM6HG256X_SFLP_GRAVITY_VECTOR_TAG;
      break;

    case 0x18:
      val->tag = ISM6HG256X_HG_XL_PEAK_TAG;
      break;

    case 0x19:
      val->tag = ISM6HG256X_SENSORHUB_NACK_TAG;
      break;

    case 0x1A:
      val->tag = ISM6HG256X_MLC_RESULT_TAG;
      break;

    case 0x1B:
      val->tag = ISM6HG256X_MLC_FILTER;
      break;

    case 0x1C:
      val->tag = ISM6HG256X_MLC_FEATURE;
      break;

    case 0x1D:
      val->tag = ISM6HG256X_XL_HG_TAG;
      break;

    case 0x1E:
      val->tag = ISM6HG256X_GY_ENHANCED_EIS;
      break;

    case 0x1F:
      val->tag = ISM6HG256X_FSM_RESULT_TAG;
      break;

    default:
      val->tag = ISM6HG256X_FIFO_EMPTY;
      break;
  }

  val->cnt = fifo_data_out_tag.tag_cnt;

  val->data[0] = buff[1];
  val->data[1] = buff[2];
  val->data[2] = buff[3];
  val->data[3] = buff[4];
  val->data[4] = buff[5];
  val->data[5] = buff[6];

  return ret;
}


int32_t ism6hg256x_fifo_stpcnt_batch_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_emb_func_fifo_en_a_t emb_func_fifo_en_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_FIFO_EN_A, (uint8_t *)&emb_func_fifo_en_a, 1);
  if (ret != 0)
  {
    goto exit;
  }
  emb_func_fifo_en_a.step_counter_fifo_en = val & 0x01U;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_FIFO_EN_A, (uint8_t *)&emb_func_fifo_en_a, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fifo_stpcnt_batch_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_emb_func_fifo_en_a_t emb_func_fifo_en_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_FIFO_EN_A, (uint8_t *)&emb_func_fifo_en_a, 1);
  if (ret != 0)
  {
    goto exit;
  }
  *val = emb_func_fifo_en_a.step_counter_fifo_en & 0x01U;

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fifo_fsm_batch_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_emb_func_fifo_en_b_t emb_func_fifo_en_b = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_FIFO_EN_B, (uint8_t *)&emb_func_fifo_en_b, 1);
  if (ret != 0)
  {
    goto exit;
  }
  emb_func_fifo_en_b.fsm_fifo_en = val & 0x01U;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_FIFO_EN_B, (uint8_t *)&emb_func_fifo_en_b, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fifo_fsm_batch_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_emb_func_fifo_en_b_t emb_func_fifo_en_b = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_FIFO_EN_B, (uint8_t *)&emb_func_fifo_en_b, 1);
  if (ret != 0)
  {
    goto exit;
  }
  *val = emb_func_fifo_en_b.fsm_fifo_en;

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fifo_mlc_batch_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_emb_func_fifo_en_a_t emb_func_fifo_en_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_FIFO_EN_A, (uint8_t *)&emb_func_fifo_en_a, 1);
  if (ret != 0)
  {
    goto exit;
  }
  emb_func_fifo_en_a.mlc_fifo_en = val & 0x01U;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_FIFO_EN_A, (uint8_t *)&emb_func_fifo_en_a, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fifo_mlc_batch_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_emb_func_fifo_en_a_t emb_func_fifo_en_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_FIFO_EN_A, (uint8_t *)&emb_func_fifo_en_a, 1);
  if (ret != 0)
  {
    goto exit;
  }
  *val = emb_func_fifo_en_a.mlc_fifo_en;

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fifo_mlc_filt_batch_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_emb_func_fifo_en_b_t emb_func_fifo_en_b = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_FIFO_EN_B, (uint8_t *)&emb_func_fifo_en_b, 1);
  if (ret != 0)
  {
    goto exit;
  }
  emb_func_fifo_en_b.mlc_filter_feature_fifo_en = val & 0x01U;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_FIFO_EN_B, (uint8_t *)&emb_func_fifo_en_b, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fifo_mlc_filt_batch_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_emb_func_fifo_en_b_t emb_func_fifo_en_b = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_FIFO_EN_B, (uint8_t *)&emb_func_fifo_en_b, 1);
  if (ret != 0)
  {
    goto exit;
  }
  *val = emb_func_fifo_en_b.mlc_filter_feature_fifo_en;

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fifo_sh_batch_target_set(const stmdev_ctx_t *ctx, uint8_t idx, uint8_t val)
{
  ism6hg256x_tgt0_config_t tgt_config = {0};
  int32_t ret = 0;

  if (idx > 3U)
  {
    return -1;
  }

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TGT0_CONFIG + (uint8_t)(idx * 3U), (uint8_t *)&tgt_config,
                            1);
  if (ret != 0)
  {
    goto exit;
  }
  tgt_config.batch_ext_sens_0_en = val & 0x01U;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_TGT0_CONFIG + (uint8_t)(idx * 3U),
                              (uint8_t *)&tgt_config, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fifo_sh_batch_target_get(const stmdev_ctx_t *ctx, uint8_t idx, uint8_t *val)
{
  ism6hg256x_tgt0_config_t tgt_config = {0};
  int32_t ret = 0;

  if (idx > 3U)
  {
    return -1;
  }

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TGT0_CONFIG + (uint8_t)(idx * 3U), (uint8_t *)&tgt_config,
                            1);
  if (ret != 0)
  {
    goto exit;
  }
  *val = tgt_config.batch_ext_sens_0_en;

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fifo_sflp_batch_set(const stmdev_ctx_t *ctx,
                                       ism6hg256x_fifo_sflp_raw_t val)
{
  ism6hg256x_emb_func_fifo_en_a_t emb_func_fifo_en_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_FIFO_EN_A, (uint8_t *)&emb_func_fifo_en_a, 1);
  if (ret != 0)
  {
    goto exit;
  }
  emb_func_fifo_en_a.sflp_game_fifo_en = val.game_rotation;
  emb_func_fifo_en_a.sflp_gravity_fifo_en = val.gravity;
  emb_func_fifo_en_a.sflp_gbias_fifo_en = val.gbias;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_FIFO_EN_A,
                              (uint8_t *)&emb_func_fifo_en_a, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fifo_sflp_batch_get(const stmdev_ctx_t *ctx,
                                       ism6hg256x_fifo_sflp_raw_t *val)
{
  ism6hg256x_emb_func_fifo_en_a_t emb_func_fifo_en_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_FIFO_EN_A, (uint8_t *)&emb_func_fifo_en_a, 1);
  if (ret != 0)
  {
    goto exit;
  }
  val->game_rotation = emb_func_fifo_en_a.sflp_game_fifo_en;
  val->gravity = emb_func_fifo_en_a.sflp_gravity_fifo_en;
  val->gbias = emb_func_fifo_en_a.sflp_gbias_fifo_en;

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_filt_anti_spike_set(const stmdev_ctx_t *ctx,
                                       ism6hg256x_filt_anti_spike_t val)
{
  ism6hg256x_if_cfg_t if_cfg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF_CFG, (uint8_t *)&if_cfg, 1);

  if (ret == 0)
  {
    if_cfg.asf_ctrl = (uint8_t)val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_IF_CFG, (uint8_t *)&if_cfg, 1);
  }

  return ret;
}


int32_t ism6hg256x_filt_anti_spike_get(const stmdev_ctx_t *ctx,
                                       ism6hg256x_filt_anti_spike_t *val)
{
  ism6hg256x_if_cfg_t if_cfg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF_CFG, (uint8_t *)&if_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (if_cfg.asf_ctrl)
  {
    case 0x00:
      *val = ISM6HG256X_AUTO;
      break;

    case 0x01:
      *val = ISM6HG256X_ALWAYS_ACTIVE;
      break;

    default:
      *val = ISM6HG256X_AUTO;
      break;
  }

  return ret;
}


int32_t ism6hg256x_filt_settling_mask_set(const stmdev_ctx_t *ctx,
                                          ism6hg256x_filt_settling_mask_t val)
{
  ism6hg256x_emb_func_cfg_t emb_func_cfg = {0};
  ism6hg256x_ui_int_ois_t ui_int_ois = {0};
  ism6hg256x_ctrl4_t ctrl4 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL4, (uint8_t *)&ctrl4, 1);
  if (ret != 0)
  {
    return ret;
  }
  ctrl4.drdy_mask = val.drdy;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL4, (uint8_t *)&ctrl4, 1);
  if (ret != 0)
  {
    return ret;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_CFG, (uint8_t *)&emb_func_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }
  emb_func_cfg.emb_func_irq_mask_xl_settl = val.irq_xl;
  emb_func_cfg.emb_func_irq_mask_xl_hg_settl = val.irq_xl_hg;
  emb_func_cfg.emb_func_irq_mask_g_settl = val.irq_g;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_CFG, (uint8_t *)&emb_func_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_INT_OIS, (uint8_t *)&ui_int_ois, 1);
  if (ret != 0)
  {
    return ret;
  }
  ui_int_ois.drdy_mask_ois = val.ois_drdy;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_UI_INT_OIS, (uint8_t *)&ui_int_ois, 1);

  return ret;
}


int32_t ism6hg256x_filt_settling_mask_get(const stmdev_ctx_t *ctx,
                                          ism6hg256x_filt_settling_mask_t *val)
{
  ism6hg256x_emb_func_cfg_t emb_func_cfg = {0};
  ism6hg256x_ui_int_ois_t ui_int_ois = {0};
  ism6hg256x_ctrl4_t ctrl4 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL4, (uint8_t *)&ctrl4, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_CFG, (uint8_t *)&emb_func_cfg, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_UI_INT_OIS, (uint8_t *)&ui_int_ois, 1);
  if (ret != 0)
  {
    return ret;
  }
  val->ois_drdy = ui_int_ois.drdy_mask_ois;

  val->irq_xl = emb_func_cfg.emb_func_irq_mask_xl_settl;
  val->irq_g = emb_func_cfg.emb_func_irq_mask_g_settl;
  val->drdy = ctrl4.drdy_mask;

  return ret;
}

int32_t ism6hg256x_filt_ois_settling_mask_set(const stmdev_ctx_t *ctx,
                                              ism6hg256x_filt_ois_settling_mask_t val)
{
  ism6hg256x_if2_int_ois_t if2_int_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF2_INT_OIS, (uint8_t *)&if2_int_ois, 1);

  if (ret == 0)
  {
    if2_int_ois.drdy_mask_ois = val.ois_drdy;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_IF2_INT_OIS, (uint8_t *)&if2_int_ois, 1);
  }

  return ret;
}


int32_t ism6hg256x_filt_ois_settling_mask_get(const stmdev_ctx_t *ctx,
                                              ism6hg256x_filt_ois_settling_mask_t *val)
{

  ism6hg256x_if2_int_ois_t if2_int_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF2_INT_OIS, (uint8_t *)&if2_int_ois, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->ois_drdy = if2_int_ois.drdy_mask_ois;

  return ret;
}


int32_t ism6hg256x_filt_gy_lp1_bandwidth_set(const stmdev_ctx_t *ctx,
                                             ism6hg256x_filt_gy_lp1_bandwidth_t val)
{
  ism6hg256x_ctrl6_t ctrl6 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL6, (uint8_t *)&ctrl6, 1);
  if (ret == 0)
  {
    ctrl6.lpf1_g_bw = (uint8_t)val & 0x07u;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL6, (uint8_t *)&ctrl6, 1);
  }

  return ret;
}


int32_t ism6hg256x_filt_gy_lp1_bandwidth_get(const stmdev_ctx_t *ctx,
                                             ism6hg256x_filt_gy_lp1_bandwidth_t *val)
{
  ism6hg256x_ctrl6_t ctrl6 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL6, (uint8_t *)&ctrl6, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl6.lpf1_g_bw)
  {
    case 0x00:
      *val = ISM6HG256X_GY_ULTRA_LIGHT;
      break;

    case 0x01:
      *val = ISM6HG256X_GY_VERY_LIGHT;
      break;

    case 0x02:
      *val = ISM6HG256X_GY_LIGHT;
      break;

    case 0x03:
      *val = ISM6HG256X_GY_MEDIUM;
      break;

    case 0x04:
      *val = ISM6HG256X_GY_STRONG;
      break;

    case 0x05:
      *val = ISM6HG256X_GY_VERY_STRONG;
      break;

    case 0x06:
      *val = ISM6HG256X_GY_AGGRESSIVE;
      break;

    case 0x07:
      *val = ISM6HG256X_GY_XTREME;
      break;

    default:
      *val = ISM6HG256X_GY_ULTRA_LIGHT;
      break;
  }

  return ret;
}


int32_t ism6hg256x_filt_gy_lp1_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_ctrl7_t ctrl7 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL7, (uint8_t *)&ctrl7, 1);
  if (ret == 0)
  {
    ctrl7.lpf1_g_en = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL7, (uint8_t *)&ctrl7, 1);
  }

  return ret;
}



int32_t ism6hg256x_filt_gy_lp1_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_ctrl7_t ctrl7 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL7, (uint8_t *)&ctrl7, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = ctrl7.lpf1_g_en;

  return ret;
}

int32_t ism6hg256x_filt_xl_setup(const stmdev_ctx_t *ctx, ism6hg256x_xl_filter filter,
                                 ism6hg256x_filt_xl_lp2_bandwidth_t bw, uint8_t hp_ref_mode_xl)
{

  int32_t ret = 0;
  ism6hg256x_ctrl8_t ctrl8 = {0};
  ism6hg256x_ctrl9_t ctrl9 = {0};

  if ((filter == ISM6HG256X_XL_FILT_HP && bw == ISM6HG256X_XL_ULTRA_LIGHT) ||
      (hp_ref_mode_xl == 1U && filter != ISM6HG256X_XL_FILT_HP) ||
      // if bw == 0 slope filter is used instead of digital HP filter
      (filter == ISM6HG256X_XL_FILT_HP_SLOPE && (uint8_t)bw != 0x0U))
  {
    ret = -1;
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL8, (uint8_t *)&ctrl8, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);

  if (ret != 0)
  {
    goto exit;
  }

  if (filter == ISM6HG256X_XL_FILT_LP_LPF2)
  {
    ctrl9.hp_slope_xl_en = 0U;
    ctrl9.lpf2_xl_en = 1U;
  }
  else if (filter == ISM6HG256X_XL_FILT_LP_LPF1)
  {
    ctrl9.hp_slope_xl_en = 0U;
    ctrl9.lpf2_xl_en = 0U;
  }
  else if (filter == ISM6HG256X_XL_FILT_HP)
  {
    ctrl9.hp_slope_xl_en = 1U;
    ctrl9.lpf2_xl_en = 0U;
  }
  else if (filter == ISM6HG256X_XL_FILT_HP_SLOPE)
  {
    ctrl9.hp_slope_xl_en = 1U;
    ctrl9.lpf2_xl_en = 0U;
  }
  else
  {
    ret = -1;
    goto exit;
  }

  ctrl8.hp_lpf2_xl_bw = (uint8_t)bw & 0x07U;
  ctrl9.hp_ref_mode_xl = hp_ref_mode_xl & 0x01U;

  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL8, (uint8_t *)&ctrl8, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);

exit:
  return ret;
}


int32_t ism6hg256x_filt_xl_lp2_bandwidth_set(const stmdev_ctx_t *ctx,
                                             ism6hg256x_filt_xl_lp2_bandwidth_t val)
{
  ism6hg256x_ctrl8_t ctrl8 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL8, (uint8_t *)&ctrl8, 1);
  if (ret == 0)
  {
    ctrl8.hp_lpf2_xl_bw = (uint8_t)val & 0x07U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL8, (uint8_t *)&ctrl8, 1);
  }

  return ret;
}


int32_t ism6hg256x_filt_xl_lp2_bandwidth_get(const stmdev_ctx_t *ctx,
                                             ism6hg256x_filt_xl_lp2_bandwidth_t *val)
{
  ism6hg256x_ctrl8_t ctrl8 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL8, (uint8_t *)&ctrl8, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl8.hp_lpf2_xl_bw)
  {
    case 0x00:
      *val = ISM6HG256X_XL_ULTRA_LIGHT;
      break;

    case 0x01:
      *val = ISM6HG256X_XL_VERY_LIGHT;
      break;

    case 0x02:
      *val = ISM6HG256X_XL_LIGHT;
      break;

    case 0x03:
      *val = ISM6HG256X_XL_MEDIUM;
      break;

    case 0x04:
      *val = ISM6HG256X_XL_STRONG;
      break;

    case 0x05:
      *val = ISM6HG256X_XL_VERY_STRONG;
      break;

    case 0x06:
      *val = ISM6HG256X_XL_AGGRESSIVE;
      break;

    case 0x07:
      *val = ISM6HG256X_XL_XTREME;
      break;

    default:
      *val = ISM6HG256X_XL_ULTRA_LIGHT;
      break;
  }

  return ret;
}


int32_t ism6hg256x_filt_xl_lp2_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_ctrl9_t ctrl9 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);
  if (ret == 0)
  {
    ctrl9.lpf2_xl_en = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);
  }

  return ret;
}


int32_t ism6hg256x_filt_xl_lp2_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_ctrl9_t ctrl9 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = ctrl9.lpf2_xl_en;

  return ret;
}

int32_t ism6hg256x_filt_xl_hp_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_ctrl9_t ctrl9 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);
  if (ret == 0)
  {
    ctrl9.hp_slope_xl_en = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);
  }

  return ret;
}


int32_t ism6hg256x_filt_xl_hp_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_ctrl9_t ctrl9 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = ctrl9.hp_slope_xl_en;

  return ret;
}

int32_t ism6hg256x_filt_xl_fast_settling_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_ctrl9_t ctrl9 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);
  if (ret == 0)
  {
    ctrl9.xl_fastsettl_mode = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);
  }

  return ret;
}


int32_t ism6hg256x_filt_xl_fast_settling_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_ctrl9_t ctrl9 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = ctrl9.xl_fastsettl_mode;

  return ret;
}

int32_t ism6hg256x_filt_xl_hp_mode_set(const stmdev_ctx_t *ctx,
                                       ism6hg256x_filt_xl_hp_mode_t val)
{
  ism6hg256x_ctrl9_t ctrl9 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);
  if (ret == 0)
  {
    ctrl9.hp_ref_mode_xl = (uint8_t)val & 0x01U;
    ctrl9.hp_slope_xl_en = ((uint8_t)val >> 1) & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);
  }

  return ret;
}


int32_t ism6hg256x_filt_xl_hp_mode_get(const stmdev_ctx_t *ctx,
                                       ism6hg256x_filt_xl_hp_mode_t *val)
{
  ism6hg256x_ctrl9_t ctrl9 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL9, (uint8_t *)&ctrl9, 1);

  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl9.hp_ref_mode_xl | (ctrl9.hp_slope_xl_en << 1))
  {
    case 0x02:
      *val = ISM6HG256X_HP_MD_NORMAL_SLOPE_ON;
      break;

    case 0x00:
      *val = ISM6HG256X_HP_MD_NORMAL_SLOPE_OFF;
      break;

    case 0x03:
      *val = ISM6HG256X_HP_MD_REFERENCE;
      break;

    default:
      *val = ISM6HG256X_HP_MD_NORMAL_SLOPE_OFF;
      break;
  }

  return ret;
}


int32_t ism6hg256x_filt_wkup_act_feed_set(const stmdev_ctx_t *ctx,
                                          ism6hg256x_filt_wkup_act_feed_t val)
{
  ism6hg256x_wake_up_ths_t wake_up_ths = {0};
  ism6hg256x_tap_cfg0_t tap_cfg0 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_WAKE_UP_THS, (uint8_t *)&wake_up_ths, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1);
  if (ret != 0)
  {
    return ret;
  }

  tap_cfg0.slope_fds = (uint8_t)val & 0x01U;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1);
  if (ret != 0)
  {
    return ret;
  }

  wake_up_ths.usr_off_on_wu = ((uint8_t)val & 0x02U) >> 1;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_WAKE_UP_THS, (uint8_t *)&wake_up_ths, 1);

  return ret;
}


int32_t ism6hg256x_filt_wkup_act_feed_get(const stmdev_ctx_t *ctx,
                                          ism6hg256x_filt_wkup_act_feed_t *val)
{
  ism6hg256x_wake_up_ths_t wake_up_ths = {0};
  ism6hg256x_tap_cfg0_t tap_cfg0 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_WAKE_UP_THS, (uint8_t *)&wake_up_ths, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch ((wake_up_ths.usr_off_on_wu << 1) + tap_cfg0.slope_fds)
  {
    case 0x00:
      *val = ISM6HG256X_WK_FEED_SLOPE;
      break;

    case 0x01:
      *val = ISM6HG256X_WK_FEED_HIGH_PASS;
      break;

    case 0x03:
      *val = ISM6HG256X_WK_FEED_LP_WITH_OFFSET;
      break;

    default:
      *val = ISM6HG256X_WK_FEED_SLOPE;
      break;
  }

  return ret;
}


int32_t ism6hg256x_mask_trigger_xl_settl_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_tap_cfg0_t tap_cfg0 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1);

  if (ret == 0)
  {
    tap_cfg0.hw_func_mask_xl_settl = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1);
  }

  return ret;
}


int32_t ism6hg256x_mask_trigger_xl_settl_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_tap_cfg0_t tap_cfg0 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = tap_cfg0.hw_func_mask_xl_settl;

  return ret;
}


int32_t ism6hg256x_filt_sixd_feed_set(const stmdev_ctx_t *ctx,
                                      ism6hg256x_filt_sixd_feed_t val)
{
  ism6hg256x_tap_cfg0_t tap_cfg0 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1);

  if (ret == 0)
  {
    tap_cfg0.low_pass_on_6d = (uint8_t)val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1);
  }

  return ret;
}


int32_t ism6hg256x_filt_sixd_feed_get(const stmdev_ctx_t *ctx,
                                      ism6hg256x_filt_sixd_feed_t *val)
{
  ism6hg256x_tap_cfg0_t tap_cfg0 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (tap_cfg0.low_pass_on_6d)
  {
    case 0x00:
      *val = ISM6HG256X_SIXD_FEED_ODR_DIV_2;
      break;

    case 0x01:
      *val = ISM6HG256X_SIXD_FEED_LOW_PASS;
      break;

    default:
      *val = ISM6HG256X_SIXD_FEED_ODR_DIV_2;
      break;
  }

  return ret;
}

int32_t ism6hg256x_filt_gy_eis_lp_bandwidth_set(const stmdev_ctx_t *ctx,
                                                ism6hg256x_filt_gy_eis_lp_bandwidth_t val)
{
  ism6hg256x_ctrl_eis_t ctrl_eis = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL_EIS, (uint8_t *)&ctrl_eis, 1);

  if (ret == 0)
  {
    ctrl_eis.lpf_g_eis_bw = (uint8_t)val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL_EIS, (uint8_t *)&ctrl_eis, 1);
  }

  return ret;
}


int32_t ism6hg256x_filt_gy_eis_lp_bandwidth_get(const stmdev_ctx_t *ctx,
                                                ism6hg256x_filt_gy_eis_lp_bandwidth_t *val)
{
  ism6hg256x_ctrl_eis_t ctrl_eis = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL_EIS, (uint8_t *)&ctrl_eis, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl_eis.lpf_g_eis_bw)
  {
    case 0x00:
      *val = ISM6HG256X_EIS_LP_NORMAL;
      break;

    case 0x01:
      *val = ISM6HG256X_EIS_LP_LIGHT;
      break;

    default:
      *val = ISM6HG256X_EIS_LP_NORMAL;
      break;
  }

  return ret;
}


int32_t ism6hg256x_filt_gy_ois_lp_bandwidth_set(const stmdev_ctx_t *ctx,
                                                ism6hg256x_filt_gy_ois_lp_bandwidth_t val)
{
  ism6hg256x_ui_ctrl2_ois_t ui_ctrl2_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_CTRL2_OIS, (uint8_t *)&ui_ctrl2_ois, 1);

  if (ret == 0)
  {
    ui_ctrl2_ois.lpf1_g_ois_bw = (uint8_t)val & 0x03U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_UI_CTRL2_OIS, (uint8_t *)&ui_ctrl2_ois, 1);
  }

  return ret;
}


int32_t ism6hg256x_filt_gy_ois_lp_bandwidth_get(const stmdev_ctx_t *ctx,
                                                ism6hg256x_filt_gy_ois_lp_bandwidth_t *val)
{

  ism6hg256x_ui_ctrl2_ois_t ui_ctrl2_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_CTRL2_OIS, (uint8_t *)&ui_ctrl2_ois, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ui_ctrl2_ois.lpf1_g_ois_bw)
  {
    case 0x00:
      *val = ISM6HG256X_OIS_GY_LP_NORMAL;
      break;

    case 0x01:
      *val = ISM6HG256X_OIS_GY_LP_STRONG;
      break;

    case 0x02:
      *val = ISM6HG256X_OIS_GY_LP_AGGRESSIVE;
      break;

    case 0x03:
      *val = ISM6HG256X_OIS_GY_LP_LIGHT;
      break;

    default:
      *val = ISM6HG256X_OIS_GY_LP_NORMAL;
      break;
  }

  return ret;
}


int32_t ism6hg256x_filt_xl_ois_lp_bandwidth_set(const stmdev_ctx_t *ctx,
                                                ism6hg256x_filt_xl_ois_lp_bandwidth_t val)
{
  ism6hg256x_ui_ctrl3_ois_t ui_ctrl3_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_CTRL3_OIS, (uint8_t *)&ui_ctrl3_ois, 1);

  if (ret == 0)
  {
    ui_ctrl3_ois.lpf_xl_ois_bw = (uint8_t)val & 0x07U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_UI_CTRL3_OIS, (uint8_t *)&ui_ctrl3_ois, 1);
  }

  return ret;
}


int32_t ism6hg256x_filt_xl_ois_lp_bandwidth_get(const stmdev_ctx_t *ctx,
                                                ism6hg256x_filt_xl_ois_lp_bandwidth_t *val)
{
  ism6hg256x_ui_ctrl3_ois_t ui_ctrl3_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_CTRL3_OIS, (uint8_t *)&ui_ctrl3_ois, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ui_ctrl3_ois.lpf_xl_ois_bw)
  {
    case 0x00:
      *val = ISM6HG256X_OIS_XL_LP_ULTRA_LIGHT;
      break;

    case 0x01:
      *val = ISM6HG256X_OIS_XL_LP_VERY_LIGHT;
      break;

    case 0x02:
      *val = ISM6HG256X_OIS_XL_LP_LIGHT;
      break;

    case 0x03:
      *val = ISM6HG256X_OIS_XL_LP_NORMAL;
      break;

    case 0x04:
      *val = ISM6HG256X_OIS_XL_LP_STRONG;
      break;

    case 0x05:
      *val = ISM6HG256X_OIS_XL_LP_VERY_STRONG;
      break;

    case 0x06:
      *val = ISM6HG256X_OIS_XL_LP_AGGRESSIVE;
      break;

    case 0x07:
      *val = ISM6HG256X_OIS_XL_LP_XTREME;
      break;

    default:
      *val = ISM6HG256X_OIS_XL_LP_ULTRA_LIGHT;
      break;
  }

  return ret;
}


int32_t ism6hg256x_fsm_permission_set(const stmdev_ctx_t *ctx,
                                      ism6hg256x_fsm_permission_t val)
{
  ism6hg256x_func_cfg_access_t func_cfg_access = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);

  if (ret == 0)
  {
    func_cfg_access.fsm_wr_ctrl_en = (uint8_t)val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  }

  return ret;
}


int32_t ism6hg256x_fsm_permission_get(const stmdev_ctx_t *ctx,
                                      ism6hg256x_fsm_permission_t *val)
{
  ism6hg256x_func_cfg_access_t func_cfg_access = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (func_cfg_access.fsm_wr_ctrl_en)
  {
    case 0x00:
      *val = ISM6HG256X_PROTECT_CTRL_REGS;
      break;

    case 0x01:
      *val = ISM6HG256X_WRITE_CTRL_REG;
      break;

    default:
      *val = ISM6HG256X_PROTECT_CTRL_REGS;
      break;
  }

  return ret;
}


int32_t ism6hg256x_fsm_permission_status(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_ctrl_status_t ctrl_status = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL_STATUS, (uint8_t *)&ctrl_status, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = ctrl_status.fsm_wr_ctrl_status;

  return ret;
}


int32_t ism6hg256x_fsm_mode_set(const stmdev_ctx_t *ctx, ism6hg256x_fsm_mode_t val)
{
  ism6hg256x_emb_func_en_b_t emb_func_en_b = {0};
  ism6hg256x_fsm_enable_t fsm_enable = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_EN_B, (uint8_t *)&emb_func_en_b, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_FSM_ENABLE, (uint8_t *)&fsm_enable, 1);
  if (ret != 0)
  {
    goto exit;
  }

  if ((val.fsm1_en | val.fsm2_en | val.fsm3_en | val.fsm4_en
       | val.fsm5_en | val.fsm6_en | val.fsm7_en | val.fsm8_en) == PROPERTY_ENABLE)
  {
    emb_func_en_b.fsm_en = PROPERTY_ENABLE;
  }
  else
  {
    emb_func_en_b.fsm_en = PROPERTY_DISABLE;
  }

  fsm_enable.fsm1_en = val.fsm1_en;
  fsm_enable.fsm2_en = val.fsm2_en;
  fsm_enable.fsm3_en = val.fsm3_en;
  fsm_enable.fsm4_en = val.fsm4_en;
  fsm_enable.fsm5_en = val.fsm5_en;
  fsm_enable.fsm6_en = val.fsm6_en;
  fsm_enable.fsm7_en = val.fsm7_en;
  fsm_enable.fsm8_en = val.fsm8_en;

  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FSM_ENABLE, (uint8_t *)&fsm_enable, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_EN_B, (uint8_t *)&emb_func_en_b, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fsm_mode_get(const stmdev_ctx_t *ctx, ism6hg256x_fsm_mode_t *val)
{
  ism6hg256x_fsm_enable_t fsm_enable = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FSM_ENABLE, (uint8_t *)&fsm_enable, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  val->fsm1_en = fsm_enable.fsm1_en;
  val->fsm2_en = fsm_enable.fsm2_en;
  val->fsm3_en = fsm_enable.fsm3_en;
  val->fsm4_en = fsm_enable.fsm4_en;
  val->fsm5_en = fsm_enable.fsm5_en;
  val->fsm6_en = fsm_enable.fsm6_en;
  val->fsm7_en = fsm_enable.fsm7_en;
  val->fsm8_en = fsm_enable.fsm8_en;

  return ret;
}


int32_t ism6hg256x_fsm_long_cnt_set(const stmdev_ctx_t *ctx, uint16_t val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  buff[1] = (uint8_t)(val / 256U);
  buff[0] = (uint8_t)(val - (buff[1] * 256U));

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_FSM_LONG_COUNTER_L, (uint8_t *)&buff[0], 2);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fsm_long_cnt_get(const stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_FSM_LONG_COUNTER_L, &buff[0], 2);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  *val = (uint16_t)(buff[0] | ((uint16_t)buff[1] << 8));

  return ret;
}


int32_t ism6hg256x_fsm_out_get(const stmdev_ctx_t *ctx, ism6hg256x_fsm_out_t *val)
{
  int32_t ret = {0};

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_FSM_OUTS1, (uint8_t *)val, 8);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fsm_data_rate_set(const stmdev_ctx_t *ctx,
                                     ism6hg256x_fsm_data_rate_t val)
{
  ism6hg256x_fsm_odr_t fsm_odr = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_FSM_ODR, (uint8_t *)&fsm_odr, 1);
  if (ret != 0)
  {
    goto exit;
  }
  fsm_odr.fsm_odr = (uint8_t)val & 0x07U;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FSM_ODR, (uint8_t *)&fsm_odr, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fsm_data_rate_get(const stmdev_ctx_t *ctx,
                                     ism6hg256x_fsm_data_rate_t *val)
{
  ism6hg256x_fsm_odr_t fsm_odr = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_FSM_ODR, (uint8_t *)&fsm_odr, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  switch (fsm_odr.fsm_odr)
  {
    case 0x00:
      *val = ISM6HG256X_FSM_15Hz;
      break;

    case 0x01:
      *val = ISM6HG256X_FSM_30Hz;
      break;

    case 0x02:
      *val = ISM6HG256X_FSM_60Hz;
      break;

    case 0x03:
      *val = ISM6HG256X_FSM_120Hz;
      break;

    case 0x04:
      *val = ISM6HG256X_FSM_240Hz;
      break;

    case 0x05:
      *val = ISM6HG256X_FSM_480Hz;
      break;

    case 0x06:
      *val = ISM6HG256X_FSM_960Hz;
      break;

    default:
      *val = ISM6HG256X_FSM_15Hz;
      break;
  }

  return ret;
}

/*
 * Original conversion routines taken from: https://github.com/numpy/numpy
 *
 * uint16_t npy_floatbits_to_halfbits(uint32_t f);
 * uint16_t npy_float_to_half(float_t f);
 * uint32_t npy_halfbits_to_floatbits(uint16_t h)
 * float_t  npy_half_to_float(uint16_t h)
 *
 * Released under BSD-3-Clause License
 */

#ifndef NPY_HALF_GENERATE_OVERFLOW
#define NPY_HALF_GENERATE_OVERFLOW  0 /* do not trigger FP overflow */
#endif
#ifndef NPY_HALF_GENERATE_UNDERFLOW
#define NPY_HALF_GENERATE_UNDERFLOW 0 /* do not trigger FP underflow */
#endif
#ifndef NPY_HALF_ROUND_TIES_TO_EVEN
#define NPY_HALF_ROUND_TIES_TO_EVEN 1
#endif

static uint16_t npy_floatbits_to_halfbits(uint32_t f)
{
  uint32_t f_exp = 0, f_sig = 0;
  uint16_t h_sgn = 0, h_exp = 0, h_sig = 0;

  h_sgn = (uint16_t)((f & 0x80000000u) >> 16);
  f_exp = (f & 0x7f800000u);

  /* Exponent overflow/NaN converts to signed inf/NaN */
  if (f_exp >= 0x47800000u)
  {
    if (f_exp == 0x7f800000u)
    {
      /* Inf or NaN */
      f_sig = (f & 0x007fffffu);
      if (f_sig != 0U)
      {
        /* NaN - propagate the flag in the significand... */
        uint16_t ret = (uint16_t)(0x7c00u + (f_sig >> 13));
        /* ...but make sure it stays a NaN */
        if (ret == 0x7c00u)
        {
          ret++;
        }
        return h_sgn + ret;
      }
      else
      {
        /* signed inf */
        return (uint16_t)(h_sgn + 0x7c00u);
      }
    }
    else
    {
      /* overflow to signed inf */
#if NPY_HALF_GENERATE_OVERFLOW
      npy_set_floatstatus_overflow();
#endif
      return (uint16_t)(h_sgn + 0x7c00u);
    }
  }

  /* Exponent underflow converts to a subnormal half or signed zero */
  if (f_exp <= 0x38000000u)
  {
    /*
     * Signed zeros, subnormal floats, and floats with small
     * exponents all convert to signed zero half-floats.
     */
    if (f_exp < 0x33000000u)
    {
#if NPY_HALF_GENERATE_UNDERFLOW
      /* If f != 0, it underflowed to 0 */
      if ((f & 0x7fffffff) != 0)
      {
        npy_set_floatstatus_underflow();
      }
#endif
      return h_sgn;
    }
    /* Make the subnormal significand */
    f_exp >>= 23;
    f_sig = (0x00800000u + (f & 0x007fffffu));
#if NPY_HALF_GENERATE_UNDERFLOW
    /* If it's not exactly represented, it underflowed */
    if ((f_sig & (((uint32_t)1 << (126 - f_exp)) - 1)) != 0)
    {
      npy_set_floatstatus_underflow();
    }
#endif
    /*
     * Usually the significand is shifted by 13. For subnormals an
     * additional shift needs to occur. This shift is one for the largest
     * exponent giving a subnormal `f_exp = 0x38000000 >> 23 = 112`, which
     * offsets the new first bit. At most the shift can be 1+10 bits.
     */
    f_sig >>= (113U - f_exp);
    /* Handle rounding by adding 1 to the bit beyond half precision */
#if NPY_HALF_ROUND_TIES_TO_EVEN
    /*
     * If the last bit in the half significand is 0 (already even), and
     * the remaining bit pattern is 1000...0, then we do not add one
     * to the bit after the half significand. However, the (113 - f_exp)
     * shift can lose up to 11 bits, so the || checks them in the original.
     * In all other cases, we can just add one.
     */
    if (((f_sig & 0x00003fffu) != 0x00001000u) || (f & 0x000007ffu))
    {
      f_sig += 0x00001000u;
    }
#else
    f_sig += 0x00001000u;
#endif
    h_sig = (uint16_t)(f_sig >> 13);
    /*
     * If the rounding causes a bit to spill into h_exp, it will
     * increment h_exp from zero to one and h_sig will be zero.
     * This is the correct result.
     */
    return (uint16_t)(h_sgn + h_sig);
  }

  /* Regular case with no overflow or underflow */
  h_exp = (uint16_t)((f_exp - 0x38000000u) >> 13);
  /* Handle rounding by adding 1 to the bit beyond half precision */
  f_sig = (f & 0x007fffffu);
#if NPY_HALF_ROUND_TIES_TO_EVEN
  /*
   * If the last bit in the half significand is 0 (already even), and
   * the remaining bit pattern is 1000...0, then we do not add one
   * to the bit after the half significand.  In all other cases, we do.
   */
  if ((f_sig & 0x00003fffu) != 0x00001000u)
  {
    f_sig += 0x00001000u;
  }
#else
  f_sig += 0x00001000u;
#endif
  h_sig = (uint16_t)(f_sig >> 13);
  /*
   * If the rounding causes a bit to spill into h_exp, it will
   * increment h_exp by one and h_sig will be zero.  This is the
   * correct result.  h_exp may increment to 15, at greatest, in
   * which case the result overflows to a signed inf.
   */
#if NPY_HALF_GENERATE_OVERFLOW
  h_sig += h_exp;
  if (h_sig == 0x7c00u)
  {
    npy_set_floatstatus_overflow();
  }
  return (uint16_t)(h_sgn + h_sig);
#else
  return (uint16_t)(h_sgn + h_exp + h_sig);
#endif
}

static uint16_t npy_float_to_half(float_t f)
{
  uint32_t bits;

  (void)memcpy(&bits, &f, sizeof(bits));

  return npy_floatbits_to_halfbits(bits);
}

static uint32_t npy_halfbits_to_floatbits(uint16_t h)
{
  uint16_t h_exp = (h & 0x7c00u);
  uint32_t f_sgn = ((uint32_t)h & 0x8000u) << 16;
  uint32_t result = 0u;

  switch (h_exp)
  {
    case 0x0000u:   // 0 or subnormal
    {
      uint16_t h_sig = (uint16_t)(h & 0x03ffu);
      // Signed zero
      if (h_sig == 0u)
      {
        result = f_sgn;
        break;
      }
      // Subnormal
      h_sig <<= 1;
      while ((h_sig & 0x0400u) == 0u)
      {
        h_sig <<= 1;
        h_exp++;
      }
      uint32_t f_exp = ((uint32_t)(127u - 15u - h_exp)) << 23;
      uint32_t f_sig = ((uint32_t)h_sig & 0x03ffu) << 13;
      result = f_sgn + f_exp + f_sig;
      break;
    }
    case 0x7c00u: // inf or NaN
      // All-ones exponent and a copy of the significand
      result = f_sgn + 0x7f800000u + (((uint32_t)h & 0x03ffu) << 13);
      break;
    default: // normalized
      // Just need to adjust the exponent and shift
      result = f_sgn + ((((uint32_t)h & 0x7fffu) + 0x1c000u) << 13);
      break;
  }

  return result;
}

static float_t npy_half_to_float(uint16_t h)
{
  float_t f = 0;

  uint32_t bits = npy_halfbits_to_floatbits(h);

  (void)memcpy(&f, &bits, sizeof(bits));

  return f;
}


int32_t ism6hg256x_sflp_game_gbias_set(const stmdev_ctx_t *ctx,
                                       const ism6hg256x_sflp_gbias_t *val)
{
  ism6hg256x_sflp_data_rate_t sflp_odr = ISM6HG256X_SFLP_15Hz;
  uint16_t gbias_hf[3] = {0};
  float_t k = 0.005f;
  int32_t ret = 0;

  ret = ism6hg256x_sflp_data_rate_get(ctx, &sflp_odr);
  if (ret != 0)
  {
    return ret;
  }

  /* Calculate k factor */
  switch (sflp_odr)
  {
    default:
    case ISM6HG256X_SFLP_15Hz:
      k = 0.04f;
      break;
    case ISM6HG256X_SFLP_30Hz:
      k = 0.02f;
      break;
    case ISM6HG256X_SFLP_60Hz:
      k = 0.01f;
      break;
    case ISM6HG256X_SFLP_120Hz:
      k = 0.005f;
      break;
    case ISM6HG256X_SFLP_240Hz:
      k = 0.0025f;
      break;
    case ISM6HG256X_SFLP_480Hz:
      k = 0.00125f;
      break;
  }

  /* compute gbias as half precision float in order to be put in embedded advanced feature register */
  gbias_hf[0] = npy_float_to_half(val->gbias_x * (3.14159265358979323846f / 180.0f) / k);
  gbias_hf[1] = npy_float_to_half(val->gbias_y * (3.14159265358979323846f / 180.0f) / k);
  gbias_hf[2] = npy_float_to_half(val->gbias_z * (3.14159265358979323846f / 180.0f) / k);

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_SFLP_GBIASX_INIT_L, (uint8_t *)&gbias_hf[0], 6);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_fsm_ext_sens_sensitivity_set(const stmdev_ctx_t *ctx, uint16_t val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  buff[1] = (uint8_t)(val / 256U);
  buff[0] = (uint8_t)(val - (buff[1] * 256U));
  ret = ism6hg256x_ln_pg_write(ctx, ISM6HG256X_FSM_EXT_SENSITIVITY_L, (uint8_t *)&buff[0], 2);

  return ret;
}

int32_t ism6hg256x_fsm_ext_sens_sensitivity_get(const stmdev_ctx_t *ctx,
                                                uint16_t *val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_FSM_EXT_SENSITIVITY_L, &buff[0], 2);
  if (ret != 0)
  {
    return ret;
  }

  *val = (uint16_t)(buff[0] | ((uint16_t)buff[1] << 8));

  return ret;
}


int32_t ism6hg256x_fsm_ext_sens_offset_set(const stmdev_ctx_t *ctx,
                                           ism6hg256x_xl_fsm_ext_sens_offset_t val)
{
  uint8_t buff[6] = {0};
  int32_t ret = 0;

  buff[1] = (uint8_t)(val.x / 256U);
  buff[0] = (uint8_t)(val.x - (buff[1] * 256U));
  buff[3] = (uint8_t)(val.y / 256U);
  buff[2] = (uint8_t)(val.y - (buff[3] * 256U));
  buff[5] = (uint8_t)(val.z / 256U);
  buff[4] = (uint8_t)(val.z - (buff[5] * 256U));
  ret = ism6hg256x_ln_pg_write(ctx, ISM6HG256X_FSM_EXT_OFFX_L, (uint8_t *)&buff[0], 6);

  return ret;
}


int32_t ism6hg256x_fsm_ext_sens_offset_get(const stmdev_ctx_t *ctx,
                                           ism6hg256x_xl_fsm_ext_sens_offset_t *val)
{
  uint8_t buff[6] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_FSM_EXT_OFFX_L, &buff[0], 6);
  if (ret != 0)
  {
    return ret;
  }

  val->x = (uint16_t)(buff[0] | ((uint16_t)buff[1] << 8));
  val->y = (uint16_t)(buff[2] | ((uint16_t)buff[3] << 8));
  val->z = (uint16_t)(buff[4] | ((uint16_t)buff[5] << 8));

  return ret;
}

int32_t ism6hg256x_fsm_ext_sens_matrix_set(const stmdev_ctx_t *ctx,
                                           ism6hg256x_xl_fsm_ext_sens_matrix_t val)
{
  uint8_t buff[12] = {0};
  int32_t ret = 0;

  buff[1] = (uint8_t)(val.xx / 256U);
  buff[0] = (uint8_t)(val.xx - (buff[1] * 256U));
  buff[3] = (uint8_t)(val.xy / 256U);
  buff[2] = (uint8_t)(val.xy - (buff[3] * 256U));
  buff[5] = (uint8_t)(val.xz / 256U);
  buff[4] = (uint8_t)(val.xz - (buff[5] * 256U));
  buff[7] = (uint8_t)(val.yy / 256U);
  buff[6] = (uint8_t)(val.yy - (buff[7] * 256U));
  buff[9] = (uint8_t)(val.yz / 256U);
  buff[8] = (uint8_t)(val.yz - (buff[9] * 256U));
  buff[11] = (uint8_t)(val.zz / 256U);
  buff[10] = (uint8_t)(val.zz - (buff[11] * 256U));
  ret = ism6hg256x_ln_pg_write(ctx, ISM6HG256X_FSM_EXT_MATRIX_XX_L, (uint8_t *)&buff[0], 12);

  return ret;
}


int32_t ism6hg256x_fsm_ext_sens_matrix_get(const stmdev_ctx_t *ctx,
                                           ism6hg256x_xl_fsm_ext_sens_matrix_t *val)
{
  uint8_t buff[12] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_FSM_EXT_MATRIX_XX_L, &buff[0], 12);
  if (ret != 0)
  {
    return ret;
  }

  val->xx = (uint16_t)(buff[0] | ((uint16_t)buff[1] << 8));
  val->xy = (uint16_t)(buff[2] | ((uint16_t)buff[3] << 8));
  val->xz = (uint16_t)(buff[4] | ((uint16_t)buff[5] << 8));
  val->yy = (uint16_t)(buff[6] | ((uint16_t)buff[7] << 8));
  val->yz = (uint16_t)(buff[8] | ((uint16_t)buff[9] << 8));
  val->zz = (uint16_t)(buff[10] | ((uint16_t)buff[11] << 8));

  return ret;
}


int32_t ism6hg256x_fsm_ext_sens_z_orient_set(const stmdev_ctx_t *ctx,
                                             ism6hg256x_fsm_ext_sens_z_orient_t val)
{
  ism6hg256x_ext_cfg_a_t ext_cfg_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EXT_CFG_A, (uint8_t *)&ext_cfg_a, 1);
  if (ret != 0)
  {
    return ret;
  }
  ext_cfg_a.ext_z_axis = (uint8_t)val & 0x07U;
  ret = ism6hg256x_ln_pg_write(ctx, ISM6HG256X_EXT_CFG_A, (uint8_t *)&ext_cfg_a, 1);

  return ret;
}


int32_t ism6hg256x_fsm_ext_sens_z_orient_get(const stmdev_ctx_t *ctx,
                                             ism6hg256x_fsm_ext_sens_z_orient_t *val)
{
  ism6hg256x_ext_cfg_a_t ext_cfg_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EXT_CFG_A, (uint8_t *)&ext_cfg_a, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ext_cfg_a.ext_z_axis)
  {
    case 0x00:
      *val = ISM6HG256X_Z_EQ_Y;
      break;

    case 0x01:
      *val = ISM6HG256X_Z_EQ_MIN_Y;
      break;

    case 0x02:
      *val = ISM6HG256X_Z_EQ_X;
      break;

    case 0x03:
      *val = ISM6HG256X_Z_EQ_MIN_X;
      break;

    case 0x04:
      *val = ISM6HG256X_Z_EQ_MIN_Z;
      break;

    case 0x05:
      *val = ISM6HG256X_Z_EQ_Z;
      break;

    default:
      *val = ISM6HG256X_Z_EQ_Y;
      break;
  }

  return ret;
}


int32_t ism6hg256x_fsm_ext_sens_y_orient_set(const stmdev_ctx_t *ctx,
                                             ism6hg256x_fsm_ext_sens_y_orient_t val)
{
  ism6hg256x_ext_cfg_a_t ext_cfg_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EXT_CFG_A, (uint8_t *)&ext_cfg_a, 1);
  if (ret == 0)
  {
    ext_cfg_a.ext_y_axis = (uint8_t)val & 0x7U;
    ret = ism6hg256x_ln_pg_write(ctx, ISM6HG256X_EXT_CFG_A, (uint8_t *)&ext_cfg_a, 1);
  }

  return ret;
}


int32_t ism6hg256x_fsm_ext_sens_y_orient_get(const stmdev_ctx_t *ctx,
                                             ism6hg256x_fsm_ext_sens_y_orient_t *val)
{
  ism6hg256x_ext_cfg_a_t ext_cfg_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EXT_CFG_A, (uint8_t *)&ext_cfg_a, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ext_cfg_a.ext_y_axis)
  {
    case 0x00:
      *val = ISM6HG256X_Y_EQ_Y;
      break;

    case 0x01:
      *val = ISM6HG256X_Y_EQ_MIN_Y;
      break;

    case 0x02:
      *val = ISM6HG256X_Y_EQ_X;
      break;

    case 0x03:
      *val = ISM6HG256X_Y_EQ_MIN_X;
      break;

    case 0x04:
      *val = ISM6HG256X_Y_EQ_MIN_Z;
      break;

    case 0x05:
      *val = ISM6HG256X_Y_EQ_Z;
      break;

    default:
      *val = ISM6HG256X_Y_EQ_Y;
      break;
  }

  return ret;
}


int32_t ism6hg256x_fsm_ext_sens_x_orient_set(const stmdev_ctx_t *ctx,
                                             ism6hg256x_fsm_ext_sens_x_orient_t val)
{
  ism6hg256x_ext_cfg_b_t ext_cfg_b = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EXT_CFG_B, (uint8_t *)&ext_cfg_b, 1);
  if (ret == 0)
  {
    ext_cfg_b.ext_x_axis = (uint8_t)val & 0x7U;
    ret = ism6hg256x_ln_pg_write(ctx, ISM6HG256X_EXT_CFG_B, (uint8_t *)&ext_cfg_b, 1);
  }

  return ret;
}


int32_t ism6hg256x_fsm_ext_sens_x_orient_get(const stmdev_ctx_t *ctx,
                                             ism6hg256x_fsm_ext_sens_x_orient_t *val)
{
  ism6hg256x_ext_cfg_b_t ext_cfg_b = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EXT_CFG_B, (uint8_t *)&ext_cfg_b, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ext_cfg_b.ext_x_axis)
  {
    case 0x00:
      *val = ISM6HG256X_X_EQ_Y;
      break;

    case 0x01:
      *val = ISM6HG256X_X_EQ_MIN_Y;
      break;

    case 0x02:
      *val = ISM6HG256X_X_EQ_X;
      break;

    case 0x03:
      *val = ISM6HG256X_X_EQ_MIN_X;
      break;

    case 0x04:
      *val = ISM6HG256X_X_EQ_MIN_Z;
      break;

    case 0x05:
      *val = ISM6HG256X_X_EQ_Z;
      break;

    default:
      *val = ISM6HG256X_X_EQ_Y;
      break;
  }

  return ret;
}


int32_t ism6hg256x_xl_hg_peak_tracking_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_emb_func_init_b_t emb_func_init_b = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_INIT_B, (uint8_t *)&emb_func_init_b, 1);
  if (ret != 0)
  {
    goto exit;
  }
  emb_func_init_b.pt_init = val & 0x01U;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_INIT_B, (uint8_t *)&emb_func_init_b, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_xl_hg_peak_tracking_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_emb_func_init_b_t emb_func_init_b = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_INIT_B, (uint8_t *)&emb_func_init_b, 1);
  if (ret != 0)
  {
    goto exit;
  }
  *val = emb_func_init_b.pt_init & 0x01U;

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_xl_hg_sensitivity_set(const stmdev_ctx_t *ctx, uint16_t val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  buff[1] = (uint8_t)(val / 256U);
  buff[0] = (uint8_t)(val - (buff[1] * 256U));
  ret = ism6hg256x_ln_pg_write(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_XL_HG_SENSITIVITY_L,
                               (uint8_t *)&buff[0], 2);

  return ret;
}


int32_t ism6hg256x_xl_hg_sensitivity_get(const stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_XL_HG_SENSITIVITY_L, &buff[0],
                              2);
  if (ret != 0)
  {
    return ret;
  }

  *val = (uint16_t)(buff[0] | ((uint16_t)buff[1] << 8));

  return ret;
}


int32_t ism6hg256x_fsm_long_cnt_timeout_set(const stmdev_ctx_t *ctx, uint16_t val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  buff[1] = (uint8_t)(val / 256U);
  buff[0] = (uint8_t)(val - (buff[1] * 256U));
  ret = ism6hg256x_ln_pg_write(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_FSM_LC_TIMEOUT_L,
                               (uint8_t *)&buff[0], 2);

  return ret;
}

int32_t ism6hg256x_fsm_long_cnt_timeout_get(const stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_FSM_LC_TIMEOUT_L, &buff[0],
                              2);
  if (ret != 0)
  {
    return ret;
  }

  *val = (uint16_t)(buff[0] | ((uint16_t)buff[1] << 8));

  return ret;
}


int32_t ism6hg256x_fsm_number_of_programs_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_fsm_programs_t fsm_programs = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_FSM_PROGRAMS,
                              (uint8_t *)&fsm_programs, 1);
  if (ret == 0)
  {
    fsm_programs.fsm_n_prog = val;
    ret = ism6hg256x_ln_pg_write(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_FSM_PROGRAMS,
                                 (uint8_t *)&fsm_programs, 1);
  }

  return ret;
}


int32_t ism6hg256x_fsm_number_of_programs_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_fsm_programs_t fsm_programs = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_FSM_PROGRAMS,
                              (uint8_t *)&fsm_programs, 1);
  if (ret == 0)
  {
    *val = fsm_programs.fsm_n_prog;
  }

  return ret;
}


int32_t ism6hg256x_fsm_start_address_set(const stmdev_ctx_t *ctx, uint16_t val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  buff[1] = (uint8_t)(val / 256U);
  buff[0] = (uint8_t)(val - (buff[1] * 256U));
  ret = ism6hg256x_ln_pg_write(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_FSM_START_ADD_L,
                               (uint8_t *)&buff[0], 2);

  return ret;
}


int32_t ism6hg256x_fsm_start_address_get(const stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_FSM_START_ADD_L, &buff[0], 2);
  if (ret != 0)
  {
    return ret;
  }

  *val = (uint16_t)(buff[0] | ((uint16_t)buff[1] << 8));

  return ret;
}


int32_t ism6hg256x_ff_time_windows_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_wake_up_dur_t wake_up_dur = {0};
  ism6hg256x_free_fall_t free_fall = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
  if (ret != 0)
  {
    return ret;
  }
  wake_up_dur.ff_dur = ((uint8_t)val & 0x20U) >> 5;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
  if (ret != 0)
  {
    return ret;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FREE_FALL, (uint8_t *)&free_fall, 1);
  if (ret != 0)
  {
    return ret;
  }
  free_fall.ff_dur = (uint8_t)val & 0x1FU;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_FREE_FALL, (uint8_t *)&free_fall, 1);

  return ret;
}


int32_t ism6hg256x_ff_time_windows_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_wake_up_dur_t wake_up_dur = {0};
  ism6hg256x_free_fall_t free_fall = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_FREE_FALL, (uint8_t *)&free_fall, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = (uint8_t)(wake_up_dur.ff_dur << 5) | free_fall.ff_dur;

  return ret;
}


int32_t ism6hg256x_ff_thresholds_set(const stmdev_ctx_t *ctx,
                                     ism6hg256x_ff_thresholds_t val)
{
  ism6hg256x_free_fall_t free_fall = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FREE_FALL, (uint8_t *)&free_fall, 1);
  if (ret == 0)
  {
    free_fall.ff_ths = (uint8_t)val & 0x7U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FREE_FALL, (uint8_t *)&free_fall, 1);
  }

  return ret;
}


int32_t ism6hg256x_ff_thresholds_get(const stmdev_ctx_t *ctx,
                                     ism6hg256x_ff_thresholds_t *val)
{
  ism6hg256x_free_fall_t free_fall = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FREE_FALL, (uint8_t *)&free_fall, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (free_fall.ff_ths)
  {
    case 0x00:
      *val = ISM6HG256X_156_mg;
      break;

    case 0x01:
      *val = ISM6HG256X_219_mg;
      break;

    case 0x02:
      *val = ISM6HG256X_250_mg;
      break;

    case 0x03:
      *val = ISM6HG256X_312_mg;
      break;

    case 0x04:
      *val = ISM6HG256X_344_mg;
      break;

    case 0x05:
      *val = ISM6HG256X_406_mg;
      break;

    case 0x06:
      *val = ISM6HG256X_469_mg;
      break;

    case 0x07:
      *val = ISM6HG256X_500_mg;
      break;

    default:
      *val = ISM6HG256X_156_mg;
      break;
  }

  return ret;
}


int32_t ism6hg256x_mlc_set(const stmdev_ctx_t *ctx, ism6hg256x_mlc_mode_t val)
{
  ism6hg256x_emb_func_en_b_t emb_en_b = {0};
  ism6hg256x_emb_func_en_a_t emb_en_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_en_a, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_EN_B, (uint8_t *)&emb_en_b, 1);
  if (ret != 0)
  {
    goto exit;
  }

  switch (val)
  {
    case ISM6HG256X_MLC_OFF:
      emb_en_a.mlc_before_fsm_en = 0;
      emb_en_b.mlc_en = 0;
      break;
    case ISM6HG256X_MLC_ON:
      emb_en_a.mlc_before_fsm_en = 0;
      emb_en_b.mlc_en = 1;
      break;
    case ISM6HG256X_MLC_ON_BEFORE_FSM:
      emb_en_a.mlc_before_fsm_en = 1;
      emb_en_b.mlc_en = 0;
      break;
    default:
      // default used: MLC_OFF
      emb_en_a.mlc_before_fsm_en = 0;
      emb_en_b.mlc_en = 0;
      break;
  }

  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_en_a, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_EN_B, (uint8_t *)&emb_en_b, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_mlc_get(const stmdev_ctx_t *ctx, ism6hg256x_mlc_mode_t *val)
{
  ism6hg256x_emb_func_en_b_t emb_en_b = {0};
  ism6hg256x_emb_func_en_a_t emb_en_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_en_a, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_EN_B, (uint8_t *)&emb_en_b, 1);
  if (ret != 0)
  {
    goto exit;
  }

  if (emb_en_a.mlc_before_fsm_en == 0U && emb_en_b.mlc_en == 0U)
  {
    *val = ISM6HG256X_MLC_OFF;
  }
  else if (emb_en_a.mlc_before_fsm_en == 0U && emb_en_b.mlc_en == 1U)
  {
    *val = ISM6HG256X_MLC_ON;
  }
  else if (emb_en_a.mlc_before_fsm_en == 1U)
  {
    *val = ISM6HG256X_MLC_ON_BEFORE_FSM;
  }
  else
  {
    /* Do nothing */
  }

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_mlc_data_rate_set(const stmdev_ctx_t *ctx,
                                     ism6hg256x_mlc_data_rate_t val)
{
  ism6hg256x_mlc_odr_t mlc_odr = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_MLC_ODR, (uint8_t *)&mlc_odr, 1);
  if (ret != 0)
  {
    goto exit;
  }

  mlc_odr.mlc_odr = (uint8_t)val & 0x07U;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_MLC_ODR, (uint8_t *)&mlc_odr, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_mlc_data_rate_get(const stmdev_ctx_t *ctx,
                                     ism6hg256x_mlc_data_rate_t *val)
{
  ism6hg256x_mlc_odr_t mlc_odr = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_MLC_ODR, (uint8_t *)&mlc_odr, 1);
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  switch (mlc_odr.mlc_odr)
  {
    case 0x00:
      *val = ISM6HG256X_MLC_15Hz;
      break;

    case 0x01:
      *val = ISM6HG256X_MLC_30Hz;
      break;

    case 0x02:
      *val = ISM6HG256X_MLC_60Hz;
      break;

    case 0x03:
      *val = ISM6HG256X_MLC_120Hz;
      break;

    case 0x04:
      *val = ISM6HG256X_MLC_240Hz;
      break;

    case 0x05:
      *val = ISM6HG256X_MLC_480Hz;
      break;

    case 0x06:
      *val = ISM6HG256X_MLC_960Hz;
      break;

    default:
      *val = ISM6HG256X_MLC_15Hz;
      break;
  }

  return ret;
}


int32_t ism6hg256x_mlc_out_get(const stmdev_ctx_t *ctx, ism6hg256x_mlc_out_t *val)
{
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret == 0)
  {
    ret = ism6hg256x_read_reg(ctx, ISM6HG256X_MLC1_SRC, (uint8_t *)val, 8);
  }
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_mlc_ext_sens_sensitivity_set(const stmdev_ctx_t *ctx, uint16_t val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  buff[1] = (uint8_t)(val / 256U);
  buff[0] = (uint8_t)(val - (buff[1] * 256U));

  ret = ism6hg256x_ln_pg_write(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_MLC_EXT_SENSITIVITY_L,
                               (uint8_t *)&buff[0], 2);

  return ret;
}

int32_t ism6hg256x_mlc_ext_sens_sensitivity_get(const stmdev_ctx_t *ctx,
                                                uint16_t *val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_MLC_EXT_SENSITIVITY_L,
                              &buff[0], 2);
  if (ret != 0)
  {
    return ret;
  }

  *val = (uint16_t)(buff[0] | ((uint16_t)buff[1] << 8));

  return ret;
}

int32_t ism6hg256x_ois_ctrl_mode_set(const stmdev_ctx_t *ctx,
                                     ism6hg256x_ois_ctrl_mode_t val)
{
  ism6hg256x_func_cfg_access_t func_cfg_access = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  if (ret == 0)
  {
    func_cfg_access.ois_ctrl_from_ui = (uint8_t)val & 0x1U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  }

  return ret;
}


int32_t ism6hg256x_ois_ctrl_mode_get(const stmdev_ctx_t *ctx,
                                     ism6hg256x_ois_ctrl_mode_t *val)
{
  ism6hg256x_func_cfg_access_t func_cfg_access = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (func_cfg_access.ois_ctrl_from_ui)
  {
    case 0x00:
      *val = ISM6HG256X_OIS_CTRL_FROM_OIS;
      break;

    case 0x01:
      *val = ISM6HG256X_OIS_CTRL_FROM_UI;
      break;

    default:
      *val = ISM6HG256X_OIS_CTRL_FROM_OIS;
      break;
  }

  return ret;
}


int32_t ism6hg256x_ois_reset_set(const stmdev_ctx_t *ctx, int8_t val)
{
  ism6hg256x_func_cfg_access_t func_cfg_access = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  if (ret == 0)
  {
    func_cfg_access.if2_reset = (uint8_t)val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  }

  return ret;
}


int32_t ism6hg256x_ois_reset_get(const stmdev_ctx_t *ctx, int8_t *val)
{
  ism6hg256x_func_cfg_access_t func_cfg_access = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
  if (ret != 0)
  {
    return ret;
  }
  *val = (int8_t)func_cfg_access.if2_reset;

  return ret;
}


int32_t ism6hg256x_ois_interface_pull_up_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_pin_ctrl_t pin_ctrl = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);
  if (ret == 0)
  {
    pin_ctrl.ois_pu_dis = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);
  }

  return ret;
}


int32_t ism6hg256x_ois_interface_pull_up_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_pin_ctrl_t pin_ctrl = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);
  if (ret != 0)
  {
    return ret;
  }
  *val = pin_ctrl.ois_pu_dis;

  return ret;
}


int32_t ism6hg256x_ois_handshake_from_ui_set(const stmdev_ctx_t *ctx,
                                             ism6hg256x_ois_handshake_t val)
{
  ism6hg256x_ui_handshake_ctrl_t ui_handshake_ctrl = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_HANDSHAKE_CTRL, (uint8_t *)&ui_handshake_ctrl, 1);
  if (ret == 0)
  {
    ui_handshake_ctrl.ui_shared_ack = val.ack;
    ui_handshake_ctrl.ui_shared_req = val.req;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_UI_HANDSHAKE_CTRL, (uint8_t *)&ui_handshake_ctrl, 1);
  }

  return ret;
}

int32_t ism6hg256x_ois_handshake_from_ui_get(const stmdev_ctx_t *ctx,
                                             ism6hg256x_ois_handshake_t *val)
{
  ism6hg256x_ui_handshake_ctrl_t ui_handshake_ctrl = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_HANDSHAKE_CTRL, (uint8_t *)&ui_handshake_ctrl, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->ack = ui_handshake_ctrl.ui_shared_ack;
  val->req = ui_handshake_ctrl.ui_shared_req;

  return ret;
}


int32_t ism6hg256x_ois_handshake_from_ois_set(const stmdev_ctx_t *ctx,
                                              ism6hg256x_ois_handshake_t val)
{
  ism6hg256x_if2_handshake_ctrl_t if2_handshake_ctrl = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF2_HANDSHAKE_CTRL, (uint8_t *)&if2_handshake_ctrl, 1);
  if (ret == 0)
  {
    if2_handshake_ctrl.if2_shared_ack = val.ack;
    if2_handshake_ctrl.if2_shared_req = val.req;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_IF2_HANDSHAKE_CTRL, (uint8_t *)&if2_handshake_ctrl, 1);
  }

  return ret;
}

int32_t ism6hg256x_ois_handshake_from_ois_get(const stmdev_ctx_t *ctx,
                                              ism6hg256x_ois_handshake_t *val)
{
  ism6hg256x_if2_handshake_ctrl_t if2_handshake_ctrl = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF2_HANDSHAKE_CTRL, (uint8_t *)&if2_handshake_ctrl, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->ack = if2_handshake_ctrl.if2_shared_ack;
  val->req = if2_handshake_ctrl.if2_shared_req;

  return ret;
}


int32_t ism6hg256x_ois_shared_set(const stmdev_ctx_t *ctx, uint8_t val[6])
{
  int32_t ret = 0;

  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_UI_IF2_SHARED_0, val, 6);

  return ret;
}

int32_t ism6hg256x_ois_shared_get(const stmdev_ctx_t *ctx, uint8_t val[6])
{
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_IF2_SHARED_0, val, 6);

  return ret;
}


int32_t ism6hg256x_ois_on_if2_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_ui_ctrl1_ois_t ui_ctrl1_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_CTRL1_OIS, (uint8_t *)&ui_ctrl1_ois, 1);
  if (ret == 0)
  {
    ui_ctrl1_ois.if2_spi_read_en = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_UI_CTRL1_OIS, (uint8_t *)&ui_ctrl1_ois, 1);
  }

  return ret;
}

int32_t ism6hg256x_ois_on_if2_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_ui_ctrl1_ois_t ui_ctrl1_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_CTRL1_OIS, (uint8_t *)&ui_ctrl1_ois, 1);
  if (ret != 0)
  {
    return ret;
  }
  *val = ui_ctrl1_ois.if2_spi_read_en;

  return ret;
}


int32_t ism6hg256x_ois_chain_set(const stmdev_ctx_t *ctx, ism6hg256x_ois_chain_t val)
{
  ism6hg256x_ui_ctrl1_ois_t ui_ctrl1_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_CTRL1_OIS, (uint8_t *)&ui_ctrl1_ois, 1);
  if (ret == 0)
  {
    ui_ctrl1_ois.ois_g_en = val.gy;
    ui_ctrl1_ois.ois_xl_en = val.xl;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_UI_CTRL1_OIS, (uint8_t *)&ui_ctrl1_ois, 1);
  }

  return ret;
}


int32_t ism6hg256x_ois_chain_get(const stmdev_ctx_t *ctx, ism6hg256x_ois_chain_t *val)
{
  ism6hg256x_ui_ctrl1_ois_t ui_ctrl1_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_CTRL1_OIS, (uint8_t *)&ui_ctrl1_ois, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->gy = ui_ctrl1_ois.ois_g_en;
  val->xl = ui_ctrl1_ois.ois_xl_en;

  return ret;
}


int32_t ism6hg256x_ois_gy_full_scale_set(const stmdev_ctx_t *ctx,
                                         ism6hg256x_ois_gy_full_scale_t val)
{
  ism6hg256x_ui_ctrl2_ois_t ui_ctrl2_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_CTRL2_OIS, (uint8_t *)&ui_ctrl2_ois, 1);
  if (ret == 0)
  {
    ui_ctrl2_ois.fs_g_ois = (uint8_t)val & 0x07U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_UI_CTRL2_OIS, (uint8_t *)&ui_ctrl2_ois, 1);
  }

  return ret;
}


int32_t ism6hg256x_ois_gy_full_scale_get(const stmdev_ctx_t *ctx,
                                         ism6hg256x_ois_gy_full_scale_t *val)
{
  ism6hg256x_ui_ctrl2_ois_t ui_ctrl2_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_CTRL2_OIS, (uint8_t *)&ui_ctrl2_ois, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ui_ctrl2_ois.fs_g_ois)
  {
    case 0x01:
      *val = ISM6HG256X_OIS_250dps;
      break;

    case 0x02:
      *val = ISM6HG256X_OIS_500dps;
      break;

    case 0x03:
      *val = ISM6HG256X_OIS_1000dps;
      break;

    case 0x04:
      *val = ISM6HG256X_OIS_2000dps;
      break;

    default:
      *val = ISM6HG256X_OIS_250dps;
      break;
  }

  return ret;
}


int32_t ism6hg256x_ois_xl_full_scale_set(const stmdev_ctx_t *ctx,
                                         ism6hg256x_ois_xl_full_scale_t val)
{
  ism6hg256x_ui_ctrl3_ois_t ui_ctrl3_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_CTRL3_OIS, (uint8_t *)&ui_ctrl3_ois, 1);
  if (ret == 0)
  {
    ui_ctrl3_ois.fs_xl_ois = (uint8_t)val & 0x3U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_UI_CTRL3_OIS, (uint8_t *)&ui_ctrl3_ois, 1);
  }

  return ret;
}


int32_t ism6hg256x_ois_xl_full_scale_get(const stmdev_ctx_t *ctx,
                                         ism6hg256x_ois_xl_full_scale_t *val)
{
  ism6hg256x_ui_ctrl3_ois_t ui_ctrl3_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_CTRL3_OIS, (uint8_t *)&ui_ctrl3_ois, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ui_ctrl3_ois.fs_xl_ois)
  {
    case 0x00:
      *val = ISM6HG256X_OIS_2g;
      break;

    case 0x01:
      *val = ISM6HG256X_OIS_4g;
      break;

    case 0x02:
      *val = ISM6HG256X_OIS_8g;
      break;

    case 0x03:
      *val = ISM6HG256X_OIS_16g;
      break;

    default:
      *val = ISM6HG256X_OIS_2g;
      break;
  }

  return ret;
}


int32_t ism6hg256x_6d_threshold_set(const stmdev_ctx_t *ctx,
                                    ism6hg256x_6d_threshold_t val)
{
  ism6hg256x_tap_ths_6d_t tap_ths_6d = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1);
  if (ret == 0)
  {
    tap_ths_6d.sixd_ths = (uint8_t)val & 0x03U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1);
  }

  return ret;
}


int32_t ism6hg256x_6d_threshold_get(const stmdev_ctx_t *ctx,
                                    ism6hg256x_6d_threshold_t *val)
{
  ism6hg256x_tap_ths_6d_t tap_ths_6d = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (tap_ths_6d.sixd_ths)
  {
    case 0x00:
      *val = ISM6HG256X_DEG_80;
      break;

    case 0x01:
      *val = ISM6HG256X_DEG_70;
      break;

    case 0x02:
      *val = ISM6HG256X_DEG_60;
      break;

    case 0x03:
      *val = ISM6HG256X_DEG_50;
      break;

    default:
      *val = ISM6HG256X_DEG_80;
      break;
  }

  return ret;
}


int32_t ism6hg256x_4d_mode_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_tap_ths_6d_t tap_ths_6d = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1);
  if (ret == 0)
  {
    tap_ths_6d.d4d_en = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1);
  }

  return ret;
}


int32_t ism6hg256x_4d_mode_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_tap_ths_6d_t tap_ths_6d = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = tap_ths_6d.d4d_en;

  return ret;
}


int32_t ism6hg256x_i3c_config_set(const stmdev_ctx_t *ctx,
                                  ism6hg256x_i3c_config_t val)
{
  ism6hg256x_pin_ctrl_t pin_ctrl = {0};
  ism6hg256x_ctrl5_t ctrl5 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL5, (uint8_t *)&ctrl5, 1);
  if (ret == 0)
  {
    pin_ctrl.ibhr_por_en = (uint8_t)val.rst_mode & 0x01U;
    ctrl5.bus_act_sel = (uint8_t)val.ibi_time & 0x03U;
    ctrl5.if2_ta0_pid = (uint8_t)val.if2_ta0_pid & 0x01U;

    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);
    ret += ism6hg256x_write_reg(ctx, ISM6HG256X_CTRL5, (uint8_t *)&ctrl5, 1);
  }

  return ret;
}


int32_t ism6hg256x_i3c_config_get(const stmdev_ctx_t *ctx,
                                  ism6hg256x_i3c_config_t *val)
{
  ism6hg256x_pin_ctrl_t pin_ctrl = {0};
  ism6hg256x_ctrl5_t ctrl5 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (pin_ctrl.ibhr_por_en)
  {
    case 0x00:
      val->rst_mode = ISM6HG256X_SW_RST_DYN_ADDRESS_RST;
      break;

    case 0x01:
      val->rst_mode = ISM6HG256X_I3C_GLOBAL_RST;
      break;

    default:
      val->rst_mode = ISM6HG256X_SW_RST_DYN_ADDRESS_RST;
      break;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_CTRL5, (uint8_t *)&ctrl5, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ctrl5.bus_act_sel)
  {
    case 0x00:
      val->ibi_time = ISM6HG256X_IBI_50us;
      break;

    case 0x01:
      val->ibi_time = ISM6HG256X_IBI_2us;
      break;

    case 0x02:
      val->ibi_time = ISM6HG256X_IBI_1ms;
      break;

    case 0x03:
      val->ibi_time = ISM6HG256X_IBI_50ms;
      break;

    default:
      val->ibi_time = ISM6HG256X_IBI_50us;
      break;
  }

  val->if2_ta0_pid = ctrl5.if2_ta0_pid;

  return ret;
}


int32_t ism6hg256x_sh_controller_interface_pull_up_set(const stmdev_ctx_t *ctx,
                                                       uint8_t val)
{
  ism6hg256x_if_cfg_t if_cfg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF_CFG, (uint8_t *)&if_cfg, 1);
  if (ret == 0)
  {
    if_cfg.shub_pu_en = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_IF_CFG, (uint8_t *)&if_cfg, 1);
  }

  return ret;
}


int32_t ism6hg256x_sh_controller_interface_pull_up_get(const stmdev_ctx_t *ctx,
                                                       uint8_t *val)
{
  ism6hg256x_if_cfg_t if_cfg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF_CFG, (uint8_t *)&if_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = if_cfg.shub_pu_en;

  return ret;
}


int32_t ism6hg256x_sh_read_data_raw_get(const stmdev_ctx_t *ctx, uint8_t *val,
                                        uint8_t len)
{
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_SENSOR_HUB_1, val, len);
exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sh_target_connected_set(const stmdev_ctx_t *ctx,
                                           ism6hg256x_sh_target_connected_t val)
{
  ism6hg256x_controller_config_t controller_config = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);
  if (ret != 0)
  {
    goto exit;
  }
  controller_config.aux_sens_on = (uint8_t)val & 0x3U;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sh_target_connected_get(const stmdev_ctx_t *ctx,
                                           ism6hg256x_sh_target_connected_t *val)
{
  ism6hg256x_controller_config_t controller_config = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  switch (controller_config.aux_sens_on)
  {
    case 0x00:
      *val = ISM6HG256X_TGT_0;
      break;

    case 0x01:
      *val = ISM6HG256X_TGT_0_1;
      break;

    case 0x02:
      *val = ISM6HG256X_TGT_0_1_2;
      break;

    case 0x03:
      *val = ISM6HG256X_TGT_0_1_2_3;
      break;

    default:
      *val = ISM6HG256X_TGT_0;
      break;
  }

  return ret;
}


int32_t ism6hg256x_sh_controller_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_controller_config_t controller_config = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);
  if (ret != 0)
  {
    goto exit;
  }

  controller_config.controller_on = val & 0x01U;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sh_controller_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_controller_config_t controller_config = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);
  if (ret != 0)
  {
    goto exit;
  }

  *val = controller_config.controller_on;

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sh_pass_through_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_controller_config_t controller_config = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);
  if (ret != 0)
  {
    goto exit;
  }

  controller_config.pass_through_mode = val & 0x01U;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sh_pass_through_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_controller_config_t controller_config = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);

  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);
  if (ret != 0)
  {
    goto exit;
  }

  *val = controller_config.pass_through_mode;

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sh_syncro_mode_set(const stmdev_ctx_t *ctx,
                                      ism6hg256x_sh_syncro_mode_t val)
{
  ism6hg256x_controller_config_t controller_config = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);

  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);
  if (ret != 0)
  {
    goto exit;
  }

  controller_config.start_config = (uint8_t)val & 0x01U;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sh_syncro_mode_get(const stmdev_ctx_t *ctx,
                                      ism6hg256x_sh_syncro_mode_t *val)
{
  ism6hg256x_controller_config_t controller_config = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  switch (controller_config.start_config)
  {
    case 0x00:
      *val = ISM6HG256X_SH_TRG_XL_GY_DRDY;
      break;

    case 0x01:
      *val = ISM6HG256X_SH_TRIG_INT2;
      break;

    default:
      *val = ISM6HG256X_SH_TRG_XL_GY_DRDY;
      break;
  }

  return ret;
}


int32_t ism6hg256x_sh_write_mode_set(const stmdev_ctx_t *ctx,
                                     ism6hg256x_sh_write_mode_t val)
{
  ism6hg256x_controller_config_t controller_config = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);
  if (ret != 0)
  {
    goto exit;
  }

  controller_config.write_once = (uint8_t)val & 0x01U;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sh_write_mode_get(const stmdev_ctx_t *ctx,
                                     ism6hg256x_sh_write_mode_t *val)
{
  ism6hg256x_controller_config_t controller_config = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  switch (controller_config.write_once)
  {
    case 0x00:
      *val = ISM6HG256X_EACH_SH_CYCLE;
      break;

    case 0x01:
      *val = ISM6HG256X_ONLY_FIRST_CYCLE;
      break;

    default:
      *val = ISM6HG256X_EACH_SH_CYCLE;
      break;
  }

  return ret;
}


int32_t ism6hg256x_sh_reset_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_controller_config_t controller_config = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);
  if (ret != 0)
  {
    goto exit;
  }

  controller_config.rst_controller_regs = val & 0x01U;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sh_reset_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_controller_config_t controller_config = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_CONTROLLER_CONFIG, (uint8_t *)&controller_config, 1);
  if (ret != 0)
  {
    goto exit;
  }

  *val = controller_config.rst_controller_regs;

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sh_cfg_write(const stmdev_ctx_t *ctx,
                                ism6hg256x_sh_cfg_write_t *val)
{
  ism6hg256x_tgt0_add_t reg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  reg.target0_add = (val->tgt0_add >> 1) & 0x07U;
  reg.rw_0 = 0;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_TGT0_ADD, (uint8_t *)&reg, 1);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_TGT0_SUBADD,
                             &(val->tgt0_subadd), 1);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_DATAWRITE_TGT0,
                             &(val->tgt0_data), 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sh_data_rate_set(const stmdev_ctx_t *ctx,
                                    ism6hg256x_sh_data_rate_t val)
{
  ism6hg256x_tgt0_config_t tgt0_config = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_TGT0_CONFIG, (uint8_t *)&tgt0_config, 1);
  if (ret != 0)
  {
    goto exit;
  }

  tgt0_config.shub_odr = (uint8_t)val & 0x07U;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_TGT0_CONFIG, (uint8_t *)&tgt0_config, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sh_data_rate_get(const stmdev_ctx_t *ctx,
                                    ism6hg256x_sh_data_rate_t *val)
{
  ism6hg256x_tgt0_config_t tgt0_config = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_TGT0_CONFIG, (uint8_t *)&tgt0_config, 1);
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  switch (tgt0_config.shub_odr)
  {
    case 0x00:
      *val = ISM6HG256X_SH_1Hz875;
      break;
    case 0x01:
      *val = ISM6HG256X_SH_15Hz;
      break;

    case 0x02:
      *val = ISM6HG256X_SH_30Hz;
      break;

    case 0x03:
      *val = ISM6HG256X_SH_60Hz;
      break;

    case 0x04:
      *val = ISM6HG256X_SH_120Hz;
      break;

    case 0x05:
      *val = ISM6HG256X_SH_240Hz;
      break;

    case 0x06:
      *val = ISM6HG256X_SH_480Hz;
      break;

    default:
      *val = ISM6HG256X_SH_15Hz;
      break;
  }

  return ret;
}


int32_t ism6hg256x_sh_tgt_cfg_read(const stmdev_ctx_t *ctx, uint8_t idx,
                                   ism6hg256x_sh_cfg_read_t *val)
{
  ism6hg256x_tgt0_add_t tgt_add = {0};
  ism6hg256x_tgt0_config_t tgt_config = {0};
  int32_t ret = 0;

  if (idx > 3)
  {
    return -1;
  }

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_SENSOR_HUB_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  tgt_add.target0_add = (val->tgt_add >> 1) & 0x07U;
  tgt_add.rw_0 = 1;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_TGT0_ADD + (uint8_t)(idx * 3U),
                             (uint8_t *)&tgt_add, 1);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_TGT0_SUBADD + (uint8_t)(idx * 3U),
                             &(val->tgt_subadd), 1);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TGT0_CONFIG + (uint8_t)(idx * 3U),
                            (uint8_t *)&tgt_config, 1);
  if (ret != 0)
  {
    goto exit;
  }

  tgt_config.target0_numop = val->tgt_len & 0x07U;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_TGT0_CONFIG + (uint8_t)(idx * 3U),
                             (uint8_t *)&tgt_config, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sh_status_get(const stmdev_ctx_t *ctx,
                                 ism6hg256x_status_controller_t *val)
{
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_STATUS_CONTROLLER_MAINPAGE, (uint8_t *) val, 1);

  return ret;
}


int32_t ism6hg256x_ui_sdo_pull_up_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_pin_ctrl_t pin_ctrl = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);
  if (ret == 0)
  {
    pin_ctrl.sdo_pu_en = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);
  }

  return ret;
}


int32_t ism6hg256x_ui_sdo_pull_up_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_pin_ctrl_t pin_ctrl = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = pin_ctrl.sdo_pu_en;

  return ret;
}

int32_t ism6hg256x_pad_strength_set(const stmdev_ctx_t *ctx, ism6hg256x_pad_strength_t val)
{
  ism6hg256x_pin_ctrl_t pin_ctrl = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);
  if (ret == 0)
  {
    pin_ctrl.io_pad_strength = val & 0x03U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);
  }

  return ret;
}


int32_t ism6hg256x_pad_strength_get(const stmdev_ctx_t *ctx, ism6hg256x_pad_strength_t *val)
{
  ism6hg256x_pin_ctrl_t pin_ctrl = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_PIN_CTRL, (uint8_t *)&pin_ctrl, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (pin_ctrl.io_pad_strength)
  {
    case 0x00:
      *val = ISM6HG256X_PAD_LOW_STRENGTH;
      break;

    case 0x01:
      *val = ISM6HG256X_PAD_MIDDLE_STRENGTH;
      break;

    case 0x02:
    default:
      *val = ISM6HG256X_PAD_HIGH_STRENGTH;
      break;
  }

  return ret;
}


int32_t ism6hg256x_ui_i2c_i3c_mode_set(const stmdev_ctx_t *ctx,
                                       ism6hg256x_ui_i2c_i3c_mode_t val)
{
  ism6hg256x_if_cfg_t if_cfg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF_CFG, (uint8_t *)&if_cfg, 1);
  if (ret == 0)
  {
    if_cfg.i2c_i3c_disable = (uint8_t)val & 0x1U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_IF_CFG, (uint8_t *)&if_cfg, 1);
  }

  return ret;
}


int32_t ism6hg256x_ui_i2c_i3c_mode_get(const stmdev_ctx_t *ctx,
                                       ism6hg256x_ui_i2c_i3c_mode_t *val)
{
  ism6hg256x_if_cfg_t if_cfg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF_CFG, (uint8_t *)&if_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (if_cfg.i2c_i3c_disable)
  {
    case 0x00:
      *val = ISM6HG256X_I2C_I3C_ENABLE;
      break;

    case 0x01:
      *val = ISM6HG256X_I2C_I3C_DISABLE;
      break;

    default:
      *val = ISM6HG256X_I2C_I3C_ENABLE;
      break;
  }

  return ret;
}


int32_t ism6hg256x_spi_mode_set(const stmdev_ctx_t *ctx, ism6hg256x_spi_mode_t val)
{
  ism6hg256x_if_cfg_t if_cfg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF_CFG, (uint8_t *)&if_cfg, 1);
  if (ret == 0)
  {
    if_cfg.sim = (uint8_t)val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_IF_CFG, (uint8_t *)&if_cfg, 1);
  }

  return ret;
}


int32_t ism6hg256x_spi_mode_get(const stmdev_ctx_t *ctx, ism6hg256x_spi_mode_t *val)
{
  ism6hg256x_if_cfg_t if_cfg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF_CFG, (uint8_t *)&if_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (if_cfg.sim)
  {
    case 0x00:
      *val = ISM6HG256X_SPI_4_WIRE;
      break;

    case 0x01:
      *val = ISM6HG256X_SPI_3_WIRE;
      break;

    default:
      *val = ISM6HG256X_SPI_4_WIRE;
      break;
  }

  return ret;
}


int32_t ism6hg256x_ui_sda_pull_up_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_if_cfg_t if_cfg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF_CFG, (uint8_t *)&if_cfg, 1);
  if (ret == 0)
  {
    if_cfg.sda_pu_en = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_IF_CFG, (uint8_t *)&if_cfg, 1);
  }

  return ret;
}


int32_t ism6hg256x_ui_sda_pull_up_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_if_cfg_t if_cfg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_IF_CFG, (uint8_t *)&if_cfg, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = if_cfg.sda_pu_en;

  return ret;
}

int32_t ism6hg256x_if2_spi_mode_set(const stmdev_ctx_t *ctx, ism6hg256x_spi_mode_t val)
{
  ism6hg256x_ui_ctrl1_ois_t ui_ctrl1_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_CTRL1_OIS, (uint8_t *)&ui_ctrl1_ois, 1);
  if (ret == 0)
  {
    ui_ctrl1_ois.sim_ois = (uint8_t)val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_UI_CTRL1_OIS, (uint8_t *)&ui_ctrl1_ois, 1);
  }

  return ret;
}


int32_t ism6hg256x_if2_spi_mode_get(const stmdev_ctx_t *ctx, ism6hg256x_spi_mode_t *val)
{
  ism6hg256x_ui_ctrl1_ois_t ui_ctrl1_ois = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_UI_CTRL1_OIS, (uint8_t *)&ui_ctrl1_ois, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (ui_ctrl1_ois.sim_ois)
  {
    case 0x00:
      *val = ISM6HG256X_SPI_4_WIRE;
      break;

    case 0x01:
      *val = ISM6HG256X_SPI_3_WIRE;
      break;

    default:
      *val = ISM6HG256X_SPI_4_WIRE;
      break;
  }

  return ret;
}


int32_t ism6hg256x_sigmot_mode_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_emb_func_en_a_t emb_func_en_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);
  if (ret != 0)
  {
    goto exit;
  }
  emb_func_en_a.sign_motion_en = val & 0x01U;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sigmot_mode_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_emb_func_en_a_t emb_func_en_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);
  if (ret != 0)
  {
    goto exit;
  }
  *val = emb_func_en_a.sign_motion_en;

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_stpcnt_mode_set(const stmdev_ctx_t *ctx,
                                   ism6hg256x_stpcnt_mode_t val)
{
  ism6hg256x_emb_func_en_a_t emb_func_en_a = {0};
  ism6hg256x_emb_func_en_b_t emb_func_en_b = {0};
  ism6hg256x_pedo_cmd_reg_t pedo_cmd_reg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_EN_B, (uint8_t *)&emb_func_en_b, 1);
  if (ret != 0)
  {
    goto exit;
  }

  if ((val.false_step_rej == PROPERTY_ENABLE)
      && ((emb_func_en_a.mlc_before_fsm_en & emb_func_en_b.mlc_en) ==
          PROPERTY_DISABLE))
  {
    emb_func_en_a.mlc_before_fsm_en = PROPERTY_ENABLE;
  }

  emb_func_en_a.pedo_en = val.step_counter_enable;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  if (ret == 0)
  {
    ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_PEDO_CMD_REG,
                                (uint8_t *)&pedo_cmd_reg, 1);
    if (ret != 0)
    {
      return ret;
    }
    pedo_cmd_reg.fp_rejection_en = val.false_step_rej;
    ret += ism6hg256x_ln_pg_write(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_PEDO_CMD_REG,
                                  (uint8_t *)&pedo_cmd_reg, 1);
  }

  return ret;
}


int32_t ism6hg256x_stpcnt_mode_get(const stmdev_ctx_t *ctx,
                                   ism6hg256x_stpcnt_mode_t *val)
{
  ism6hg256x_emb_func_en_a_t emb_func_en_a = {0};
  ism6hg256x_pedo_cmd_reg_t pedo_cmd_reg = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_PEDO_CMD_REG,
                              (uint8_t *)&pedo_cmd_reg, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->false_step_rej = pedo_cmd_reg.fp_rejection_en;
  val->step_counter_enable = emb_func_en_a.pedo_en;

  return ret;
}


int32_t ism6hg256x_stpcnt_steps_get(const stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_STEP_COUNTER_L, &buff[0], 2);
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  *val = (uint16_t)(buff[0] | ((uint16_t)buff[1] << 8));

  return ret;
}


int32_t ism6hg256x_stpcnt_rst_step_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_emb_func_src_t emb_func_src = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_SRC, (uint8_t *)&emb_func_src, 1);
  if (ret != 0)
  {
    goto exit;
  }

  emb_func_src.pedo_rst_step = val & 0x01U;
  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_SRC, (uint8_t *)&emb_func_src, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_stpcnt_rst_step_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_emb_func_src_t emb_func_src = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_SRC, (uint8_t *)&emb_func_src, 1);
  if (ret != 0)
  {
    goto exit;
  }
  *val = emb_func_src.pedo_rst_step;

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_stpcnt_debounce_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_pedo_deb_steps_conf_t pedo_deb_steps_conf = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_PEDO_DEB_STEPS_CONF,
                              (uint8_t *)&pedo_deb_steps_conf, 1);
  if (ret == 0)
  {
    pedo_deb_steps_conf.deb_step = val;
    ret = ism6hg256x_ln_pg_write(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_PEDO_DEB_STEPS_CONF,
                                 (uint8_t *)&pedo_deb_steps_conf, 1);
  }

  return ret;
}


int32_t ism6hg256x_stpcnt_debounce_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_pedo_deb_steps_conf_t pedo_deb_steps_conf = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_PEDO_DEB_STEPS_CONF,
                              (uint8_t *)&pedo_deb_steps_conf, 1);
  if (ret != 0)
  {
    return ret;
  }
  *val = pedo_deb_steps_conf.deb_step;

  return ret;
}


int32_t ism6hg256x_stpcnt_period_set(const stmdev_ctx_t *ctx, uint16_t val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  buff[1] = (uint8_t)(val / 256U);
  buff[0] = (uint8_t)(val - (buff[1] * 256U));
  ret = ism6hg256x_ln_pg_write(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_PEDO_SC_DELTAT_L,
                               (uint8_t *)&buff[0], 2);

  return ret;
}


int32_t ism6hg256x_stpcnt_period_get(const stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t buff[2] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_ln_pg_read(ctx, ISM6HG256X_EMB_ADV_PG_1 + ISM6HG256X_PEDO_SC_DELTAT_L, &buff[0],
                              2);
  if (ret != 0)
  {
    return ret;
  }

  *val = (uint16_t)(buff[0] | ((uint16_t)buff[1] << 8));

  return ret;
}


int32_t ism6hg256x_sflp_game_rotation_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_emb_func_en_a_t emb_func_en_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);
  if (ret != 0)
  {
    goto exit;
  }

  emb_func_en_a.sflp_game_en = val & 0x01U;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_EN_A,
                              (uint8_t *)&emb_func_en_a, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sflp_game_rotation_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_emb_func_en_a_t emb_func_en_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);
  if (ret != 0)
  {
    goto exit;
  }
  *val = emb_func_en_a.sflp_game_en;

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sflp_game_rotation_reset(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_emb_func_init_a_t emb_func_init_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_INIT_A, (uint8_t *)&emb_func_init_a, 1);
  if (ret != 0)
  {
    goto exit;
  }
  emb_func_init_a.sflp_game_init = val & 0x01U;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_INIT_A, (uint8_t *)&emb_func_init_a, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sflp_data_rate_set(const stmdev_ctx_t *ctx,
                                      ism6hg256x_sflp_data_rate_t val)
{
  ism6hg256x_sflp_odr_t sflp_odr = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    goto exit;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_SFLP_ODR, (uint8_t *)&sflp_odr, 1);
  if (ret != 0)
  {
    goto exit;
  }

  sflp_odr.sflp_game_odr = (uint8_t)val & 0x07U;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_SFLP_ODR, (uint8_t *)&sflp_odr, 1);

exit:
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_sflp_data_rate_get(const stmdev_ctx_t *ctx,
                                      ism6hg256x_sflp_data_rate_t *val)
{
  ism6hg256x_sflp_odr_t sflp_odr = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_SFLP_ODR, (uint8_t *)&sflp_odr, 1);
  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  switch (sflp_odr.sflp_game_odr)
  {
    case 0x00:
      *val = ISM6HG256X_SFLP_15Hz;
      break;

    case 0x01:
      *val = ISM6HG256X_SFLP_30Hz;
      break;

    case 0x02:
      *val = ISM6HG256X_SFLP_60Hz;
      break;

    case 0x03:
      *val = ISM6HG256X_SFLP_120Hz;
      break;

    case 0x04:
      *val = ISM6HG256X_SFLP_240Hz;
      break;

    case 0x05:
      *val = ISM6HG256X_SFLP_480Hz;
      break;

    default:
      *val = ISM6HG256X_SFLP_15Hz;
      break;
  }

  return ret;
}


int32_t ism6hg256x_tap_detection_set(const stmdev_ctx_t *ctx,
                                     ism6hg256x_tap_detection_t val)
{
  ism6hg256x_tap_cfg0_t tap_cfg0 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1);
  if (ret == 0)
  {
    tap_cfg0.tap_x_en = val.tap_x_en;
    tap_cfg0.tap_y_en = val.tap_y_en;
    tap_cfg0.tap_z_en = val.tap_z_en;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1);
  }

  return ret;
}


int32_t ism6hg256x_tap_detection_get(const stmdev_ctx_t *ctx,
                                     ism6hg256x_tap_detection_t *val)
{
  ism6hg256x_tap_cfg0_t tap_cfg0 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->tap_x_en = tap_cfg0.tap_x_en;
  val->tap_y_en = tap_cfg0.tap_y_en;
  val->tap_z_en = tap_cfg0.tap_z_en;

  return ret;
}


int32_t ism6hg256x_tap_thresholds_set(const stmdev_ctx_t *ctx,
                                      ism6hg256x_tap_thresholds_t val)
{
  ism6hg256x_tap_ths_6d_t tap_ths_6d = {0};
  ism6hg256x_tap_cfg2_t tap_cfg2 = {0};
  ism6hg256x_tap_cfg1_t tap_cfg1 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_CFG1, (uint8_t *)&tap_cfg1, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_CFG2, (uint8_t *)&tap_cfg2, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1);
  if (ret != 0)
  {
    return ret;
  }

  tap_cfg1.tap_ths_x = val.x;
  tap_cfg2.tap_ths_y = val.y;
  tap_ths_6d.tap_ths_z = val.z;

  ret = ism6hg256x_write_reg(ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_TAP_CFG2, (uint8_t *)&tap_cfg2, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_TAP_CFG1, (uint8_t *)&tap_cfg1, 1);

  return ret;
}


int32_t ism6hg256x_tap_thresholds_get(const stmdev_ctx_t *ctx,
                                      ism6hg256x_tap_thresholds_t *val)
{
  ism6hg256x_tap_ths_6d_t tap_ths_6d = {0};
  ism6hg256x_tap_cfg2_t tap_cfg2 = {0};
  ism6hg256x_tap_cfg1_t tap_cfg1 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_CFG1, (uint8_t *)&tap_cfg1, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_CFG2, (uint8_t *)&tap_cfg2, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_THS_6D, (uint8_t *)&tap_ths_6d, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->x  = tap_cfg1.tap_ths_x;
  val->y = tap_cfg2.tap_ths_y;
  val->z = tap_ths_6d.tap_ths_z;

  return ret;
}


int32_t ism6hg256x_tap_axis_priority_set(const stmdev_ctx_t *ctx,
                                         ism6hg256x_tap_axis_priority_t val)
{
  ism6hg256x_tap_cfg1_t tap_cfg1 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_CFG1, (uint8_t *)&tap_cfg1, 1);
  if (ret == 0)
  {
    tap_cfg1.tap_priority = (uint8_t)val & 0x7U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_TAP_CFG1, (uint8_t *)&tap_cfg1, 1);
  }

  return ret;
}


int32_t ism6hg256x_tap_axis_priority_get(const stmdev_ctx_t *ctx,
                                         ism6hg256x_tap_axis_priority_t *val)
{
  ism6hg256x_tap_cfg1_t tap_cfg1 = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_CFG1, (uint8_t *)&tap_cfg1, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (tap_cfg1.tap_priority)
  {
    case 0x00 :
      *val = ISM6HG256X_XYZ ;
      break;

    case 0x01 :
      *val = ISM6HG256X_YXZ ;
      break;

    case 0x02:
      *val = ISM6HG256X_XZY;
      break;

    case 0x03 :
      *val = ISM6HG256X_ZYX ;
      break;

    case 0x05 :
      *val = ISM6HG256X_YZX ;
      break;

    case 0x06 :
      *val = ISM6HG256X_ZXY ;
      break;

    default:
      *val = ISM6HG256X_XYZ ;
      break;
  }

  return ret;
}


int32_t ism6hg256x_tap_time_windows_set(const stmdev_ctx_t *ctx,
                                        ism6hg256x_tap_time_windows_t val)
{
  ism6hg256x_tap_dur_t tap_dur = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1);
  if (ret == 0)
  {
    tap_dur.shock = val.shock;
    tap_dur.quiet = val.quiet;
    tap_dur.dur = val.tap_gap;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1);
  }

  return ret;
}

int32_t ism6hg256x_tap_time_windows_get(const stmdev_ctx_t *ctx,
                                        ism6hg256x_tap_time_windows_t *val)
{
  ism6hg256x_tap_dur_t tap_dur = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TAP_DUR, (uint8_t *)&tap_dur, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->shock = tap_dur.shock;
  val->quiet = tap_dur.quiet;
  val->tap_gap = tap_dur.dur;

  return ret;
}


int32_t ism6hg256x_tap_mode_set(const stmdev_ctx_t *ctx, ism6hg256x_tap_mode_t val)
{
  ism6hg256x_wake_up_ths_t wake_up_ths = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_WAKE_UP_THS, (uint8_t *)&wake_up_ths, 1);
  if (ret == 0)
  {
    wake_up_ths.single_double_tap = (uint8_t)val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_WAKE_UP_THS, (uint8_t *)&wake_up_ths, 1);
  }

  return ret;
}


int32_t ism6hg256x_tap_mode_get(const stmdev_ctx_t *ctx, ism6hg256x_tap_mode_t *val)
{
  ism6hg256x_wake_up_ths_t wake_up_ths = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_WAKE_UP_THS, (uint8_t *)&wake_up_ths, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (wake_up_ths.single_double_tap)
  {
    case 0x00:
      *val = ISM6HG256X_ONLY_SINGLE;
      break;

    case 0x01:
      *val = ISM6HG256X_BOTH_SINGLE_DOUBLE;
      break;

    default:
      *val = ISM6HG256X_ONLY_SINGLE;
      break;
  }

  return ret;
}


int32_t ism6hg256x_tilt_mode_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_emb_func_en_a_t emb_func_en_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);
  emb_func_en_a.tilt_en = val & 0x01U;
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);

  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_tilt_mode_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_emb_func_en_a_t emb_func_en_a = {0};
  int32_t ret = 0;

  ret = ism6hg256x_mem_bank_set(ctx, ISM6HG256X_EMBED_FUNC_MEM_BANK);
  if (ret != 0)
  {
    return ret;
  }

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_EMB_FUNC_EN_A, (uint8_t *)&emb_func_en_a, 1);
  *val = emb_func_en_a.tilt_en;

  ret += ism6hg256x_mem_bank_set(ctx, ISM6HG256X_MAIN_MEM_BANK);

  return ret;
}


int32_t ism6hg256x_timestamp_raw_get(const stmdev_ctx_t *ctx, uint32_t *val)
{
  uint8_t buff[4] = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_TIMESTAMP0, &buff[0], 4);
  if (ret != 0)
  {
    return ret;
  }

  *val = (uint32_t)buff[3] << 8;
  *val = (*val | (uint32_t)buff[2]) << 8;
  *val = (*val | (uint32_t)buff[1]) << 8;
  *val = (*val | (uint32_t)buff[0]) << 8;

  return ret;
}


int32_t ism6hg256x_timestamp_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  ism6hg256x_functions_enable_t functions_enable = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1);
  if (ret == 0)
  {
    functions_enable.timestamp_en = val & 0x01U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1);
  }

  return ret;
}


int32_t ism6hg256x_timestamp_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  ism6hg256x_functions_enable_t functions_enable = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1);
  if (ret != 0)
  {
    return ret;
  }

  *val = functions_enable.timestamp_en;

  return ret;
}


int32_t ism6hg256x_act_mode_set(const stmdev_ctx_t *ctx, ism6hg256x_act_mode_t val)
{
  ism6hg256x_functions_enable_t functions_enable = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1);
  if (ret == 0)
  {
    functions_enable.inact_en = (uint8_t)val & 0x3U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1);
  }

  return ret;
}


int32_t ism6hg256x_act_mode_get(const stmdev_ctx_t *ctx, ism6hg256x_act_mode_t *val)
{
  ism6hg256x_functions_enable_t functions_enable = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_FUNCTIONS_ENABLE, (uint8_t *)&functions_enable, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (functions_enable.inact_en)
  {
    case 0x00:
      *val = ISM6HG256X_XL_AND_GY_NOT_AFFECTED;
      break;

    case 0x01:
      *val = ISM6HG256X_XL_LOW_POWER_GY_NOT_AFFECTED;
      break;

    case 0x02:
      *val = ISM6HG256X_XL_LOW_POWER_GY_SLEEP;
      break;

    case 0x03:
      *val = ISM6HG256X_XL_LOW_POWER_GY_POWER_DOWN;
      break;

    default:
      *val = ISM6HG256X_XL_AND_GY_NOT_AFFECTED;
      break;
  }

  return ret;
}


int32_t ism6hg256x_act_from_sleep_to_act_dur_set(const stmdev_ctx_t *ctx,
                                                 ism6hg256x_act_from_sleep_to_act_dur_t val)
{
  ism6hg256x_inactivity_dur_t inactivity_dur = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_INACTIVITY_DUR, (uint8_t *)&inactivity_dur, 1);
  if (ret == 0)
  {
    inactivity_dur.inact_dur = (uint8_t)val & 0x3U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_INACTIVITY_DUR, (uint8_t *)&inactivity_dur, 1);
  }

  return ret;
}


int32_t ism6hg256x_act_from_sleep_to_act_dur_get(const stmdev_ctx_t *ctx,
                                                 ism6hg256x_act_from_sleep_to_act_dur_t *val)
{
  ism6hg256x_inactivity_dur_t inactivity_dur = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_INACTIVITY_DUR, (uint8_t *)&inactivity_dur, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (inactivity_dur.inact_dur)
  {
    case 0x00:
      *val = ISM6HG256X_SLEEP_TO_ACT_AT_1ST_SAMPLE;
      break;

    case 0x01:
      *val = ISM6HG256X_SLEEP_TO_ACT_AT_2ND_SAMPLE;
      break;

    case 0x02:
      *val = ISM6HG256X_SLEEP_TO_ACT_AT_3RD_SAMPLE;
      break;

    case 0x03:
      *val = ISM6HG256X_SLEEP_TO_ACT_AT_4th_SAMPLE;
      break;

    default:
      *val = ISM6HG256X_SLEEP_TO_ACT_AT_1ST_SAMPLE;
      break;
  }

  return ret;
}


int32_t ism6hg256x_act_sleep_xl_odr_set(const stmdev_ctx_t *ctx,
                                        ism6hg256x_act_sleep_xl_odr_t val)
{
  ism6hg256x_inactivity_dur_t inactivity_dur = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_INACTIVITY_DUR, (uint8_t *)&inactivity_dur, 1);
  if (ret == 0)
  {
    inactivity_dur.xl_inact_odr = (uint8_t)val & 0x03U;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_INACTIVITY_DUR, (uint8_t *)&inactivity_dur, 1);
  }

  return ret;
}


int32_t ism6hg256x_act_sleep_xl_odr_get(const stmdev_ctx_t *ctx,
                                        ism6hg256x_act_sleep_xl_odr_t *val)
{
  ism6hg256x_inactivity_dur_t inactivity_dur = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_INACTIVITY_DUR, (uint8_t *)&inactivity_dur, 1);
  if (ret != 0)
  {
    return ret;
  }

  switch (inactivity_dur.xl_inact_odr)
  {
    case 0x00:
      *val = ISM6HG256X_1Hz875;
      break;

    case 0x01:
      *val = ISM6HG256X_15Hz;
      break;

    case 0x02:
      *val = ISM6HG256X_30Hz;
      break;

    case 0x03:
      *val = ISM6HG256X_60Hz;
      break;

    default:
      *val = ISM6HG256X_1Hz875;
      break;
  }

  return ret;
}


int32_t ism6hg256x_act_thresholds_set(const stmdev_ctx_t *ctx,
                                      const ism6hg256x_act_thresholds_t *val)
{
  ism6hg256x_inactivity_ths_t inactivity_ths = {0};
  ism6hg256x_inactivity_dur_t inactivity_dur = {0};
  ism6hg256x_wake_up_ths_t wake_up_ths = {0};
  ism6hg256x_wake_up_dur_t wake_up_dur = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_INACTIVITY_DUR, (uint8_t *)&inactivity_dur, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_INACTIVITY_THS, (uint8_t *)&inactivity_ths, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_WAKE_UP_THS, (uint8_t *)&wake_up_ths, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
  if (ret != 0)
  {
    return ret;
  }

  inactivity_dur.wu_inact_ths_w = val->inactivity_cfg.wu_inact_ths_w & 0x07U;
  inactivity_dur.xl_inact_odr = val->inactivity_cfg.xl_inact_odr & 0x03U;
  inactivity_dur.inact_dur = val->inactivity_cfg.inact_dur & 0x03U;

  inactivity_ths.inact_ths = val->inactivity_ths & 0x3FU;
  wake_up_ths.wk_ths = val->threshold & 0x3FU;
  wake_up_dur.wake_dur = val->duration & 0x03U;

  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_INACTIVITY_DUR, (uint8_t *)&inactivity_dur, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_INACTIVITY_THS, (uint8_t *)&inactivity_ths, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_WAKE_UP_THS, (uint8_t *)&wake_up_ths, 1);
  ret += ism6hg256x_write_reg(ctx, ISM6HG256X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);

  return ret;
}


int32_t ism6hg256x_act_thresholds_get(const stmdev_ctx_t *ctx,
                                      ism6hg256x_act_thresholds_t *val)
{
  ism6hg256x_inactivity_dur_t inactivity_dur = {0};
  ism6hg256x_inactivity_ths_t inactivity_ths = {0};
  ism6hg256x_wake_up_ths_t wake_up_ths = {0};
  ism6hg256x_wake_up_dur_t wake_up_dur = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_INACTIVITY_DUR, (uint8_t *)&inactivity_dur, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_INACTIVITY_THS, (uint8_t *)&inactivity_ths, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_WAKE_UP_THS, (uint8_t *)&wake_up_ths, 1);
  ret += ism6hg256x_read_reg(ctx, ISM6HG256X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->inactivity_cfg.wu_inact_ths_w = inactivity_dur.wu_inact_ths_w;
  val->inactivity_cfg.xl_inact_odr = inactivity_dur.xl_inact_odr;
  val->inactivity_cfg.inact_dur = inactivity_dur.inact_dur;

  val->inactivity_ths = inactivity_ths.inact_ths;
  val->threshold = wake_up_ths.wk_ths;
  val->duration = wake_up_dur.wake_dur;

  return ret;
}


int32_t ism6hg256x_act_wkup_time_windows_set(const stmdev_ctx_t *ctx,
                                             ism6hg256x_act_wkup_time_windows_t val)
{
  ism6hg256x_wake_up_dur_t wake_up_dur = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
  if (ret == 0)
  {
    wake_up_dur.wake_dur = val.shock;
    wake_up_dur.sleep_dur = val.quiet;
    ret = ism6hg256x_write_reg(ctx, ISM6HG256X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
  }

  return ret;
}

/**
  * @brief  Time windows configuration for Wake Up - Activity - Inactivity (SLEEP, WAKE). Duration to go in sleep mode. Default value: 0000 (this corresponds to 16 ODR) 1 LSB = 512/ODR_XL time. Wake up duration event. 1 LSB = 1/ODR_XL time. [get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Time windows configuration for Wake Up - Activity - Inactivity (SLEEP, WAKE). Duration to go in sleep mode. Default value: 0000 (this corresponds to 16 ODR) 1 LSB = 512/ODR_XL time. Wake up duration event. 1 LSB = 1/ODR_XL time.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t ism6hg256x_act_wkup_time_windows_get(const stmdev_ctx_t *ctx,
                                             ism6hg256x_act_wkup_time_windows_t *val)
{
  ism6hg256x_wake_up_dur_t wake_up_dur = {0};
  int32_t ret = 0;

  ret = ism6hg256x_read_reg(ctx, ISM6HG256X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
  if (ret != 0)
  {
    return ret;
  }

  val->shock = wake_up_dur.wake_dur;
  val->quiet = wake_up_dur.sleep_dur;

  return ret;
}
