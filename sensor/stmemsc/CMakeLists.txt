# Makefile - STMems_Standard_C_drivers
#
# Copyright (c) 2019 STMicroelectronics
#
# SPDX-License-Identifier: Apache-2.0

set(stmems_pids
  a3g4250d
  ais2dw12
  ais328dq
  ais3624dq
  asm330lhh
  h3lis331dl
  hts221
  i3g4250d
  iis2dh
  iis2dlpc
  iis2mdc
  iis328dq
  iis3dhhc
  iis3dwb
  ism303dac
  ism330dhcx
  ism330dlc
  l20g20is
  l3gd20h
  lis2de12
  lis2dh12
  lis2ds12
  lis2dtw12
  lis2dw12
  lis2hh12
  lis2mdl
  lis331dlh
  lis3de
  lis3dhh
  lis3dh
  lis3mdl
  lps22hb
  lps22hh
  lps25hb
  lps27hhw
  lps33hw
  lps33w
  lsm303agr
  lsm303ah
  lsm6ds3
  lsm6ds3tr_c
  lsm6dsl
  lsm6dsm
  lsm6dso
  lsm6dsox
  lsm6dsr
  lsm9ds1
  stts751
  )

foreach(stmems_pid ${stmems_pids})
  string(TOUPPER ${stmems_pid} pid_to_upper)
  if(CONFIG_USE_STDC_${pid_to_upper})
    zephyr_include_directories(
        ${stmems_pid}_STdC/driver/
        )
    zephyr_sources(
        ${stmems_pid}_STdC/driver/${stmems_pid}_reg.c
        )
  endif()
endforeach()
