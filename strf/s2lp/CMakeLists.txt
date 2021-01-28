# Copyright (c) 2021 Nikos Oikonomou <nikoikonomou92@gmail.com>
#
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(
	  S2LP_Library/Inc
	  )

zephyr_sources(
	S2LP_Library/Src/S2LP_Commands.c
	S2LP_Library/Src/S2LP_Csma.c
	S2LP_Library/Src/S2LP_Fifo.c
	S2LP_Library/Src/S2LP_General.c
	S2LP_Library/Src/S2LP_Gpio.c
	S2LP_Library/Src/S2LP_PacketHandler.c
	S2LP_Library/Src/S2LP_PktBasic.c
	S2LP_Library/Src/S2LP_PktStack.c
	S2LP_Library/Src/S2LP_PktWMbus.c
	S2LP_Library/Src/S2LP_Qi.c
	S2LP_Library/Src/S2LP_Radio.c
	S2LP_Library/Src/S2LP_Timer_ex.c
	S2LP_Library/Src/S2LP_Timer.c
	S2LP_Library/Src/S2LP_Types.c
	)

