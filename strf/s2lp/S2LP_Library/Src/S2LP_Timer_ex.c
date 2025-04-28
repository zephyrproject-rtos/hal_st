/**
 * @file    S2LP_Timer_ex.c
 * @author  LowPower RF BU - AMG
 * @version 1.2.0
 * @date    October 31, 2016
 * @brief   This file provides functions to manage S2-LP debug.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors 
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "S2LP_Timer.h"
#include "S2LP_Timer_ex.h"
#include "S2LP_Regs.h"
#include "MCU_Interface.h"


/** @addtogroup S2LP_Libraries
 * @{
 */


/** @addtogroup S2LP_Types
 * @{
 */

/** @defgroup Types_Private_Functions                 Types Private Functions
 * @{
 */


void S2LPTimerSetRxTimerMs(float fDesiredMsec)
{
  S2LPTimerSetRxTimerUs((uint32_t)(fDesiredMsec*1000));
}

void S2LPTimerGetRxTimer(float* pfTimeoutMsec, uint8_t* pcCounter , uint8_t* pcPrescaler)
{
  uint32_t timeoutUsec;
  
  S2LPTimerGetRxTimerUs(&timeoutUsec, pcCounter , pcPrescaler);
    
  (*pfTimeoutMsec)=((float)timeoutUsec)/1000;
}

void S2LPTimerSetWakeUpTimerMs(float fDesiredMsec)
{
  S2LPTimerSetWakeUpTimerUs((uint32_t)(fDesiredMsec*1000));
}

void S2LPTimerSetWakeUpTimerReloadMs(float fDesiredMsec)
{
  S2LPTimerSetWakeUpTimerReloadUs((uint32_t)(fDesiredMsec*1000));
}

void S2LPTimerGetWakeUpTimer(float* pfWakeUpMsec, uint8_t* pcCounter , uint8_t* pcPrescaler, uint8_t* pcMulti)
{
  uint32_t timeoutUsec;
  
  S2LPTimerGetWakeUpTimerUs(&timeoutUsec, pcCounter , pcPrescaler, pcMulti);
    
  (*pfWakeUpMsec)=((float)timeoutUsec)/1000;
}

void S2LPTimerGetWakeUpTimerReload(float* pfWakeUpReloadMsec, uint8_t* pcCounter, uint8_t* pcPrescaler, uint8_t* pcMulti)
{
  uint32_t timeoutUsec;
  
  S2LPTimerGetWakeUpTimerReloadUs(&timeoutUsec, pcCounter , pcPrescaler, pcMulti);
    
  (*pfWakeUpReloadMsec)=((float)timeoutUsec)/1000;
}



/**
 * @}
 */



/**
 * @}
 */



/**
 * @}
 */



/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
