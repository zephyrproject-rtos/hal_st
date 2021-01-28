/**
 * @file    S2LP_Timer_ex.h
 * @author  LowPower RF BU - AMG
 * @version 1.3.0
 * @date    10-July-2017
 * @brief   Configuration and management of S2-LP Timers using floating point.
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
 * This module provides API to configure the S2-LP timing mechanisms using floating points.
 *
 * <b>Example:</b>
 * @code
 *   ...
 *
 *   S2LPTimerSetRxTimeoutMs(50.0);
 * 
 *   ...
 *
 * @endcode
 *
 * <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __S2LP_TIMER_EX_H
#define __S2LP_TIMER_EX_H


/* Includes ------------------------------------------------------------------*/

#include "S2LP_Timer_ex.h"


#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @addtogroup S2LP_Libraries
 * @{
 */


/**
 * @defgroup S2LP_Timer_ex               Timer Wrapper
 * @brief Configuration and management of S2LP Timers using floating point.
 * @details See the file <i>@ref S2LP_Timer.h</i> for more details.
 * @{
 */


/**
 * @defgroup Timer_ex_Exported_Types       Timer Wrapper Exported Types
 * @{
 */




/**
 * @defgroup Timer_ex_Exported_Functions           Timer Wrapper Exported Functions
 * @{
 */
void S2LPTimerSetRxTimerMs(float fDesiredMsec);
void S2LPTimerGetRxTimer(float* pfTimeoutMsec, uint8_t* pcCounter , uint8_t* pcPrescaler);
void S2LPTimerSetWakeUpTimerMs(float fDesiredMsec);
void S2LPTimerSetWakeUpTimerReloadMs(float fDesiredMsec);
void S2LPTimerGetWakeUpTimer(float* pfWakeUpMsec, uint8_t* pcCounter , uint8_t* pcPrescaler, uint8_t* pcMulti);
void S2LPTimerGetWakeUpTimerReload(float* pfWakeUpReloadMsec, uint8_t* pcCounter, uint8_t* pcPrescaler, uint8_t* pcMulti);
void S2LPTimerComputeWakeUpValues(float fDesiredMsec , uint8_t* pcCounter , uint8_t* pcPrescaler);

/**
 * @}
 */

/**
 * @}
 */


/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/

