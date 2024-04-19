/**
 * @file    S2LP_Fifo.h
 * @author  LowPower RF BU - AMG
 * @version 1.3.0
 * @date    10-July-2017
 * @brief   Configuration and management of S2-LP Fifo.
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
* This module allows the user to manage the linear FIFO. The functions exported
* here can be used to set the thresholds for the FIFO almost full / empty alarm
* interrupts or to get the total number of elements inside the FIFO.
*
* <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __S2LP_FIFO_H
#define __S2LP_FIFO_H


/* Includes ------------------------------------------------------------------*/
#include "S2LP_Regs.h"
#include "S2LP_Types.h"


#ifdef __cplusplus
 extern "C" {
#endif


/**
 * @addtogroup S2LP_Libraries
 * @{
 */


/**
 * @defgroup S2LP_Fifo           FIFO
 * @brief Configuration and management of S2LP FIFO.
 * @details See the file <i>@ref S2LP_Fifo.h</i> for more details.
 * @{
 */

/**
 * @defgroup Fifo_Exported_Types   FIFO Exported Types
 * @{
 */


/**
 * @}
 */


/**
 * @defgroup Fifo_Exported_Constants       FIFO Exported Constants
 * @{
 */

/**
 * @}
 */


/**
 * @defgroup Fifo_Exported_Macros          FIFO Exported Macros
 * @{
 */


/**
 * @}
 */


/**
 * @defgroup Fifo_Exported_Functions                       FIFO Exported Functions
 * @{
 */

uint8_t S2LPFifoReadNumberBytesRxFifo(void);
uint8_t S2LPFifoReadNumberBytesTxFifo(void);
void S2LPFifoSetAlmostFullThresholdRx(uint8_t cThrRxFifo);
uint8_t S2LPFifoGetAlmostFullThresholdRx(void);
void S2LPFifoSetAlmostEmptyThresholdRx(uint8_t cThrRxFifo);
uint8_t S2LPFifoGetAlmostEmptyThresholdRx(void);
void S2LPFifoSetAlmostFullThresholdTx(uint8_t cThrTxFifo);
uint8_t S2LPFifoGetAlmostFullThresholdTx(void);
void S2LPFifoSetAlmostEmptyThresholdTx(uint8_t cThrTxFifo);
uint8_t S2LPFifoGetAlmostEmptyThresholdTx(void);
void S2LPFifoMuxRxFifoIrqEnable(SFunctionalState xNewState);

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

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
