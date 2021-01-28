/**
 * @file    S2LP_General.h
 * @author  LowPower RF BU - AMG
 * @version 1.3.0
 * @date    10-July-2017
 * @brief   Configuration and management of S2-LP General functionalities.
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
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __S2LP_GENERAL_H
#define __S2LP_GENERAL_H


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
 * @defgroup S2LP_General     General
 * @brief Configuration and management of S2LP General functionalities.
 * @details See the file <i>@ref S2LP_General.h</i> for more details.
 * @{
 */

/**
 * @defgroup General_Exported_Types     General Exported Types
 * @{
 */


/**
 * @brief  S2LP version type enumeration
 */

typedef enum {
  MODE_EXT_XO  = 0,
  MODE_EXT_XIN = 0x80,
} ModeExtRef;


/**
 * @}
 */


/**
 * @defgroup General_Exported_Constants         General Exported Constants
 * @{
 */


/**
 * @}
 */


/**
 * @defgroup General_Exported_Macros            General Exported Macros
 * @{
 */
#define S2LPGeneralLibraryVersion() "S2LP_Libraries_v.1.3.0"


/**
 * @}
 */


/**
 * @defgroup General_Exported_Functions         General Exported Functions
 * @{
 */

uint8_t S2LPGeneralGetDevicePN(void);
uint8_t S2LPGeneralGetVersion(void);
void S2LPGeneralSetExtRef(ModeExtRef xExtMode);
ModeExtRef S2LPGeneralGetExtRef(void);
void S2LPRadioSetExternalSmpsMode(SFunctionalState xNewState);
void S2LPRefreshStatus(void);

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
