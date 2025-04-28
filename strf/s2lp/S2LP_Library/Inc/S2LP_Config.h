/**
 * @file    S2LP_Config.h
 * @author  LowPower RF BU - AMG
 * @version 1.3.0
 * @date    10-July-2017
 * @brief   S2LP Configuration and useful defines .
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
* This file is used to include all or a part of the S2LP
* libraries into the application program which will be used.
* Moreover some important parameters are defined here and the
* user is allowed to edit them.
*
* <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __S2LP_CONFIG_H
#define __S2LP_CONFIG_H


  /* Includes ------------------------------------------------------------------*/
#include "S2LP_Regs.h"
#include "S2LP_Commands.h"
#include "S2LP_Csma.h"
#include "S2LP_General.h"
#include "S2LP_Gpio.h"
#include "S2LP_Timer.h"
#include "S2LP_Fifo.h"
#include "S2LP_PacketHandler.h"
#include "S2LP_PktBasic.h"
#include "S2LP_PktWMbus.h"
#include "S2LP_PktStack.h"
#include "S2LP_Radio.h"
#include "S2LP_Qi.h"
#include "MCU_Interface.h"
#include "S2LP_Types.h"


#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup S2LP_Libraries        S2LP Libraries
 * @brief This firmware implements libraries which allow the user
 * to manage the features of S2LP without knowing the hardware details.
 * @details The <i>S2LP_Libraries</i> modules are totally platform independent. The library provides one
 * module for each device feature. Each module refers to some functions whose prototypes are located in the
 * header file <i>@ref MCU_Interface.h</i>. The user who want to use these libraries on a particular
 * platform has to implement these functions respecting them signatures.
 * @{
 */

/** @defgroup S2LP_Configuration      Configuration
 * @brief S2LP Configuration and useful defines.
 * @details See the file <i>@ref S2LP_Config.h</i> for more details.
 * @{
 */


/** @defgroup Configuration_Exported_Types      Configuration Exported Types
 * @{
 */

/**
 * @}
 */


/** @defgroup Configuration_Exported_Constants  Configuration Exported Constants
 * @{
 */
#define DIG_DOMAIN_XTAL_THRESH  30000000        /*!< Digital domain logic threshold for XTAL in MHz */

/**
 * @}
 */


/** @defgroup Configuration_Exported_Macros     Configuration Exported Macros
 * @{
 */

  
/**
 * @}
 */


/** @defgroup Configuration_Exported_Functions  Configuration Exported Functions
 * @{
 */

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

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
