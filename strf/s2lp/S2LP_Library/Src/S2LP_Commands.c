/**
 * @file    S2LP_Commands.c
 * @author  LowPower RF BU - AMG
 * @version 1.3.0
 * @date    10-July-2017
 * @brief   Management of S2-LP Commands.
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
 *
 * <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
 */


/* Includes ------------------------------------------------------------------*/
#include "S2LP_Commands.h"
#include "MCU_Interface.h"



/**
 * @addtogroup S2LP_Libraries
 * @{
 */


/**
 * @addtogroup S2LP_Commands
 * @{
 */


/**
 * @defgroup Commands_Private_TypesDefinitions  Commands Private TypesDefinitions
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Commands_Private_Defines           Commands Private Defines
 * @{
 */

/**
 *@}
 */

/**
 * @defgroup Commands_Private_Macros            Commands Private Macros
 * @{
 */

#define IS_S2LP_CMD(CMD)  (CMD == CMD_TX || \
                             CMD == CMD_RX || \
                             CMD == CMD_READY || \
                             CMD == CMD_STANDBY || \
                             CMD == CMD_SLEEP || \
                             CMD == CMD_LOCKRX || \
                             CMD == CMD_LOCKTX || \
                             CMD == CMD_SABORT || \
                             CMD == CMD_LDC_RELOAD || \
                             CMD == CMD_SEQUENCE_UPDATE || \
                             CMD == CMD_SRES || \
                             CMD == CMD_FLUSHRXFIFO || \
                             CMD == CMD_FLUSHTXFIFO \
                            )

/**
 *@}
 */


/**
 * @defgroup Commands_Private_Variables         Commands Private Variables
 * @{
 */

/**
 *@}
 */



/**
 * @defgroup Commands_Private_FunctionPrototypes        Commands Private Function Prototypes
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Commands_Private_Functions                 Commands Private Functions
 * @{
 */

/**
 * @brief  Send a specific command to S2LP.
 * @param  xCommandCode code of the command to send.
           This parameter can be any value of @ref S2LPCmd.
 * @retval None.
 */
void S2LPCmdStrobeCommand(S2LPCmd xCommandCode)
{
  /* Check the parameters */
  s_assert_param(IS_S2LP_CMD(xCommandCode));

  g_xStatus = S2LPSpiCommandStrobes((uint8_t) xCommandCode);
}


/**
 *@}
 */


/**
 *@}
 */


/**
 *@}
 */




/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
