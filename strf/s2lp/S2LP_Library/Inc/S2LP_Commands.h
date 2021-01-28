/**
 * @file    S2LP_Commands.h
 * @author  LowPower RF BU - AMG
 * @version 1.3.0
 * @date    April 12, 2017
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
* In this module can be found all the API used to strobe commands to
* S2LP.
* Every command strobe is an SPI transaction with a specific command code.
*
* <b>Example:</b>
* @code
*   ...
*
*   S2LPCmdStrobeRx();
*
*   ...
* @endcode
*
* <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __S2LP_COMMANDS_H
#define __S2LP_COMMANDS_H


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
 * @defgroup S2LP_Commands    Commands
 * @brief Management of S2LP Commands.
 * @details See the file <i>@ref S2LP_Commands.h</i> for more details.
 * @{
 */

/**
 * @defgroup Commands_Exported_Types    Commands Exported Types
 * @{
 */

/**
  * @brief  S2LP Commands codes enumeration
  */
typedef enum
{
  CMD_TX =  ((uint8_t)(0x60)),                    /*!< Start to transmit; valid only from READY */
  CMD_RX =  ((uint8_t)(0x61)),                    /*!< Start to receive; valid only from READY */
  CMD_READY =  ((uint8_t)(0x62)),                 /*!< Go to READY; valid only from STANDBY or SLEEP or LOCK */
  CMD_STANDBY =  ((uint8_t)(0x63)),               /*!< Go to STANDBY; valid only from READY */
  CMD_SLEEP = ((uint8_t)(0x64)),                  /*!< Go to SLEEP; valid only from READY */
  CMD_LOCKRX = ((uint8_t)(0x65)),                 /*!< Go to LOCK state by using the RX configuration of the synth; valid only from READY */
  CMD_LOCKTX = ((uint8_t)(0x66)),                 /*!< Go to LOCK state by using the TX configuration of the synth; valid only from READY */
  CMD_SABORT = ((uint8_t)(0x67)),                 /*!< Force exit form TX or RX states and go to READY state; valid only from TX or RX */
  CMD_LDC_RELOAD = ((uint8_t)(0x68)),             /*!< LDC Mode: Reload the LDC timer with the value stored in the  LDC_PRESCALER / COUNTER  registers; valid from all states  */
  CMD_RCO_CALIB =  ((uint8_t)(0x69)),             /*!< Start (or re-start) the RCO calibration */
  CMD_SRES = ((uint8_t)(0x70)),                   /*!< Reset of all digital part, except SPI registers */
  CMD_FLUSHRXFIFO = ((uint8_t)(0x71)),            /*!< Clean the RX FIFO; valid from all states */
  CMD_FLUSHTXFIFO = ((uint8_t)(0x72)),            /*!< Clean the TX FIFO; valid from all states */
  CMD_SEQUENCE_UPDATE =  ((uint8_t)(0x73)),       /*!< Autoretransmission: Reload the Packet sequence counter with the value stored in the PROTOCOL[2] register valid from all states */
} S2LPCmd;


/**
 * @}
 */


/**
 * @defgroup Commands_Exported_Constants        Commands Exported Constants
 * @{
 */


/**
 * @}
 */


/**
 * @defgroup Commands_Exported_Macros   Commands Exported Macros
 * @{
 */

/**
 * @brief  Sends the TX command to S2-LP. Start to transmit.
 * @param  None.
 * @retval None.
 * @note: this macro sets the SMPS switching frequency to 5.46MHz about for ETSI regulation compliancy.
 */

#define S2LPCmdStrobeTx()         {uint8_t tmp=0x9C;\
                                    Config_RangeExt(PA_TX,S2LPManagementGetRangeExtender());\
                                     S2LPSpiWriteRegisters(0x76,1,&tmp);\
                                     S2LPCmdStrobeCommand(CMD_TX);}

/**
 * @brief  Sends the RX command to S2-LP. Start to receive.
 * @param  None.
 * @retval None.
 * @note: this macro sets the SMPS switching frequency to 3.12MHz.
 */

#define S2LPCmdStrobeRx()         {uint8_t tmp=0x90;\
                                    Config_RangeExt(PA_RX,S2LPManagementGetRangeExtender());\
                                    S2LPSpiWriteRegisters(0x76,1,&tmp);\
                                    S2LPCmdStrobeCommand(CMD_RX);}

#define S2LPCmdStrobeReady()      S2LPCmdStrobeCommand(CMD_READY)
#define S2LPCmdStrobeStandby()    S2LPCmdStrobeCommand(CMD_STANDBY)
#define S2LPCmdStrobeSleep()      S2LPCmdStrobeCommand(CMD_SLEEP)
#define S2LPCmdStrobeLockRx()     S2LPCmdStrobeCommand(CMD_LOCKRX)
#define S2LPCmdStrobeLockTx()     S2LPCmdStrobeCommand(CMD_LOCKTX)
#define S2LPCmdStrobeSabort()     S2LPCmdStrobeCommand(CMD_SABORT)
#define S2LPCmdStrobeLdcReload()  S2LPCmdStrobeCommand(CMD_LDC_RELOAD)
#define S2LPCmdStrobeSequenceUpdate() S2LPCmdStrobeCommand(CMD_SEQUENCE_UPDATE)
#define S2LPCmdStrobeSres()          S2LPCmdStrobeCommand(CMD_SRES)
#define S2LPCmdStrobeFlushRxFifo()    S2LPCmdStrobeCommand(CMD_FLUSHRXFIFO)
#define S2LPCmdStrobeFlushTxFifo()    S2LPCmdStrobeCommand(CMD_FLUSHTXFIFO)


/**
 * @}
 */


/**
 * @defgroup Commands_Exported_Functions    Commands Exported Functions
 * @{
 */
  
void S2LPCmdStrobeCommand(S2LPCmd xCommandCode);


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
