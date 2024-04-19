/**
 * @file    S2LP_Types.h
 * @author  LowPower RF BU - AMG
 * @version 1.3.0
 * @date    April 12, 2017
 * @brief   Header file for S2-LP types.
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
 * This module provide some types definitions which will be used in
 * all the modules of this library. Here is defined also the global
 * variable @ref g_xStatus which contains the status of S2-LP and
 * is updated every time an SPI transaction occurs.
 *
 *
 * <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __S2LP_GENERICTYPES_H
#define __S2LP_GENERICTYPES_H


/* Includes ------------------------------------------------------------------*/

/* Include all integer types definitions */
#include <stdint.h>
#include <stdio.h>



#ifdef __cplusplus
 extern "C" {
#endif


/**
 * @addtogroup S2LP_Libraries
 * @{
 */


/**
 * @defgroup S2LP_Types       Types
 * @brief Module for S2LP types definition.
 * * @details See the file <i>@ref S2LP_Types.h</i> for more details.
 * @{
 */

/**
 * @defgroup Types_Exported_Types       Types Exported Types
 * @{
 */

/**
 * @brief  S2LP Functional state. Used to enable or disable a specific option.
 */
typedef enum {
  S_DISABLE = 0,
  S_ENABLE = !S_DISABLE
} SFunctionalState;


/**
 * @brief  S2LP Flag status. Used to control the state of a flag.
 */
typedef enum {
  S_RESET = 0,
  S_SET = !S_RESET
} SFlagStatus;


/**
 * @brief  boolean type enumeration.
 */
typedef enum {
  S_FALSE = 0,
  S_TRUE  = !S_FALSE
} SBool;


/**
 * @brief  S2LP States enumeration.
 */
typedef enum {
  MC_STATE_READY             =0x00,  /*!< READY */
  MC_STATE_SLEEP_NOFIFO      =0x01,  /*!< SLEEP NO FIFO RETENTION */
  MC_STATE_STANDBY           =0x02,  /*!< STANDBY */
  MC_STATE_SLEEP             =0x03,  /*!< SLEEP */
  MC_STATE_LOCKON            =0x0C,  /*!< LOCKON */
  MC_STATE_RX                =0x30,  /*!< RX */
  MC_STATE_LOCK_ST           =0x14,  /*!< LOCK_ST */
  MC_STATE_TX                =0x5C,  /*!< TX */
  MC_STATE_SYNTH_SETUP       =0x50   /*!< SYNTH_SETUP */
} S2LPState;


/**
 * @brief S2LP Status. This definition represents the single field of the S2LP
 *        status returned on each SPI transaction, equal also to the MC_STATE registers.
 *        This field-oriented structure allows user to address in simple way the single
 *        field of the S2LP status.
 *        The user shall define a variable of S2LPStatus type to access on S2LP status fields.
 * @note  The fields order in the structure depends on used endianness (little or big
 *        endian). The actual definition is valid ONLY for LITTLE ENDIAN mode. Be sure to
 *        change opportunely the fields order when use a different endianness.
 */
typedef struct {
  uint8_t XO_ON:1;           /*!< XO is operating state */
  S2LPState MC_STATE: 7;     /*!< The state of the Main Controller of S2LP @ref S2LPState */
  uint8_t ERROR_LOCK: 1;     /*!< RCO calibration error */
  uint8_t RX_FIFO_EMPTY: 1;  /*!< RX FIFO is empty */
  uint8_t TX_FIFO_FULL: 1;   /*!< TX FIFO is full */
  uint8_t ANT_SELECT: 1;     /*!< Currently selected antenna */
  uint8_t RCCAL_OK: 1;       /*!< RCO successfully terminated */
  uint8_t : 3;               /*!< This 3 bits field are reserved and equal to 2 */
}S2LPStatus;


/**
 * @}
 */


/**
 * @defgroup Types_Exported_Constants   Types Exported Constants
 * @{
 */


/**
 * @}
 */

/**
 * @defgroup Types_Exported_Variables   Types Exported Variables
 * @{
 */

extern volatile S2LPStatus g_xStatus;

/**
 * @}
 */

/**
 * @defgroup Types_Exported_Macros              Types Exported Macros
 * @{
 */

#define IS_SFUNCTIONAL_STATE(STATE)   (STATE == S_DISABLE || STATE == S_ENABLE)
#define IS_SFLAG_STATUS(STATUS)   (STATUS == S_RESET || STATUS == S_SET)
#define IS_SBOOL(STATUS)   (STATUS == S_FALSE || STATUS == S_TRUE)

#define S_ABS(a) ((a)>0?(a):-(a))

/**
 * @}
 */


/**
 * @defgroup Types_Exported_Functions   Types Exported Functions
 * @{
 */

#ifdef  S2LP_USE_FULL_ASSERT
 /**
   * @brief  The s_assert_param macro is used for function's parameters check.
   * @param  expr If expr is false, it calls assert_failed function which reports
   *         the name of the source file and the source line number of the call
   *         that failed. If expr is true, it returns no value.
   * @retval None
   */
  #define s_assert_param(expr) ((expr) ? (void)0 : s_assert_failed((uint8_t *)__FILE__, __LINE__))
  void s_assert_failed(uint8_t* file, uint32_t line);
#elif  S2LP_USE_VCOM_ASSERT
  /**
   * @brief  The s_assert_param macro is used for function's parameters check.
   * @param  expr  If expr is false, it calls assert_failed function which reports
   *         the name of the source file and the source line number of the call
   *         that failed. If expr is true, it returns no value.
   * @retval None
   */
  #define s_assert_param(expr) ((expr) ? (void)0 : s_assert_failed((uint8_t *)__FILE__, __LINE__,#expr))
  void s_assert_failed(uint8_t* file, uint32_t line, char* expression);
#else
  #define s_assert_param(expr)        {}
#endif

/**
 *@}
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
