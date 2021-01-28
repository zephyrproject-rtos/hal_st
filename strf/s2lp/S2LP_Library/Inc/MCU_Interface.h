/**
* @file    MCU_Interface.h
* @author  LowPower RF BU - AMG
* @version 1.1.0
* @date    July 1, 2016
* @brief   Header file for low level S2LP SPI driver.
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
* This header file constitutes an interface to the SPI driver used to
* communicate with S2LP.
* It exports some function prototypes to write/read registers and FIFOs
* and to send command strobes.
* Since the S2LP libraries are totally platform independent, the implementation
* of these functions are not provided here. The user have to implement these functions
* taking care to keep the exported prototypes.
*
* These functions are:
*
* <ul>
* <li>S2LPSpiInit</i>
* <li>S2LPSpiWriteRegisters</i>
* <li>S2LPSpiReadRegisters</i>
* <li>S2LPSpiCommandStrobes</i>
* <li>S2LPSpiWriteLinearFifo</i>
* <li>S2LPSpiReadLinearFifo</i>
* </ul>
*
* @note An example of SPI driver implementation is available in the <i>Sdk_Eval</i> library.
*
*
* <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MCU_INTERFACE_H
#define __MCU_INTERFACE_H


/* Includes ------------------------------------------------------------------*/
#include "S2LP_Config.h"
#include "S2LP_Util.h"
#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup S2LP_Libraries
 * @{
 */

/** @defgroup S2LP_SPI_Driver         SPI Driver
 * @brief Header file for low level S2LP SPI driver.
 * @details See the file <i>@ref MCU_Interface.h</i> for more details.
 * @{
 */


/** @defgroup SPI_Exported_Types        SPI Exported Types
 * @{
 */

typedef S2LPStatus StatusBytes;

/**
 * @}
 */


/** @defgroup SPI_Exported_Constants    SPI Exported Constants
 * @{
 */

/**
 * @}
 */


/** @defgroup SPI_Exported_Functions    SPI Exported Functions
 * @{
 */

void RadioSpiInit(void);
void RadioSpiDeinit(void);
StatusBytes RadioSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes RadioSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes RadioSpiCommandStrobes(uint8_t cCommandCode);
StatusBytes RadioSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes RadioSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
//static void SPI_Error(void);

void RadioEnterShutdown(void);
void RadioExitShutdown(void);
//SFlagStatus RadioCheckShutdown(void);

/**
 * @}
 */


/** @defgroup SPI_Exported_Macros       SPI Exported Macros
 * @{
 */

#define S2LPEnterShutdown                                  RadioEnterShutdown
#define S2LPExitShutdown                                   RadioExitShutdown
#define S2LPCheckShutdown                                  (SFlagStatus)RadioCheckShutdown //(S2LPFlagStatus)SdkEvalCheckShutdown

#define S2LPSpiInit                                                  RadioSpiInit
#define S2LPSpiDeinit                                                RadioSpiDeinit
#define S2LPSpiWriteRegisters(cRegAddress, cNbBytes, pcBuffer)       RadioSpiWriteRegisters(cRegAddress, cNbBytes, pcBuffer)
#define S2LPSpiReadRegisters(cRegAddress, cNbBytes, pcBuffer)        RadioSpiReadRegisters(cRegAddress, cNbBytes, pcBuffer)
#define S2LPSpiCommandStrobes(cCommandCode)                          RadioSpiCommandStrobes(cCommandCode)
#define S2LPSpiWriteFifo(cNbBytes, pcBuffer)                         RadioSpiWriteFifo(cNbBytes, pcBuffer)
#define S2LPSpiReadFifo(cNbBytes, pcBuffer)                          RadioSpiReadFifo(cNbBytes, pcBuffer)
//#define SdkEvalSpiReadRegisters(cRegAddress, cNbBytes, pcBuffer)     S2LPSpiReadRegisters(cRegAddress, cNbBytes, pcBuffer)
//#define SdkEvalSpiWriteRegisters(cRegAddress, cNbBytes, pcBuffer)    S2LPSpiWriteRegisters(cRegAddress, cNbBytes, pcBuffer)    
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
