/**
 * @file    MCU_Interface_template.h
 * @author  LowPower RF BU - AMG
 * @version 1.1.0
 * @date    July 1, 2016
 * @brief   Header file for low level S2LP SPI driver. 
 * 			To be copied in the application floder and customized according to the SPI driver used.
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
* <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MCU_INTERFACE_H
#define __MCU_INTERFACE_H


/* Includes ------------------------------------------------------------------*/
#include "S2LP_Config.h"

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




/** @defgroup SPI_Exported_Functions    SPI Exported Functions
 * @{
 */


void SdkEvalSpiInit(void);
void SdkEvalSpiDeinit(void);
StatusBytes SdkEvalSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes SdkEvalSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes SdkEvalSpiCommandStrobes(uint8_t cCommandCode);
StatusBytes SdkEvalSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
StatusBytes SdkEvalSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer);

void SdkEvalEnterShutdown(void);
void SdkEvalExitShutdown(void);
SFlagStatus SdkEvalCheckShutdown(void);

/**
 * @}
 */


/** @defgroup SPI_Exported_Macros       SPI Exported Macros
 * @{
 */

#define S2LPEnterShutdown                                  SdkEvalEnterShutdown
#define S2LPExitShutdown                                   SdkEvalExitShutdown
#define S2LPCheckShutdown                                  (S2LPFlagStatus)SdkEvalCheckShutdown

#define S2LPSpiInit                                                  SdkEvalSpiInit
#define S2LPSpiDeinit                                                  SdkEvalSpiDeinit
#define S2LPSpiWriteRegisters(cRegAddress, cNbBytes, pcBuffer)       SdkEvalSpiWriteRegisters(cRegAddress, cNbBytes, pcBuffer)
#define S2LPSpiReadRegisters(cRegAddress, cNbBytes, pcBuffer)        SdkEvalSpiReadRegisters(cRegAddress, cNbBytes, pcBuffer)
#define S2LPSpiCommandStrobes(cCommandCode)                          SdkEvalSpiCommandStrobes(cCommandCode)
#define S2LPSpiWriteFifo(cNbBytes, pcBuffer)                   SdkEvalSpiWriteFifo(cNbBytes, pcBuffer)
#define S2LPSpiReadFifo(cNbBytes, pcBuffer)                    SdkEvalSpiReadFifo(cNbBytes, pcBuffer)

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
