/**
 * @file    S2LP_PacketHandler.h
 * @author  LowPower RF BU - AMG
 * @version 1.3.0
 * @date    10-July-2017
 * @brief   Configuration and management of the common features of S2-LP packets.
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
 * This module provides all the common functions and definitions used by the
 * packets modules.
 * Here are also defined all the generic enumeration types that are redefined
 * in the specific packets modules, but every enumeration value is referred
 * to this module. So the user who wants to configure the preamble of a Basic,
 * or a STack packet has to use the enumeration values defined here.
 *
 * <b>Example:</b>
 * @code
 *
 *   ...
 *
 *   S2LPPktBasicSetPreambleLength(PKT_PREAMBLE_LENGTH_18BYTES);
 *
 *   ...
 *
 * @endcode
 *
 *
 * <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __S2LP_PKT_COMMON_H
#define __S2LP_PKT_COMMON_H

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
 * @defgroup S2LP_PktCommon           Pkt Common
 * @brief Configuration and management of the common features of S2LP packets.
 * @details See the file <i>@ref S2LP_PktCommon.h</i> for more details.
 * @{
 */

/**
 * @defgroup PktCommon_Exported_Types   Pkt Common Exported Types
 * @{
 */


/**
 * @brief  CRC length in bytes enumeration.
 */
typedef enum {
  PKT_NO_CRC               = 0x00, /*!< No CRC                              */
  PKT_CRC_MODE_8BITS       = 0x20, /*!< CRC length 8 bits  - poly: 0x07     */
  PKT_CRC_MODE_16BITS_1    = 0x40, /*!< CRC length 16 bits - poly: 0x8005   */
  PKT_CRC_MODE_16BITS_2    = 0x60, /*!< CRC length 16 bits - poly: 0x1021   */
  PKT_CRC_MODE_24BITS      = 0x80, /*!< CRC length 24 bits - poly: 0x864CFB */
  PKT_CRC_MODE_32BITS      = 0xA0, /*!< CRC length 32 bits - poly: 0x04C011BB7 */
} PktCrcMode;


/**
 * @brief  Direct transmission mode enumeration for SPIRIT.
 */
typedef enum
{
  NORMAL_TX_MODE = 0x00,          /*!< Normal mode, no direct transmission is used */
  DIRECT_TX_FIFO_MODE = 0x04,     /*!< Source is FIFO: payload bits are continuously read from the TX FIFO */
  DIRECT_TX_GPIO_MODE = 0x08,     /*!< Source is GPIO: payload bits are continuously read from one of the GPIO ports and transmitted without any processing */
  PN9_TX_MODE = 0x0C              /*!< A pseudorandom binary sequence is generated internally */
} DirectTx;


/**
 * @brief  Direct receive mode enumeration for SPIRIT.
 */
typedef enum
{
  NORMAL_RX_MODE = 0x00,          /*!< Normal mode, no direct reception is used */
  DIRECT_RX_FIFO_MODE = 0x10,     /*!< Destination is FIFO: payload bits are continuously written to the RX FIFO and not subjected to any processing*/
  DIRECT_RX_GPIO_MODE = 0x20      /*!< Destination is GPIO: payload bits are continuously written to one of the GPIO ports and not subjected to any processing*/
} DirectRx;


/**
 *@}
 */


/**
 * @defgroup PktCommon_Exported_Constants               Pkt Common Exported Constants
 * @{
 */


/**
 *@}
 */


/**
 * @defgroup PktCommon_Exported_Macros                  Pkt Common Exported Macros
 * @{
 */

#define S2LPSetPreambleLengthByte(xPreambleLength)      S2LPSetPreambleLength(4*xPreambleLength)
#define S2LPSetSyncLengthByte(cSyncLength)              S2LPSetSyncLength(8*cSyncLength)

#define IS_PREAMBLE_LEN(VAL)  (VAL<=1024)
#define IS_SYNC_LEN(VAL)      (VAL<=64)

#define IS_PKT_LEN_FIELD_WID(LENGTH)   ((LENGTH == PKT_LENGTH_FIELD_1BYTE) || \
                                        (LENGTH == PKT_LENGTH_FIELD_2BYTE))


#define IS_PKT_CRC_MODE(MODE)   ((MODE == PKT_NO_CRC) || \
                                (MODE == PKT_CRC_MODE_8BITS)  || \
                                (MODE == PKT_CRC_MODE_16BITS_1)  || \
                                (MODE == PKT_CRC_MODE_16BITS_2) || \
                                (MODE == PKT_CRC_MODE_24BITS))
/**
 *@}
 */


/**
 * @defgroup PktCommon_Exported_Functions               Pkt Common Exported Functions
 * @{
 */

void S2LPSetPreambleLength(uint16_t cPreambleLength);
uint16_t S2LPGetPreambleLength(void);
void S2LPSetSyncLength(uint8_t cSyncLength);
uint8_t S2LPGetSyncLength(void);
void S2LPSetSyncWords(uint32_t lSyncWords, uint8_t xSyncLength);
void S2LPGetSyncWords(uint32_t* lSyncWords, uint8_t* cSyncLength);
void S2LPPacketHandlerWhitening(SFunctionalState xNewState);
void S2LPPacketHandlerFec(SFunctionalState xNewState);
void S2LPPacketHandler3OutOf6(SFunctionalState xNewState);
void S2LPPacketHandlerManchester(SFunctionalState xNewState);
uint8_t S2LPGetPacketFormat(void);
void S2LPPktCommonFilterOnCrc(SFunctionalState xNewState);
uint8_t S2LPGetReceivedDestinationAddress(void);
uint8_t S2LPGetReceivedSourceAddress(void);
uint8_t S2LPGetMyAddress(void);
uint8_t S2LPGetBroadcastAddress(void);
uint8_t S2LPGetMulticastAddress(void);
uint8_t S2LPGetRxSourceMask(void);
uint8_t S2LPGetRxSourceReferenceAddress(void);
void S2LPPacketHandlerSetTxMode(DirectTx xNewState);
void S2LPPacketHandlerSetRxMode(DirectRx xNewState);
DirectTx S2LPPacketHandlerGetTxMode(void);
DirectRx S2LPPacketHandlerGetRxMode(void);
void S2LPPacketHandlerSetExtendedLenField(SFunctionalState xExtendedLenField);
void S2LPPacketHandlerSwap4FSKSymbol(SFunctionalState xSwapSymbol);
void S2LPPacketHandlerSwapFifoEndianess(SFunctionalState xEnableSwap);
void S2LPPacketHandlerSwapPreamblePattern(SFunctionalState xEnableSwap);
void S2LPPacketHandlerSetCrcMode(PktCrcMode xPktCrcMode);
PktCrcMode S2LPPacketHandlerGetCrcMode(void);
void S2LPPacketHandlerSelectSecondarySync(SFunctionalState xSecondarySync);
void S2LPPacketHandlerSetAutoPcktFilter(SFunctionalState xNewState);
void S2LPPacketHandlerSetRxPersistentMode(SFunctionalState xNewState);
void S2LPPacketHandlerSetSrcAddrFlt(SFunctionalState xNewState);
void S2LPPacketHandlerSetVariableLength(SFunctionalState xVarLen);
void S2LPSetDualSyncWords(uint32_t lSyncWords);
void S2LPGetDualSyncWords(uint32_t* lSyncWords);
void S2LPSetRxSourceMask(uint8_t address);
void S2LPSetRxSourceReferenceAddress(uint8_t address);
void S2LPSetBroadcastAddress(uint8_t address);
void S2LPSetMulticastAddress(uint8_t address);
void S2LPSetMyAddress(uint8_t address);
uint8_t S2LPGetMyAddress(void);


/**
 *@}
 */

/**
 *@}
 */


/**
 *@}
 */

#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
