/**
 * @file    S2LP_PktStack.h
 * @author  LowPower RF BU - AMG
 * @version 1.3.0
 * @date    10-July-2017
 * @brief   Configuration and management of S2-LP STack packets.
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
 * This module can be used to manage the configuration of S2LP STack
 * packets, and it is quite similar to the Basic packets one since the
 * STack packets can be considered an extension of Basic.
 * The user can obtain a packet configuration filling the structure
 * <i>@ref PktStackInit</i>, defining in it some general parameters
 * for the S2-LP STack packet format.
 * Another structure the user can fill is <i>@ref PktStackAddressesInit</i>
 * to define the addresses which will be used during the communication.
 * Moreover, functions to set the payload length and the destination address
 * are provided.
 *
 * <b>Example:</b>
 * @code
 *
 * PktStackInit stackInit={
 *   32,                                // preamble length in bits
 *   32,                                // sync word length in bits
 *   0x88888888,                        // sync word
 *   S_ENABLE,                          // variable or fixed payload length
 *   S_DISABLE,                         // extended length field width (used only for variable length)
 *   PKT_NO_CRC,                        // CRC mode
 *   S_DISABLE,                         // FEC
 *   S_ENABLE                           // whitening
 * };
 *
 * PktStackAddressesInit addressInit={
 *   S_ENABLE,                          // enable/disable filtering on my address
 *   0x34,                              // my address (address of the current node)
 *   S_DISABLE,                         // enable/disable filtering on multicast address
 *   0xEE,                              // multicast address
 *   S_DISABLE,                         // enable/disable filtering on broadcast address
 *   0xFF                               // broadcast address
 * };
 *
 * ...
 *
 * S2LPPktStackInit(&stackInit);
 * S2LPPktStackAddressesInit(&addressInit);
 *
 * ...
 *
 * S2LPPktStackSetPayloadLength(20);
 * S2LPPktStackSetDestinationAddress(0x44);
 *
 * ...
 *
 * @endcode
 *
 * The module provides some other functions that can be used to modify
 * or read only some configuration parameters.
 *
 * <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __S2LP_PKT_STACK_H
#define __S2LP_PKT_STACK_H

/* Includes ------------------------------------------------------------------*/

#include "S2LP_Regs.h"
#include "S2LP_Types.h"
#include "S2LP_PacketHandler.h"

#ifdef __cplusplus
 extern "C" {
#endif


/**
 * @addtogroup S2LP_Libraries
 * @{
 */


/**
 * @defgroup S2LP_PktStack    Pkt STack
 * @brief Configuration and management of S2LP STack packets.
 * @details See the file <i>@ref S2LP_PktStack.h</i> for more details.
 * @{
 */

/**
 * @defgroup PktStack_Exported_Types    Pkt STack Exported Types
 * @{
 */

   
/**
 * @brief  CRC length in bytes enumeration.
 */
typedef PktCrcMode     StackCrcMode;


/**
 * @brief  S2LP STack Packet Init structure definition.
 */
typedef struct {
  uint16_t           xPreambleLength;         /*!< Set the preamble length of packet. From 1 to 1024 chip sequence. */
  uint8_t            xSyncLength;             /*!< Set the sync word length of packet in bits. From 1 to 64 bits. */
  uint32_t           lSyncWords;	            /*!< Set the sync words in MSB. */
  SFunctionalState   xFixVarLength;           /*!< Enable the variable length mode. */
  SFunctionalState   cExtendedPktLenField;    /*!< Extend the length field from 1 byte to 2 bytes. Variable length mode only. */
  StackCrcMode       xCrcMode;              	/*!< Set the CRC type. @ref StackCrcMode */
  SFunctionalState   xFec;                    /*!< Enable the FEC/Viterbi. */
  SFunctionalState   xDataWhitening;          /*!< Enable the data whitening. */
} PktStackInit;


/**
 * @brief  S2LP STack packet address structure definition. This structure allows users to specify
 *         the node/multicast/broadcast addresses and the correspondent filtering options.
 */
typedef struct {
  SFunctionalState   xFilterOnMyAddress;         /*!< If set packet is received if its destination address matches with cMyAddress. */
  uint8_t            cMyAddress;                 /*!< Set the MyAddress. */
  SFunctionalState   xFilterOnMulticastAddress;  /*!< If set packet is received if its destination address matches with cMulticastAddress. */
  uint8_t            cMulticastAddress;          /*!< Set the Multicast address */
  SFunctionalState   xFilterOnBroadcastAddress;  /*!< If set packet is received if its destination address matches with cBroadcastAddress. */
  uint8_t            cBroadcastAddress;          /*!< Set the Broadcast address */
} PktStackAddressesInit;


/**
 *@}
 */


/**
 * @defgroup PktStack_Exported_Macros   Pkt STack Exported Macros
 * @{
 */

/**
 * @brief  Sets the PREAMBLE Length mode for S2LP STack packets.
 * @param  xPreambleLength length of PREAMBLE field in bytes.
 *         This parameter can be any value of @ref StackPreambleLength.
 * @retval None.
 */
#define S2LPPktStackSetPreambleLength(xPreambleLength)                        S2LPSetPreambleLength(xPreambleLength)


/**
 * @brief  Returns the PREAMBLE Length mode for S2LP STack packets.
 * @param  None.
 * @retval uint8_t Preamble length in bytes.
 */
#define S2LPPktStackGetPreambleLength()                                      S2LPGetPreambleLength()


/**
 * @brief  Sets the SYNC Length for S2LP STack packets.
 * @param  xSyncLength length of SYNC field in bytes.
 *         This parameter can be any value of @ref StackSyncLength.
 * @retval None.
 */
#define S2LPPktStackSetSyncLength(xSyncLength)                                S2LPSetSyncLength((PktSyncLength)xSyncLength)


/**
 * @brief  Returns the SYNC Length for S2LP STack packets.
 * @param  None.
 * @retval uint8_t Sync length in bytes.
 */
#define S2LPPktStackGetSyncLength()                                           S2LPGetSyncLength()



/**
 * @brief  Enables or Disables the CRC filtering.
 * @param  xNewState new state for CRC_CHECK.
 *         This parameter can be S_ENABLE or S_DISABLE.
 * @retval None.
 */
#define S2LPPktStackFilterOnCrc(xNewState)                                    S2LPPktCommonFilterOnCrc(xNewState)



/**
 * @brief  Enables or Disables WHITENING for S2LP STack packets.
 * @param  xNewState new state for WHITENING mode.
 *         This parameter can be S_ENABLE or S_DISABLE.
 * @retval None.
 */
#define S2LPPktStackWhitening(xNewState)                                     S2LPWhitening(xNewState)


/**
 * @brief  Enables or Disables FEC for S2LP STack packets.
 * @param  xNewState new state for FEC mode.
 *         This parameter can be S_ENABLE or S_DISABLE.
 * @retval None.
 */
#define S2LPPktStackFec(xNewState)                                            S2LPFec(xNewState)


/**
 * @brief  Sets multiple SYNC words for S2LP STack packets.
 * @param  lSyncWords SYNC words to be set with format: 0x|SYNC1|SYNC2|SYNC3|SYNC4|.
 *         This parameter is a uint32_t.
 * @param  xSyncLength SYNC length in bytes. The 32bit word passed will be stored in the SYNCx registers from the MSb
 *         until the number of bytes in xSyncLength has been stored.
 *         This parameter is a @ref StackSyncLength.
 * @retval None.
 */
#define S2LPPktStackSetSyncWords(lSyncWords, xSyncLength)                    S2LPSetSyncWords(lSyncWords,xSyncLength)


/**
 *@}
 */


/**
 * @defgroup PktStack_Exported_Functions        Pkt STack Exported Functions
 * @{
 */

void S2LPPktStackInit(PktStackInit* pxPktStackInit);
void S2LPPktStackGetInfo(PktStackInit* pxPktStackInit);
void S2LPPktStackAddressesInit(PktStackAddressesInit* pxPktStackAddresses);
void S2LPPktStackGetAddressesInfo(PktStackAddressesInit* pxPktStackAddresses);
void S2LPPktStackSetFormat(void);
void S2LPPktStackSetPayloadLength(uint16_t nPayloadLength);
uint16_t S2LPPktStackGetPayloadLength(void);
void S2LPPktStackAckRequest(SFunctionalState xNewState);
void S2LPPktStackAutoAck(SFunctionalState xNewState);
void S2LPPktStackNRetx(uint8_t nRetx);
void S2LPPktStackPiggybacking(SFunctionalState xNewState);
SFlagStatus S2LPPktStackGetTXAckRequest(void);
uint8_t S2LPPktStackGetNReTx(void);

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

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
