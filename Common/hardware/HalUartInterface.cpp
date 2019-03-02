/**
  *****************************************************************************
  * @file    HalUartInterface.cpp
  * @author  Tyler Gamvrelis
  *
  * @defgroup HalUartInterface
  * @brief    Implements UartInterface using HAL functions
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "HalUartInterface.h"




/********************************** Externs **********************************/
// These functions are not defined in the F767xx's HAL drivers, so we drag them
// over from the F446xx's HAL drivers
#if defined(USE_MANUAL_UART_ABORT_DEFINITIONS)
/**
  * @brief  Abort ongoing Transmit transfer (blocking mode).
  * @param  huart UART handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable PPP Interrupts
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval HAL status
*/
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart)
{
  /* Disable TXEIE and TCIE interrupts */
  CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));

  /* Disable the UART DMA Tx request if enabled */
  if(HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAT))
  {
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);

    /* Abort the UART DMA Tx channel : use blocking DMA Abort API (no callback) */
    if(huart->hdmatx != NULL)
    {
      /* Set the UART DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      huart->hdmatx->XferAbortCallback = NULL;

      HAL_DMA_Abort(huart->hdmatx);
    }
  }

  /* Reset Tx transfer counter */
  huart->TxXferCount = 0x00U;

  /* Restore huart->gState to Ready */
  huart->gState = HAL_UART_STATE_READY;

  return HAL_OK;
}

/**
  * @brief  Abort ongoing Receive transfer (blocking mode).
  * @param  huart UART handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable PPP Interrupts
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval HAL status
*/
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart)
{
  /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
  CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

  /* Disable the UART DMA Rx request if enabled */
  if(HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR))
  {
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Abort the UART DMA Rx channel : use blocking DMA Abort API (no callback) */
    if(huart->hdmarx != NULL)
    {
      /* Set the UART DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      huart->hdmarx->XferAbortCallback = NULL;

      HAL_DMA_Abort(huart->hdmarx);
    }
  }

  /* Reset Rx transfer counter */
  huart->RxXferCount = 0x00U;

  /* Restore huart->RxState to Ready */
  huart->RxState = HAL_UART_STATE_READY;

  return HAL_OK;
}
#endif




namespace hal{
/***************************** HalUartInterface ******************************/
// Public
// ----------------------------------------------------------------------------
HalUartInterface::HalUartInterface(){;}

HalUartInterface::~HalUartInterface(){;}

HAL_StatusTypeDef HalUartInterface::transmitPoll(
    const UART_HandleTypeDef* uartHandlePtr,
    uint8_t* arrTransmit,
    size_t numBytes,
    uint32_t timeout
) const
{
    HAL_StatusTypeDef status =  HAL_UART_Transmit(
        const_cast<UART_HandleTypeDef*>(uartHandlePtr),
        arrTransmit,
        numBytes,
        timeout
    );

    return status;
}

HAL_StatusTypeDef HalUartInterface::receivePoll(
    const UART_HandleTypeDef* uartHandlePtr,
    uint8_t* arrReceive,
    size_t numBytes,
    uint32_t timeout
) const
{
    HAL_StatusTypeDef status =  HAL_UART_Receive(
        const_cast<UART_HandleTypeDef*>(uartHandlePtr),
        arrReceive,
        numBytes,
        timeout
    );

    return status;
}

#if defined(THREADED)
HAL_StatusTypeDef HalUartInterface::transmitIT(
    const UART_HandleTypeDef* uartHandlePtr,
    uint8_t* arrTransmit,
    size_t numBytes
) const
{
    HAL_StatusTypeDef status =  HAL_UART_Transmit_IT(
        const_cast<UART_HandleTypeDef*>(uartHandlePtr),
        arrTransmit,
        numBytes
    );

    return status;
}

HAL_StatusTypeDef HalUartInterface::receiveIT(
    const UART_HandleTypeDef* uartHandlePtr,
    uint8_t* arrReceive,
    size_t numBytes
) const
{
    HAL_StatusTypeDef status =  HAL_UART_Receive_IT(
        const_cast<UART_HandleTypeDef*>(uartHandlePtr),
        arrReceive,
        numBytes
    );

    return status;
}

HAL_StatusTypeDef HalUartInterface::transmitDMA(
    const UART_HandleTypeDef* uartHandlePtr,
    uint8_t* arrTransmit,
    size_t numBytes
) const
{
    HAL_StatusTypeDef status =  HAL_UART_Transmit_DMA(
        const_cast<UART_HandleTypeDef*>(uartHandlePtr),
        arrTransmit,
        numBytes
    );

    return status;
}

HAL_StatusTypeDef HalUartInterface::receiveDMA(
    const UART_HandleTypeDef* uartHandlePtr,
    uint8_t* arrReceive,
    size_t numBytes
) const
{
    HAL_StatusTypeDef status =  HAL_UART_Receive_DMA(
        const_cast<UART_HandleTypeDef*>(uartHandlePtr),
        arrReceive,
        numBytes
    );

    return status;
}

__IO uint32_t HalUartInterface::getDmaRxInstanceNDTR(
    const UART_HandleTypeDef* uartHandlePtr
) const
{
    return const_cast<UART_HandleTypeDef*>(uartHandlePtr)->hdmarx->Instance->NDTR;
}

#endif

void HalUartInterface::abortTransmit(
    const UART_HandleTypeDef* uartHandlePtr
) const
{
    HAL_UART_AbortTransmit(
        const_cast<UART_HandleTypeDef*>(uartHandlePtr)
    );
}

void HalUartInterface::abortReceive(
    const UART_HandleTypeDef* uartHandlePtr
) const
{
    HAL_UART_AbortReceive(
        const_cast<UART_HandleTypeDef*>(uartHandlePtr)
    );
}

__IO uint32_t HalUartInterface::getErrorCode(
    const UART_HandleTypeDef* uartHandlePtr
) const
{
    return const_cast<UART_HandleTypeDef*>(uartHandlePtr)->ErrorCode;
}

} // end namespace uart

/**
 * @}
 */
/* end - HalUartInterface */
