/**
  *****************************************************************************
  * @file   CircularDmaBuffer.cpp
  * @author Robert
  *
  * @defgroup circular_dma_buffer
  * @ingroup circular_dma_buffer
  * @brief Manages a circular buffer communicating using UART in circular DMA mode.
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "CircularDmaBuffer.h"




/******************************** File-local *********************************/
namespace{
// Constants
// ----------------------------------------------------------------------------




// Types & enums
// ----------------------------------------------------------------------------




// Classes and structs
// ----------------------------------------------------------------------------




// Variables
// ----------------------------------------------------------------------------




// Functions
// ----------------------------------------------------------------------------

size_t readBuffImpl(const uint8_t*   buff_p,
                    const size_t&    size,
                    const size_t&    head,
                    size_t&          tail,
                    uint8_t*         out_buff)
{
    size_t numReceived = 0;

    if (tail > head) {
        while (tail < size) {
            out_buff[numReceived++] = buff_p[tail++];
        }
        tail = 0;
    }

    while (tail < head) {
        out_buff[numReceived++] = buff_p[tail++];
    }

    return numReceived;
}

} // end anonymous namespace




namespace uart{
// Constants
// ----------------------------------------------------------------------------




/************************** insert class name here ***************************/
// Public
// ----------------------------------------------------------------------------
CircularDmaBuffer::CircularDmaBuffer() {

}

CircularDmaBuffer::CircularDmaBuffer(
    const UART_HandleTypeDef *uart_handle_in,
    const UartInterface* hw_if_in,
    const uint16_t transmission_size_in,
    const size_t buff_size_in,
    uint8_t *buff_p_in
) :
    m_uart_handle(uart_handle_in),
    m_hw_if(hw_if_in),
    m_transmission_size(transmission_size_in),
    m_buff_size(buff_size_in),
    m_buff_p(buff_p_in) {

}

/**
 * @brief Checks for failing conditions that make the buffer inoperable.
 * @return true if no failing conditions detected, false if one or more failing conditions are detected.
 *
 */
bool CircularDmaBuffer::selfCheck() const {
    bool ok = m_transmission_size == m_buff_size; // TODO: should support m_buff_size > m_transmission_size as well.
    return ok;
}

/**
 * @brief Updates m_buff_head to point to 1 past the last entry written to m_buffer_p by the DMA transfer.
 * @return The index that m_buff_head is at after updating.
 */
size_t CircularDmaBuffer::updateHead() {
    return (m_buff_head = m_buff_size - static_cast<size_t>(m_hw_if->getDmaRxInstanceNDTR(m_uart_handle))); // TODO: NDTR should come through the UartInterface so can test this
}

/**
 * @brief Sets m_buff_tail equal to m_buff_head, discarding any unread received bytes in m_buff_p.
 * @return the number of unread received bytes discarded.
 *
 */
size_t CircularDmaBuffer::catchupTail() {
    return (m_buff_tail = m_buff_head);
}

/**
 * @brief Checks if new received data is available to read in m_buff_p.
 * @return true if new data is available, no if not.
 */
bool CircularDmaBuffer::dataAvail() const {
    return m_buff_head != m_buff_tail;
}

/**
 * @brief Reads new data in m_buff_p into out_buff, without updating m_buff_tail.
 * @return number of bytes read from m_buff_p into out_buff.
 */
size_t CircularDmaBuffer::peekBuff(uint8_t *out_buff) const {
    size_t tailIdx = m_buff_tail;
    return readBuffImpl(m_buff_p, m_buff_size, m_buff_head, tailIdx, out_buff);
}

/**
 * @brief Reads new data in m_buff_p into out_buff, updating the m_buff_tail.
 * @return number of bytes read from m_buff_p into out_buff.
 */
size_t CircularDmaBuffer::readBuff(uint8_t *out_buff) {
    return readBuffImpl(m_buff_p, m_buff_size, m_buff_head, m_buff_tail, out_buff);
}

void CircularDmaBuffer::initiate() const {
    m_hw_if->receiveDMA(const_cast<UART_HandleTypeDef*>(m_uart_handle),
            const_cast<uint8_t*>(m_buff_p), m_transmission_size);
}

void CircularDmaBuffer::reinitiateIfError() const {
    if(m_uart_handle->ErrorCode != HAL_UART_ERROR_NONE){
        m_hw_if->abortReceive(const_cast<UART_HandleTypeDef*>(m_uart_handle));
        this->initiate();
    }
}

const UART_HandleTypeDef* CircularDmaBuffer::getUartHandle() const {
    return m_uart_handle;
}

const UartInterface* CircularDmaBuffer::getHwIf() const {
    return m_hw_if;
}

const uint16_t CircularDmaBuffer::getTransmissionSize() const {
    return m_transmission_size;
}

const size_t CircularDmaBuffer::getBuffSize() const {
    return m_buff_size;

}

const uint8_t* CircularDmaBuffer::getBuffP() const {
    return m_buff_p;
}

size_t CircularDmaBuffer::getBuffHead() const {
    return m_buff_head;
}

size_t CircularDmaBuffer::getBuffTail() const {
    return m_buff_tail;
}

// Protected
// ----------------------------------------------------------------------------




// Private
// ----------------------------------------------------------------------------


} // end namespace uart

// TODO: run atop of UartInterface, add unit tests


/**
 * @}
 */
/* end - circular_dma_buffer */
