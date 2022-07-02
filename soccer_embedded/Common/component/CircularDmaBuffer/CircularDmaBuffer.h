/**
  *****************************************************************************
  * @file
  * @author  Robert
  *
  * @defgroup Header
  * @ingroup circular_dma_buffer
  * @{
  *****************************************************************************
  */

// TODO: data structures namespace
// TODO: think about overrun check and implement. try buffer with m_transmission_size != m_buff_size
//   tygamvrelis: Technically the buffer size is dependent only on the baud rate and how often the buffer is checked, not the transmission size


#ifndef CIRCULAR_DMA_BUFFER_H
#define CIRCULAR_DMA_BUFFER_H




/********************************* Includes **********************************/
#include "UartInterface.h"


using hal::UartInterface;


/***************************** CircularDmaBuffer ******************************/
namespace uart{
// Constants
// ----------------------------------------------------------------------------




// Types & enums
// ----------------------------------------------------------------------------


// Classes and structs
// ----------------------------------------------------------------------------
class CircularDmaBuffer{
public:
    CircularDmaBuffer();
    CircularDmaBuffer(
        const UART_HandleTypeDef *m_uart_handle,
        const UartInterface* m_hw_if,
        const uint16_t m_transmission_size,
        const size_t m_buff_size,
        uint8_t *m_buff_p
    );
    bool selfCheck() const;
    size_t updateHead();
    size_t catchupTail();
    bool dataAvail() const;
    size_t peekBuff(uint8_t *out_buff) const;
    size_t readBuff(uint8_t *out_buff);
    void initiate() const;
    void reinitiateIfError() const;

    const UART_HandleTypeDef* getUartHandle() const;
    const UartInterface* getHwIf() const;
    const uint16_t getTransmissionSize() const;
    const size_t getBuffSize() const;
    const uint8_t* getBuffP() const;
    size_t getBuffHead() const;
    size_t getBuffTail() const;
private:
    const UART_HandleTypeDef *m_uart_handle = nullptr;
    const UartInterface *m_hw_if = nullptr;
    const uint16_t m_transmission_size = 0;
    const size_t m_buff_size = 0;
    const uint8_t *m_buff_p = nullptr;
    size_t m_buff_head = 0;
    size_t m_buff_tail = 0;
};



// Functions
// ----------------------------------------------------------------------------


} // end namespace uart




/***************************** Inline functions ******************************/




/**
 * @}
 */
/* end - Header */

#endif /* TEMPLATE_CPP_H */
