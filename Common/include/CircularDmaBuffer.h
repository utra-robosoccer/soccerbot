/**
  *****************************************************************************
  * @file    CircularDmaBuffer.h
  * @author  Robert
  *
  * @defgroup Header
  * @ingroup circular_dma_buffer
  * @{
  *****************************************************************************
  */




#ifndef CIRCULAR_DMA_BUFFER_H
#define CIRCULAR_DMA_BUFFER_H




/********************************* Includes **********************************/
#include "UartInterface.h"



/************************** insert module name here **************************/
namespace uart{
// Constants
// ----------------------------------------------------------------------------




// Types & enums
// ----------------------------------------------------------------------------


// Classes and structs
// ----------------------------------------------------------------------------
class CircularDmaBuffer{
public:
    CircularDmaBuffer(
        UART_HandleTypeDef *uart_handle_in,
        const UartInterface* hw_if_in,
        uint16_t transmission_size_in,
        size_t buff_size_in,
        uint8_t *buff_p_in
    );
    bool selfCheck() const;
    bool overrunCheck() const;
    size_t updateHead();
    size_t catchupTail();
    bool dataAvail() const;
    size_t peekBuff(uint8_t *out_buff) const;
    size_t readBuff(uint8_t *out_buff);
    void initiate();
    void restartIfError();
private:
    UART_HandleTypeDef *m_uart_handle = nullptr;
    const UartInterface* m_hw_if = nullptr;
    uint16_t m_transmission_size = 0;
    size_t m_buff_size = 0;
    uint8_t *m_buff_p = nullptr;
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
