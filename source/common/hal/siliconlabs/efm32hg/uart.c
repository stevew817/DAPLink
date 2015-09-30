/* CMSIS-DAP Interface Firmware
 * Copyright (c) 2009-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "uart.h"
#include "IO_Config.h"
#include "string.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "em_gpio.h"

// Size must be 2^n for using quick wrap around
#define UART_BUFFER_SIZE    (32)

struct {
    uint8_t  data[UART_BUFFER_SIZE];
    volatile uint32_t idx_in;
    volatile uint32_t idx_out;
    volatile uint32_t cnt_in;
    volatile uint32_t cnt_out;
} write_buffer, read_buffer;

uint32_t tx_in_progress = 0;
uint32_t resetted = 1;

void clear_buffers(void)
{
    memset((void*)&read_buffer, 0xBB, sizeof(read_buffer.data));
    read_buffer.idx_in = 0;
    read_buffer.idx_out = 0;
    read_buffer.cnt_in = 0;
    read_buffer.cnt_out = 0;
    memset((void*)&write_buffer, 0xBB, sizeof(read_buffer.data));
    write_buffer.idx_in = 0;
    write_buffer.idx_out = 0;
    write_buffer.cnt_in = 0;
    write_buffer.cnt_out = 0;
}

void serial_baud(uint32_t baudrate);

int32_t uart_initialize(void)
{
    // enable clock to uart (HFPERCLK should already have the right selection)
    CMU_ClockEnable(UART_CLK, true);
  
    USART_Reset(UART);
    // set output location
    UART->ROUTE = (UART->ROUTE & _LEUART_ROUTE_LOCATION_MASK) | UART_LOC;
  
    clear_buffers();
    tx_in_progress = 0;
    return 1;
}

int32_t uart_uninitialize(void)
{
    USART_IntDisable(UART, USART_IF_TXBL | USART_IF_RXDATAV);
    USART_Enable(UART, usartDisable);
    
    clear_buffers();
    tx_in_progress = 0;
    return 1;
}

int32_t uart_reset(void)
{
    resetted = 1;

    // enable clock to uart (HFPERCLK should already have the right selection)
    CMU_ClockEnable(UART_CLK, true);
  
    // disable interrupt
    USART_IntDisable(UART, USART_IF_TXBL | USART_IF_RXDATAV);
    USART_Reset(UART);
    // set output location
    UART->ROUTE = (UART->ROUTE & _USART_ROUTE_LOCATION_MASK) | UART_LOC;
    // reset function
    clear_buffers();
    tx_in_progress = 0;
    
    return 1;
}

int32_t uart_set_configuration(UART_Configuration *config)
{
    USART_InitAsync_TypeDef internal_config = USART_INITASYNC_DEFAULT;
    // disable interrupt
    clear_buffers();
    
    switch(config->DataBits) {
      case UART_DATA_BITS_8:
      default:
        internal_config.databits = usartDatabits8;
        break;
    }
    
    switch(config->Parity) {
      case UART_PARITY_ODD:
        internal_config.parity = usartOddParity;
        break;
      case UART_PARITY_EVEN:
        internal_config.parity = usartEvenParity;
        break;
      case UART_PARITY_NONE:
      default:
        internal_config.parity = usartNoParity;
        break;
    }
    
    switch(config->StopBits) {
      case UART_STOP_BITS_1_5:
        internal_config.stopbits = usartStopbits1;
        break;
      case UART_STOP_BITS_2:
        internal_config.stopbits = usartStopbits2;
        break;
      case UART_STOP_BITS_1:
      default:
        internal_config.stopbits = usartStopbits1;
        break;
    }
    
    internal_config.enable = usartDisable;
    internal_config.baudrate = config->Baudrate;
    
    USART_Reset(UART);
    USART_InitAsync(UART, &internal_config);
    
    if(resetted == 0) {
    
      USART_Enable(UART, usartEnable);
      
      // Enable UART interrupt
      NVIC_EnableIRQ(UART_RX_IRQn);
			NVIC_EnableIRQ(UART_TX_IRQn);
      USART_IntEnable(UART, USART_IEN_RXDATAV);
      
      // enable GPIO pins used by leuart
      CMU_ClockEnable(cmuClock_GPIO, true);
      GPIO_PinModeSet(PIN_UART_RX_PORT, PIN_UART_RX_BIT, gpioModeInput, 0); 
      GPIO_PinModeSet(PIN_UART_TX_PORT, PIN_UART_TX_BIT, gpioModePushPull, 0);
    } else {
      resetted = 0;
    }
    return 1;
}

int32_t uart_get_configuration(UART_Configuration *config)
{
    return 1;
}

int32_t uart_write_free(void)
{
    return UART_BUFFER_SIZE - (write_buffer.cnt_in - write_buffer.cnt_out);
}

int32_t uart_write_data (uint8_t *data, uint16_t size)
{
    uint32_t cnt;
    int16_t  len_in_buf;
    if (size == 0) {
        return 0;
    }
    cnt = 0;
    while (size--) {
        len_in_buf = write_buffer.cnt_in - write_buffer.cnt_out;
        if (len_in_buf < UART_BUFFER_SIZE) {
            write_buffer.data[write_buffer.idx_in++] = *data++;
            write_buffer.idx_in &= (UART_BUFFER_SIZE - 1);
            write_buffer.cnt_in++;
            cnt++;
        }
    }
    if (!tx_in_progress) {
        // Wait for D register to be free
        while(!(USART_StatusGet(UART) & USART_STATUS_TXBL));
        tx_in_progress = 1;
        // Write the first byte into D
        USART_Tx(UART, write_buffer.data[write_buffer.idx_out++]);
        write_buffer.idx_out &= (UART_BUFFER_SIZE - 1);
        write_buffer.cnt_out++;
        // enable TX interrupt
        USART_IntEnable(UART, USART_IEN_TXBL);
    }
    return cnt;
}

int32_t uart_read_data(uint8_t *data, uint16_t size)
{
    uint32_t cnt;
    if (size == 0) {
        return 0;
    }
    
    cnt = 0;
    while (size--) {
        if (read_buffer.cnt_in != read_buffer.cnt_out) {
            *data++ = read_buffer.data[read_buffer.idx_out++];
            read_buffer.idx_out &= (UART_BUFFER_SIZE - 1);
            read_buffer.cnt_out++;
            cnt++;
        } else {
            break;
        }
    }
    return cnt;
}

void UART_TX_IRQHandler(void)
{
    uint32_t status;
    // read interrupt status
    status = USART_IntGet(UART);
    // handle character to transmit
    if (write_buffer.cnt_in != write_buffer.cnt_out) {
        // if TDRE is empty
        if (status & USART_IF_TXBL) {
            USART_Tx(UART, write_buffer.data[write_buffer.idx_out++]);
            write_buffer.idx_out &= (UART_BUFFER_SIZE - 1);
            write_buffer.cnt_out++;
            tx_in_progress = 1;
        }
    } else {
        // disable TX interrupt
        USART_IntDisable(UART, USART_IEN_TXBL);
        tx_in_progress = 0;
    }
}

void UART_RX_IRQHandler(void)
{
    uint32_t status;
    // read interrupt status
    status = USART_IntGet(UART);
    // handle received character
    if (status & USART_IF_RXDATAV) {
        read_buffer.data[read_buffer.idx_in++] = USART_Rx(UART);
        read_buffer.idx_in &= (UART_BUFFER_SIZE - 1);
        read_buffer.cnt_in++;
    }
}
