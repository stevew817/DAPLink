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

#ifndef __IO_CONFIG_H__
#define __IO_CONFIG_H__

#include "em_device.h"

// Debug Port I/O Pins

// SWCLK Pin                    PD4
#define PIN_SWCLK_PORT          gpioPortD
//MODEL or MODEH depending on BIT
#define PIN_SWCLK_MODE          MODEL
#define PIN_SWCLK_GPIO          (3)
#define PIN_SWCLK_BIT           (4)
//_N is normalized bit (AND with 0xF)
#define PIN_SWCLK_BIT_N         (4)
#define PIN_SWCLK               (1<<PIN_SWCLK_BIT)

// SWDIO Pin                    PD5
#define PIN_SWDIO_PORT          gpioPortD
//MODEL or MODEH depending on BIT
#define PIN_SWDIO_MODE          MODEL
#define PIN_SWDIO_GPIO          (3)
#define PIN_SWDIO_BIT           (5)
//_N is normalized bit (AND with 0xF)
#define PIN_SWDIO_BIT_N         (5)
#define PIN_SWDIO               (1<<PIN_SWDIO_BIT)

// nRESET Pin                   PD6
#define PIN_nRESET_PORT         gpioPortD
//MODEL or MODEH depending on BIT
#define PIN_nRESET_MODE         MODEL
#define PIN_nRESET_GPIO         (3)
#define PIN_nRESET_BIT          (6)
//_N is normalized bit (AND with 0xF)
#define PIN_nRESET_BIT_N        (6)
#define PIN_nRESET              (1<<PIN_nRESET_BIT)

// Debug Unit LEDs

// HID_LED (LED0) PF4
#define PIN_HID_LED_PORT        gpioPortF
#define PIN_HID_LED_GPIO        (5)
#define PIN_HID_LED_BIT         (4)
#define PIN_HID_LED             (1<<PIN_HID_LED_BIT)

// MSC_LED (LED0) PF4
#define PIN_MSC_LED_PORT        gpioPortF
#define PIN_MSC_LED_GPIO        (5)
#define PIN_MSC_LED_BIT         (4)
#define PIN_MSC_LED             (1<<PIN_MSC_LED_BIT)

// CDC_LED (LED1) PF5
#define PIN_CDC_LED_PORT        gpioPortF
#define PIN_CDC_LED_GPIO        (5)
#define PIN_CDC_LED_BIT         (5)
#define PIN_CDC_LED             (1<<PIN_CDC_LED_BIT)

// SW RESET BUTTON (PB0) PC9
#define PIN_SW_RESET_PORT       gpioPortC
#define PIN_SW_RESET_GPIO       (2)
#define PIN_SW_RESET_BIT        (9)
#define PIN_SW_RESET            (1<<PIN_SW_RESET_BIT)

// Connected LED                Not available

// Target Running LED           Not available

// USB IRQ Macros
#define DISABLE_USB_IRQ()       NVIC_DisableIRQ(USB_IRQn)
#define ENABLE_USB_IRQ()        NVIC_EnableIRQ(USB_IRQn)

// UART
#define UART_NAME               (USART0)
// RX PE11
#define PIN_UART_RX_PORT		gpioPortE
#define PIN_UART_RX_MODE        MODEH
#define PIN_UART_RX_GPIO        (4)
#define PIN_UART_RX_BIT         (11)
#define PIN_UART_RX_BIT_N       (3)
#define PIN_UART_RX             (1<<PIN_UART_RX_BIT)
// TX PE10
#define PIN_UART_TX_PORT		gpioPortE
#define PIN_UART_TX_MODE        MODEH
#define PIN_UART_TX_GPIO        (4)
#define PIN_UART_TX_BIT         (10)
#define PIN_UART_TX_BIT_N       (2)
#define PIN_UART_TX             (1<<PIN_UART_TX_BIT)

#define UART_LOC                USART_ROUTE_LOCATION_LOC0

#define UART                    USART0
#define UART_HF_REF_FREQ        (24000000u)
#define UART_CLK                cmuClock_USART0
#define UART_RX_IRQn            USART0_RX_IRQn
#define UART_RX_IRQHandler      USART0_RX_IRQHandler
#define UART_TX_IRQn            USART0_TX_IRQn
#define UART_TX_IRQHandler      USART0_TX_IRQHandler


// GPIO macros
#define GPIO_SET_MODEL(port, pin, mode)      \
            GPIO->P[port].MODEL = (GPIO->P[port].MODEL & ~((uint32_t)0xF << (pin * 4))) | ((uint32_t)mode << (pin * 4))
#define GPIO_SET_MODEH(port, pin, mode)      \
            GPIO->P[port].MODEH = (GPIO->P[port].MODEL & ~((uint32_t)0xF << ((pin - 8) * 4))) | ((uint32_t)mode << ((pin - 8) * 4))

#define GPIO_MODE_INPUT         (0x1)
#define GPIO_MODE_INPUTPULL     (0x2)
#define GPIO_MODE_OUTPUT        (0x4)
#define GPIO_MODE_OUTPUTDRIVE   (0x5)
#define GPIO_MODE_DISABLED      (0x0)

#endif
