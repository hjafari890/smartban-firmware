#include <ti/devices/cc13x2_cc26x2/driverlib/ioc.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/uart.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/prcm.h>
#include <ti/devices/cc13x2_cc26x2/inc/hw_memmap.h>

#include "pins.h"
#include "uart_stream.h"

#define UART_BAUD 115200

void UART_Init(void)
{
    PRCMPeripheralRunEnable(PRCM_PERIPH_UART0);
    PRCMLoadSet();
    while (!PRCMLoadGet()) {}

    IOCPinTypeUart(UART0_BASE, PIN_UART_RX, PIN_UART_TX,
                   IOID_UNUSED, IOID_UNUSED);

    UARTConfigSetExpClk(UART0_BASE, 48000000, UART_BAUD,
                         UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART0_BASE);
    UARTEnable(UART0_BASE);
}

void UART_SendFrame(uint16_t seq, int32_t sample)
{
    uint8_t b[8];
    b[0] = 0xAA;
    b[1] = 0x55;
    b[2] = (uint8_t)(seq >> 8);
    b[3] = (uint8_t)(seq & 0xFF);
    b[4] = (uint8_t)((sample >> 16) & 0xFF);
    b[5] = (uint8_t)((sample >> 8) & 0xFF);
    b[6] = (uint8_t)(sample & 0xFF);
    b[7] = b[2] ^ b[3] ^ b[4] ^ b[5] ^ b[6];

    for (int i = 0; i < 8; i++) {
        UARTCharPut(UART0_BASE, b[i]); /* blocking; fine at this data rate/sample period */
    }
}
