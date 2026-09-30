/* =========================================================================
 * main.c - Bare-metal ECG-only bring-up for the biomedical sensor shield
 * on the CC2652R1 LaunchPad. No RTOS: single superloop, one GPIO interrupt.
 *
 * Drop ads1292.c/.h, uart_stream.c/.h and pins.h into your existing CCS
 * project alongside this file (see firmware/README.md for the full
 * integration steps), then flash and open a serial terminal or the
 * gui/ Python app at 115200 baud.
 * ========================================================================= */
#include <stdint.h>
#include "ads1292.h"
#include "uart_stream.h"

/* If your CCS project's SysConfig-generated code already sets up the
 * system clock / power policy in a way that conflicts with the manual
 * PRCM calls in ads1292.c / uart_stream.c, call it here first. Most
 * empty/NoRTOS SimpleLink templates leave this to the startup file, in
 * which case there is nothing extra to do. */
static void SystemInit(void)
{
    /* Intentionally empty -- see the comment above. */
}

int main(void)
{
    SystemInit();

    UART_Init();
    ADS1292_Init();

    if (!ADS1292_SelfTest()) {
        /* Wiring/SPI problem: keep announcing it instead of doing nothing,
         * so it's visible from the GUI/terminal rather than a silent hang.
         * seq = 0xFFFF is reserved as the "error" marker; the GUI/terminal
         * side should treat it as diagnostic, not an ECG sample. */
        while (1) {
            UART_SendFrame(0xFFFF, ADS1292_LastId());
        }
    }

    ADS1292_StartContinuous();

    uint16_t seq = 0;
    while (1) {
        if (ADS1292_DrdyAsserted()) {
            int32_t sample = ADS1292_ReadSample();
            UART_SendFrame(seq++, sample);
        }
        /* Nothing else in the loop on purpose -- keep the DRDY-to-UART
         * path as short and predictable as possible for v1. */
    }
}
