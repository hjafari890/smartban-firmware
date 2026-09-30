/* =========================================================================
 * uart_stream.h - Fixed 8-byte binary frame per ECG sample, sent to the PC.
 *
 * Frame layout (matches gui/serial_reader.py -- keep both sides in sync):
 *
 *   byte 0   : 0xAA  (sync)
 *   byte 1   : 0x55  (sync)
 *   byte 2-3 : sequence number, uint16, big-endian, wraps at 65535
 *   byte 4-6 : ECG sample, int24, big-endian, two's complement
 *   byte 7   : checksum = byte2 ^ byte3 ^ byte4 ^ byte5 ^ byte6
 *
 * At 500 SPS this is 4000 bytes/s -- comfortably inside 115200 baud
 * (~11.5 kB/s), leaving headroom for UART framing overhead.
 * ========================================================================= */
#ifndef UART_STREAM_H
#define UART_STREAM_H

#include <stdint.h>

void UART_Init(void);
void UART_SendFrame(uint16_t seq, int32_t sample);

#endif /* UART_STREAM_H */
