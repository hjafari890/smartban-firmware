/* =========================================================================
 * ads1292.h - Minimal bare-metal ADS1292R driver, single ECG channel only.
 *
 * Channel 2 (respiration) is powered down -- this driver only brings up
 * channel 1 (ECG) with right-leg-drive enabled, which is the single biggest
 * lever for a clean signal (it actively cancels mains common-mode noise
 * instead of leaving it for software filtering later).
 * ========================================================================= */
#ifndef ADS1292_H
#define ADS1292_H

#include <stdint.h>
#include <stdbool.h>

#define ADS1292_SAMPLE_RATE_SPS   500

/* Configures SPI + control GPIOs, powers the 1V8 rail, resets the
 * ADS1292R, and programs it for single-channel ECG. Call once at startup,
 * before any other ADS1292_* function. */
void ADS1292_Init(void);

/* Reads the ID register as a wiring sanity check. Returns true if a
 * plausible (non-zero, non-0xFF) ID came back. Silicon revisions can
 * report slightly different exact values, so this checks "something
 * real answered" rather than one exact byte -- call ADS1292_LastId()
 * afterwards if you want to log/print the raw value. */
bool ADS1292_SelfTest(void);
uint8_t ADS1292_LastId(void);

/* Enters continuous-conversion mode (RDATAC) and raises START. After
 * this, DRDY pulses low once per sample period. */
void ADS1292_StartContinuous(void);

/* True exactly once per DRDY pulse; clears itself on read. The GPIO ISR
 * only sets a flag -- all SPI traffic happens in the main loop, not in
 * interrupt context. */
bool ADS1292_DrdyAsserted(void);

/* Call after ADS1292_DrdyAsserted() returns true. Does the SPI burst
 * read and returns the sign-extended 24-bit channel-1 sample. */
int32_t ADS1292_ReadSample(void);

#endif /* ADS1292_H */
