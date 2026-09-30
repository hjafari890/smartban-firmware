/* =========================================================================
 * pins.h - CC2652R1 (LAUNCHXL-CC26X2R1) pin map for the ECG-only bring-up
 *
 * These are DIO/IOID numbers, i.e. the argument you pass to DriverLib GPIO
 * and IOC functions -- NOT physical LaunchPad header pin numbers.
 *
 * The ADS1292R + shared-bus lines below come from the verified trace of the
 * shield netlist / EasyEDA schematic. The UART pair is the standard XDS110
 * backchannel used on essentially every CC13xx/CC26xx LaunchPad -- it was
 * NOT re-verified against this specific shield, so double check it against
 * your LAUNCHXL-CC26X2R1 pinout diagram before relying on it.
 * ========================================================================= */
#ifndef PINS_H
#define PINS_H

#include <ti/devices/cc13x2_cc26x2/driverlib/ioc.h>

/* ---- SPI bus (shared with the onboard ADXL362 IMU) ---------------------- */
#define PIN_SPI_SCLK        IOID_10   /* Shield J1.14 (via R63) */
#define PIN_SPI_MOSI        IOID_9    /* Shield J2.11 (via R62) */
#define PIN_SPI_MISO        IOID_8    /* Shield J2.13 (direct)  */

/* ---- ADS1292R control lines ---------------------------------------------- */
#define PIN_ECG_CS           IOID_11   /* Shield J2.5  (via R61 jumper)     */
#define PIN_ECG_DRDY         IOID_23   /* Shield J1.4                       */
#define PIN_ECG_START        IOID_24   /* Shield J1.12                      */
#define PIN_ECG_PWDN         IOID_22   /* Shield J1.10 (PWDN/RESET#)        */

/* ---- Board-level lines that MUST be driven for the ECG path to work ----- */
#define PIN_1V8_EN           IOID_21   /* Shield J1.16 - enables the ADS1292R LDO rail */
#define PIN_IMU_CS            IOID_15   /* ADXL362 CS - hold HIGH so it stays off the shared SPI bus */

/* ---- UART to PC (XDS110 backchannel, standard on CC13xx/CC26xx LaunchPads,
 * NOT independently re-verified for this shield -- confirm against your
 * board's pinout diagram) --------------------------------------------------- */
#define PIN_UART_TX           IOID_3
#define PIN_UART_RX           IOID_2

#endif /* PINS_H */
