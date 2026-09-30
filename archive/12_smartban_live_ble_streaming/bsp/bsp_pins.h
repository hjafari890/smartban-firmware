/*
 * ============================================================================
 * bsp_pins.h
 * Board Support Package - CC2652R1 LaunchPad & SmartBAN Shield Rev 3.5 Pinout
 * ============================================================================
 */

#ifndef BSP_PINS_H_
#define BSP_PINS_H_

#include <stdint.h>
#include <ti/drivers/GPIO.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/ioc.h>
#include "ti_drivers_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CC2652R1 Physical Device IO IDs (IOID_x) matching BoosterPack Headers
 */
#define BSP_DIO_UART_RX         IOID_2      /* DIO 2: UART2 Receive line (115200 baud) */
#define BSP_DIO_UART_TX         IOID_3      /* DIO 3: UART2 Transmit line (115200 baud) */
#define BSP_DIO_I2C_SCL         IOID_4      /* DIO 4: Shared I2C0 Clock (3.3V) */
#define BSP_DIO_I2C_SDA         IOID_5      /* DIO 5: Shared I2C0 Data (3.3V) */

#define BSP_DIO_SPI_SCLK        IOID_10     /* DIO 10: Shared SSI0 Serial Clock */
#define BSP_DIO_SPI_STD_MOSI    IOID_9      /* DIO 9: Standard MOSI / ADS1292 DIN / ADXL362 SDO (Swapped) */
#define BSP_DIO_SPI_STD_MISO    IOID_8      /* DIO 8: Standard MISO / ADS1292 DOUT / ADXL362 SDI (Swapped) */

#define BSP_DIO_ECG_CS          IOID_11     /* DIO 11: ADS1292R Chip Select (Active LOW) */
#define BSP_DIO_ECG_PWDN        IOID_22     /* DIO 22: ADS1292R Power Down / Reset (Active LOW) */
#define BSP_DIO_ECG_DRDY        IOID_23     /* DIO 23: ADS1292R Data Ready Interrupt (Falling edge) */
#define BSP_DIO_ECG_START       IOID_24     /* DIO 24: ADS1292R Conversion Start / Oscillator Enable */

#define BSP_DIO_IMU_CS          IOID_15     /* DIO 15: ADXL362 Chip Select (Active LOW) */
#define BSP_DIO_IMU_SW          IOID_18     /* DIO 18: IMU Q1 Discharge Switch Gate (Held LOW) */
#define BSP_DIO_IMU_INT1        IOID_26     /* DIO 26: ADXL362 Interrupt 1 (Activity / DRDY) */
#define BSP_DIO_IMU_INT2        IOID_27     /* DIO 27: ADXL362 Interrupt 2 (Inactivity / Awake) */

#define BSP_DIO_1V8_EN          IOID_21     /* DIO 21: AP2112K-1.8 LDO Enable (Active HIGH) */
#define BSP_DIO_I2C_EN          IOID_30     /* DIO 30: PCA9306 Level Translator Enable (Active HIGH) */

#define BSP_DIO_OPT_INT         IOID_25     /* DIO 25: OPT4041 Ambient Light Threshold Interrupt */
#define BSP_DIO_VCNL_INT        IOID_12     /* DIO 12: VCNL4040 Proximity Interrupt */
#define BSP_DIO_BTN_INT         IOID_29     /* DIO 29: PCAL6408A Button Expander Interrupt */

#define BSP_DIO_LED_RED         IOID_6      /* DIO 6: CC26X2R1 LaunchPad LED0 (Red) */
#define BSP_DIO_LED_GREEN       IOID_7      /* DIO 7: CC26X2R1 LaunchPad LED1 (Green) */

/*
 * NOTE: MAX32664C / MAXM86161 Optical Biometric Subsystem is EXCLUDED
 * due to known PCB layout flaw (series resistors on private I2C bus and
 * misrouted MFIO boot pin on Header J2). DIO 19 and DIO 28 are unassigned.
 */

#ifdef __cplusplus
}
#endif

#endif /* BSP_PINS_H_ */
