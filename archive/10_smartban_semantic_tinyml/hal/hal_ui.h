/*
 * ============================================================================
 * hal_ui.h
 * Hardware Abstraction Layer - NXP PCAL6408A 6-Button Expander & WCH CH455H UI
 * ============================================================================
 */

#ifndef HAL_UI_H_
#define HAL_UI_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PCAL6408A I2C Address and Registers */
#define PCAL6408A_I2C_ADDR          0x20
#define PCAL6408A_REG_INPUT         0x00
#define PCAL6408A_REG_OUTPUT        0x01
#define PCAL6408A_REG_POLARITY      0x02
#define PCAL6408A_REG_CONFIG        0x03
#define PCAL6408A_REG_OUT_DRV0      0x40
#define PCAL6408A_REG_IN_LATCH      0x42
#define PCAL6408A_REG_PUD_EN        0x43
#define PCAL6408A_REG_PUD_SEL       0x44
#define PCAL6408A_REG_INT_MASK      0x45

/* Tactile Buttons Mapping (Active LOW on hardware, active HIGH after inversion) */
#define HAL_UI_BTN_SW1_ECG          (1 << 0) /* P0: SW1 ECG Waveform view */
#define HAL_UI_BTN_SW6_MODE         (1 << 1) /* P1: SW6 Mode / Stream Toggle */
#define HAL_UI_BTN_SW3_PPG          (1 << 2) /* P2: SW3 PPG view */
#define HAL_UI_BTN_SW5_RESET        (1 << 3) /* P3: SW5 System reset / re-init */
#define HAL_UI_BTN_SW2_IR           (1 << 4) /* P4: SW2 FIR Temperature view */
#define HAL_UI_BTN_SW4_BT           (1 << 5) /* P5: SW4 IMU Zero-G Tare / BT */
#define HAL_UI_BTN_MASK_ALL         0x3F

/* CH455H 7-Segment & LED Driver I2C Command Addresses */
#define CH455_CMD_SYS               0x24 /* System command address (0x71 = Display ON, max brightness) */
#define CH455_DIG0_ADDR             0x34 /* Hundreds digit */
#define CH455_DIG1_ADDR             0x35 /* Tens digit (Bit 7 = Decimal Point) */
#define CH455_DIG2_ADDR             0x36 /* Ones digit */
#define CH455_DIG3_ADDR             0x37 /* Mode LEDs: Bit 0=LED0, Bit 1=LED1, Bit 2=LED2, Bit 3=LED3 */

#define CH455_LED_0                 (1 << 0)
#define CH455_LED_1                 (1 << 1)
#define CH455_LED_2                 (1 << 2)
#define CH455_LED_3                 (1 << 3)

/**
 * @brief Initialize both PCAL6408A button expander and CH455H display controller.
 * @return true if initialized successfully.
 */
bool hal_ui_init(void);

/**
 * @brief Poll and debounce tactile buttons.
 *        Updates internal debounced state and generates single-cycle press/release events.
 * @param current_mask Output currently held buttons mask.
 * @param pressed_edges Output single-shot button press edge events.
 * @param released_edges Output single-shot button release edge events.
 * @return true on valid read.
 */
bool hal_ui_poll_buttons(uint8_t *current_mask, uint8_t *pressed_edges, uint8_t *released_edges);

/**
 * @brief Quick read of currently debounced button mask.
 */
uint8_t hal_ui_get_buttons(void);

/**
 * @brief Display an integer number (0..999) across the 3-digit 7-segment display.
 * @param val Value to display. Values > 999 will display "---".
 */
void hal_ui_display_number(uint16_t val);

/**
 * @brief Display clinical Heart Rate (BPM) on CH455H 3-digit 7-segment display
 *        with synchronized heartbeat decimal-point & 4-LED flash (from 08_ecg_dedicated).
 */
void hal_ui_display_bpm_beat(uint16_t bpm_val, bool flash_on);

/**
 * @brief Display a 1-decimal floating point number (e.g. 36.7) with decimal point on DIG1.
 * @param val Float value to display (e.g. 0.0 to 99.9).
 */
void hal_ui_display_float1(float val);

/**
 * @brief Display 3 alphanumeric characters on the 7-segment display (e.g. "tEP", "PrS", "ECG", "CAL").
 * @param c0 Hundreds digit character.
 * @param c1 Tens digit character.
 * @param c2 Ones digit character.
 */
void hal_ui_display_text(char c0, char c1, char c2);

/**
 * @brief Control the 4 discrete status LEDs on DIG3.
 * @param leds Bitmask of LEDs to enable (bits 0..3).
 */
void hal_ui_set_mode_leds(uint8_t leds);

/**
 * @brief Blank all 7-segment digits and turn off mode LEDs.
 */
void hal_ui_clear_display(void);

/**
 * @brief Put CH455H display controller into ultra-low-power sleep mode.
 */
void hal_ui_sleep(void);

/**
 * @brief Wake CH455H display controller from sleep mode and enable display.
 */
void hal_ui_wake(void);

/**
 * @brief Get font pattern for a given character.
 */
uint8_t hal_ui_char_to_segment(char c);

#ifdef __cplusplus
}
#endif

#endif /* HAL_UI_H_ */
