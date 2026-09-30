/*
 * ============================================================================
 * hal_ui.c
 * Hardware Abstraction Layer - NXP PCAL6408A & WCH CH455H Implementation
 * ============================================================================
 */

#include "hal_ui.h"
#include "../bsp/bsp_pins.h"
#include "../bsp/bsp_power.h"
#include "../bsp/bsp_i2c.h"
#include <unistd.h>

static bool    s_pcal_online = false;
static bool    s_ch455_online = false;

static uint8_t s_debounced_state = 0;
static uint8_t s_prev_state = 0;
static uint8_t s_raw_prev = 0;
static uint8_t s_debounce_count = 0;

/* 7-Segment Digit Font (0..9) */
static const uint8_t s_digit_font[10] = {
    0x3F, /* 0: ABCDEF  */
    0x06, /* 1: BC      */
    0x5B, /* 2: ABDEG   */
    0x4F, /* 3: ABCDG   */
    0x66, /* 4: BCFG    */
    0x6D, /* 5: ACDFG   */
    0x7D, /* 6: ACDEFG  */
    0x07, /* 7: ABC     */
    0x7F, /* 8: ABCDEFG */
    0x6F  /* 9: ABCDFG  */
};

uint8_t hal_ui_char_to_segment(char c)
{
    if (c >= '0' && c <= '9') {
        return s_digit_font[c - '0'];
    }

    switch (c) {
        case 'A': case 'a': return 0x77;
        case 'B': case 'b': return 0x7C;
        case 'C':           return 0x39;
        case 'c':           return 0x58;
        case 'D': case 'd': return 0x5E;
        case 'E': case 'e': return 0x79;
        case 'F': case 'f': return 0x71;
        case 'G': case 'g': return 0x3D;
        case 'H':           return 0x76;
        case 'h':           return 0x74;
        case 'I': case 'i': return 0x06;
        case 'J': case 'j': return 0x1E;
        case 'L': case 'l': return 0x38;
        case 'N': case 'n': return 0x54;
        case 'O': case 'o': return 0x5C;
        case 'P': case 'p': return 0x73;
        case 'R': case 'r': return 0x50;
        case 'S': case 's': return 0x6D;
        case 'T': case 't': return 0x78;
        case 'U':           return 0x3E;
        case 'u':           return 0x1C;
        case 'Y': case 'y': return 0x6E;
        case '-':           return 0x40;
        case '_':           return 0x08;
        case ' ':           return 0x00;
        default:            return 0x00;
    }
}

bool hal_ui_init(void)
{
    bsp_power_set_i2c_shifter(true);
    bsp_i2c_init();

    /* 1. Configure PCAL6408A 8-Bit GPIO Expander */
    uint8_t test_val = 0;
    s_pcal_online = false;
    for (int retry = 0; retry < 3; retry++) {
        if (bsp_i2c_write_reg8(PCAL6408A_I2C_ADDR, PCAL6408A_REG_CONFIG, 0xFF)) {
            /* Disable input latching on P0..P5 so power-up rail glitches never latch false presses */
            bsp_i2c_write_reg8(PCAL6408A_I2C_ADDR, PCAL6408A_REG_IN_LATCH, 0x00);
            /* Enable pull-up/pull-down on P0..P5 */
            bsp_i2c_write_reg8(PCAL6408A_I2C_ADDR, PCAL6408A_REG_PUD_EN, 0x3F);
            /* Select pull-UP resistors (active-low button wiring) */
            bsp_i2c_write_reg8(PCAL6408A_I2C_ADDR, PCAL6408A_REG_PUD_SEL, 0x3F);
            /* Mask interrupts on all pins (polled at 20 Hz) */
            bsp_i2c_write_reg8(PCAL6408A_I2C_ADDR, PCAL6408A_REG_INT_MASK, 0xFF);
            /* Read input register twice to clear any power-on state */
            bsp_i2c_read_reg8(PCAL6408A_I2C_ADDR, PCAL6408A_REG_INPUT, &test_val);
            bsp_i2c_read_reg8(PCAL6408A_I2C_ADDR, PCAL6408A_REG_INPUT, &test_val);
            s_pcal_online = true;
            break;
        }
        usleep(10000);
    }

    /* 2. Configure CH455H 7-Segment & LED Driver */
    /* System command 0x71: Enable display, 100% brightness, no sleep */
    if (bsp_i2c_write_cmd(CH455_CMD_SYS, 0x71)) {
        s_ch455_online = true;
        hal_ui_clear_display();
    } else {
        s_ch455_online = false;
    }

    /* Initialize state to current physical pin levels so boot never triggers a rising edge */
    uint8_t init_raw = 0xFF;
    if (s_pcal_online && bsp_i2c_read_reg8(PCAL6408A_I2C_ADDR, PCAL6408A_REG_INPUT, &init_raw)) {
        uint8_t init_curr = (~init_raw) & HAL_UI_BTN_MASK_ALL;
        s_debounced_state = init_curr;
        s_prev_state = init_curr;
        s_raw_prev = init_curr;
    } else {
        s_debounced_state = 0;
        s_prev_state = 0;
        s_raw_prev = 0;
    }
    s_debounce_count = 0;

    return (s_pcal_online || s_ch455_online);
}

bool hal_ui_poll_buttons(uint8_t *current_mask, uint8_t *pressed_edges, uint8_t *released_edges)
{
    if (!s_pcal_online) {
        uint8_t test = 0;
        if (bsp_i2c_read_reg8(PCAL6408A_I2C_ADDR, PCAL6408A_REG_INPUT, &test)) {
            s_pcal_online = true;
        } else {
            if (current_mask) *current_mask = 0;
            if (pressed_edges) *pressed_edges = 0;
            if (released_edges) *released_edges = 0;
            return false;
        }
    }

    uint8_t raw = 0xFF;
    if (!bsp_i2c_read_reg8(PCAL6408A_I2C_ADDR, PCAL6408A_REG_INPUT, &raw)) {
        return false;
    }

    /* Invert bits P0..P5: Hardware buttons pull to ground (active LOW) */
    uint8_t curr = (~raw) & HAL_UI_BTN_MASK_ALL;

    /* Require 2 consecutive matching 50ms polls (100ms debounce) to reject I2C/power glitches */
    if (curr == s_raw_prev) {
        s_debounced_state = curr;
    }
    s_raw_prev = curr;

    uint8_t pressed = (s_debounced_state & ~s_prev_state);
    uint8_t released = (~s_debounced_state & s_prev_state);
    s_prev_state = s_debounced_state;

    if (current_mask) *current_mask = s_debounced_state;
    if (pressed_edges) *pressed_edges = pressed;
    if (released_edges) *released_edges = released;

    return true;
}

uint8_t hal_ui_get_buttons(void)
{
    return s_debounced_state;
}

void hal_ui_display_number(uint16_t val)
{
    if (!s_ch455_online) return;

    if (val > 999) {
        hal_ui_display_text('-', '-', '-');
        return;
    }

    uint8_t h = (uint8_t)(val / 100);
    uint8_t t = (uint8_t)((val % 100) / 10);
    uint8_t o = (uint8_t)(val % 10);

    uint8_t seg0 = (h > 0) ? s_digit_font[h] : 0x00; /* Blank leading zero on hundreds */
    uint8_t seg1 = (h > 0 || t > 0) ? s_digit_font[t] : 0x00; /* Blank leading zero on tens */
    uint8_t seg2 = s_digit_font[o];

    bsp_i2c_write_cmd(CH455_DIG0_ADDR, seg0);
    bsp_i2c_write_cmd(CH455_DIG1_ADDR, seg1);
    bsp_i2c_write_cmd(CH455_DIG2_ADDR, seg2);
}

void hal_ui_display_bpm_beat(uint16_t bpm_val, bool flash_on)
{
    if (!s_ch455_online) return;
    uint8_t seg0, seg1, seg2;
    if (bpm_val < 30U || bpm_val > 220U) {
        /* Waiting for rhythm lock: show '---' */
        seg0 = 0x40U | (flash_on ? 0x80U : 0x00U);
        seg1 = 0x40U | (flash_on ? 0x80U : 0x00U);
        seg2 = 0x40U | (flash_on ? 0x80U : 0x00U);
    } else {
        uint16_t val = bpm_val % 1000U;
        uint8_t h = (uint8_t)(val / 100U);
        uint8_t t = (uint8_t)((val % 100U) / 10U);
        uint8_t o = (uint8_t)(val % 10U);

        /* Blank leading hundreds digit when BPM < 100 for clean clinical readout (e.g. ' 72') */
        seg0 = (h > 0U ? s_digit_font[h] : 0x00U) | (flash_on ? 0x80U : 0x00U);
        seg1 = s_digit_font[t] | (flash_on ? 0x80U : 0x00U);
        seg2 = s_digit_font[o] | (flash_on ? 0x80U : 0x00U);
    }

    bsp_i2c_write_cmd(CH455_DIG0_ADDR, seg0);
    bsp_i2c_write_cmd(CH455_DIG1_ADDR, seg1);
    bsp_i2c_write_cmd(CH455_DIG2_ADDR, seg2);
    /* Flash all 4 discrete LEDs on beat (0x0F), or keep LED0 active (0x01) */
    bsp_i2c_write_cmd(CH455_DIG3_ADDR, flash_on ? 0x0FU : 0x01U);
}

void hal_ui_display_float1(float val)
{
    if (!s_ch455_online) return;

    if (val < 0.0f || val >= 100.0f) {
        hal_ui_display_text('-', '-', '-');
        return;
    }

    uint16_t scaled = (uint16_t)(val * 10.0f + 0.5f);
    uint8_t t = (uint8_t)(scaled / 100);
    uint8_t o = (uint8_t)((scaled % 100) / 10);
    uint8_t f = (uint8_t)(scaled % 10);

    uint8_t seg0 = (t > 0) ? s_digit_font[t] : 0x00;
    uint8_t seg1 = s_digit_font[o] | 0x80; /* Bit 7 enables Decimal Point on digit 1 */
    uint8_t seg2 = s_digit_font[f];

    bsp_i2c_write_cmd(CH455_DIG0_ADDR, seg0);
    bsp_i2c_write_cmd(CH455_DIG1_ADDR, seg1);
    bsp_i2c_write_cmd(CH455_DIG2_ADDR, seg2);
}

void hal_ui_display_text(char c0, char c1, char c2)
{
    if (!s_ch455_online) return;

    bsp_i2c_write_cmd(CH455_DIG0_ADDR, hal_ui_char_to_segment(c0));
    bsp_i2c_write_cmd(CH455_DIG1_ADDR, hal_ui_char_to_segment(c1));
    bsp_i2c_write_cmd(CH455_DIG2_ADDR, hal_ui_char_to_segment(c2));
}

void hal_ui_set_mode_leds(uint8_t leds)
{
    if (!s_ch455_online) return;
    /* DIG3 (0x37) drives the 4 discrete mode LEDs on bits 0..3 */
    bsp_i2c_write_cmd(CH455_DIG3_ADDR, leds & 0x0F);
}

void hal_ui_clear_display(void)
{
    if (!s_ch455_online) return;

    /* Blank all segments and mode LEDs (display controller remains active) */
    bsp_i2c_write_cmd(CH455_DIG0_ADDR, 0x00);
    bsp_i2c_write_cmd(CH455_DIG1_ADDR, 0x00);
    bsp_i2c_write_cmd(CH455_DIG2_ADDR, 0x00);
    bsp_i2c_write_cmd(CH455_DIG3_ADDR, 0x00);
}

void hal_ui_sleep(void)
{
    if (!s_ch455_online) return;

    /* Blank all segments and discrete mode LEDs */
    bsp_i2c_write_cmd(CH455_DIG0_ADDR, 0x00);
    bsp_i2c_write_cmd(CH455_DIG1_ADDR, 0x00);
    bsp_i2c_write_cmd(CH455_DIG2_ADDR, 0x00);
    bsp_i2c_write_cmd(CH455_DIG3_ADDR, 0x00);

    /* Command CH455H internal driver into low-power sleep / oscillator off */
    bsp_i2c_write_cmd(CH455_CMD_SYS, 0x00);
}

void hal_ui_wake(void)
{
    if (!s_ch455_online) return;

    /* Restore CH455H normal display operation: 100% brightness, no sleep */
    bsp_i2c_write_cmd(CH455_CMD_SYS, 0x71);
}
