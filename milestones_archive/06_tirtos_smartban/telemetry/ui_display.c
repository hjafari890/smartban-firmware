/*
 * ============================================================================
 * ui_display.c
 * SmartBAN Sensor Node Firmware - CH455 7-Segment & LED Visualization Engine
 * Platform: CC2652R1 LaunchPad + SmartBAN Shield Rev 3.5 (SDK 8.33, TI-RTOS7)
 * Milestone: M4 (Serial Testbed Interface, CLI & Visual UI)
 * ============================================================================
 */

#include "telemetry/ui_display.h"
#include "hal/hal_ui.h"
#include "bsp/bsp_i2c.h"
#include <string.h>
#include <stdio.h>

/* Enhanced 7-Segment Font with Synthesized 'M' and 'W' Glyphs */
uint8_t ui_display_char_to_seg(char c)
{
    if (c >= '0' && c <= '9') {
        return hal_ui_char_to_segment(c);
    }
    switch (c) {
        case 'M': case 'm': return 0x37; /* Inverted U with double uprights (Standard 7-seg M: A,B,C,E,F) */
        case 'W': case 'w': return 0x3E; /* Wide bottom U (Standard 7-seg W: B,C,D,E,F) */
        default:            return hal_ui_char_to_segment(c);
    }
}

void ui_display_init(ui_display_state_t *state, const ui_display_config_t *cfg)
{
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(ui_display_state_t));

    if (cfg != NULL) {
        state->config = *cfg;
    } else {
        state->config.beat_pulse_ticks  = 2U;  /* 2 ticks @ 10 Hz = 200 ms */
        state->config.alarm_flash_ticks = 3U;  /* 3 ticks @ 10 Hz = ~1.67 Hz */
        state->config.alarm_hold_ticks  = 30U; /* 30 ticks @ 10 Hz = 3.0 s */
    }

    state->active_view  = UI_VIEW_HR;
    state->active_alarm = UI_ALARM_NONE;
    state->flash_phase  = true;
}

void ui_display_reset(ui_display_state_t *state)
{
    if (state == NULL) {
        return;
    }
    ui_display_config_t cfg = state->config;
    ui_display_init(state, &cfg);
}

void ui_display_set_view(ui_display_state_t *state, ui_view_mode_t view)
{
    if (state == NULL) {
        return;
    }
    state->active_view = view;
}

ui_view_mode_t ui_display_get_view(const ui_display_state_t *state)
{
    return (state != NULL) ? state->active_view : UI_VIEW_HR;
}

ui_view_mode_t ui_display_cycle_view(ui_display_state_t *state)
{
    if (state == NULL) {
        return UI_VIEW_HR;
    }

    switch (state->active_view) {
        case UI_VIEW_HR:     state->active_view = UI_VIEW_TEMP;   break;
        case UI_VIEW_TEMP:   state->active_view = UI_VIEW_STATUS; break;
        case UI_VIEW_STATUS: state->active_view = UI_VIEW_HR;     break;
        default:             state->active_view = UI_VIEW_HR;     break;
    }

    /* Manually cycling views dismisses any latched alarm view hold */
    state->alarm_hold_timer = 0;
    return state->active_view;
}

void ui_display_notify_beat(ui_display_state_t *state)
{
    if (state == NULL) {
        return;
    }
    state->beat_pulse_timer = state->config.beat_pulse_ticks;
}

void ui_display_render_text(const char *text, int8_t dp_digit)
{
    if (text == NULL) {
        return;
    }

    uint8_t segs[3] = {0, 0, 0};
    size_t len = strlen(text);

    if (len >= 3) {
        segs[0] = ui_display_char_to_seg(text[0]);
        segs[1] = ui_display_char_to_seg(text[1]);
        segs[2] = ui_display_char_to_seg(text[2]);
    } else if (len == 2) {
        segs[0] = 0x00; /* Blank leading */
        segs[1] = ui_display_char_to_seg(text[0]);
        segs[2] = ui_display_char_to_seg(text[1]);
    } else if (len == 1) {
        segs[0] = 0x00;
        segs[1] = 0x00;
        segs[2] = ui_display_char_to_seg(text[0]);
    }

    if (dp_digit >= 0 && dp_digit <= 2) {
        segs[dp_digit] |= 0x80; /* Bit 7 enables Decimal Point */
    }

    bsp_i2c_write_cmd(CH455_DIG0_ADDR, segs[0]);
    bsp_i2c_write_cmd(CH455_DIG1_ADDR, segs[1]);
    bsp_i2c_write_cmd(CH455_DIG2_ADDR, segs[2]);
}

void ui_display_set_leds(uint8_t led_mask)
{
    hal_ui_set_mode_leds(led_mask);
}

void ui_display_clear(void)
{
    hal_ui_clear_display();
}

void ui_display_update(ui_display_state_t *state, const ui_display_data_t *data)
{
    if (state == NULL || data == NULL) {
        return;
    }

    state->total_ticks++;

    /* 1. Flash phase generation for emergency alarm display and LED 3 */
    state->flash_counter++;
    if (state->flash_counter >= state->config.alarm_flash_ticks) {
        state->flash_counter = 0;
        state->flash_phase = !state->flash_phase;
    }

    /* 2. Beat pulse countdown for dynamic decimal point pulse */
    bool dp_active = false;
    if (data->qrs_beat_event) {
        state->beat_pulse_timer = state->config.beat_pulse_ticks;
    }
    if (state->beat_pulse_timer > 0) {
        state->beat_pulse_timer--;
        dp_active = true;
    }

    /* 3. Alarm Evaluation & Priority Override */
    ui_alarm_type_t current_alarm = UI_ALARM_NONE;
    if (data->fall_detected) {
        current_alarm = UI_ALARM_FALL;
    } else if (data->alert_mask & ALERT_FLAG_TACHYCARDIA) {
        current_alarm = UI_ALARM_TACHYCARDIA;
    } else if (data->alert_mask & ALERT_FLAG_BRADYCARDIA) {
        current_alarm = UI_ALARM_BRADYCARDIA;
    } else if (data->alert_mask & ALERT_FLAG_ARRHYTHMIA) {
        current_alarm = UI_ALARM_ARRHYTHMIA;
    } else if (data->alert_mask & ALERT_FLAG_FEVER) {
        current_alarm = UI_ALARM_FEVER;
    } else if (data->alert_mask & ALERT_FLAG_HYPOTHERMIA) {
        current_alarm = UI_ALARM_HYPOTHERMIA;
    }

    if (current_alarm != UI_ALARM_NONE) {
        state->active_alarm = current_alarm;
        state->alarm_hold_timer = state->config.alarm_hold_ticks;
    } else if (state->alarm_hold_timer > 0) {
        state->alarm_hold_timer--;
    } else {
        state->active_alarm = UI_ALARM_NONE;
    }

    /* 4. Display View Rendering */
    if (state->active_alarm != UI_ALARM_NONE) {
        /* Mode 3: Emergency Alert Override */
        const char *alarm_str = "ALr";
        switch (state->active_alarm) {
            case UI_ALARM_FALL:        alarm_str = "FAL"; break;
            case UI_ALARM_TACHYCARDIA: alarm_str = "tAC"; break;
            case UI_ALARM_BRADYCARDIA: alarm_str = "brA"; break;
            case UI_ALARM_ARRHYTHMIA:  alarm_str = "Arr"; break;
            case UI_ALARM_FEVER:       alarm_str = "Hot"; break;
            case UI_ALARM_HYPOTHERMIA: alarm_str = "CLd"; break;
            default:                   alarm_str = "ALr"; break;
        }

        if (state->flash_phase) {
            ui_display_render_text(alarm_str, -1);
        } else {
            /* Flashing OFF phase: blank 7-segment display */
            bsp_i2c_write_cmd(CH455_DIG0_ADDR, 0x00);
            bsp_i2c_write_cmd(CH455_DIG1_ADDR, 0x00);
            bsp_i2c_write_cmd(CH455_DIG2_ADDR, 0x00);
        }
    } else {
        /* Normal Selected View Rendering */
        switch (state->active_view) {
            case UI_VIEW_HR: {
                /* Mode 0: Heart Rate (" 72") with QRS Beat DP Pulse */
                if (data->heart_rate_bpm == 0) {
                    ui_display_render_text("---", -1);
                } else {
                    char hr_str[5];
                    snprintf(hr_str, sizeof(hr_str), "%3u", (unsigned int)data->heart_rate_bpm);
                    /* Pulsing decimal point on Digit 2 (ones digit) */
                    ui_display_render_text(hr_str, dp_active ? 2 : -1);
                }
                break;
            }

            case UI_VIEW_TEMP: {
                /* Mode 1: Calibrated Skin Temperature ("36.4") */
                if (data->skin_temp_c < 10.0f || data->skin_temp_c > 50.0f) {
                    ui_display_render_text("---", -1);
                } else {
                    uint16_t scaled = (uint16_t)(data->skin_temp_c * 10.0f + 0.5f);
                    uint8_t t = (uint8_t)(scaled / 100U);
                    uint8_t o = (uint8_t)((scaled % 100U) / 10U);
                    uint8_t f = (uint8_t)(scaled % 10U);

                    uint8_t seg0 = (t > 0) ? hal_ui_char_to_segment((char)('0' + t)) : 0x00;
                    uint8_t seg1 = hal_ui_char_to_segment((char)('0' + o)) | 0x80; /* Decimal point on DIG1 */
                    uint8_t seg2 = hal_ui_char_to_segment((char)('0' + f));

                    bsp_i2c_write_cmd(CH455_DIG0_ADDR, seg0);
                    bsp_i2c_write_cmd(CH455_DIG1_ADDR, seg1);
                    bsp_i2c_write_cmd(CH455_DIG2_ADDR, seg2);
                }
                break;
            }

            case UI_VIEW_STATUS: {
                /* Mode 2: System Operating Mode ("SEM", "RAW") */
                if (data->stream_mode == STREAM_MODE_SEMANTIC) {
                    ui_display_render_text("SEM", -1);
                } else {
                    ui_display_render_text("RAW", -1);
                }
                break;
            }

            default:
                ui_display_render_text("---", -1);
                break;
        }
    }

    /* 5. Update Mode Status LEDs (DIG3: bits 0..3) */
    uint8_t led_mask = 0;
    if (data->stream_mode == STREAM_MODE_SEMANTIC) {
        led_mask |= UI_LED_SEMANTIC;
    } else {
        led_mask |= UI_LED_RAW;
    }

    if (data->skin_contact) {
        led_mask |= UI_LED_CONTACT;
    }

    /* Alert LED 3 flashes synchronously at 2 Hz during active alarm */
    if ((state->active_alarm != UI_ALARM_NONE) && state->flash_phase) {
        led_mask |= UI_LED_ALERT;
    }

    hal_ui_set_mode_leds(led_mask);
}

bool ui_button_debounce_tick(uint8_t raw_pressed, uint8_t *debounced_mask,
                             uint8_t *pressed_edges, uint8_t *released_edges)
{
    static uint8_t s_debounced = 0;
    static uint8_t s_prev = 0;
    static uint8_t s_lockout[6] = {0, 0, 0, 0, 0, 0};

    for (int i = 0; i < 6; i++) {
        uint8_t bit = (1U << i);
        if (s_lockout[i] > 0) {
            s_lockout[i]--;
            /* In refractory period: keep existing debounced state locked */
        } else {
            if (raw_pressed & bit) {
                /* New physical press detected */
                if (!(s_debounced & bit)) {
                    s_debounced |= bit;
                    s_lockout[i] = 2U; /* 2 ticks @ 10 Hz = 200 ms refractory lockout */
                }
            } else {
                s_debounced &= ~bit;
            }
        }
    }

    uint8_t p_edges = (s_debounced & ~s_prev);
    uint8_t r_edges = (~s_debounced & s_prev);
    s_prev = s_debounced;

    if (debounced_mask != NULL) *debounced_mask = s_debounced;
    if (pressed_edges != NULL)  *pressed_edges = p_edges;
    if (released_edges != NULL) *released_edges = r_edges;

    return true;
}
