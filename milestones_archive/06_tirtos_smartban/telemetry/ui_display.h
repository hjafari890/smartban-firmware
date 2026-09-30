/*
 * ============================================================================
 * ui_display.h
 * SmartBAN Sensor Node Firmware - CH455 7-Segment & LED Visualization Engine
 * Target: CC2652R1 (ARM Cortex-M4F), SimpleLink SDK 8.33, TI-RTOS7
 *
 * Milestone M4: Serial Testbed Interface, CLI & Visual UI
 * ============================================================================
 */

#ifndef TELEMETRY_UI_DISPLAY_H_
#define TELEMETRY_UI_DISPLAY_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "edgeai/edgeai_fusion.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * UI Display View Modes & Alarm Types
 * ============================================================================ */

/**
 * @brief UI 7-segment display view modes.
 */
typedef enum {
    UI_VIEW_HR     = 0, /**< Mode 0: Heart Rate (" 72") with pulsing QRS decimal point */
    UI_VIEW_TEMP   = 1, /**< Mode 1: Calibrated Skin Temperature ("36.4") */
    UI_VIEW_STATUS = 2, /**< Mode 2: System Telemetry Mode ("SEM", "RAW") */
    UI_VIEW_ALARM  = 3  /**< Mode 3: Emergency Alert Override ("FAL", "tAC", "brA", "Arr") */
} ui_view_mode_t;

/**
 * @brief Emergency alert display categories.
 */
typedef enum {
    UI_ALARM_NONE         = 0, /**< Normal operation, no alarm override */
    UI_ALARM_FALL         = 1, /**< Confirmed fall impact ("FAL") */
    UI_ALARM_TACHYCARDIA  = 2, /**< Cardiac tachycardia ("tAC") */
    UI_ALARM_BRADYCARDIA  = 3, /**< Cardiac bradycardia ("brA") */
    UI_ALARM_ARRHYTHMIA   = 4, /**< Cardiac arrhythmia ("Arr") */
    UI_ALARM_HYPOTHERMIA  = 5, /**< Hypothermia alert ("CLd") */
    UI_ALARM_FEVER        = 6  /**< Hyperthermia / Fever alert ("Hot") */
} ui_alarm_type_t;

/* ============================================================================
 * Mode Status LEDs Bitmask (CH455 DIG3 / 0x37)
 * ============================================================================ */
#define UI_LED_SEMANTIC     (1U << 0) /**< Bit 0: Lit when Semantic Mode is active */
#define UI_LED_RAW          (1U << 1) /**< Bit 1: Lit when Raw Stream Mode is active */
#define UI_LED_CONTACT      (1U << 2) /**< Bit 2: Lit when skin contact is verified */
#define UI_LED_ALERT        (1U << 3) /**< Bit 3: Flashes when clinical anomaly is active */
#define UI_LED_ALL_MASK     0x0FU

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief Telemetry UI input data snapshot supplied to the display engine.
 */
typedef struct {
    uint8_t                heart_rate_bpm;    /**< Instantaneous heart rate (bpm) */
    float                  skin_temp_c;       /**< Calibrated skin temperature (degC) */
    smartban_stream_mode_t stream_mode;       /**< STREAM_MODE_SEMANTIC or STREAM_MODE_RAW */
    bool                   skin_contact;      /**< True if on-body contact confirmed */
    uint16_t               alert_mask;        /**< Active alert flags bitfield */
    bool                   fall_detected;     /**< True if fall impact confirmed */
    bool                   qrs_beat_event;    /**< True if QRS beat detected this cycle */
} ui_display_data_t;

/**
 * @brief Configuration parameters for the visualization engine.
 */
typedef struct {
    uint8_t beat_pulse_ticks;   /**< Duration of QRS decimal point pulse (ticks @ 10 Hz, default: 2 = 200 ms) */
    uint8_t alarm_flash_ticks;  /**< Period of emergency alarm flashing (ticks @ 10 Hz, default: 3 = ~1.67 Hz) */
    uint8_t alarm_hold_ticks;   /**< Minimum duration to display emergency alarm (ticks @ 10 Hz, default: 30 = 3s) */
} ui_display_config_t;

/**
 * @brief Visualization engine internal state structure.
 */
typedef struct {
    ui_view_mode_t      active_view;        /**< Currently selected view mode */
    ui_alarm_type_t     active_alarm;       /**< Currently active alarm type */
    uint8_t             beat_pulse_timer;   /**< Countdown timer for QRS DP pulse */
    uint8_t             alarm_hold_timer;   /**< Countdown timer for alarm view hold */
    uint8_t             flash_counter;      /**< Counter for LED/Display flashing */
    bool                flash_phase;        /**< Current toggle phase of 2 Hz flash */
    ui_display_config_t config;             /**< Engine configuration thresholds */
    uint32_t            total_ticks;        /**< Monotonic 100 ms tick counter */
} ui_display_state_t;

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/**
 * @brief Initialize the visualization engine state and timing parameters.
 * @param state Pointer to UI display state structure.
 * @param cfg Optional custom configuration (pass NULL for standard defaults).
 */
void ui_display_init(ui_display_state_t *state, const ui_display_config_t *cfg);

/**
 * @brief Reset visualization engine state.
 * @param state Pointer to UI display state structure.
 */
void ui_display_reset(ui_display_state_t *state);

/**
 * @brief Explicitly select the active display view mode.
 * @param state Pointer to UI display state structure.
 * @param view Target view mode (UI_VIEW_HR, UI_VIEW_TEMP, UI_VIEW_STATUS).
 */
void ui_display_set_view(ui_display_state_t *state, ui_view_mode_t view);

/**
 * @brief Get the currently active display view mode.
 * @param state Pointer to UI display state structure.
 * @return Current view mode.
 */
ui_view_mode_t ui_display_get_view(const ui_display_state_t *state);

/**
 * @brief Cycle to the next display view mode (SW1 press action).
 *        Cycles: HR -> TEMP -> STATUS -> HR.
 * @param state Pointer to UI display state structure.
 * @return Newly active view mode.
 */
ui_view_mode_t ui_display_cycle_view(ui_display_state_t *state);

/**
 * @brief Notify the visualization engine of a confirmed QRS cardiac beat.
 *        Triggers a dynamic decimal point pulse on the 7-segment display.
 * @param state Pointer to UI display state structure.
 */
void ui_display_notify_beat(ui_display_state_t *state);

/**
 * @brief Periodic 10 Hz visualization update engine.
 *        Evaluates alarms, updates QRS pulses, renders the selected 7-segment
 *        view to CH455H digits DIG0..DIG2, and updates discrete mode LEDs on DIG3.
 * @param state Pointer to UI display state structure.
 * @param data Pointer to latest sensor/telemetry data snapshot.
 */
void ui_display_update(ui_display_state_t *state, const ui_display_data_t *data);

/**
 * @brief Render up to 4 characters of text across the display.
 *        Handles leading zero blanking, decimal points, and 3-digit hardware mapping.
 * @param text Up to 4 ASCII characters (null-terminated or 4-chars).
 * @param dp_digit Index of digit (0..2) to append decimal point (-1 for none).
 */
void ui_display_render_text(const char *text, int8_t dp_digit);

/**
 * @brief Direct low-level control of the 4 mode status LEDs on DIG3.
 * @param led_mask Bitmask of LEDs to illuminate (UI_LED_SEMANTIC, UI_LED_RAW, etc.).
 */
void ui_display_set_leds(uint8_t led_mask);

/**
 * @brief Clear all display digits and extinguish all mode LEDs.
 */
void ui_display_clear(void);

/**
 * @brief Enhanced 7-segment font lookup supporting digits, symbols, and synthesized 'M' (0x37) and 'W' (0x3E).
 * @param c Input ASCII character.
 * @return 7-segment bitmask.
 */
uint8_t ui_display_char_to_seg(char c);

/**
 * @brief Multi-sample button debouncer with 200 ms refractory lockout.
 *        Guarantees 0 ms latency on initial press, masks mechanical bounce,
 *        and prevents lost taps on PCAL6408A auto-clearing input latches.
 * @param raw_pressed Raw pressed button bitmask from expander (active HIGH).
 * @param debounced_mask Pointer to destination debounced state mask.
 * @param pressed_edges Pointer to destination single-cycle press edge events.
 * @param released_edges Pointer to destination single-cycle release edge events.
 * @return true if button state is valid.
 */
bool ui_button_debounce_tick(uint8_t raw_pressed, uint8_t *debounced_mask,
                             uint8_t *pressed_edges, uint8_t *released_edges);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_UI_DISPLAY_H_ */
