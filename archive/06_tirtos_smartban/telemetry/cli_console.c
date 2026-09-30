/*
 * ============================================================================
 * cli_console.c
 * SmartBAN TI-RTOS7 Interactive Serial CLI Console & Command Parser
 * Platform: CC2652R1 LaunchPad + SmartBAN Shield Rev 3.5 (SDK 8.33, TI-RTOS7)
 * Milestone: M4 (Serial Testbed Interface & Interactive CLI)
 * ============================================================================
 */

#include "telemetry/cli_console.h"
#include "telemetry/telemetry_uart.h"
#include "edgeai/edgeai_fusion.h"
#include "syscfg/ti_drivers_config.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/dpl/ClockP.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/sys_ctrl.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/ioc.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/cpu.h>

/* ANSI sequence filter states */
typedef enum {
    ANSI_STATE_NORMAL = 0,
    ANSI_STATE_ESC,
    ANSI_STATE_CSI
} ansi_state_t;

/* Internal Console State */
static UART2_Handle      s_uart_handle = NULL;
static cli_state_t       s_cli_state = CLI_STATE_STANDBY_ARMED;
static uint32_t          s_watchdog_ticks = 0;
static bool              s_echo_enabled = true;
static volatile bool     s_cli_wake_requested = false;
static ansi_state_t      s_ansi_state = ANSI_STATE_NORMAL;

static char              s_line_buf[CLI_LINE_BUF_SIZE];
static size_t            s_line_len = 0;

/* Configurable Anomaly Thresholds */
static uint16_t          s_tachy_threshold_bpm = 100U;
static uint16_t          s_brady_threshold_bpm = 50U;
static float             s_fall_threshold_g    = 3.00f;

/* Case-insensitive string comparison helper */
static int cli_strcasecmp(const char *s1, const char *s2)
{
    while (*s1 && *s2) {
        int c1 = tolower((unsigned char)*s1);
        int c2 = tolower((unsigned char)*s2);
        if (c1 != c2) {
            return (c1 - c2);
        }
        s1++;
        s2++;
    }
    return (int)(tolower((unsigned char)*s1) - tolower((unsigned char)*s2));
}

/* ============================================================================
 * GPIO Falling Edge Wake ISR on DIO 2 (UART RX Pin)
 * ============================================================================ */
static void cli_rx_pin_wake_isr(uint_least8_t index)
{
    (void)index;
    /* UART Start Bit falling edge detected in AON domain (~14 us wakeup).
     * Signal Task_Telemetry_UI to restore UART RX peripheral. */
    s_cli_wake_requested = true;
}

/* ============================================================================
 * Command Handlers
 * ============================================================================ */

static int cmd_help_handler(int argc, char *argv[], char *res, size_t max_len)
{
    (void)argc;
    (void)argv;
    return snprintf(res, max_len,
        "===============================================================\r\n"
        " SmartBAN Serial CLI Console - Available Commands:\r\n"
        "===============================================================\r\n"
        "  HELP                       - Display this command reference\r\n"
        "  MODE [SEMANTIC|RAW|TOGGLE] - Query or switch telemetry streaming mode\r\n"
        "  STATUS                     - Display detailed system diagnostics & stacks\r\n"
        "  THRESHOLD [tachy|brady|fall] [val] - Configure dynamic anomaly thresholds\r\n"
        "  STREAM [START|STOP]        - Enable or pause serial telemetry output\r\n"
        "  RESET                      - Perform clean MCU reset via SysCtlSystemReset\r\n"
        "===============================================================\r\n");
}

static int cmd_mode_handler(int argc, char *argv[], char *res, size_t max_len)
{
    if (argc == 1) {
        smartban_stream_mode_t cur = edgeai_get_stream_mode();
        return snprintf(res, max_len,
            "[MODE] Current: %s (%s)\r\n",
            cur == STREAM_MODE_SEMANTIC ? "SEMANTIC" : "RAW STREAM",
            cur == STREAM_MODE_SEMANTIC ? ">95%% Data Reduction, 1 Hz/Alert" : "Continuous Evaluation Stream");
    }

    if (cli_strcasecmp(argv[1], "SEMANTIC") == 0) {
        edgeai_set_stream_mode(STREAM_MODE_SEMANTIC);
        return snprintf(res, max_len,
            "[MODE] Switched to SEMANTIC (1 Hz / Alert Bursts, >95%% Data Reduction)\r\n");
    } else if (cli_strcasecmp(argv[1], "RAW") == 0) {
        edgeai_set_stream_mode(STREAM_MODE_RAW);
        return snprintf(res, max_len,
            "[MODE] Switched to RAW STREAM (50 Hz Continuous Evaluation Stream)\r\n");
    } else if (cli_strcasecmp(argv[1], "TOGGLE") == 0) {
        smartban_stream_mode_t nm = edgeai_toggle_stream_mode();
        return snprintf(res, max_len,
            "[MODE] Toggled to %s\r\n",
            nm == STREAM_MODE_SEMANTIC ? "SEMANTIC (>95%% Reduction)" : "RAW STREAM (Evaluation)");
    }

    return snprintf(res, max_len,
        "[ERROR] Invalid mode '%s'. Syntax: MODE [SEMANTIC|RAW|TOGGLE]\r\n", argv[1]);
}

/* Weak symbol or forward reference for detailed main task status */
__attribute__((weak)) void main_format_system_status(char *buf, size_t max_len);

static int cmd_status_handler(int argc, char *argv[], char *res, size_t max_len)
{
    (void)argc;
    (void)argv;

    /* If main_format_system_status is linked in main_tirtos.c, invoke it */
    if (main_format_system_status != NULL) {
        main_format_system_status(res, max_len);
        return (int)strlen(res);
    }

    /* Fallback default diagnostic status */
    telemetry_stats_t tstats;
    telemetry_uart_get_stats(&tstats);
    uint32_t q_count = telemetry_uart_get_queue_count();
    smartban_stream_mode_t smode = edgeai_get_stream_mode();

    return snprintf(res, max_len,
        "---------------------------------------------------------------\r\n"
        " SmartBAN CC2652R1 System Diagnostics & Health Status:\r\n"
        "---------------------------------------------------------------\r\n"
        "  Operating Mode : %s\r\n"
        "  CLI Power State: %s (Watchdog: %lu s)\r\n"
        "  Telemetry Queue: %lu / %u records (Drops: %lu)\r\n"
        "  Telemetry Stats: Sent Sem: %lu, Raw: %lu, Bytes: %lu, Err: %lu\r\n"
        "  Thresholds     : Tachy: %u bpm, Brady: %u bpm, Fall: %.2f g\r\n"
        "  Sensors Status : ADS1292(OK) ADXL362(OK) MLX90632(OK) OPT4041(OK) VCNL4040(OK)\r\n"
        "---------------------------------------------------------------\r\n",
        smode == STREAM_MODE_SEMANTIC ? "SEMANTIC (>95% Reduction)" : "RAW STREAM",
        s_cli_state == CLI_STATE_ACTIVE_SESSION ? "ACTIVE_SESSION" : "STANDBY_ARMED",
        (unsigned long)(s_watchdog_ticks / 10U),
        (unsigned long)q_count, TELEMETRY_QUEUE_CAPACITY, (unsigned long)tstats.queue_overflow_drops,
        (unsigned long)tstats.semantic_frames_sent, (unsigned long)tstats.raw_frames_sent,
        (unsigned long)tstats.total_bytes_sent, (unsigned long)tstats.uart_write_errors,
        (unsigned int)s_tachy_threshold_bpm, (unsigned int)s_brady_threshold_bpm,
        (double)s_fall_threshold_g);
}

static int cmd_threshold_handler(int argc, char *argv[], char *res, size_t max_len)
{
    if (argc == 1) {
        return snprintf(res, max_len,
            "[THRESHOLD] Current Anomaly Configurations:\r\n"
            "  Tachycardia : %u bpm  (Permitted: 80..220 bpm)\r\n"
            "  Bradycardia : %u bpm  (Permitted: 30..70 bpm)\r\n"
            "  Fall Impact : %.2f g   (Permitted: 1.50..8.00 g)\r\n",
            s_tachy_threshold_bpm, s_brady_threshold_bpm, (double)s_fall_threshold_g);
    }

    if (argc >= 3) {
        if (cli_strcasecmp(argv[1], "TACHY") == 0) {
            int val = atoi(argv[2]);
            if (val >= 80 && val <= 220) {
                s_tachy_threshold_bpm = (uint16_t)val;
                return snprintf(res, max_len,
                    "[THRESHOLD] Tachycardia threshold updated to %u bpm\r\n", s_tachy_threshold_bpm);
            }
            return snprintf(res, max_len,
                "[ERROR] Tachycardia threshold %d out of range [80..220] bpm\r\n", val);
        } else if (cli_strcasecmp(argv[1], "BRADY") == 0) {
            int val = atoi(argv[2]);
            if (val >= 30 && val <= 70) {
                s_brady_threshold_bpm = (uint16_t)val;
                return snprintf(res, max_len,
                    "[THRESHOLD] Bradycardia threshold updated to %u bpm\r\n", s_brady_threshold_bpm);
            }
            return snprintf(res, max_len,
                "[ERROR] Bradycardia threshold %d out of range [30..70] bpm\r\n", val);
        } else if (cli_strcasecmp(argv[1], "FALL") == 0) {
            float val = (float)atof(argv[2]);
            if (val >= 1.50f && val <= 8.00f) {
                s_fall_threshold_g = val;
                return snprintf(res, max_len,
                    "[THRESHOLD] Fall impact threshold updated to %.2f g\r\n", (double)s_fall_threshold_g);
            }
            return snprintf(res, max_len,
                "[ERROR] Fall impact threshold %.2f out of range [1.50..8.00] g\r\n", (double)val);
        }
    }

    return snprintf(res, max_len,
        "[ERROR] Syntax: THRESHOLD [tachy|brady|fall] <value>\r\n");
}

static int cmd_stream_handler(int argc, char *argv[], char *res, size_t max_len)
{
    if (argc == 1) {
        return snprintf(res, max_len,
            "[STREAM] Automated Telemetry Streaming is %s\r\n",
            telemetry_uart_is_streaming_enabled() ? "ENABLED" : "PAUSED");
    }

    if (cli_strcasecmp(argv[1], "START") == 0) {
        telemetry_uart_set_streaming_enabled(true);
        return snprintf(res, max_len,
            "[STREAM] Automated Telemetry Streaming ENABLED\r\n");
    } else if (cli_strcasecmp(argv[1], "STOP") == 0) {
        telemetry_uart_set_streaming_enabled(false);
        return snprintf(res, max_len,
            "[STREAM] Automated Telemetry Streaming PAUSED\r\n");
    }

    return snprintf(res, max_len,
        "[ERROR] Syntax: STREAM [START|STOP]\r\n");
}

static int cmd_reset_handler(int argc, char *argv[], char *res, size_t max_len)
{
    (void)argc;
    (void)argv;
    int len = snprintf(res, max_len,
        "[SYSTEM] Initiating clean MCU hardware reset via SysCtlSystemReset()...\r\n");
    if (s_uart_handle != NULL && len > 0) {
        size_t written = 0;
        UART2_write(s_uart_handle, res, (size_t)len, &written);
    }
    /* Small spin-wait for TX shift register to flush */
    for (volatile uint32_t i = 0; i < 480000; i++) {
        __asm__ __volatile__("nop");
    }
    SysCtrlSystemReset();
    return len;
}

/* Dispatch Command Lookup Table */
static const cli_command_entry_t s_command_table[] = {
    {"HELP",      "HELP",                             "Display command reference",                cmd_help_handler},
    {"?",         "?",                                "Display command reference",                cmd_help_handler},
    {"MODE",      "MODE [SEMANTIC|RAW|TOGGLE]",       "Query or switch streaming mode",           cmd_mode_handler},
    {"STATUS",    "STATUS",                           "Display system diagnostics & stack info",  cmd_status_handler},
    {"THRESHOLD", "THRESHOLD [tachy|brady|fall] [v]", "Configure anomaly thresholds",            cmd_threshold_handler},
    {"STREAM",    "STREAM [START|STOP]",              "Enable or pause telemetry stream",         cmd_stream_handler},
    {"RESET",     "RESET",                            "Software reset via SysCtlSystemReset",     cmd_reset_handler}
};

#define COMMAND_TABLE_SIZE (sizeof(s_command_table) / sizeof(s_command_table[0]))

/* ============================================================================
 * Public APIs
 * ============================================================================ */

bool cli_console_init(UART2_Handle uart_handle)
{
    s_uart_handle = uart_handle;
    s_cli_state = CLI_STATE_STANDBY_ARMED;
    s_watchdog_ticks = 0;
    s_echo_enabled = true;
    s_cli_wake_requested = false;
    s_ansi_state = ANSI_STATE_NORMAL;
    s_line_len = 0;
    s_line_buf[0] = '\0';

    /* Arm Standby by default */
    cli_console_enter_standby();
    return true;
}

void cli_console_set_echo(bool enable)
{
    s_echo_enabled = enable;
}

bool cli_console_get_echo(void)
{
    return s_echo_enabled;
}

bool cli_console_is_typing_active(void)
{
    return (s_line_len > 0);
}

cli_state_t cli_console_get_state(void)
{
    return s_cli_state;
}

void cli_console_print(const char *str)
{
    if (s_uart_handle == NULL || str == NULL) {
        return;
    }
    size_t len = strlen(str);
    if (len == 0) return;
    size_t written = 0;
    UART2_write(s_uart_handle, str, len, &written);
}

void cli_console_enter_standby(void)
{
    if (s_uart_handle != NULL) {
        UART2_rxDisable(s_uart_handle);
    }

    /* Configure DIO 2 (UART RX) as input with pull-up and falling-edge interrupt */
    GPIO_setConfig(CONFIG_GPIO_UART2_0_RX, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(CONFIG_GPIO_UART2_0_RX, cli_rx_pin_wake_isr);
    GPIO_clearInt(CONFIG_GPIO_UART2_0_RX);
    GPIO_enableInt(CONFIG_GPIO_UART2_0_RX);

    s_cli_state = CLI_STATE_STANDBY_ARMED;
    s_watchdog_ticks = 0;
    s_line_len = 0;
    s_line_buf[0] = '\0';
}

void cli_console_wake_session(void)
{
    /* Disable GPIO edge interrupt on DIO 2 */
    GPIO_disableInt(CONFIG_GPIO_UART2_0_RX);

    /* Remap DIO 2 back to UART0 RX peripheral function in IOC */
    IOCPortConfigureSet(IOID_2, IOC_PORT_MCU_UART0_RX, IOC_STD_INPUT);

    /* Enable UART2 RX (asserts standby constraint during interactive typing) */
    if (s_uart_handle != NULL) {
        UART2_rxEnable(s_uart_handle);
    }

    s_watchdog_ticks = CLI_SESSION_TIMEOUT_TICKS;
    s_cli_state = CLI_STATE_ACTIVE_SESSION;
    s_cli_wake_requested = false;
    s_line_len = 0;
    s_line_buf[0] = '\0';

    cli_console_print("\r\n[SmartBAN Awakened from Standby]\r\nSmartBAN> ");
}

int cli_console_execute_line(char *line, char *response_buf, size_t max_len)
{
    if (line == NULL || response_buf == NULL || max_len == 0) {
        return -1;
    }

    char *argv[CLI_MAX_ARGS];
    int argc = 0;
    char *p = line;

    /* In-place whitespace tokenizer */
    while (*p != '\0' && argc < (int)CLI_MAX_ARGS) {
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
            *p++ = '\0';
        }
        if (*p != '\0') {
            argv[argc++] = p;
            while (*p != '\0' && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') {
                p++;
            }
        }
    }

    if (argc == 0) {
        response_buf[0] = '\0';
        return 0;
    }

    /* Dispatch match against command table */
    for (size_t i = 0; i < COMMAND_TABLE_SIZE; i++) {
        if (cli_strcasecmp(argv[0], s_command_table[i].name) == 0) {
            return s_command_table[i].handler(argc, argv, response_buf, max_len);
        }
    }

    /* Unknown command fallback */
    return snprintf(response_buf, max_len,
        "[ERROR] Unknown command '%s'. Type HELP for available commands.\r\n", argv[0]);
}

void cli_console_process_char(char c)
{
    /* 1. ANSI Escape Sequence Filter */
    if (s_ansi_state == ANSI_STATE_NORMAL) {
        if (c == 0x1B) {
            s_ansi_state = ANSI_STATE_ESC;
            return;
        }
    } else if (s_ansi_state == ANSI_STATE_ESC) {
        if (c == '[') {
            s_ansi_state = ANSI_STATE_CSI;
            return;
        } else {
            s_ansi_state = ANSI_STATE_NORMAL;
            return;
        }
    } else if (s_ansi_state == ANSI_STATE_CSI) {
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '~') {
            s_ansi_state = ANSI_STATE_NORMAL;
            return;
        }
        return; /* Consume parameters in CSI sequence */
    }

    /* 2. Destructive Backspace & DEL (0x08 / 0x7F) */
    if (c == '\b' || c == 0x7F) {
        if (s_line_len > 0) {
            s_line_len--;
            s_line_buf[s_line_len] = '\0';
            if (s_echo_enabled) {
                cli_console_print("\b \b");
            }
        }
        s_watchdog_ticks = CLI_SESSION_TIMEOUT_TICKS;
        return;
    }

    /* 3. Line Terminators (CR, LF, CRLF) */
    if (c == '\r' || c == '\n') {
        if (s_echo_enabled) {
            cli_console_print("\r\n");
        }

        if (s_line_len > 0) {
            char response[512];
            cli_console_execute_line(s_line_buf, response, sizeof(response));
            if (response[0] != '\0') {
                cli_console_print(response);
            }
            s_line_len = 0;
            s_line_buf[0] = '\0';
        }

        if (s_echo_enabled && s_cli_state == CLI_STATE_ACTIVE_SESSION) {
            cli_console_print("SmartBAN> ");
        }

        s_watchdog_ticks = CLI_SESSION_TIMEOUT_TICKS;
        return;
    }

    /* 4. Printable Characters (0x20 .. 0x7E) */
    if (c >= 0x20 && c <= 0x7E) {
        if (s_line_len < (CLI_LINE_BUF_SIZE - 1U)) {
            s_line_buf[s_line_len++] = c;
            s_line_buf[s_line_len] = '\0';

            if (s_echo_enabled) {
                char echo_buf[2] = {c, '\0'};
                cli_console_print(echo_buf);
            }
        }
        s_watchdog_ticks = CLI_SESSION_TIMEOUT_TICKS;
    }
}

void cli_console_tick_10hz(void)
{
    /* 1. Check if GPIO wake event requested session activation */
    if (s_cli_wake_requested) {
        cli_console_wake_session();
    }

    /* 2. If session is active, read pending RX characters and service watchdog */
    if (s_cli_state == CLI_STATE_ACTIVE_SESSION) {
        if (s_uart_handle != NULL) {
            char c = 0;
            size_t bytes_read = 0;
            /* Read characters from UART2 FIFO without blocking */
            while (UART2_read(s_uart_handle, &c, 1, &bytes_read) == UART2_STATUS_SUCCESS && bytes_read > 0) {
                cli_console_process_char(c);
            }
        }

        /* Watchdog countdown */
        if (s_watchdog_ticks > 0) {
            s_watchdog_ticks--;
            if (s_watchdog_ticks == 0) {
                cli_console_print("\r\n[CLI Timeout: Returning to Standby Sleep (~1.0 uA)]\r\n");
                cli_console_enter_standby();
            }
        }
    }
}

/* ============================================================================
 * Threshold Accessors
 * ============================================================================ */

uint16_t cli_get_tachy_threshold(void)
{
    return s_tachy_threshold_bpm;
}

bool cli_set_tachy_threshold(uint16_t bpm)
{
    if (bpm >= 80 && bpm <= 220) {
        s_tachy_threshold_bpm = bpm;
        return true;
    }
    return false;
}

uint16_t cli_get_brady_threshold(void)
{
    return s_brady_threshold_bpm;
}

bool cli_set_brady_threshold(uint16_t bpm)
{
    if (bpm >= 30 && bpm <= 70) {
        s_brady_threshold_bpm = bpm;
        return true;
    }
    return false;
}

float cli_get_fall_threshold(void)
{
    return s_fall_threshold_g;
}

bool cli_set_fall_threshold(float g)
{
    if (g >= 1.50f && g <= 8.00f) {
        s_fall_threshold_g = g;
        return true;
    }
    return false;
}
