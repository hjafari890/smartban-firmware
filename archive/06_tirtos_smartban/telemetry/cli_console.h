/*
 * ============================================================================
 * cli_console.h
 * SmartBAN TI-RTOS7 Interactive Serial CLI Console & Command Parser
 * Platform: CC2652R1 LaunchPad + SmartBAN Shield Rev 3.5 (SDK 8.33, TI-RTOS7)
 * Milestone: M4 (Serial Testbed Interface & Interactive CLI)
 * ============================================================================
 */

#ifndef CLI_CONSOLE_H_
#define CLI_CONSOLE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <ti/drivers/UART2.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CLI_LINE_BUF_SIZE         (128U)  /**< Maximum command line length */
#define CLI_MAX_ARGS              (6U)    /**< Maximum tokens per command line */
#define CLI_SESSION_TIMEOUT_MS    (10000U)/**< Inactivity timeout before standby sleep (10s) */
#define CLI_SESSION_TIMEOUT_TICKS (100U)  /**< 10 seconds at 10 Hz */

/**
 * @brief CLI Operating and Power States.
 */
typedef enum {
    CLI_STATE_STANDBY_ARMED = 0, /**< Idle: RX disabled, DIO 2 edge wake armed (~1.0 uA Standby) */
    CLI_STATE_ACTIVE_SESSION     /**< Active: RX enabled, watchdog armed, echoing commands */
} cli_state_t;

/**
 * @brief Function pointer signature for CLI command handlers.
 * @param argc Number of parsed arguments.
 * @param argv Array of null-terminated string tokens.
 * @param response_buf Output buffer for command response text.
 * @param max_len Maximum length of response buffer.
 * @return 0 on success; non-zero error code.
 */
typedef int (*cli_cmd_handler_t)(int argc, char *argv[], char *response_buf, size_t max_len);

/**
 * @brief CLI Command Table Descriptor.
 */
typedef struct {
    const char       *name;        /**< Command string (e.g. "MODE") */
    const char       *syntax;      /**< Parameter syntax guide */
    const char       *help_desc;   /**< Human-readable help summary */
    cli_cmd_handler_t handler;     /**< Callback execution function */
} cli_command_entry_t;

/* ============================================================================
 * Console Management APIs
 * ============================================================================ */

/**
 * @brief Initialize the CLI Console subsystem.
 * @param uart_handle Initialized UART2 handle.
 * @return true on success; false otherwise.
 */
bool cli_console_init(UART2_Handle uart_handle);

/**
 * @brief Ingest a single character from the UART2 RX buffer.
 *        Handles editing, destructive backspace, ANSI filter, and line completion.
 * @param c Incoming ASCII character.
 */
void cli_console_process_char(char c);

/**
 * @brief Periodic CLI maintenance tick (Called at 10 Hz from Task_Telemetry_UI).
 *        Evaluates session timeout, services standby transitions, and reads UART RX.
 */
void cli_console_tick_10hz(void);

/**
 * @brief Enable or disable local character echo.
 * @param enable true for human terminal sessions; false for automated test harnesses.
 */
void cli_console_set_echo(bool enable);

/**
 * @brief Query whether local character echo is enabled.
 */
bool cli_console_get_echo(void);

/**
 * @brief Check if the user is currently typing a command (line_len > 0).
 *        Used by Telemetry Streamer to hold off periodic frames.
 */
bool cli_console_is_typing_active(void);

/**
 * @brief Query current CLI power/session state.
 */
cli_state_t cli_console_get_state(void);

/**
 * @brief Manually awaken the interactive CLI session (e.g. via hardware button press).
 */
void cli_console_wake_session(void);

/**
 * @brief Manually transition CLI to standby armed mode (~1.0 uA).
 */
void cli_console_enter_standby(void);

/**
 * @brief Print string directly to UART TX console.
 * @param str Null-terminated string.
 */
void cli_console_print(const char *str);

/**
 * @brief Execute a raw command line string directly (useful for tests and automation).
 * @param line Command line text.
 * @param response_buf Output buffer for response text.
 * @param max_len Size of response buffer.
 * @return 0 on success, non-zero error code.
 */
int cli_console_execute_line(char *line, char *response_buf, size_t max_len);

/* ============================================================================
 * Dynamic Threshold Configuration Accessors
 * ============================================================================ */

uint16_t cli_get_tachy_threshold(void);
bool     cli_set_tachy_threshold(uint16_t bpm);
uint16_t cli_get_brady_threshold(void);
bool     cli_set_brady_threshold(uint16_t bpm);
float    cli_get_fall_threshold(void);
bool     cli_set_fall_threshold(float g);

#ifdef __cplusplus
}
#endif

#endif /* CLI_CONSOLE_H_ */
