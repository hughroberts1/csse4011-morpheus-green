/**
 * @file cmd.h
 * @author Hugh Roberts
 * @brief Contains function prototypes for LED and system time shell commands
 * @version 0.1
 * @date 2022-04-04
 * @copyright Copyright (c) 2022
 */

static int cmd_red_led_ctrl_on(const struct shell *, size_t, char **);
static int cmd_green_led_ctrl_on(const struct shell *, size_t, char **);
static int cmd_blue_led_ctrl_on(const struct shell *, size_t, char**);
static int cmd_red_led_ctrl_off(const struct shell *, size_t, char **);
static int cmd_green_led_ctrl_off(const struct shell *, size_t, char **);
static int cmd_blue_led_ctrl_off(const struct shell *, size_t, char **);

static int cmd_red_led_ctrl_toggle(const struct shell *, size_t, char **);
static int cmd_green_led_ctrl_toggle(const struct shell *, size_t, char **);
static int cmd_blue_led_ctrl_toggle(const struct shell *, size_t, char **);

/* Declare command handler prototypes */
static int cmd_sys_time_f(const struct shell *, size_t, char **);
static int cmd_sys_time(const struct shell *, size_t, char **);