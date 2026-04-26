#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include "heater_pwm_shell.h"

LOG_MODULE_REGISTER(heater_pwm, LOG_LEVEL_INF);

/*
 * Software PWM on the heater output GPIO.
 *
 * 2 Hz period = 500 ms total.  5% duty = 25 ms on, 475 ms off.
 * A single periodic timer fires every 25 ms (GCD of 25 and 475).
 * On each tick the output is driven high for the first tick and low for
 * the remaining 19 ticks, giving exactly 25/500 = 5% duty at 2 Hz.
 */

#define PWM_PERIOD_MS   5000
#define PWM_ON_MS       2500
#define PWM_TICK_MS     2500          /* GCD(25, 475) */
#define PWM_TICKS       (PWM_PERIOD_MS / PWM_TICK_MS)   /* 20 ticks per period */
#define PWM_ON_TICKS    (PWM_ON_MS    / PWM_TICK_MS)    /* 1 tick high          */

static const struct gpio_dt_spec heater_gpio =
	GPIO_DT_SPEC_GET(DT_ALIAS(heater0), gpios);

static struct k_timer pwm_timer;
static volatile bool pwm_running;
static int tick_count;

static void pwm_timer_fn(struct k_timer *t)
{
	ARG_UNUSED(t);

	tick_count = (tick_count + 1) % PWM_TICKS;
	gpio_pin_set_dt(&heater_gpio, tick_count < PWM_ON_TICKS ? 1 : 0);
}

static int pwm_start(void)
{
	if (!gpio_is_ready_dt(&heater_gpio)) {
		LOG_ERR("Heater GPIO not ready");
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&heater_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		LOG_ERR("Heater GPIO configure failed: %d", ret);
		return ret;
	}

	tick_count = PWM_TICKS - 1; /* next tick rolls to 0 → first tick goes high */
	pwm_running = true;
	k_timer_start(&pwm_timer, K_MSEC(PWM_TICK_MS), K_MSEC(PWM_TICK_MS));
	LOG_INF("Heater PWM started: 0.2 Hz, 10%% duty on GPIO%u",
		heater_gpio.pin);
	return 0;
}

static void pwm_stop(void)
{
	k_timer_stop(&pwm_timer);
	pwm_running = false;
	gpio_pin_set_dt(&heater_gpio, 0);
	LOG_INF("Heater PWM stopped");
}

static int cmd_heater_pwm_start(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (pwm_running) {
		shell_print(sh, "Heater PWM already running");
		return 0;
	}

	int ret = pwm_start();
	if (ret) {
		shell_error(sh, "Failed to start heater PWM: %d", ret);
	} else {
		shell_print(sh, "Heater PWM started (2 Hz, 5%% duty)");
	}
	return ret;
}

static int cmd_heater_pwm_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!pwm_running) {
		shell_print(sh, "Heater PWM not running");
		return 0;
	}

	pwm_stop();
	shell_print(sh, "Heater PWM stopped");
	return 0;
}

static int cmd_heater_pwm_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Heater PWM: %s", pwm_running ? "running" : "stopped");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_heater_pwm,
	SHELL_CMD(start,  NULL, "Start 2 Hz 5%% duty PWM on heater GPIO", cmd_heater_pwm_start),
	SHELL_CMD(stop,   NULL, "Stop heater PWM",                         cmd_heater_pwm_stop),
	SHELL_CMD(status, NULL, "Show heater PWM state",                   cmd_heater_pwm_status),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(heater_pwm, &sub_heater_pwm,
	"Control heater output software PWM", NULL);

static int heater_pwm_init(void)
{
	k_timer_init(&pwm_timer, pwm_timer_fn, NULL);
	return 0;
}

SYS_INIT(heater_pwm_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);