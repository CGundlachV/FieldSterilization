#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/shell/shell.h>

#include "mlx90614_read.h"

LOG_MODULE_REGISTER(fieldsterilization, LOG_LEVEL_INF);

#ifdef CONFIG_DISPLAY
#include <zephyr/drivers/display.h>
#include <lvgl.h>
#include <lvgl_zephyr.h>
#endif

#ifndef CONFIG_FIELDSTERILIZATION_SETPOINT_CENTIC
#define CONFIG_FIELDSTERILIZATION_SETPOINT_CENTIC 12100
#endif

#ifndef CONFIG_FIELDSTERILIZATION_MAX_SAFE_CENTIC
#define CONFIG_FIELDSTERILIZATION_MAX_SAFE_CENTIC 13500
#endif

#ifndef CONFIG_FIELDSTERILIZATION_VISUAL_MIN_CENTIC
#define CONFIG_FIELDSTERILIZATION_VISUAL_MIN_CENTIC 2000
#endif

#ifndef CONFIG_FIELDSTERILIZATION_VISUAL_MAX_CENTIC
#define CONFIG_FIELDSTERILIZATION_VISUAL_MAX_CENTIC 18000
#endif

#ifndef CONFIG_FIELDSTERILIZATION_CONTROL_WINDOW_MS
#define CONFIG_FIELDSTERILIZATION_CONTROL_WINDOW_MS 1000
#endif

#ifndef CONFIG_FIELDSTERILIZATION_CONTROL_KP_MILLI
#define CONFIG_FIELDSTERILIZATION_CONTROL_KP_MILLI 8000
#endif

#ifndef CONFIG_FIELDSTERILIZATION_CONTROL_KI_MILLI
#define CONFIG_FIELDSTERILIZATION_CONTROL_KI_MILLI 1000
#endif

#ifndef CONFIG_FIELDSTERILIZATION_SENSOR_TIMEOUT_MS
#define CONFIG_FIELDSTERILIZATION_SENSOR_TIMEOUT_MS 1500
#endif

#ifndef CONFIG_FIELDSTERILIZATION_BUTTON_DEBOUNCE_MS
#define CONFIG_FIELDSTERILIZATION_BUTTON_DEBOUNCE_MS 250
#endif

#define SAMPLE_PERIOD_MS            100
#define UI_PERIOD_MS                100
#define LOG_PERIOD_MS               1000
#define SENSOR_RETRY_PERIOD_MS      2000
#define CONTROL_DEADBAND_C          0.5f
#define FAULT_RESET_MARGIN_C        2.0f
#define SENSOR_FILTER_ALPHA         0.25f

#define SETPOINT_C                  ((float)CONFIG_FIELDSTERILIZATION_SETPOINT_CENTIC / 100.0f)
#define MAX_SAFE_C                  ((float)CONFIG_FIELDSTERILIZATION_MAX_SAFE_CENTIC / 100.0f)
#define VISUAL_MIN_C                ((float)CONFIG_FIELDSTERILIZATION_VISUAL_MIN_CENTIC / 100.0f)
#define VISUAL_MAX_C                ((float)CONFIG_FIELDSTERILIZATION_VISUAL_MAX_CENTIC / 100.0f)
#define CONTROL_WINDOW_MS           CONFIG_FIELDSTERILIZATION_CONTROL_WINDOW_MS
#define CONTROL_KP                  ((float)CONFIG_FIELDSTERILIZATION_CONTROL_KP_MILLI / 1000.0f)
#define CONTROL_KI                  ((float)CONFIG_FIELDSTERILIZATION_CONTROL_KI_MILLI / 1000.0f)
#define SENSOR_TIMEOUT_MS           CONFIG_FIELDSTERILIZATION_SENSOR_TIMEOUT_MS
#define BUTTON_DEBOUNCE_MS          CONFIG_FIELDSTERILIZATION_BUTTON_DEBOUNCE_MS

BUILD_ASSERT(CONTROL_WINDOW_MS >= MSEC_PER_SEC,
	     "heater control window must be at least 1000 ms");
BUILD_ASSERT(CONFIG_FIELDSTERILIZATION_MAX_SAFE_CENTIC >
	     CONFIG_FIELDSTERILIZATION_SETPOINT_CENTIC,
	     "max safe temperature must be above the setpoint");
BUILD_ASSERT(CONFIG_FIELDSTERILIZATION_VISUAL_MAX_CENTIC >
	     CONFIG_FIELDSTERILIZATION_VISUAL_MIN_CENTIC,
	     "visual max temperature must be above visual min temperature");

#if DT_NODE_EXISTS(DT_ALIAS(sw0))
#define HAS_RUN_BUTTON 1
static const struct gpio_dt_spec run_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback run_button_cb;
static K_SEM_DEFINE(run_button_sem, 0, 1);
static volatile int64_t last_button_irq_ms;
#else
#define HAS_RUN_BUTTON 0
#endif

#if DT_NODE_EXISTS(DT_ALIAS(heater0))
#define HAS_HEATER_OUTPUT 1
static const struct gpio_dt_spec heater_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(heater0), gpios);
#else
#define HAS_HEATER_OUTPUT 0
#endif

#if DT_NODE_EXISTS(DT_ALIAS(led0))
#define HAS_STATUS_LED 1
static const struct gpio_dt_spec status_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#else
#define HAS_STATUS_LED 0
#endif

#ifdef CONFIG_DISPLAY
#if DT_NODE_EXISTS(DT_ALIAS(backlight0))
#define HAS_DISPLAY_BACKLIGHT 1
static const struct gpio_dt_spec display_backlight = GPIO_DT_SPEC_GET(DT_ALIAS(backlight0), gpios);
#else
#define HAS_DISPLAY_BACKLIGHT 0
#endif
#endif

enum app_fault {
	APP_FAULT_NONE,
	APP_FAULT_SENSOR,
	APP_FAULT_OVERTEMP,
	APP_FAULT_OUTPUT,
};

struct app_state {
	bool running;
	bool sensor_initialized;
	bool sensor_online;
	bool have_sample;
	bool heater_ready;
	bool heater_on;
	enum app_fault fault;
	uint8_t sensor_failures;
	uint8_t duty_pct;
	float ambient_c;
	float object_c;
	float filtered_c;
	float control_integral;
	int64_t last_good_sensor_ms;
	int64_t last_sensor_retry_ms;
	int64_t last_control_ms;
	int64_t output_window_start_ms;
	int64_t run_started_ms;
};

static struct app_state app;

static float clampf(float value, float min_value, float max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

static const char *fault_text(enum app_fault fault)
{
	switch (fault) {
	case APP_FAULT_NONE:
		return "none";
	case APP_FAULT_SENSOR:
		return "sensor";
	case APP_FAULT_OVERTEMP:
		return "overtemp";
	case APP_FAULT_OUTPUT:
		return "output";
	default:
		return "unknown";
	}
}

static bool elapsed(int64_t now_ms, int64_t *last_ms, int32_t period_ms)
{
	if (*last_ms == 0 || (now_ms - *last_ms) >= period_ms) {
		*last_ms = now_ms;
		return true;
	}

	return false;
}

static int heater_apply(struct app_state *state, bool active)
{
#if HAS_HEATER_OUTPUT
	if (!state->heater_ready) {
		state->heater_on = false;
		return active ? -ENODEV : 0;
	}

	int ret = gpio_pin_set_dt(&heater_gpio, active ? 1 : 0);
	if (ret == 0) {
		state->heater_on = active;
	}
	return ret;
#else
	state->heater_on = false;
	return active ? -ENODEV : 0;
#endif
}

static int status_led_apply(bool on)
{
#if HAS_STATUS_LED
	if (!gpio_is_ready_dt(&status_led)) {
		return -ENODEV;
	}

	return gpio_pin_set_dt(&status_led, on ? 1 : 0);
#else
	ARG_UNUSED(on);
	return -ENODEV;
#endif
}

static int status_led_configure(void)
{
#if HAS_STATUS_LED
	if (!gpio_is_ready_dt(&status_led)) {
		LOG_WRN("Status LED GPIO is not ready");
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&status_led, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		LOG_WRN("Status LED configure failed: %d", ret);
		return ret;
	}

	(void)status_led_apply(false);
	LOG_INF("Status LED ready on %s pin %u", status_led.port->name, status_led.pin);
	return 0;
#else
	return -ENODEV;
#endif
}

static bool status_led_should_be_on(const struct app_state *state, int64_t now_ms)
{
	int32_t phase_ms;

	if (state->heater_on) {
		return true;
	}

	switch (state->fault) {
	case APP_FAULT_SENSOR:
	case APP_FAULT_OUTPUT:
		return ((now_ms / 125) % 2) == 0;
	case APP_FAULT_OVERTEMP:
		phase_ms = now_ms % 1000;
		return phase_ms < 120 || (phase_ms >= 240 && phase_ms < 360);
	case APP_FAULT_NONE:
	default:
		break;
	}

	if (!state->running && state->sensor_online && state->have_sample) {
		return (now_ms % 1000) < 180;
	}

	return false;
}

static void status_led_update(const struct app_state *state, int64_t now_ms)
{
#if HAS_STATUS_LED
	static bool last_state;
	bool new_state = status_led_should_be_on(state, now_ms);

	if (new_state != last_state) {
		(void)status_led_apply(new_state);
		last_state = new_state;
	}
#else
	ARG_UNUSED(state);
	ARG_UNUSED(now_ms);
#endif
}

static void controller_stop(struct app_state *state)
{
	state->running = false;
	state->duty_pct = 0;
	state->control_integral = 0.0f;
	(void)heater_apply(state, false);
}

static void latch_fault(struct app_state *state, enum app_fault fault)
{
	if (state->fault != fault) {
		LOG_ERR("Fault latched: %s", fault_text(fault));
	}
	state->fault = fault;
	controller_stop(state);
}

static void clear_recoverable_faults(struct app_state *state)
{
	if (!state->have_sample || state->running) {
		return;
	}

	if (state->fault == APP_FAULT_SENSOR && state->sensor_online) {
		LOG_INF("Fault cleared: sensor");
		state->fault = APP_FAULT_NONE;
	}

	if (state->fault == APP_FAULT_OVERTEMP &&
	    state->filtered_c < (MAX_SAFE_C - FAULT_RESET_MARGIN_C)) {
		LOG_INF("Fault cleared: overtemp, temp %.1f C below %.1f C reset point",
			(double)state->filtered_c,
			(double)(MAX_SAFE_C - FAULT_RESET_MARGIN_C));
		state->fault = APP_FAULT_NONE;
	}
}

static int heater_configure(struct app_state *state)
{
#if HAS_HEATER_OUTPUT
	if (!gpio_is_ready_dt(&heater_gpio)) {
		LOG_ERR("Heater GPIO is not ready");
		state->heater_ready = false;
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&heater_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		LOG_ERR("Heater GPIO configure failed: %d", ret);
		state->heater_ready = false;
		return ret;
	}

	state->heater_ready = true;
	LOG_INF("Heater output ready on %s pin %u", heater_gpio.port->name, heater_gpio.pin);
	return 0;
#else
	LOG_ERR("No heater0 devicetree alias; heater output is unavailable");
	state->heater_ready = false;
	return -ENODEV;
#endif
}

#if HAS_RUN_BUTTON
static void button_pressed_isr(const struct device *dev, struct gpio_callback *cb,
			       uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	if ((pins & BIT(run_button.pin)) == 0) {
		return;
	}

	int64_t now_ms = k_uptime_get();
	if ((now_ms - last_button_irq_ms) >= BUTTON_DEBOUNCE_MS) {
		last_button_irq_ms = now_ms;
		k_sem_give(&run_button_sem);
	}
}
#endif

static int button_configure(void)
{
#if HAS_RUN_BUTTON
	if (!gpio_is_ready_dt(&run_button)) {
		LOG_ERR("Run button GPIO is not ready");
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&run_button, GPIO_INPUT);
	if (ret) {
		LOG_ERR("Run button configure failed: %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&run_button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_ERR("Run button interrupt configure failed: %d", ret);
		return ret;
	}

	gpio_init_callback(&run_button_cb, button_pressed_isr, BIT(run_button.pin));
	ret = gpio_add_callback(run_button.port, &run_button_cb);
	if (ret) {
		LOG_ERR("Run button callback add failed: %d", ret);
		return ret;
	}

	LOG_INF("Run/stop button ready on %s pin %u", run_button.port->name, run_button.pin);
	return 0;
#else
	LOG_WRN("No sw0 devicetree alias; run/stop button is unavailable");
	return -ENODEV;
#endif
}

static bool button_event_pending(void)
{
#if HAS_RUN_BUTTON
	return k_sem_take(&run_button_sem, K_NO_WAIT) == 0;
#else
	return false;
#endif
}

static int sensor_try_init(struct app_state *state, int64_t now_ms)
{
	if (state->sensor_initialized) {
		return 0;
	}

	if (state->last_sensor_retry_ms != 0 &&
	    (now_ms - state->last_sensor_retry_ms) < SENSOR_RETRY_PERIOD_MS) {
		return -EAGAIN;
	}

	state->last_sensor_retry_ms = now_ms;
	int ret = mlx90614_sensor_init();
	if (ret == 0) {
		state->sensor_initialized = true;
		state->sensor_failures = 0;
		LOG_INF("MLX90614 sensor initialized");
	} else {
		state->sensor_initialized = false;
		state->sensor_online = false;
		LOG_WRN("MLX90614 init failed: %d", ret);
	}

	return ret;
}

static void sensor_sample(struct app_state *state, int64_t now_ms)
{
	if (sensor_try_init(state, now_ms) != 0) {
		return;
	}

	float ambient;
	float object;
	int ret = mlx90614_read_temperature(&ambient, &object);
	if (ret == 0) {
		state->ambient_c = ambient;
		state->object_c = object;
		if (!state->have_sample) {
			state->filtered_c = object;
			state->have_sample = true;
		} else {
			state->filtered_c += SENSOR_FILTER_ALPHA * (object - state->filtered_c);
		}

		state->sensor_online = true;
		state->sensor_failures = 0;
		state->last_good_sensor_ms = now_ms;

		clear_recoverable_faults(state);

		if (state->filtered_c >= MAX_SAFE_C) {
			latch_fault(state, APP_FAULT_OVERTEMP);
		}
		return;
	}

	state->sensor_failures++;
	if (state->last_good_sensor_ms == 0 ||
	    (now_ms - state->last_good_sensor_ms) >= SENSOR_TIMEOUT_MS) {
		state->sensor_online = false;
		if (state->running) {
			latch_fault(state, APP_FAULT_SENSOR);
		}
	}

	if (state->sensor_failures >= 3) {
		mlx90614_sensor_deinit();
		state->sensor_initialized = false;
		state->sensor_failures = 0;
	}

	if (state->sensor_failures == 1) {
		LOG_WRN("MLX90614 read failed: %d", ret);
	}
}

static bool can_start(const struct app_state *state)
{
	if (!state->heater_ready) {
		LOG_WRN("can_start: heater not ready");
		return false;
	}

	if (!state->sensor_online || !state->have_sample) {
		LOG_WRN("can_start: sensor not ready (online=%d have_sample=%d)",
			state->sensor_online, state->have_sample);
		return false;
	}

	if (state->filtered_c >= (MAX_SAFE_C - FAULT_RESET_MARGIN_C)) {
		LOG_WRN("can_start: temp %.1f C above reset margin %.1f C",
			(double)state->filtered_c,
			(double)(MAX_SAFE_C - FAULT_RESET_MARGIN_C));
		return false;
	}

	return true;
}

static void toggle_run(struct app_state *state, int64_t now_ms)
{
	if (state->running) {
		LOG_INF("Run stopped by button");
		controller_stop(state);
		return;
	}

	LOG_INF("Start requested");
	if (!can_start(state)) {
		if (!state->heater_ready) {
			state->fault = APP_FAULT_OUTPUT;
			LOG_WRN("Cannot start: heater output is not ready");
		} else if (!state->sensor_online || !state->have_sample) {
			state->fault = APP_FAULT_SENSOR;
			LOG_WRN("Cannot start: waiting for a valid MLX90614 reading");
		} else {
			state->fault = APP_FAULT_OVERTEMP;
			LOG_WRN("Cannot start: temperature is still above the reset margin");
		}
		controller_stop(state);
		return;
	}

	state->fault = APP_FAULT_NONE;
	state->running = true;
	state->duty_pct = 0;
	state->control_integral = 0.0f;
	state->last_control_ms = now_ms - CONTROL_WINDOW_MS;
	state->output_window_start_ms = now_ms;
	state->run_started_ms = now_ms;
	(void)heater_apply(state, false);
	LOG_INF("Run started: setpoint %.1f C", (double)SETPOINT_C);
}

static void control_update(struct app_state *state)
{
	if (!state->running || !state->sensor_online ||
	    !state->have_sample || state->fault != APP_FAULT_NONE) {
		state->duty_pct = 0;
		state->control_integral = 0.0f;
		return;
	}

	float error_c = SETPOINT_C - state->filtered_c;
	if (error_c <= -CONTROL_DEADBAND_C) {
		state->control_integral = clampf(state->control_integral - (CONTROL_KI * 2.0f),
						 0.0f, 100.0f);
		state->duty_pct = 0;
		return;
	}

	state->control_integral = clampf(state->control_integral + (error_c * CONTROL_KI),
					 0.0f, 100.0f);

	float duty = (error_c * CONTROL_KP) + state->control_integral;
	state->duty_pct = (uint8_t)(clampf(duty, 0.0f, 100.0f) + 0.5f);
}

static void heater_window_update(struct app_state *state, int64_t now_ms)
{
	bool enabled = state->running && state->sensor_online &&
		       state->fault == APP_FAULT_NONE && state->duty_pct > 0;

	if (!enabled) {
		if (heater_apply(state, false) != 0 && state->running) {
			latch_fault(state, APP_FAULT_OUTPUT);
		}
		state->output_window_start_ms = now_ms;
		return;
	}

	int64_t offset_ms = now_ms - state->output_window_start_ms;
	if (offset_ms < 0 || offset_ms >= CONTROL_WINDOW_MS) {
		state->output_window_start_ms = now_ms - (offset_ms % CONTROL_WINDOW_MS);
		offset_ms = now_ms - state->output_window_start_ms;
	}

	int64_t on_ms = ((int64_t)CONTROL_WINDOW_MS * state->duty_pct) / 100;
	bool heater_on = offset_ms < on_ms;

	if (heater_apply(state, heater_on) != 0) {
		latch_fault(state, APP_FAULT_OUTPUT);
	}
}

static void log_status(const struct app_state *state)
{
	if (state->have_sample) {
		LOG_INF("%s temp %.1f C set %.1f C duty %u%% heater %s fault %s",
			state->running ? "RUN" : "STOP",
			(double)state->filtered_c,
			(double)SETPOINT_C,
			state->duty_pct,
			state->heater_on ? "on" : "off",
			fault_text(state->fault));
	} else {
		LOG_INF("%s waiting for sensor duty %u%% heater %s fault %s",
			state->running ? "RUN" : "STOP",
			state->duty_pct,
			state->heater_on ? "on" : "off",
			fault_text(state->fault));
	}
}

#ifdef CONFIG_FIELDSTERILIZATION_I2C_SCAN_ON_BOOT
static void i2c_scan(void)
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c21), okay)
	const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c21));
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(i2c_sensor), okay)
	const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c_sensor));
#else
	const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
#endif

	if (!device_is_ready(i2c)) {
		printk("I2C not ready\n");
		return;
	}

	printk("I2C scan (0x08..0x77):\n");
	int found = 0;
	for (uint8_t addr = 0x08; addr < 0x78; addr++) {
		uint8_t dummy;
		struct i2c_msg msg = {
			.buf = &dummy,
			.len = 0,
			.flags = I2C_MSG_WRITE | I2C_MSG_STOP,
		};

		if (i2c_transfer(i2c, &msg, 1, addr) == 0) {
			printk("  found 0x%02x\n", addr);
			found++;
		}
	}
	if (!found) {
		printk("  no devices found\n");
	}
}
#endif

#ifdef CONFIG_DISPLAY
#define ARC_ANGLE_START  135
#define ARC_ANGLE_RANGE  270

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} rgb8_t;

static lv_obj_t *temp_arc;
static lv_obj_t *target_marker_arc;
static lv_obj_t *output_arc;
static lv_obj_t *temp_label;
static lv_obj_t *setpoint_label;
static lv_obj_t *status_label;
static lv_obj_t *duty_label;
static lv_obj_t *heater_label;
static lv_style_t style_arc_bg;
static lv_style_t style_temp_arc;
static lv_style_t style_target_marker;
static lv_style_t style_output_arc_bg;
static lv_style_t style_output_arc;
static lv_style_t style_temp_label;
static lv_style_t style_setpoint_label;
static lv_style_t style_status_label;
static lv_style_t style_duty_label;
static lv_style_t style_heater_label;

static rgb8_t rgb8_lerp(rgb8_t a, rgb8_t b, uint8_t t)
{
	return (rgb8_t) {
		.r = (uint8_t)((a.r * (255 - t) + b.r * t) / 255),
		.g = (uint8_t)((a.g * (255 - t) + b.g * t) / 255),
		.b = (uint8_t)((a.b * (255 - t) + b.b * t) / 255),
	};
}

static lv_color_t rgb8_to_lv(rgb8_t color)
{
	return lv_color_make(color.r, color.g, color.b);
}

static lv_color_t control_color(const struct app_state *state)
{
	static const rgb8_t COOL = {0x22, 0x8C, 0xFF};
	static const rgb8_t WARM = {0xFF, 0xB3, 0x22};
	static const rgb8_t GOOD = {0x38, 0xE0, 0x76};
	static const rgb8_t HOT = {0xFF, 0x35, 0x42};

	if (!state->have_sample || !state->sensor_online) {
		return lv_color_hex(0x5E6875);
	}
	if (state->fault != APP_FAULT_NONE) {
		return lv_color_hex(0xFF3542);
	}

	float error_c = SETPOINT_C - state->filtered_c;
	if (error_c > 10.0f) {
		return rgb8_to_lv(COOL);
	}
	if (error_c > 1.0f) {
		uint8_t t = (uint8_t)(((10.0f - error_c) / 9.0f) * 255.0f);
		return rgb8_to_lv(rgb8_lerp(COOL, WARM, t));
	}
	if (error_c >= -1.0f) {
		return rgb8_to_lv(GOOD);
	}
	if (error_c >= -8.0f) {
		uint8_t t = (uint8_t)(((-error_c - 1.0f) / 7.0f) * 255.0f);
		return rgb8_to_lv(rgb8_lerp(GOOD, HOT, t));
	}
	return rgb8_to_lv(HOT);
}

static int32_t temp_to_visual_value(float temp_c)
{
	float clamped = clampf(temp_c, VISUAL_MIN_C, VISUAL_MAX_C);
	float span = VISUAL_MAX_C - VISUAL_MIN_C;

	return (int32_t)(((clamped - VISUAL_MIN_C) * 1000.0f / span) + 0.5f);
}

static uint16_t visual_value_to_angle(int32_t value)
{
	value = CLAMP(value, 0, 1000);
	return ARC_ANGLE_START + (uint16_t)((ARC_ANGLE_RANGE * value) / 1000);
}

static void display_backlight_full(void)
{
#if HAS_DISPLAY_BACKLIGHT
	if (!gpio_is_ready_dt(&display_backlight)) {
		LOG_WRN("Display backlight GPIO is not ready");
		return;
	}

	int ret = gpio_pin_configure_dt(&display_backlight, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		LOG_WRN("Display backlight GPIO configure failed: %d", ret);
		return;
	}

	LOG_INF("Display backlight on %s pin %u", display_backlight.port->name,
		display_backlight.pin);
#endif
}

static void ui_create(lv_obj_t *screen, lv_coord_t w, lv_coord_t h)
{
	lv_coord_t size = MIN(w, h) - 10;
	lv_coord_t output_size = size - 54;

	lv_obj_set_style_bg_color(screen, lv_color_black(), LV_PART_MAIN);
	lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

	lv_style_init(&style_arc_bg);
	lv_style_set_arc_color(&style_arc_bg, lv_color_hex(0x101820));
	lv_style_set_arc_width(&style_arc_bg, 14);

	lv_style_init(&style_temp_arc);
	lv_style_set_arc_color(&style_temp_arc, lv_color_hex(0x26D9FF));
	lv_style_set_arc_width(&style_temp_arc, 14);
	lv_style_set_arc_rounded(&style_temp_arc, true);

	temp_arc = lv_arc_create(screen);
	lv_obj_set_size(temp_arc, size, size);
	lv_obj_center(temp_arc);
	lv_obj_remove_style(temp_arc, NULL, LV_PART_KNOB);
	lv_obj_set_style_pad_all(temp_arc, 0, LV_PART_KNOB);
	lv_obj_add_style(temp_arc, &style_arc_bg, LV_PART_MAIN);
	lv_obj_add_style(temp_arc, &style_temp_arc, LV_PART_INDICATOR);
	lv_arc_set_bg_angles(temp_arc, ARC_ANGLE_START, ARC_ANGLE_START + ARC_ANGLE_RANGE);
	lv_arc_set_angles(temp_arc, ARC_ANGLE_START, ARC_ANGLE_START);
	lv_arc_set_range(temp_arc, 0, 1000);
	lv_obj_clear_flag(temp_arc, LV_OBJ_FLAG_CLICKABLE);

	lv_style_init(&style_target_marker);
	lv_style_set_arc_color(&style_target_marker, lv_color_white());
	lv_style_set_arc_width(&style_target_marker, 5);
	lv_style_set_arc_rounded(&style_target_marker, true);

	target_marker_arc = lv_arc_create(screen);
	lv_obj_set_size(target_marker_arc, size - 2, size - 2);
	lv_obj_center(target_marker_arc);
	lv_obj_remove_style(target_marker_arc, NULL, LV_PART_KNOB);
	lv_obj_set_style_pad_all(target_marker_arc, 0, LV_PART_KNOB);
	lv_obj_add_style(target_marker_arc, &style_target_marker, LV_PART_MAIN);
	lv_obj_set_style_arc_opa(target_marker_arc, LV_OPA_TRANSP, LV_PART_INDICATOR);
	lv_arc_set_range(target_marker_arc, 0, 1000);
	lv_obj_clear_flag(target_marker_arc, LV_OBJ_FLAG_CLICKABLE);

	int32_t target_value = temp_to_visual_value(SETPOINT_C);
	uint16_t target_angle = visual_value_to_angle(target_value);
	lv_arc_set_bg_angles(target_marker_arc, target_angle - 3, target_angle + 3);
	lv_arc_set_angles(target_marker_arc, target_angle - 3, target_angle + 3);

	lv_style_init(&style_output_arc_bg);
	lv_style_set_arc_color(&style_output_arc_bg, lv_color_hex(0x151515));
	lv_style_set_arc_width(&style_output_arc_bg, 8);

	lv_style_init(&style_output_arc);
	lv_style_set_arc_color(&style_output_arc, lv_color_hex(0xFFB000));
	lv_style_set_arc_width(&style_output_arc, 8);
	lv_style_set_arc_rounded(&style_output_arc, true);

	output_arc = lv_arc_create(screen);
	lv_obj_set_size(output_arc, output_size, output_size);
	lv_obj_center(output_arc);
	lv_obj_remove_style(output_arc, NULL, LV_PART_KNOB);
	lv_obj_set_style_pad_all(output_arc, 0, LV_PART_KNOB);
	lv_obj_add_style(output_arc, &style_output_arc_bg, LV_PART_MAIN);
	lv_obj_add_style(output_arc, &style_output_arc, LV_PART_INDICATOR);
	lv_arc_set_bg_angles(output_arc, 205, 335);
	lv_arc_set_angles(output_arc, 205, 205);
	lv_arc_set_range(output_arc, 0, 100);
	lv_obj_clear_flag(output_arc, LV_OBJ_FLAG_CLICKABLE);

	lv_style_init(&style_status_label);
	lv_style_set_text_color(&style_status_label, lv_color_hex(0x88A0AA));
	lv_style_set_text_font(&style_status_label, &lv_font_montserrat_18);
	status_label = lv_label_create(screen);
	lv_obj_add_style(status_label, &style_status_label, LV_PART_MAIN);
	lv_obj_set_width(status_label, 180);
	lv_obj_set_style_text_align(status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	lv_label_set_text(status_label, "SENSOR");
	lv_obj_align(status_label, LV_ALIGN_CENTER, 0, -67);

	lv_style_init(&style_temp_label);
	lv_style_set_text_color(&style_temp_label, lv_color_hex(0x26D9FF));
	lv_style_set_text_font(&style_temp_label, &lv_font_montserrat_40);
	temp_label = lv_label_create(screen);
	lv_obj_add_style(temp_label, &style_temp_label, LV_PART_MAIN);
	lv_obj_set_width(temp_label, 210);
	lv_obj_set_style_text_align(temp_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	lv_label_set_text(temp_label, "--.- C");
	lv_obj_align(temp_label, LV_ALIGN_CENTER, 0, -20);

	lv_style_init(&style_setpoint_label);
	lv_style_set_text_color(&style_setpoint_label, lv_color_hex(0xC9D3D8));
	lv_style_set_text_font(&style_setpoint_label, &lv_font_montserrat_18);
	setpoint_label = lv_label_create(screen);
	lv_obj_add_style(setpoint_label, &style_setpoint_label, LV_PART_MAIN);
	lv_obj_set_width(setpoint_label, 190);
	lv_obj_set_style_text_align(setpoint_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	lv_label_set_text(setpoint_label, "SET --.- C");
	lv_obj_align(setpoint_label, LV_ALIGN_CENTER, 0, 26);

	lv_style_init(&style_duty_label);
	lv_style_set_text_color(&style_duty_label, lv_color_hex(0xFFB000));
	lv_style_set_text_font(&style_duty_label, &lv_font_montserrat_16);
	duty_label = lv_label_create(screen);
	lv_obj_add_style(duty_label, &style_duty_label, LV_PART_MAIN);
	lv_obj_set_width(duty_label, 170);
	lv_obj_set_style_text_align(duty_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	lv_label_set_text(duty_label, "OUT 0%");
	lv_obj_align(duty_label, LV_ALIGN_CENTER, 0, 58);

	lv_style_init(&style_heater_label);
	lv_style_set_text_color(&style_heater_label, lv_color_hex(0x6E7B82));
	lv_style_set_text_font(&style_heater_label, &lv_font_montserrat_14);
	heater_label = lv_label_create(screen);
	lv_obj_add_style(heater_label, &style_heater_label, LV_PART_MAIN);
	lv_obj_set_width(heater_label, 150);
	lv_obj_set_style_text_align(heater_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	lv_label_set_text(heater_label, "HEAT OFF");
	lv_obj_align(heater_label, LV_ALIGN_CENTER, 0, 80);
}

static void ui_update(const struct app_state *state)
{
	char buf[32];
	lv_color_t color = control_color(state);

	if (state->have_sample) {
		snprintk(buf, sizeof(buf), "%.1f C", (double)state->filtered_c);
		lv_label_set_text(temp_label, buf);
		lv_arc_set_value(temp_arc, temp_to_visual_value(state->filtered_c));
	} else {
		lv_label_set_text(temp_label, "--.- C");
		lv_arc_set_value(temp_arc, 0);
	}

	snprintk(buf, sizeof(buf), "SET %.1f C", (double)SETPOINT_C);
	lv_label_set_text(setpoint_label, buf);

	if (state->fault != APP_FAULT_NONE) {
		snprintk(buf, sizeof(buf), "FAULT %s", fault_text(state->fault));
		lv_label_set_text(status_label, buf);
		lv_obj_set_style_text_color(status_label, lv_color_hex(0xFF3542), LV_PART_MAIN);
	} else if (state->running) {
		lv_label_set_text(status_label, "RUN");
		lv_obj_set_style_text_color(status_label, lv_color_hex(0x38E076), LV_PART_MAIN);
	} else if (state->sensor_online) {
		lv_label_set_text(status_label, "READY");
		lv_obj_set_style_text_color(status_label, lv_color_hex(0xDCE6EA), LV_PART_MAIN);
	} else {
		lv_label_set_text(status_label, "SENSOR");
		lv_obj_set_style_text_color(status_label, lv_color_hex(0xFFB322), LV_PART_MAIN);
	}

	snprintk(buf, sizeof(buf), "OUT %u%%", state->duty_pct);
	lv_label_set_text(duty_label, buf);

	if (state->heater_on) {
		lv_label_set_text(heater_label, "HEAT ON");
		lv_obj_set_style_text_color(heater_label, lv_color_hex(0xFFB000), LV_PART_MAIN);
	} else {
		lv_label_set_text(heater_label, "HEAT OFF");
		lv_obj_set_style_text_color(heater_label, lv_color_hex(0x6E7B82), LV_PART_MAIN);
	}

	lv_obj_set_style_arc_color(temp_arc, color, LV_PART_INDICATOR);
	lv_obj_set_style_text_color(temp_label, color, LV_PART_MAIN);
	lv_obj_set_style_arc_color(output_arc,
				   state->heater_on ? lv_color_hex(0xFFB000) :
						      lv_color_hex(0x5A4000),
				   LV_PART_INDICATOR);
	lv_arc_set_value(output_arc, state->duty_pct);
}

static int display_setup(void)
{
	const struct device *display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Display device not ready");
		return -ENODEV;
	}

	int ret = display_blanking_off(display_dev);
	if (ret < 0 && ret != -ENOSYS) {
		LOG_WRN("Display blanking off failed: %d", ret);
	}

	ret = display_set_brightness(display_dev, 255);
	if (ret) {
		LOG_DBG("Display brightness API unavailable: %d", ret);
	}
	display_backlight_full();

	lv_coord_t scr_w;
	lv_coord_t scr_h;
#if LVGL_VERSION_MAJOR >= 9
	lv_display_t *disp = lv_display_get_default();
	scr_w = lv_display_get_horizontal_resolution(disp);
	scr_h = lv_display_get_vertical_resolution(disp);
#else
	lv_disp_t *disp = lv_disp_get_default();
	lv_disp_set_bg_opa(disp, LV_OPA_COVER);
	scr_w = lv_disp_get_hor_res(disp);
	scr_h = lv_disp_get_ver_res(disp);
#endif

	LOG_INF("Display ready: %dx%d", scr_w, scr_h);
	lvgl_lock();
	ui_create(lv_scr_act(), scr_w, scr_h);
	lv_timer_handler();
	lvgl_unlock();
	return 0;
}
#endif /* CONFIG_DISPLAY */

static int cmd_run_toggle(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	toggle_run(&app, k_uptime_get());
	return 0;
}

SHELL_CMD_REGISTER(heater_run, NULL, "Toggle heater run/stop", cmd_run_toggle);

static int cmd_run_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (app.running) {
		controller_stop(&app);
	}
	return 0;
}

SHELL_CMD_REGISTER(heater_stop, NULL, "Stop heater and clear duty cycle", cmd_run_stop);

int main(void)
{
	int64_t last_sample_ms = 0;
	int64_t last_log_ms = 0;
#ifdef CONFIG_DISPLAY
	int64_t last_ui_ms = 0;
#endif

	k_msleep(500);

#ifdef CONFIG_FIELDSTERILIZATION_I2C_SCAN_ON_BOOT
	i2c_scan();
#endif

	(void)heater_configure(&app);
	(void)button_configure();
	(void)status_led_configure();

#ifdef CONFIG_DISPLAY
	int ret = display_setup();
	if (ret) {
		return ret;
	}
#endif

	int64_t now_ms = k_uptime_get();
	(void)sensor_try_init(&app, now_ms);

	while (1) {
		now_ms = k_uptime_get();

		while (button_event_pending()) {
			toggle_run(&app, now_ms);
		}

		if (elapsed(now_ms, &last_sample_ms, SAMPLE_PERIOD_MS)) {
			sensor_sample(&app, now_ms);

			if (elapsed(now_ms, &app.last_control_ms, CONTROL_WINDOW_MS)) {
				control_update(&app);
			}

			heater_window_update(&app, now_ms);
		}

		status_led_update(&app, now_ms);

#ifdef CONFIG_DISPLAY
		if (elapsed(now_ms, &last_ui_ms, UI_PERIOD_MS)) {
			lvgl_lock();
			ui_update(&app);
			lvgl_unlock();
		}
		lvgl_lock();
		lv_timer_handler();
		lvgl_unlock();
#endif

		if (elapsed(now_ms, &last_log_ms, LOG_PERIOD_MS)) {
			log_status(&app);
		}

		k_msleep(10);
	}

	return 0;
}
